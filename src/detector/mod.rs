//! A sparse-sampling AprilTag detector, after napa-vision (Zachary Wilson, GPL-3.0).
//!
//! Per frame, level by level (each level doubles the sampling stride):
//!   1. Rake: point-sample the image every 3/4 of the smallest cell, so every border crossing contains
//!      a sample from its dark core. Coarser levels take every second sample of the level before.
//!   2. Threshold each sample against its 4×4-tile min/max over a 3×3 tile neighbourhood, skipping tiles
//!      whose contrast is under the noise floor measured from the frame. Rows of dark samples become
//!      runs, joined into blobs by union-find.
//!   3. Keep tag-sized, tag-shaped blobs; each one's extreme points give a rough quad.
//!   4. Fit each side to its full-resolution edge, decode once cheaply to reject non-tags, then refine
//!      coarse to fine and intersect the four lines for the corners.
//!   5. Sample the cell grid through the homography and decode with multi-index hashing.
//!   6. Repair what a plain fit gets wrong: a quiet zone darker than the background, a side fitted one
//!      cell out, and edges biased by clipped exposure.

mod decode;
mod edges;
pub mod families;
mod family;
mod geometry;
mod image;
mod level;
mod segment;

use thiserror::Error;

use decode::{border_bias, border_plausible, decode_quad, min_border};
use edges::{EdgeLevels, FitOptions, fit_sides};
pub use family::Family;
use family::Layout;
use geometry::{Homography, Quad, Vec2, grow_quad, perimeter};
use image::Sampler;
pub use image::{ImageError, ImageView};
use level::Floors;
use segment::{Component, Segmenter};

/// A level keeps blobs whose bounding box spans up to this many samples (for an 8-cell tag): a tag
/// stays isolated from its background where its one-cell margin spans about two samples, and its border
/// stays solid under the tile threshold up to a side of about 64 samples (96 rotated). The band reaches
/// 192 so large low-contrast tags are proposed before their thin quiet zone merges into the background.
const BAND_HI: usize = 192;
/// Levels continue until the frame is under this many samples across (for an 8-cell tag).
const LAST_LEVEL: usize = 144;
/// The final fit takes a profile every 1.5 px along each side, up to this many.
const FINAL_POINTS: usize = 96;
/// Noise floor tuning: sparse grids cap the floor because texture inflates the estimate; a noisy
/// sparse grid re-rakes with block averages.
const FLOOR_SIGMAS: f64 = 6.0;
const FLOOR_CAP: u8 = 64;
const NOISY_FLOOR: u8 = 42;
const BOX_SIGMAS: f64 = 7.0;

/// Detector tuning.
#[derive(Clone, Debug)]
pub struct Config {
    /// Smallest tag (black square side, in pixels) the detector guarantees to sample; 8..=30000.
    pub min_tag_px: u32,
    /// Payload bits to correct, or None for the family's recommended value; capped by the family.
    pub max_hamming: Option<u32>,
    /// Grey levels required between tag black and the white quiet zone; 1..=255.
    pub min_contrast: u8,
    /// Dark border cells required, out of tag36h11's 28; 0..=28.
    pub min_border_cells: u32,
    /// Cell sharpening strength, 0..=1; upstream AprilTag uses 0.25.
    pub sharpening: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self { min_tag_px: 32, max_hamming: None, min_contrast: 20, min_border_cells: 22, sharpening: 0.25 }
    }
}

#[derive(Debug, Error, PartialEq)]
pub enum ConfigError {
    #[error("min_tag_px must be 8..=30000")]
    MinTagPx,
    #[error("min_contrast must be at least 1")]
    MinContrast,
    #[error("min_border_cells must be 0..=28")]
    MinBorderCells,
    #[error("sharpening must be 0..=1")]
    Sharpening,
}

/// Validated settings, with the family's Hamming limit applied.
#[derive(Clone, Copy)]
pub(crate) struct Settings {
    min_tag_px: usize,
    max_hamming: u32,
    min_contrast: u8,
    min_border_cells: u32,
    sharpening: f64,
}

/// One decoded tag.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Detection {
    pub id: u32,
    /// Payload bits corrected.
    pub hamming: u32,
    /// The smaller of the white and black payload bits' mean distance from the threshold, in grey levels.
    pub decision_margin: f32,
    /// Corners in pixel-centre coordinates, in napa-vision's (OpenCV's AprilTag) order.
    pub corners: [[f64; 2]; 4],
}

impl Detection {
    pub fn center(&self) -> [f64; 2] {
        let s = self.corners.iter().fold([0.0, 0.0], |a, c| [a[0] + c[0], a[1] + c[1]]);
        [0.25 * s[0], 0.25 * s[1]]
    }
}

/// A detector for one tag family. It owns reusable buffers: use one per camera stream.
pub struct Detector {
    layout: Layout,
    settings: Settings,
    level: Vec<u8>,
    floors: Floors,
    seg: Segmenter,
    slot: Vec<i32>,
    cands: Vec<Component>,
    box_acc: Vec<u32>,
    box_hs: Vec<u32>,
    /// This frame's failed candidates (fitted quad and stride), to skip the same blob at a coarser level.
    failed: Vec<(Quad, usize)>,
}

impl Detector {
    pub fn new(family: &'static Family, config: &Config) -> Result<Self, ConfigError> {
        let layout = Layout::new(family);
        if !(8..=30000).contains(&config.min_tag_px) {
            return Err(ConfigError::MinTagPx);
        }
        if config.min_contrast == 0 {
            return Err(ConfigError::MinContrast);
        }
        if config.min_border_cells > 28 {
            return Err(ConfigError::MinBorderCells);
        }
        if !(0.0..=1.0).contains(&config.sharpening) {
            return Err(ConfigError::Sharpening);
        }
        let max_hamming = config.max_hamming.unwrap_or(layout.recommended_hamming).min(family.max_hamming());
        let settings = Settings {
            min_tag_px: config.min_tag_px as usize,
            max_hamming,
            min_contrast: config.min_contrast,
            min_border_cells: config.min_border_cells,
            sharpening: config.sharpening,
        };
        Ok(Self {
            layout,
            settings,
            level: Vec::new(),
            floors: Floors::default(),
            seg: Segmenter::default(),
            slot: Vec::new(),
            cands: Vec::new(),
            box_acc: Vec::new(),
            box_hs: Vec::new(),
            failed: Vec::new(),
        })
    }

    pub fn family(&self) -> &'static Family {
        self.layout.family
    }

    /// Detect tags, replacing the contents of `out`. Frames under 16 pixels on a side have no tags.
    pub fn detect_into(&mut self, img: &ImageView, out: &mut Vec<Detection>) {
        out.clear();
        let (width, height) = (img.width(), img.height());
        if width < 16 || height < 16 || width > u16::MAX as usize || height > u16::MAX as usize {
            return;
        }
        let view = Sampler { img: *img, invert: self.layout.family.reversed };
        let w = self.layout.width;
        let cells = self.layout.cells;
        let st = self.settings;
        // Sampling every 3/4 cell guarantees a sample inside the dark core of each border cell.
        let base = (st.min_tag_px * 3 / (4 * w)).max(1);
        let mut contrast = st.min_contrast;
        self.failed.clear();
        // Size band in level samples; adjacent levels overlap, since a blob's extent varies with phase,
        // blur and rotation.
        let lo = (7 * w + 4) / 8;
        let hi = BAND_HI * w / 8;
        let mut prev_lw = 0;
        let mut floor_stride = base;
        let mut s = base;
        loop {
            let lw = width.div_ceil(s);
            let lh = height.div_ceil(s);
            let last = lw < LAST_LEVEL * w / 8 || lh < LAST_LEVEL * w / 8;
            // 64 spare bytes: thresholding reads whole 64-sample words past the last row.
            // Never shrink: a coarser level is made in place from the finer one.
            if self.level.len() < lw * lh + 64 {
                self.level.resize(lw * lh + 64, 0);
            }
            if s == base {
                level::rake(&mut self.level, lw, lh, img, s);
                if view.invert {
                    level::invert(&mut self.level[..lw * lh]);
                }
            } else {
                level::halve(&mut self.level, lw, lh, prev_lw);
            }
            prev_lw = lw;
            if s == base || (base == 1 && s == 2 && contrast > NOISY_FLOOR) {
                // Don't blur stride-one proposals: a tiny border can be one pixel wide. Its higher floor
                // suppresses raw noise; the next, averaged level restores low contrast.
                floor_stride = s;
                let dense = s == 1 && !last;
                let (sigmas, cap) = if dense { (7.0, 255) } else { (FLOOR_SIGMAS, FLOOR_CAP) };
                contrast = self.floors.measure(&self.level, lw, lh, st.min_contrast, sigmas, cap);
                // A noisy frame: rake again averaging each sample's block, and re-measure.
                if contrast > NOISY_FLOOR && s > 1 && s <= 16 {
                    let kernel = if base == 1 && s == 2 { 3 } else { s };
                    level::box_rake(&mut self.level, lw, lh, img, s, kernel, &mut self.box_acc, &mut self.box_hs);
                    if view.invert {
                        level::invert(&mut self.level[..lw * lh]);
                    }
                    // Stride-two 3×3 means share a column: their difference has sqrt(2/3) of their noise.
                    let sigmas = if base == 1 && s == 2 { BOX_SIGMAS * 1.5f64.sqrt() } else { BOX_SIGMAS };
                    contrast = self.floors.measure(&self.level, lw, lh, st.min_contrast, sigmas, FLOOR_CAP);
                }
            }
            self.seg.threshold(&self.level, lw, lh, &self.floors, s / floor_stride);
            self.seg.label();
            self.collect_candidates(s, base, last, lo, hi);
            for ci in 0..self.cands.len() {
                let comp = self.cands[ci].clone();
                if let Some(d) = self.examine(&view, &comp, s, last, cells, out) {
                    out.push(d);
                }
            }
            if last {
                break;
            }
            s *= 2;
        }
    }

    /// Detect tags into a new vector.
    pub fn detect(&mut self, img: &ImageView) -> Vec<Detection> {
        let mut out = Vec::new();
        self.detect_into(img, &mut out);
        out
    }

    /// Keep tag-sized, tag-shaped blobs as candidates, with their extreme points.
    fn collect_candidates(&mut self, s: usize, base: usize, last: bool, min_size: usize, hi: usize) {
        // Dense levels propose small tags only; larger tags are cheaper on the next level.
        let max_size = if last { usize::MAX } else { hi };
        self.cands.clear();
        self.slot.clear();
        for b in &self.seg.boxes {
            let bw = usize::from(b.maxx - b.minx) + 1;
            let bh = usize::from(b.maxy - b.miny) + 1;
            let (big, small) = (bw.max(bh), bw.min(bh));
            // An 8:1 box admits tilts to about 80 degrees; decoding rejects thin blobs that aren't tags.
            let keep = !(!last && base == 1 && s <= 2 && small > 72)
                && small >= min_size
                && big <= max_size
                && big <= 8 * small
                && f64::from(b.count) / (bw * bh) as f64 >= 0.2;
            if keep {
                self.slot.push(self.cands.len() as i32);
                self.cands.push(Component::new(*b));
            } else {
                self.slot.push(-1);
            }
        }
        for (r, &p) in self.seg.runs.iter().zip(&self.seg.parent) {
            let c = self.slot[(-p - 1) as usize];
            if c >= 0 {
                self.cands[c as usize].add_run(r);
            }
        }
        for c in &mut self.cands {
            c.flush_row();
        }
    }

    /// Fit, decode and refine one candidate blob found at stride `s`.
    fn examine(
        &mut self,
        view: &Sampler,
        comp: &Component,
        s: usize,
        last: bool,
        cells: f64,
        out: &[Detection],
    ) -> Option<Detection> {
        let st = self.settings;
        let w = self.layout.width;
        let b = comp.bbox;
        let bw = usize::from(b.maxx - b.minx) + 1;
        let bh = usize::from(b.maxy - b.miny) + 1;
        let big = bw.max(bh);
        let sf = s as f64;
        // Levels run fine to coarse: skip a blob whose centre a finer level already decoded.
        let cc = Vec2::new(0.5 * f64::from(b.minx + b.maxx) * sf, 0.5 * f64::from(b.miny + b.maxy) * sf);
        let radius = 0.3 * big as f64 * sf + 2.0;
        if out.iter().any(|d| {
            let c = d.center();
            (c[0] - cc.x).abs() + (c[1] - cc.y).abs() < radius
        }) {
            return None;
        }
        let rough = rough_quad(comp, s)?;
        let pick_area = geometry::signed_area(&rough).abs() / (sf * sf);
        let perim = perimeter(&rough) / sf;
        // At most ~54/64 of a tag's square is dark; solid strokes and filled shapes run well past that.
        if f64::from(comp.count) > self.layout.fill_limit * (pick_area + 0.5 * perim + 1.0) {
            return None;
        }
        if pick_area < 0.3 * (bw * bh) as f64 {
            return None;
        }
        // Paired border samples reject noise before the expensive edge profiles. Permit one weak side,
        // and leave terminal proposals unscreened.
        if s <= 2 && !last && st.min_border_cells >= 22 && !sides_contrast(view, &rough, sf, cells, st.min_contrast) {
            return None;
        }
        let fit = |q: &Quad, r_out: f64, r_in: f64, outermost: bool, max_points: usize, levels: Option<EdgeLevels>| {
            fit_sides(
                view,
                q,
                &FitOptions { r_out, r_in, min_edge: f64::from(st.min_contrast), outermost, max_points, levels, cells },
            )
        };
        // Rough corners are dark samples, so the true border lies outward by at most one stride: search
        // mostly outward, which keeps payload edges out of the window.
        let mut reach = 0.5 * ((1.5 * sf + 1.0) + (0.5 * sf + 1.0));
        let (mut fine, mut edge_w) = fit(&rough, 1.5 * sf + 1.0, 0.5 * sf + 1.0, true, 6, None)?;
        // Levels overlap, so a blob that failed at a finer level often comes back: the early decode
        // depends only on the fitted quad, so one within 0.3 px of a failure fails again.
        if self.failed_before(&fine, s, 0.3) {
            return None;
        }
        // The first fit is within a fraction of a cell: decode now with two bits of slack, and refine
        // only quads that are tags.
        let mut probe = fine;
        let early = match decode_quad(&mut self.layout, view, &mut probe, &st, 2) {
            Some(d) => d,
            // The blob may include a quiet zone darker than the background around it (a grey card on a
            // bright wall): then the quad is the margin's outline, and the square its inner W/(W+2).
            None => match self.rescue_margin(view, &fine, cells) {
                Some((q, r, d)) => {
                    fine = q;
                    reach = r;
                    d
                }
                None => {
                    self.failed.push((fine, s));
                    return None;
                }
            },
        };
        let lv = EdgeLevels { black: early.levels.0, white: early.levels.1 };
        let cell = perimeter(&fine) / (4.0 * w as f64);
        // Coarse to fine: shrink the window to the previous pass's resolution, but never below the
        // measured edge width, and within half a cell so it never reaches the next payload edge.
        for pass in 0..5 {
            let step = 2.0 * reach / 14.0;
            let mut next = (1.5 * step).max(1.5).max(1.25 * edge_w);
            if next > 0.45 * cell && 0.45 * cell >= 1.0 {
                next = 0.45 * cell;
            }
            if pass > 0 && (next >= 0.9 * reach || step <= 0.35) {
                break;
            }
            reach = next;
            (fine, edge_w) = fit(&fine, reach, reach, false, 8, Some(lv))?;
        }
        // One more pass at the converged window, with every edge sample.
        let (mut finer, mut edge_w) = fit(&fine, reach, reach, false, FINAL_POINTS, Some(lv))?;
        let decoded = decode_quad(&mut self.layout, view, &mut finer, &st, 0)?;
        let mut d = decoded.detection;
        let final_lv = decoded.levels;

        // A side whose border cells read white was fitted one cell out, on the quiet zone's outer edge
        // (the payload still decodes through error correction). Move it in a cell and refit.
        let bad: u32 = (0..4).map(|k| u32::from(decoded.side_level[k] > 0.3) << k).sum();
        if bad != 0 && bad != 15 {
            let level_before: f64 = decoded.side_level.iter().sum();
            let h = Homography::square_to_quad(&finer);
            let (one, rest) = (1.0 / w as f64, (w as f64 - 1.0) / w as f64);
            let pick = |bit: u32, moved: f64, keep: f64| if bad & bit != 0 { moved } else { keep };
            let moved = h.sub_quad(pick(8, one, 0.0), pick(1, one, 0.0), pick(2, rest, 1.0), pick(4, rest, 1.0));
            let r = 0.45 * perimeter(&moved) / (4.0 * w as f64);
            let r2 = (1.25 * edge_w).max(1.5);
            // The moved side starts tilted: fit, then refit re-centred in a window just wide enough.
            let refit = (r >= 1.0).then(|| fit(&moved, r, r, false, FINAL_POINTS, Some(lv))).flatten().and_then(
                |(first, ew)| {
                    edge_w = ew;
                    if r2 < r { fit(&first, r2, r2, false, FINAL_POINTS, Some(lv)).map(|f| f.0) } else { Some(first) }
                },
            );
            if let Some(mut fixed) = refit
                && let Some(d2) = decode_quad(&mut self.layout, view, &mut fixed, &st, 0)
                && d2.detection.id == d.id
                && d2.side_level.iter().sum::<f64>() < level_before - 0.2
            {
                d = d2.detection;
                finer = fixed;
            }
        }
        // Clipped white (or black) biases every edge toward the clipped side: measure the bias from the
        // border's own width and undo it, only when clipping is visible.
        let (black, white) = final_lv;
        if (white >= 250.0 || black <= 4.0)
            && let Some((delta, n)) = border_bias(&self.layout, view, &finer, black, white)
            && n >= 4
            && delta.abs() > 0.02
            && delta.abs() < 1.5
        {
            let mut grown = grow_quad(&finer, delta);
            if let Some(d3) = decode_quad(&mut self.layout, view, &mut grown, &st, 0)
                && d3.detection.id == d.id
            {
                d = d3.detection;
            }
        }
        let c = d.center();
        let size = (d.corners[0][0] - d.corners[2][0]).abs() + (d.corners[0][1] - d.corners[2][1]).abs();
        let dup = out.iter().any(|o| {
            let oc = o.center();
            (oc[0] - c[0]).abs() + (oc[1] - c[1]).abs() < 0.25 * size + 2.0
        });
        (!dup).then_some(d)
    }

    /// Refit a failed quad's inner W/(W+2), where the black square would be if the quad is a grey quiet
    /// zone's outline. Returns the refitted quad, its window, and the decode.
    fn rescue_margin(&mut self, view: &Sampler, fine: &Quad, cells: f64) -> Option<(Quad, f64, decode::Decoded)> {
        let st = &self.settings;
        let w = self.layout.width as f64;
        let h = Homography::square_to_quad(fine);
        let (m0, m1) = (1.0 / (w + 2.0), (w + 1.0) / (w + 2.0));
        let inner = h.sub_quad(m0, m0, m1, m1);
        let r = 0.45 * perimeter(fine) / (4.0 * (w + 2.0)); // 0.45 of a cell
        // Most failed decodes aren't tags: refit only if the inner square already shows a dark border
        // inside a lighter margin, with loose limits.
        if r < 1.5
            || !border_plausible(
                &self.layout,
                view,
                &inner,
                f64::from(st.min_contrast / 2),
                min_border(&self.layout, st.min_border_cells) - 6,
            )
        {
            return None;
        }
        let opts = FitOptions {
            r_out: r,
            r_in: r,
            min_edge: f64::from(st.min_contrast),
            outermost: false,
            max_points: 8,
            levels: None,
            cells,
        };
        let (finer, _) = fit_sides(view, &inner, &opts)?;
        let mut probe = finer;
        let d = decode_quad(&mut self.layout, view, &mut probe, &self.settings, 2)?;
        Some((finer, r, d))
    }

    /// Whether a finer level already rejected this quad: every corner within `tol` of a failed quad's,
    /// in any cyclic order.
    fn failed_before(&self, q: &Quad, s: usize, tol: f64) -> bool {
        self.failed.iter().filter(|(_, fs)| *fs < s).any(|(f, _)| {
            (0..4).any(|shift| {
                (0..4).all(|k| {
                    let p = f[(k + shift) & 3];
                    (q[k].x - p.x).abs() <= tol && (q[k].y - p.y).abs() <= tol
                })
            })
        })
    }
}

/// Rough quad from a blob's extreme points: of the 16 extremes (clockwise), the four that span the
/// largest quadrilateral. Start from the best of the eight axis and diagonal extremes, then let each
/// vertex move to any extreme between its neighbours while the area grows.
fn rough_quad(comp: &Component, s: usize) -> Option<Quad> {
    let e = &comp.ext;
    let mut idx = [0usize, 4, 8, 12];
    let mut best2 = -1i64; // twice the area: (p2 - p0) × (p3 - p1)
    for i in (0..10).step_by(2) {
        for j in (i + 2..12).step_by(2) {
            for k in (j + 2..14).step_by(2) {
                for l in (k + 2..16).step_by(2) {
                    let a2 = i64::from(e[k].0 - e[i].0) * i64::from(e[l].1 - e[j].1)
                        - i64::from(e[k].1 - e[i].1) * i64::from(e[l].0 - e[j].0);
                    if a2 > best2 {
                        best2 = a2;
                        idx = [i, j, k, l];
                    }
                }
            }
        }
    }
    // Moving vertex v to point m changes twice the area by cross(m, next - prev): one dot product each.
    for _ in 0..4 {
        let mut moved = false;
        for v in 0..4 {
            let (pv, nv) = (idx[(v + 3) & 3], idx[(v + 1) & 3]);
            let ex = i64::from(e[nv].0 - e[pv].0);
            let ey = i64::from(e[nv].1 - e[pv].1);
            let gain = |m: usize| i64::from(e[m].0) * ey - i64::from(e[m].1) * ex;
            let mut cur = gain(idx[v]);
            let mut m = (pv + 1) & 15;
            while m != nv {
                if gain(m) > cur {
                    cur = gain(m);
                    idx[v] = m;
                    moved = true;
                }
                m = (m + 1) & 15;
            }
        }
        if !moved {
            break;
        }
    }
    if best2 <= 0 {
        return None;
    }
    let sf = s as f64;
    Some(idx.map(|i| Vec2::new(f64::from(e[i].0) * sf, f64::from(e[i].1) * sf)))
}

/// Paired samples half a cell either side of each side of the rough quad (grown by half a stride):
/// whether at least three sides are lighter outside than inside.
fn sides_contrast(view: &Sampler, rough: &Quad, sf: f64, cells: f64, min_contrast: u8) -> bool {
    let h = Homography::square_to_quad(&grow_quad(rough, 0.5 * sf));
    let half = 0.5 / cells;
    let good = (0..4)
        .filter(|&side| {
            let sum: f64 = (0..2)
                .map(|j| {
                    let along = 0.25 + 0.5 * j as f64;
                    let u = match side {
                        1 => 1.0,
                        3 => 0.0,
                        _ => along,
                    };
                    let v = match side {
                        0 => 0.0,
                        2 => 1.0,
                        _ => along,
                    };
                    let du = match side {
                        1 => half,
                        3 => -half,
                        _ => 0.0,
                    };
                    let dv = match side {
                        2 => half,
                        0 => -half,
                        _ => 0.0,
                    };
                    view.at_point(h.project(u + du, v + dv)) - view.at_point(h.project(u - du, v - dv))
                })
                .sum();
            sum >= f64::from(min_contrast) * 0.25
        })
        .count();
    good >= 3
}
