//! Reading a tag's cells through the homography of its quad, and decoding its payload.

use super::family::Layout;
use super::geometry::{Homography, Quad, Vec2, signed_area};
use super::image::Sampler;
use super::{Detection, Settings};

/// Mean of the two middle values of an even-length list.
fn median_even(v: &mut [f64]) -> f64 {
    v.sort_unstable_by(f64::total_cmp);
    let k = v.len() / 2;
    0.5 * (v[k - 1] + v[k])
}

/// The tag's black and white levels from its border cells and the quiet ring outside them.
pub struct BorderRead {
    pub black: f64,
    pub white: f64,
    /// Border cells darker than the black/white midpoint.
    pub dark: usize,
}

/// Cell samples of a tag grid (up to 16×16).
pub struct Grid {
    pub v: [f64; 256],
}

impl Grid {
    fn new() -> Self {
        Self { v: [0.0; 256] }
    }
}

/// Read the square's border cells (into `grid`) and the quiet ring half a cell outside them: 28 and 36
/// samples for an 8-cell square. Returns None if white - black is under `min_contrast`.
pub fn read_border(l: &Layout, im: &Sampler, h: &Homography, min_contrast: f64, grid: &mut Grid) -> Option<BorderRead> {
    let (w, o, g, cells) = (l.width, l.outer, l.grid, l.cells);
    let mut border = [0.0f64; 64];
    let mut nb = 0;
    for r in 0..w {
        let step = if r == 0 || r == w - 1 { 1 } else { w - 1 };
        for c in (0..w).step_by(step) {
            let v = im.at_point(h.project((c as f64 + 0.5) / cells, (r as f64 + 0.5) / cells));
            grid.v[(r + o) * g + c + o] = v;
            border[nb] = v;
            nb += 1;
        }
    }
    let mut ring = [0.0f64; 68];
    let mut nr = 0;
    let (near, far) = (-0.5 / cells, (w as f64 + 0.5) / cells);
    for i in 0..w + 2 {
        let u = (i as f64 - 0.5) / cells;
        ring[nr] = im.at_point(h.project(u, near));
        ring[nr + 1] = im.at_point(h.project(u, far));
        nr += 2;
    }
    for i in 1..w + 1 {
        let v = (i as f64 - 0.5) / cells;
        ring[nr] = im.at_point(h.project(near, v));
        ring[nr + 1] = im.at_point(h.project(far, v));
        nr += 2;
    }
    let mut sorted = border;
    let black = median_even(&mut sorted[..nb]);
    let white = median_even(&mut ring[..nr]);
    if white - black < min_contrast {
        return None;
    }
    let thr = 0.5 * (black + white);
    let dark = border[..nb].iter().filter(|&&v| v < thr).count();
    Some(BorderRead { black, white, dark })
}

/// `min_border_cells` counts out of tag36h11's 28 border cells; scale it to the family's.
pub fn min_border(l: &Layout, min_border_cells: u32) -> i64 {
    i64::from((min_border_cells * 4 * (l.width as u32 - 1) + 14) / 28)
}

/// Whether a quad has a plausible tag border before any edge fitting.
pub fn border_plausible(l: &Layout, im: &Sampler, q: &Quad, min_contrast: f64, min_dark: i64) -> bool {
    let h = Homography::square_to_quad(q);
    read_border(l, im, &h, min_contrast, &mut Grid::new()).is_some_and(|b| b.dark as i64 >= min_dark)
}

/// What a successful decode reports besides the detection.
pub struct Decoded {
    pub detection: Detection,
    pub levels: (f64, f64),
    /// Each side's non-corner border cells (top, right, bottom, left) as a fraction of the way from the
    /// tag's black to its white: about 0 for a true border side.
    pub side_level: [f64; 4],
}

/// Decode the tag inside quad `q` (reordered in place to clockwise winding). `slack` loosens the border
/// and Hamming limits, for a cheap early decode on unrefined corners.
pub fn decode_quad(l: &mut Layout, im: &Sampler, q: &mut Quad, s: &Settings, slack: u32) -> Option<Decoded> {
    let (w, o, g) = (l.width, l.outer, l.grid);
    if signed_area(q) < 0.0 {
        q.swap(1, 3);
    }
    let h = Homography::square_to_quad(q);
    let mut grid = Grid::new();
    // Border and quiet ring first: most candidates fail on contrast or border before the payload is read.
    let br = read_border(l, im, &h, s.min_contrast as f64, &mut grid)?;
    let (black, white) = (br.black, br.white);
    let thr = 0.5 * (black + white);
    if (br.dark as i64) < min_border(l, s.min_border_cells) - i64::from(slack) {
        return None;
    }
    let cell_at = |c: isize, r: isize| h.project((c as f64 + 0.5) / l.cells, (r as f64 + 0.5) / l.cells);
    let (wi, oi) = (w as isize, o as isize);
    // The payload, and for families with data outside the quiet ring, that ring and the data around it.
    for r in 1 - oi..wi - 1 + oi {
        for c in 1 - oi..wi - 1 + oi {
            let on_border =
                (0..wi).contains(&r) && (0..wi).contains(&c) && (r == 0 || r == wi - 1 || c == 0 || c == wi - 1);
            if o > 0 && on_border {
                continue;
            }
            grid.v[((r + oi) * g as isize + c + oi) as usize] = im.at_point(cell_at(c, r));
        }
    }
    if o > 0 {
        // The grid's outermost ring, outside the quiet ring.
        for r in -oi..wi + oi {
            let step = if r == -oi || r == wi + oi - 1 { 1 } else { (wi + 2 * oi - 1) as usize };
            for c in (-oi..wi + oi).step_by(step) {
                grid.v[((r + oi) * g as isize + c + oi) as usize] = im.at_point(cell_at(c, r));
            }
        }
    }
    let mut side_level = [0.0; 4];
    for k in 1..w - 1 {
        side_level[0] += grid.v[o * g + k + o];
        side_level[1] += grid.v[(k + o) * g + w - 1 + o];
        side_level[2] += grid.v[(w - 1 + o) * g + k + o];
        side_level[3] += grid.v[(k + o) * g + o];
    }
    for v in &mut side_level {
        *v = (*v / (w - 2) as f64 - black) / (white - black);
    }
    // Sharpen the cell samples, as upstream AprilTag does, to undo blur between neighbouring cells.
    if s.sharpening > 0.0 {
        let mut sharp = [0.0f64; 256];
        for r in 0..g {
            for c in 0..g {
                let v = grid.v[r * g + c];
                let mut lap = 4.0 * v;
                lap -= if r > 0 { grid.v[(r - 1) * g + c] } else { v };
                lap -= if r < g - 1 { grid.v[(r + 1) * g + c] } else { v };
                lap -= if c > 0 { grid.v[r * g + c - 1] } else { v };
                lap -= if c < g - 1 { grid.v[r * g + c + 1] } else { v };
                sharp[r * g + c] = v + s.sharpening * lap;
            }
        }
        grid.v[..g * g].copy_from_slice(&sharp[..g * g]);
    }

    let code = l.cell.iter().fold(0u64, |code, &c| (code << 1) | u64::from(grid.v[c] > thr));
    let max_hamming = s.max_hamming.min(l.max_hamming);
    let soft = l.soft_hamming.max(max_hamming);
    let mut found = l.nearest_code(code, if slack > 0 { soft } else { max_hamming } + slack);
    if found.is_none() && slack == 0 && soft > max_hamming {
        // Soft decision for families whose codes are too close for more hard corrections: accept up to
        // `soft` wrong bits only if they are exactly the least certain readings (nearest the threshold).
        found = l.nearest_code(code, soft).filter(|&(i, _)| {
            let wrong = code ^ l.codes[i];
            let (mut worst_wrong, mut best_right) = (0.0f64, f64::INFINITY);
            for (k, &c) in l.cell.iter().enumerate() {
                let certainty = (grid.v[c] - thr).abs();
                if wrong & l.bit(k) != 0 {
                    worst_wrong = worst_wrong.max(certainty);
                } else {
                    best_right = best_right.min(certainty);
                }
            }
            worst_wrong < best_right
        });
    }
    let (bi, hamming) = found?;
    // Three corrected bits only when the wrong cells touch: real damage (a finger, cable, glare) is
    // spatially coherent, while chance matches scatter their errors.
    if slack == 0 && hamming == 3 && !l.errors_connected(code ^ l.codes[bi]) {
        return None;
    }
    let rot = l.rotations[bi] as usize;
    // Observed grid == rot90(canonical, rot): the canonical top-left sits at corner (4 - rot) % 4.
    let mut corners = [[0.0; 2]; 4];
    for (k, c) in corners.iter_mut().enumerate() {
        let p = q[(k + 4 - rot) & 3];
        *c = [p.x, p.y];
    }
    // As upstream AprilTag: the smaller of the white and black payload bits' mean distance from the
    // threshold, in grey levels.
    let (mut ws, mut wn, mut bs, mut bn) = (0.0, 0, 0.0, 0);
    for &c in &l.cell {
        let v = grid.v[c] - thr;
        if v > 0.0 {
            ws += v;
            wn += 1;
        } else {
            bs -= v;
            bn += 1;
        }
    }
    let wm = if wn > 0 { ws / wn as f64 } else { 0.0 };
    let bm = if bn > 0 { bs / bn as f64 } else { 0.0 };
    Some(Decoded {
        detection: Detection { id: l.ids[bi], hamming, decision_margin: wm.min(bm) as f32, corners },
        levels: (black, white),
        side_level,
    })
}

/// Centre of mass of the strongest falling (white to black along the profile) or rising gradient lobe
/// in `prof[from..=to]`.
fn lobe_centroid(prof: &[f64], from: usize, to: usize, falling: bool) -> Option<f64> {
    let mut g = [0.0f64; 64];
    let mut best = 0.0;
    let mut bj = None;
    for j in from..=to {
        g[j] = if falling { prof[j - 1] - prof[j + 1] } else { prof[j + 1] - prof[j - 1] };
        if g[j] > best {
            best = g[j];
            bj = Some(j);
        }
    }
    let bj = bj?;
    let (mut jl, mut jr) = (bj, bj);
    while jl > from && g[jl - 1] > 0.05 * best {
        jl -= 1;
    }
    while jr < to && g[jr + 1] > 0.05 * best {
        jr += 1;
    }
    let (sg, sj) = (jl..=jr).fold((0.0, 0.0), |(sg, sj), j| (sg + g[j], sj + g[j] * j as f64));
    Some(sj / sg)
}

/// Clipped exposure (or any symmetric edge bias) shrinks every black region by delta: the outline by
/// delta per side and the one-cell border by 2 delta. Measuring the border's width where a white payload
/// cell lies inside it gives delta. Returns (delta, measurements), or None when the sides disagree (a
/// border that simply isn't one cell wide).
pub fn border_bias(l: &Layout, im: &Sampler, q: &Quad, black: f64, white: f64) -> Option<(f64, usize)> {
    const N: usize = 33;
    let h = Homography::square_to_quad(q);
    let mid = 0.5 * (black + white);
    let (mut sum, mut n) = (0.0, 0);
    let mut side_sum = [0.0; 4];
    let mut side_n = [0; 4];
    for side in 0..4 {
        for c in 1..l.width - 1 {
            // (a, t): position along the side and depth inward, in cells; side 0 is the top (v = 0).
            let a = (c as f64 + 0.5) / l.cells;
            let unit = |t: f64| {
                let t = t / l.cells;
                let (u, v) = match side {
                    0 => (a, t),
                    1 => (1.0 - t, a),
                    2 => (a, 1.0 - t),
                    _ => (t, a),
                };
                h.project(u, v)
            };
            if im.at_point(unit(1.5)) < mid {
                continue; // a dark payload cell
            }
            let mut prof = [0.0; N];
            let mut pts = [Vec2::default(); N];
            for j in 0..N {
                pts[j] = unit(-0.5 + 2.0 * j as f64 / (N - 1) as f64);
                prof[j] = im.at_point(pts[j]);
            }
            // Outer edge within half a cell of t = 0, inner edge within half a cell of t = 1.
            let (Some(jo), Some(ji)) =
                (lobe_centroid(&prof, 1, N / 2 - 1, true), lobe_centroid(&prof, N / 2 + 1, N - 2, false))
            else {
                continue;
            };
            let lerp = |j: f64| {
                let i = j as usize;
                pts[i] + (pts[i + 1] - pts[i]) * (j - i as f64)
            };
            let width_px = (lerp(ji) - lerp(jo)).norm();
            let cell_px = (pts[24] - pts[8]).norm(); // t = 0 to t = 1
            let d = (cell_px - width_px) / 1.75;
            sum += d;
            n += 1;
            side_sum[side] += d;
            side_n[side] += 1;
        }
    }
    if n == 0 {
        return None;
    }
    let delta = sum / n as f64;
    // Clipping moves every side alike; sides that disagree mean the border isn't one cell wide.
    for side in 0..4 {
        if side_n[side] > 0
            && (side_sum[side] / side_n[side] as f64 - delta).abs() > (0.05 + 0.25 * delta.abs()) * l.bias_slack
        {
            return None;
        }
    }
    Some((delta, n))
}
