//! Sub-pixel fitting of a quad's four sides to the tag border's outer edge, at full resolution.

use super::geometry::{Quad, Vec2, centroid, intersect};
use super::image::Sampler;

/// Edges whose ramp is wider than this many cells overlap the next edge's ramp (motion or defocus
/// blur): locate them by midpoint crossing instead of the gradient's centre of mass.
const CROSSING_CELLS: f64 = 0.6;
/// Midpoint-crossing passes: the plateau levels are re-read around each new estimate.
const CROSS_ITERS: usize = 3;
const MAX_POINTS: usize = 96;
const KMAX: usize = 15;

/// The tag's black and white grey levels, for the midpoint-crossing estimator; `black >= white`
/// disables it.
#[derive(Clone, Copy, Debug)]
pub struct EdgeLevels {
    pub black: f64,
    pub white: f64,
}

pub struct FitOptions {
    /// Search window outside and inside the current side, in pixels.
    pub r_out: f64,
    pub r_in: f64,
    /// Minimum edge contrast in grey levels.
    pub min_edge: f64,
    /// The first pass from a rough quad: shorter profiles, and prefer the outermost strong edge.
    pub outermost: bool,
    /// Profiles per side (at most 96); more than 24 means one every 1.5 px.
    pub max_points: usize,
    pub levels: Option<EdgeLevels>,
    /// Cells across the tag square.
    pub cells: f64,
}

/// Fit each side of `quad` to the outer border edge, searched from `r_out` outside to `r_in` inside along
/// the side's normal, and intersect the four lines. Returns the new quad and the mean edge width, or
/// None if a side has too few edge points or the corners move implausibly far.
pub fn fit_sides(im: &Sampler, quad: &Quad, o: &FitOptions) -> Option<(Quad, f64)> {
    let reach = 0.5 * (o.r_out + o.r_in);
    let k_len = if o.outermost { 11 } else { KMAX };
    let cen = centroid(quad);
    let mut width_sum = 0.0;
    let mut width_n = 0;
    let mut lines = [(Vec2::default(), Vec2::default()); 4];
    let step = (o.r_out + o.r_in) / (k_len - 1) as f64;

    for (k, line) in lines.iter_mut().enumerate() {
        let a = quad[k];
        let b = quad[(k + 1) & 3];
        let e = b - a;
        let len = e.norm();
        if len < 6.0 {
            return None;
        }
        let e = e * (1.0 / len);
        let mut n = Vec2::new(-e.y, e.x);
        if (cen - a).dot(n) < 0.0 {
            n = n * -1.0;
        }
        let np = ((len / if o.max_points > 24 { 1.5 } else { 4.0 }) as usize).clamp(6, o.max_points.min(MAX_POINTS));
        let mut pts = [Vec2::default(); MAX_POINTS];
        let mut wt = [0.0f64; MAX_POINTS];
        let mut good = 0;
        let dt = 0.76 / np as f64;
        // best / (2 step) >= min_edge / 4
        let min_g = o.min_edge * 0.5 * step;
        for i in 0..np {
            if good + (np - i) < 3 {
                break; // three edge points are out of reach
            }
            let t = 0.12 + dt * (i as f64 + 0.5);
            let base = a + (b - a) * t;
            let mut prof = [0.0f64; KMAX];
            for (j, p) in prof.iter_mut().enumerate().take(k_len) {
                *p = im.at_point(base + n * (-o.r_out + j as f64 * step));
            }
            // Outside (white) minus inside (black): the outer border edge is a positive peak.
            let mut g = [0.0f64; KMAX];
            let mut best = 0.0;
            let mut bj = None;
            for j in 1..k_len - 1 {
                g[j] = prof[j - 1] - prof[j + 1];
                if g[j] > best {
                    best = g[j];
                    bj = Some(j);
                }
            }
            // The outer border is the first step met going inward that lands on the darkest level:
            // payload cells further in make equally strong steps, so prefer the outermost.
            if o.outermost && bj.is_some() {
                for j in 1..k_len - 1 {
                    if 5.0 * g[j] >= 3.0 * best && (j == k_len - 2 || g[j] >= g[j + 1]) {
                        bj = Some(j);
                        best = g[j];
                        break;
                    }
                }
            }
            let Some(bj) = bj else { continue };
            if best < min_g {
                continue;
            }
            // Centre of mass of the positive gradient lobe around the peak: exact for an area-sampled
            // edge at any blur width.
            let (mut jl, mut jr) = (bj, bj);
            while jl > 1 && 20.0 * g[jl - 1] > best {
                jl -= 1;
            }
            while jr < k_len - 2 && 20.0 * g[jr + 1] > best {
                jr += 1;
            }
            let (sg, sj) = (jl..=jr).fold((0.0, 0.0), |(sg, sj), j| (sg + g[j], sj + g[j] * j as f64));
            let mut cj = if sg > 0.0 { sj / sg } else { bj as f64 };
            // A ramp wide against a cell (blur) overlaps the next edge's and biases the centre of mass;
            // the crossing of the black/white midpoint stays exact for symmetric blur under two cells.
            if let Some(lv) = o.levels
                && lv.black < lv.white
                && (jr - jl + 1) as f64 * step > CROSSING_CELLS * len / o.cells
            {
                // Levels from the cell centres either side, half a cell from the edge.
                let half = len / (2.0 * o.cells);
                for _ in 0..CROSS_ITERS {
                    let pos0 = -o.r_out + cj * step;
                    let wl = im.at_point(base + n * (pos0 - half));
                    let bl = im.at_point(base + n * (pos0 + half));
                    let mid = 0.5 * (wl + bl);
                    let mut best_c: Option<f64> = None;
                    for j in jl.saturating_sub(1)..=jr {
                        if j + 1 >= k_len || !(prof[j] >= mid && prof[j + 1] < mid) {
                            continue;
                        }
                        let c = j as f64 + (prof[j] - mid) / (prof[j] - prof[j + 1]);
                        if best_c.is_none_or(|bc| (c - cj).abs() < (bc - cj).abs()) {
                            best_c = Some(c);
                        }
                    }
                    let Some(c) = best_c else { break };
                    let settled = (c - cj).abs() < 1e-3;
                    cj = c;
                    if settled {
                        break;
                    }
                }
            }
            width_sum += (jr - jl + 1) as f64 * step;
            width_n += 1;
            pts[good] = base + n * (-o.r_out + cj * step);
            wt[good] = best;
            good += 1;
        }
        if good < 3 {
            return None;
        }
        *line = fit_line(&mut pts, &mut wt, good, reach)?;
    }

    let edge_width = if width_n > 0 { width_sum / width_n as f64 } else { 0.0 };
    let mut out = *quad;
    for k in 0..4 {
        let p = (k + 3) & 3; // corner k joins side k-1 and side k
        let c = intersect(lines[p].0, lines[p].1, lines[k].0, lines[k].1, 0.2)?;
        let mv = (c.x - quad[k].x).abs() + (c.y - quad[k].y).abs();
        if mv > 4.0 * reach + 2.0 {
            return None;
        }
        out[k] = c;
    }
    Some((out, edge_width))
}

/// Weighted total-least-squares line (point, unit direction), then one pass dropping outliers when most
/// samples agree.
fn fit_line(pts: &mut [Vec2], wt: &mut [f64], mut good: usize, reach: f64) -> Option<(Vec2, Vec2)> {
    let mut line = (Vec2::default(), Vec2::default());
    for pass in 0..2 {
        let sw: f64 = wt[..good].iter().sum();
        if sw <= 0.0 {
            return None;
        }
        let m = pts[..good].iter().zip(&wt[..good]).fold(Vec2::default(), |m, (p, w)| m + *p * *w) * (1.0 / sw);
        let (mut sxx, mut syy, mut sxy) = (0.0, 0.0, 0.0);
        for (p, w) in pts[..good].iter().zip(&wt[..good]) {
            let q = *p - m;
            sxx += w * q.x * q.x;
            syy += w * q.y * q.y;
            sxy += w * q.x * q.y;
        }
        // Principal axis from the half-angle identities: tan 2t = 2 sxy / (sxx - syy).
        let spread = sxx - syy;
        let r = (spread * spread + 4.0 * sxy * sxy).sqrt();
        let (mut ct, mut st) = (1.0, 0.0);
        if r > 0.0 {
            if spread >= 0.0 {
                ct = (0.5 * (1.0 + spread / r)).sqrt();
                st = sxy / (r * ct);
            } else {
                st = (0.5 * (1.0 - spread / r)).sqrt().copysign(sxy);
                ct = sxy / (r * st);
            }
        }
        let d = Vec2::new(ct, st);
        line = (m, d);
        if pass == 0 {
            let normal = Vec2::new(-d.y, d.x);
            let mut kept = 0;
            for i in 0..good {
                if ((pts[i] - m).dot(normal)).abs() <= 1.0 + 0.1 * reach {
                    pts[kept] = pts[i];
                    wt[kept] = wt[i];
                    kept += 1;
                }
            }
            if kept == good {
                break; // the same samples would give the same fit
            }
            if kept >= 3 && 2 * kept >= good {
                good = kept;
            } else {
                break; // too few agree: keep the full fit
            }
        }
    }
    Some(line)
}
