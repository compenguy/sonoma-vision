//! Sparse sampling ("raking") of the frame into coarse levels, and per-region noise floors.

use super::image::ImageView;

/// Side of a noise region, in level samples.
pub const REGION: usize = 64;

/// level[y][x] = img[y * s][x * s]: one point sample per stride.
pub fn rake(level: &mut [u8], lw: usize, lh: usize, img: &ImageView, s: usize) {
    for (y, row) in level.chunks_exact_mut(lw).take(lh).enumerate() {
        for (dst, v) in row.iter_mut().zip(img.row_step(y * s, s)) {
            *dst = v;
        }
    }
}

/// The next coarser level, in place: every second sample of the level before (each output sample
/// sits at or before its source).
pub fn halve(buf: &mut [u8], lw: usize, lh: usize, prev_lw: usize) {
    for y in 0..lh {
        let src = 2 * y * prev_lw;
        let dst = y * lw;
        for x in 0..lw {
            buf[dst + x] = buf[src + 2 * x];
        }
    }
}

pub fn invert(buf: &mut [u8]) {
    for v in buf {
        *v = 255 - *v;
    }
}

/// The averaging rake: level[y][x] = mean of the `kernel`² image block centred on (x * s, y * s), clipped
/// to the image. Each sample then stands for its whole block, so sensor noise drops by about `kernel`,
/// and a border cell's core still leaves every sample mostly dark.
#[allow(clippy::too_many_arguments)]
pub fn box_rake(
    level: &mut [u8],
    lw: usize,
    lh: usize,
    img: &ImageView,
    s: usize,
    kernel: usize,
    acc: &mut Vec<u32>,
    hs: &mut Vec<u32>,
) {
    let (w, h) = (img.width(), img.height());
    acc.resize(w, 0);
    hs.resize(w, 0);
    let half = (kernel - 1) / 2;
    let kk = (kernel * kernel) as u32;
    for y in 0..lh {
        let r0 = (y * s).saturating_sub(half);
        let r1 = (y * s + kernel - half).min(h);
        for (x, a) in acc.iter_mut().enumerate() {
            *a = (r0..r1).map(|r| u32::from(img.get(x, r))).sum();
        }
        // hs[c] = acc[c] + ... + acc[c + kernel - 1], for every block start that fits in the row.
        for c in 0..=w - kernel {
            hs[c] = acc[c..c + kernel].iter().sum();
        }
        let dst = &mut level[y * lw..(y + 1) * lw];
        // Whole blocks are x_lo <= x < x_hi; the rest are clipped by the image edge.
        let (mut x_lo, mut x_hi) = (half.div_ceil(s), ((w - kernel + half) / s + 1).min(lw));
        if r1 - r0 < kernel {
            (x_lo, x_hi) = (0, 0);
        }
        for x in x_lo..x_hi {
            dst[x] = ((hs[x * s - half] + kk / 2) / kk) as u8;
        }
        for x in (0..x_lo).chain(x_hi.max(x_lo)..lw) {
            let c0 = (x * s).saturating_sub(half);
            let c1 = (x * s + kernel - half).min(w);
            let sum: u32 = acc[c0..c1].iter().sum();
            let cnt = ((c1 - c0) * (r1 - r0)) as u32;
            dst[x] = ((sum + cnt / 2) / cnt) as u8;
        }
    }
}

/// Tile contrast floors per noise region, measured from the level itself.
#[derive(Default)]
pub struct Floors {
    values: Vec<u8>,
    cols: usize,
    rows: usize,
    uniform: bool,
}

/// Floor from a histogram of neighbouring-sample differences (clamped to 63): `sigmas` times the noise.
/// The median absolute difference of neighbouring samples is about 0.95 sigma for Gaussian noise, and a
/// flat noisy tile neighbourhood spans about 5.5 sigma, so 6 sigma keeps noise from forming dark networks.
fn floor_from(hist: &[u32; 64], n: u32, min_contrast: u8, sigmas: f64, cap: u8) -> u8 {
    let half = n.div_ceil(2);
    let mut seen = 0;
    let mut median = 0;
    while median < 63 {
        seen += hist[median];
        if seen >= half {
            break;
        }
        median += 1;
    }
    let floor = ((sigmas * median as f64 / 0.954 + 0.5) as u32).min(u32::from(cap));
    (floor as u8).max(min_contrast)
}

impl Floors {
    /// Measure each region's floor (from sample pairs on every 8th row), at most the frame's, which is
    /// returned. A region too small to measure takes the frame's.
    pub fn measure(&mut self, level: &[u8], lw: usize, lh: usize, min_contrast: u8, sigmas: f64, cap: u8) -> u8 {
        self.cols = lw.div_ceil(REGION);
        self.rows = lh.div_ceil(REGION);
        self.values.clear();
        let mut all = [0u32; 64];
        let mut n_all = 0;
        for ry in 0..self.rows {
            for rx in 0..self.cols {
                let mut hist = [0u32; 64];
                let x0 = rx * REGION;
                let x1 = (x0 + REGION).min(lw);
                let y1 = ((ry + 1) * REGION).min(lh);
                for y in (ry * REGION..y1).step_by(8) {
                    let row = &level[y * lw + x0..y * lw + x1];
                    for pair in row.chunks_exact(2) {
                        hist[pair[0].abs_diff(pair[1]).min(63) as usize] += 1;
                    }
                }
                let n: u32 = hist.iter().sum();
                for (a, h) in all.iter_mut().zip(&hist) {
                    *a += h;
                }
                n_all += n;
                self.values.push(if n >= 16 { floor_from(&hist, n, min_contrast, sigmas, cap) } else { 0 });
            }
        }
        let frame = if n_all > 0 { floor_from(&all, n_all, min_contrast, sigmas, cap) } else { min_contrast };
        // Never above the frame's: a region only lowers its floor where it is cleaner than the frame.
        for v in &mut self.values {
            if *v == 0 || *v > frame {
                *v = frame;
            }
        }
        self.uniform = self.values.iter().all(|&v| v == self.values[0]);
        frame
    }

    pub fn uniform(&self) -> Option<u8> {
        self.uniform.then(|| self.values[0])
    }

    /// The floor for a tile at tile coordinates (tx, ty), where a tile spans `4 * factor` samples of
    /// the grid the floors were measured on.
    #[inline]
    pub fn at(&self, tx: usize, ty: usize, factor: usize) -> u8 {
        let rx = (tx * 4 * factor / REGION).min(self.cols - 1);
        let ry = (ty * 4 * factor / REGION).min(self.rows - 1);
        self.values[ry * self.cols + rx]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn box_rake_averages_blocks() {
        let w = 12;
        let data: Vec<u8> = (0..w * w).map(|i| if (i % w) < 6 { 0 } else { 90 }).collect();
        let img = ImageView::gray(&data, w, w).unwrap();
        let (lw, lh) = (4, 4);
        let mut level = vec![0; lw * lh];
        box_rake(&mut level, lw, lh, &img, 3, 3, &mut Vec::new(), &mut Vec::new());
        // Samples at image columns 3, 6, 9: blocks 2..5, 5..8 (one dark of three), 8..11.
        assert_eq!(&level[4..8], &[0, 0, 60, 90]);
    }

    #[test]
    fn floors_follow_noise() {
        let (lw, lh) = (192, 64);
        let mut level = vec![100u8; lw * lh];
        for y in 0..lh {
            for x in 64..lw {
                level[y * lw + x] = if (x + y) % 2 == 0 { 90 } else { 110 };
            }
        }
        let mut floors = Floors::default();
        let frame = floors.measure(&level, lw, lh, 5, 6.0, 64);
        assert_eq!(floors.at(0, 0, 1), 5);
        assert_eq!(floors.at(20, 0, 1), frame);
        assert!(frame > 5);
    }
}
