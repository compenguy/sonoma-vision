//! Synthetic frames with exact ground truth, for tests and benchmarks: anti-aliased tags under
//! perspective, Gaussian blur and sensor noise.

use crate::detector::Family;

/// A small deterministic PRNG (xorshift64*), so scenes are reproducible without extra dependencies.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) | 1)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }

    /// Uniform in [0, 1).
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }

    pub fn below(&mut self, n: usize) -> usize {
        (self.unit() * n as f64) as usize
    }

    pub fn gaussian(&mut self) -> f64 {
        let u = self.unit().max(1e-300);
        let v = self.unit();
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

/// A greyscale frame being drawn.
pub struct Canvas {
    pub width: usize,
    pub height: usize,
    pub pixels: Vec<f64>,
}

/// A rendered tag's identity and the exact image positions of its square's corners (pixel-centre
/// coordinates), in the order the detector reports them.
#[derive(Clone, Debug)]
pub struct TruthTag {
    pub id: u32,
    pub corners: [[f64; 2]; 4],
}

impl Canvas {
    pub fn new(width: usize, height: usize, background: f64) -> Self {
        Self { width, height, pixels: vec![background; width * height] }
    }

    /// Draw tag `id` of `family` so that its square's corners (the unit square's (0,0), (1,0), (1,1),
    /// (0,1)) land on `corners`, with 4×4 supersampling. Printed black and white are `ink` and `paper`.
    pub fn draw_tag(&mut self, family: &Family, id: usize, corners: [[f64; 2]; 4], ink: f64, paper: f64) -> TruthTag {
        let w = family.width as i32;
        let margin = (family.total_width as i32 - w) / 2;
        let code = family.codes[id];
        let nbits = family.nbits as usize;
        // Printed colour of each cell of the total grid, relative to the square's top-left cell.
        let span = family.total_width as i32;
        let mut cell = vec![paper; (span * span) as usize];
        for cy in -margin..w + margin {
            for cx in -margin..w + margin {
                let inside = (0..w).contains(&cx) && (0..w).contains(&cy);
                let ring = |d: i32| cx >= -d && cy >= -d && cx < w + d && cy < w + d;
                let border = inside && (cx == 0 || cy == 0 || cx == w - 1 || cy == w - 1);
                let v = if border {
                    if family.reversed { paper } else { ink }
                } else if ring(1) && !inside {
                    if family.reversed { ink } else { paper }
                } else {
                    paper
                };
                cell[((cy + margin) * span + cx + margin) as usize] = v;
            }
        }
        for k in 0..nbits {
            let (cx, cy) = (i32::from(family.x[k]), i32::from(family.y[k]));
            let white = (code >> (nbits - 1 - k)) & 1 == 1;
            cell[((cy + margin) * span + cx + margin) as usize] = if white { paper } else { ink };
        }

        let h = homography(&corners);
        let inv = invert3(&h);
        // Bounding box of the whole printed tag.
        let lo = -f64::from(margin) / f64::from(w);
        let hi = 1.0 - lo;
        let ext = [(lo, lo), (hi, lo), (hi, hi), (lo, hi)].map(|(u, v)| apply(&h, u, v));
        let x0 = ext.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min).floor().max(0.0) as usize;
        let y0 = ext.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min).floor().max(0.0) as usize;
        let x1 = (ext.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max).ceil() as usize + 1).min(self.width);
        let y1 = (ext.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max).ceil() as usize + 1).min(self.height);
        const S: usize = 4;
        for py in y0..y1 {
            for px in x0..x1 {
                let (mut sum, mut hits) = (0.0, 0);
                for sy in 0..S {
                    for sx in 0..S {
                        let x = px as f64 + (sx as f64 + 0.5) / S as f64 - 0.5;
                        let y = py as f64 + (sy as f64 + 0.5) / S as f64 - 0.5;
                        let [u, v] = apply(&inv, x, y);
                        let (cx, cy) = ((u * f64::from(w)).floor() as i32, (v * f64::from(w)).floor() as i32);
                        if cx < -margin || cy < -margin || cx >= w + margin || cy >= w + margin {
                            continue;
                        }
                        sum += cell[((cy + margin) * span + cx + margin) as usize];
                        hits += 1;
                    }
                }
                if hits > 0 {
                    let p = &mut self.pixels[py * self.width + px];
                    *p = (*p * (S * S - hits) as f64 + sum) / (S * S) as f64;
                }
            }
        }
        TruthTag { id: id as u32, corners }
    }

    /// Separable Gaussian blur with standard deviation `sigma` pixels.
    pub fn blur(&mut self, sigma: f64) {
        if sigma <= 0.0 {
            return;
        }
        let r = (3.0 * sigma).ceil() as isize;
        let k: Vec<f64> = (-r..=r).map(|i| (-(i * i) as f64 / (2.0 * sigma * sigma)).exp()).collect();
        let norm: f64 = k.iter().sum();
        let (w, h) = (self.width as isize, self.height as isize);
        let mut tmp = vec![0.0; self.pixels.len()];
        for y in 0..h {
            for x in 0..w {
                let s: f64 = (-r..=r)
                    .map(|i| k[(i + r) as usize] * self.pixels[(y * w + (x + i).clamp(0, w - 1)) as usize])
                    .sum();
                tmp[(y * w + x) as usize] = s / norm;
            }
        }
        for y in 0..h {
            for x in 0..w {
                let s: f64 =
                    (-r..=r).map(|i| k[(i + r) as usize] * tmp[((y + i).clamp(0, h - 1) * w + x) as usize]).sum();
                self.pixels[(y * w + x) as usize] = s / norm;
            }
        }
    }

    /// Add Gaussian noise with standard deviation `sigma` grey levels.
    pub fn noise(&mut self, sigma: f64, rng: &mut Rng) {
        for p in &mut self.pixels {
            *p += sigma * rng.gaussian();
        }
    }

    /// Quantise to 8 bits, clipping.
    pub fn to_gray(&self) -> Vec<u8> {
        self.pixels.iter().map(|&p| p.round().clamp(0.0, 255.0) as u8).collect()
    }
}

/// A square of side `size` centred at (cx, cy), rotated by `angle`, with each corner moved by up to
/// `skew` times the side to mimic perspective.
pub fn random_quad(rng: &mut Rng, cx: f64, cy: f64, size: f64, angle: f64, skew: f64) -> [[f64; 2]; 4] {
    let (s, c) = angle.sin_cos();
    [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)].map(|(u, v)| {
        let (dx, dy) = (rng.range(-skew, skew) * size, rng.range(-skew, skew) * size);
        [cx + size * (c * u - s * v) + dx, cy + size * (s * u + c * v) + dy]
    })
}

/// Homography taking the unit square's corners to `q`, as a row-major 3×3 matrix.
fn homography(q: &[[f64; 2]; 4]) -> [f64; 9] {
    let [p0, p1, p2, p3] = *q;
    let (sx, sy) = (p0[0] - p1[0] + p2[0] - p3[0], p0[1] - p1[1] + p2[1] - p3[1]);
    let (dx1, dy1, dx2, dy2) = (p1[0] - p2[0], p1[1] - p2[1], p3[0] - p2[0], p3[1] - p2[1]);
    let den = dx1 * dy2 - dx2 * dy1;
    let g = (sx * dy2 - dx2 * sy) / den;
    let h = (dx1 * sy - sx * dy1) / den;
    [
        p1[0] - p0[0] + g * p1[0],
        p3[0] - p0[0] + h * p3[0],
        p0[0],
        p1[1] - p0[1] + g * p1[1],
        p3[1] - p0[1] + h * p3[1],
        p0[1],
        g,
        h,
        1.0,
    ]
}

fn apply(m: &[f64; 9], u: f64, v: f64) -> [f64; 2] {
    let z = m[6] * u + m[7] * v + m[8];
    [(m[0] * u + m[1] * v + m[2]) / z, (m[3] * u + m[4] * v + m[5]) / z]
}

fn invert3(m: &[f64; 9]) -> [f64; 9] {
    let [a, b, c, d, e, f, g, h, i] = *m;
    let co = [
        e * i - f * h,
        c * h - b * i,
        b * f - c * e,
        f * g - d * i,
        a * i - c * g,
        c * d - a * f,
        d * h - e * g,
        b * g - a * h,
        a * e - b * d,
    ];
    let det = a * co[0] + b * co[3] + c * co[6];
    co.map(|x| x / det)
}

/// Write a binary PGM, for inspecting scenes.
pub fn write_pgm(path: &std::path::Path, width: usize, height: usize, gray: &[u8]) -> std::io::Result<()> {
    let mut data = format!("P5\n{width} {height}\n255\n").into_bytes();
    data.extend_from_slice(gray);
    std::fs::write(path, data)
}
