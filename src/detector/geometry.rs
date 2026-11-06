//! Small planar geometry helpers for quads in pixel coordinates.

use std::ops::{Add, Mul, Sub};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn dot(self, o: Self) -> f64 {
        self.x * o.x + self.y * o.y
    }

    pub fn cross(self, o: Self) -> f64 {
        self.x * o.y - self.y * o.x
    }

    pub fn norm(self) -> f64 {
        self.x.hypot(self.y)
    }
}

impl Add for Vec2 {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y)
    }
}

impl Sub for Vec2 {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y)
    }
}

impl Mul<f64> for Vec2 {
    type Output = Self;
    fn mul(self, k: f64) -> Self {
        Self::new(self.x * k, self.y * k)
    }
}

pub type Quad = [Vec2; 4];

pub fn centroid(q: &Quad) -> Vec2 {
    (q[0] + q[1] + q[2] + q[3]) * 0.25
}

/// Positive when the corners run clockwise on screen (y down).
pub fn signed_area(q: &Quad) -> f64 {
    0.5 * (0..4).map(|k| q[k].cross(q[(k + 1) & 3])).sum::<f64>()
}

pub fn perimeter(q: &Quad) -> f64 {
    (0..4).map(|k| (q[(k + 1) & 3] - q[k]).norm()).sum()
}

/// Projective map of the unit square (0..1)² onto a quad (Heckbert): (0,0), (1,0), (1,1), (0,1) go to
/// corners 0..3.
#[derive(Clone, Copy, Debug)]
pub struct Homography([f64; 9]);

impl Homography {
    pub fn square_to_quad(q: &Quad) -> Self {
        let [p0, p1, p2, p3] = *q;
        let s = p0 - p1 + p2 - p3;
        let d1 = p1 - p2;
        let d2 = p3 - p2;
        let den = d1.cross(d2);
        let (mut g, mut h) = (0.0, 0.0);
        if den.abs() > 1e-12 && (s.x.abs() > 1e-12 || s.y.abs() > 1e-12) {
            g = s.cross(d2) / den;
            h = d1.cross(s) / den;
        }
        Self([
            p1.x - p0.x + g * p1.x,
            p3.x - p0.x + h * p3.x,
            p0.x,
            p1.y - p0.y + g * p1.y,
            p3.y - p0.y + h * p3.y,
            p0.y,
            g,
            h,
            1.0,
        ])
    }

    #[inline]
    pub fn project(&self, u: f64, v: f64) -> Vec2 {
        let m = &self.0;
        let iz = 1.0 / (m[6] * u + m[7] * v + m[8]);
        Vec2::new((m[0] * u + m[1] * v + m[2]) * iz, (m[3] * u + m[4] * v + m[5]) * iz)
    }

    /// The quad (u0, v0)..(u1, v1) of the unit square, mapped.
    pub fn sub_quad(&self, u0: f64, v0: f64, u1: f64, v1: f64) -> Quad {
        [self.project(u0, v0), self.project(u1, v0), self.project(u1, v1), self.project(u0, v1)]
    }
}

/// Intersection of lines `p + t d` and `q + s e`; None when nearly parallel (|sin| < min_sin for unit
/// directions).
pub fn intersect(p: Vec2, d: Vec2, q: Vec2, e: Vec2, min_sin: f64) -> Option<Vec2> {
    let cross = d.cross(e);
    if cross.abs() < min_sin {
        return None;
    }
    let t = (q - p).cross(e) / cross;
    Some(p + d * t)
}

/// Move every side of a convex quad outward by `d` pixels (inward for negative `d`).
pub fn grow_quad(q: &Quad, d: f64) -> Quad {
    let c = centroid(q);
    let mut pts = [Vec2::default(); 4];
    let mut dirs = [Vec2::default(); 4];
    for k in 0..4 {
        let a = q[k];
        let e = q[(k + 1) & 3] - a;
        let e = e * (1.0 / e.norm());
        let mut n = Vec2::new(e.y, -e.x);
        if (a - c).dot(n) < 0.0 {
            n = n * -1.0;
        }
        pts[k] = a + n * d;
        dirs[k] = e;
    }
    let mut out = *q;
    for k in 0..4 {
        let p = (k + 3) & 3;
        if let Some(v) = intersect(pts[p], dirs[p], pts[k], dirs[k], 0.0) {
            out[k] = v;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn homography_maps_corners() {
        let q = [Vec2::new(10.0, 10.0), Vec2::new(50.0, 12.0), Vec2::new(48.0, 60.0), Vec2::new(8.0, 55.0)];
        let h = Homography::square_to_quad(&q);
        for (k, (u, v)) in [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)].into_iter().enumerate() {
            let p = h.project(u, v);
            assert!((p - q[k]).norm() < 1e-9, "corner {k}: {p:?}");
        }
    }

    #[test]
    fn grow_square() {
        let q = [Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(10.0, 10.0), Vec2::new(0.0, 10.0)];
        let g = grow_quad(&q, 1.0);
        assert!((g[0] - Vec2::new(-1.0, -1.0)).norm() < 1e-9);
        assert!((g[2] - Vec2::new(11.0, 11.0)).norm() < 1e-9);
    }
}
