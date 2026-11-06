//! The camera model: intrinsics and OpenCV lens distortion, either estimated from a field of view or
//! loaded from a PhotonVision calibration, and the mapping between pixels and normalised image
//! coordinates (on the z = 1 plane of the camera's optical frame: x right, y down, z forward).

use anyhow::{Context, Result, bail, ensure};
use serde_json::Value;

#[derive(Clone, Debug, PartialEq)]
pub struct Intrinsics {
    pub width: usize,
    pub height: usize,
    pub fx: f64,
    pub fy: f64,
    /// Optical centre in pixel-centre coordinates (pixel (0, 0)'s centre is at 0, 0), as OpenCV's.
    pub cx: f64,
    pub cy: f64,
    /// OpenCV distortion `k1, k2, p1, p2, k3, k4, k5, k6` (the rational model; the usual five-term
    /// model has `k4..k6` zero).
    pub dist: [f64; 8],
}

impl Intrinsics {
    /// A distortion-free pinhole camera centred on the image, with diagonal field of view
    /// `diagonal_fov_deg`. Good enough to try pose solving; a calibration is far more accurate.
    pub fn from_fov(width: usize, height: usize, diagonal_fov_deg: f64) -> Self {
        let half_diagonal = (width as f64).hypot(height as f64) / 2.0;
        let f = half_diagonal / (diagonal_fov_deg.to_radians() / 2.0).tan();
        Self {
            width,
            height,
            fx: f,
            fy: f,
            cx: (width as f64 - 1.0) / 2.0,
            cy: (height as f64 - 1.0) / 2.0,
            dist: [0.0; 8],
        }
    }

    /// A calibration exported from PhotonVision (its `resolution`, row-major `cameraIntrinsics` and
    /// OpenCV `distCoeffs`).
    pub fn from_photonvision(json: &str) -> Result<Self> {
        let v: Value = serde_json::from_str(json).context("not JSON")?;
        let number = |v: &Value, what: &str| v.as_f64().with_context(|| format!("{what} is not a number"));
        let resolution = &v["resolution"];
        let width = number(&resolution["width"], "resolution.width")?;
        let height = number(&resolution["height"], "resolution.height")?;
        let data = |key: &str| -> Result<Vec<f64>> {
            v[key]["data"]
                .as_array()
                .with_context(|| format!("no {key}.data"))?
                .iter()
                .map(|x| number(x, key))
                .collect()
        };
        let k = data("cameraIntrinsics")?;
        ensure!(k.len() == 9, "cameraIntrinsics has {} values, not 9", k.len());
        let d = data("distCoeffs")?;
        // Thin-prism and tilt terms (OpenCV's 12- and 14-coefficient models) aren't modelled.
        ensure!(
            d.iter().skip(8).all(|&c| c == 0.0),
            "distortion models beyond OpenCV's rational model aren't supported"
        );
        if let Some(model) = v["lensmodel"].as_str() {
            ensure!(model.contains("OPENCV"), "lens model {model} isn't supported (OpenCV's is)");
        }
        let mut dist = [0.0; 8];
        for (slot, c) in dist.iter_mut().zip(&d) {
            *slot = *c;
        }
        ensure!(width >= 1.0 && height >= 1.0 && k[0] > 0.0 && k[4] > 0.0, "implausible calibration");
        Ok(Self { width: width as usize, height: height as usize, fx: k[0], fy: k[4], cx: k[2], cy: k[5], dist })
    }

    /// The same camera at another resolution. Only scaling is supported: a mode that crops or bins
    /// differently (a different aspect ratio) needs its own calibration.
    pub fn scaled_to(&self, width: usize, height: usize) -> Result<Self> {
        if (width, height) == (self.width, self.height) {
            return Ok(self.clone());
        }
        let (sx, sy) = (width as f64 / self.width as f64, height as f64 / self.height as f64);
        if (sx - sy).abs() > 1e-3 * sx {
            bail!("calibrated at {}x{}, which doesn't scale to {width}x{height}", self.width, self.height);
        }
        Ok(Self {
            width,
            height,
            fx: self.fx * sx,
            fy: self.fy * sy,
            cx: (self.cx + 0.5) * sx - 0.5,
            cy: (self.cy + 0.5) * sy - 0.5,
            dist: self.dist,
        })
    }

    /// Undistorted normalised image coordinates of pixel `p` (OpenCV's iterative `undistortPoints`).
    pub fn normalize(&self, p: [f64; 2]) -> [f64; 2] {
        let x0 = (p[0] - self.cx) / self.fx;
        let y0 = (p[1] - self.cy) / self.fy;
        if self.dist.iter().all(|&c| c == 0.0) {
            return [x0, y0];
        }
        let [k1, k2, p1, p2, k3, k4, k5, k6] = self.dist;
        let (mut x, mut y) = (x0, y0);
        for _ in 0..20 {
            let r2 = x * x + y * y;
            let icdist = (1.0 + ((k6 * r2 + k5) * r2 + k4) * r2) / (1.0 + ((k3 * r2 + k2) * r2 + k1) * r2);
            let dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
            let dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
            (x, y) = ((x0 - dx) * icdist, (y0 - dy) * icdist);
        }
        [x, y]
    }

    /// The pixel that normalised image point `n` lands on, distortion included.
    pub fn project(&self, n: [f64; 2]) -> [f64; 2] {
        let [k1, k2, p1, p2, k3, k4, k5, k6] = self.dist;
        let [x, y] = n;
        let r2 = x * x + y * y;
        let radial = (1.0 + ((k3 * r2 + k2) * r2 + k1) * r2) / (1.0 + ((k6 * r2 + k5) * r2 + k4) * r2);
        let xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        [self.fx * xd + self.cx, self.fy * yd + self.cy]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fov_sets_focal_length() {
        // 90° diagonal: the corners are at 45°, so f is half the diagonal.
        let c = Intrinsics::from_fov(640, 480, 90.0);
        assert!((c.fx - 400.0).abs() < 1e-9);
        assert_eq!((c.cx, c.cy), (319.5, 239.5));
        let corner = c.normalize([-0.5 + 640.0, -0.5 + 480.0]);
        assert!((corner[0].hypot(corner[1]) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn distortion_round_trips() {
        let c = Intrinsics {
            width: 1280,
            height: 720,
            fx: 900.0,
            fy: 905.0,
            cx: 640.3,
            cy: 361.2,
            dist: [-0.3, 0.12, 0.001, -0.0007, -0.02, 0.01, 0.0, 0.002],
        };
        for p in [[0.0, 0.0], [100.0, 650.0], [1279.0, 719.0], [640.0, 360.0]] {
            let back = c.project(c.normalize(p));
            assert!((back[0] - p[0]).abs() < 1e-6 && (back[1] - p[1]).abs() < 1e-6, "{p:?} -> {back:?}");
        }
    }

    #[test]
    fn loads_photonvision_and_scales() {
        let json = r#"{
            "resolution": {"width": 1280.0, "height": 720.0},
            "cameraIntrinsics": {"rows": 3, "cols": 3, "type": 6,
                "data": [910.0, 0.0, 641.5, 0.0, 912.0, 357.5, 0.0, 0.0, 1.0]},
            "distCoeffs": {"rows": 1, "cols": 8, "type": 6,
                "data": [0.05, -0.1, 0.001, 0.002, 0.03, 0.0, 0.0, 0.0]},
            "lensmodel": "LENSMODEL_OPENCV"
        }"#;
        let c = Intrinsics::from_photonvision(json).unwrap();
        assert_eq!((c.width, c.height, c.fx, c.fy, c.cx, c.cy), (1280, 720, 910.0, 912.0, 641.5, 357.5));
        assert_eq!(c.dist[..5], [0.05, -0.1, 0.001, 0.002, 0.03]);
        let half = c.scaled_to(640, 360).unwrap();
        assert_eq!((half.fx, half.cx, half.cy), (455.0, 320.5, 178.5));
        assert!(c.scaled_to(640, 480).is_err(), "a different aspect ratio needs its own calibration");
        assert!(Intrinsics::from_photonvision(r#"{"resolution": {}}"#).is_err());
    }
}
