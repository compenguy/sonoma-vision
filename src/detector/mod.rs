//! AprilTag detection with napa-vision's C core (Zachary Wilson, GPL-3.0; vendor/napa-vision, compiled
//! by build.rs): a sparse-sampling detector that samples a sparse grid, skips regions without contrast,
//! groups dark runs into blobs, rejects non-tags with a cheap early decode, and fits edges and decodes
//! only the tags, at full resolution.
//!
//! This module wraps it: settings, borrowed luma frames (including packed YUYV/UYVY), and the tag
//! families' tables, which the synthetic frames in [`crate::synth`] draw from.

pub mod families;
mod family;
mod image;
mod luma;
mod napa;

use thiserror::Error;

pub use family::Family;
pub use image::{ImageError, ImageView};

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
    #[error("napa-vision's core couldn't make a {0} detector")]
    Napa(&'static str),
}

/// Validated settings, with the family's Hamming limit applied.
#[derive(Clone, Copy)]
pub(crate) struct Settings {
    min_tag_px: u32,
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
    family: &'static Family,
    core: napa::NapaCore,
}

impl Detector {
    pub fn new(family: &'static Family, config: &Config) -> Result<Self, ConfigError> {
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
        let max_hamming = config.max_hamming.unwrap_or_else(|| family.recommended_hamming()).min(family.max_hamming());
        let settings = Settings {
            min_tag_px: config.min_tag_px,
            max_hamming,
            min_contrast: config.min_contrast,
            min_border_cells: config.min_border_cells,
            sharpening: config.sharpening,
        };
        let core = napa::NapaCore::new(family, &settings).ok_or(ConfigError::Napa(family.name))?;
        Ok(Self { family, core })
    }

    pub fn family(&self) -> &'static Family {
        self.family
    }

    /// Detect tags, replacing the contents of `out`. Frames under 16 pixels on a side have no tags.
    pub fn detect_into(&mut self, img: &ImageView, out: &mut Vec<Detection>) {
        out.clear();
        self.core.detect(img, out);
    }

    /// Detect tags into a new vector.
    pub fn detect(&mut self, img: &ImageView) -> Vec<Detection> {
        let mut out = Vec::new();
        self.detect_into(img, &mut out);
        out
    }
}
