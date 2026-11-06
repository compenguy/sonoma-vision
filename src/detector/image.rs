//! Borrowed views of 8-bit luma frames, read in place from whatever buffer holds them.

use thiserror::Error;

/// An 8-bit luma image borrowed from a larger buffer: a greyscale frame, the Y plane of NV12, or the
/// interleaved Y bytes of a packed YUYV/UYVY frame. Pixel `(x, y)` is `data[y * stride + x * pitch]`.
#[derive(Clone, Copy, Debug)]
pub struct ImageView<'a> {
    data: &'a [u8],
    width: usize,
    height: usize,
    stride: usize,
    pitch: usize,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum ImageError {
    #[error("image dimensions must be non-zero")]
    Empty,
    #[error("pixel pitch must be at least 1 and the row stride must cover a row")]
    Layout,
    #[error("buffer of {len} bytes is too small for the image (needs {needed})")]
    TooSmall { len: usize, needed: usize },
}

impl<'a> ImageView<'a> {
    pub fn new(data: &'a [u8], width: usize, height: usize, stride: usize, pitch: usize) -> Result<Self, ImageError> {
        if width == 0 || height == 0 {
            return Err(ImageError::Empty);
        }
        if pitch == 0 || stride < (width - 1) * pitch + 1 {
            return Err(ImageError::Layout);
        }
        let needed = (height - 1) * stride + (width - 1) * pitch + 1;
        if data.len() < needed {
            return Err(ImageError::TooSmall { len: data.len(), needed });
        }
        Ok(Self { data, width, height, stride, pitch })
    }

    /// A tightly packed greyscale image.
    pub fn gray(data: &'a [u8], width: usize, height: usize) -> Result<Self, ImageError> {
        Self::new(data, width, height, width, 1)
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    #[inline]
    pub fn get(&self, x: usize, y: usize) -> u8 {
        self.data[y * self.stride + x * self.pitch]
    }

    /// Every `step`th pixel of row `y`, starting at column 0.
    #[inline]
    pub(crate) fn row_step(&self, y: usize, step: usize) -> impl Iterator<Item = u8> + 'a {
        let start = y * self.stride;
        self.data[start..].iter().step_by(step * self.pitch).copied()
    }

    pub(crate) fn data(&self) -> &'a [u8] {
        self.data
    }

    pub(crate) fn stride(&self) -> usize {
        self.stride
    }

    pub(crate) fn pitch(&self) -> usize {
        self.pitch
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_short_buffers() {
        assert!(matches!(ImageView::gray(&[0; 15], 4, 4), Err(ImageError::TooSmall { .. })));
        assert!(ImageView::new(&[0; 31], 4, 4, 8, 2).is_ok());
    }

    #[test]
    fn packed_luma_reads_every_other_byte() {
        // YUYV: Y0 U Y1 V ...
        let data = [10, 0, 20, 0, 30, 0, 40, 0];
        let img = ImageView::new(&data, 2, 2, 4, 2).unwrap();
        assert_eq!(img.get(1, 0), 20);
        assert_eq!(img.get(0, 1), 30);
    }
}
