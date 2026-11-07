// Derived from NAPA code.
// Original work: Copyright (c) 2026 Zachary Wilson.
// Modifications: Copyright (c) 2026 William Page.
// SPDX-License-Identifier: GPL-3.0-only
// Copies and works based on this file must keep this notice and credit Zachary Wilson.

//! napa-vision's C core (vendor/napa-vision, compiled by build.rs) behind [`super::Detector`].

use std::ffi::{CString, c_char, c_int, c_void};
use std::ptr::NonNull;

use super::image::ImageView;
use super::{Detection, Family, Settings, luma};

#[repr(C)]
struct NapaParams {
    min_tag_px: c_int,
    max_hamming: c_int,
    min_contrast: c_int,
    min_border_cells: c_int,
    sharpening_x100: c_int,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct NapaDetection {
    id: c_int,
    hamming: c_int,
    decision_margin: f32,
    corners: [f64; 8],
}

/// napa_family_layout: a family's layout and Hamming limits.
#[repr(C)]
pub(crate) struct FamilyInfo {
    pub ncodes: c_int,
    pub nbits: c_int,
    pub width: c_int,
    pub total_width: c_int,
    pub reversed: c_int,
    pub min_distance: c_int,
    pub max_hamming: c_int,
    pub recommended_hamming: c_int,
    pub x: [i8; 64],
    pub y: [i8; 64],
}

unsafe extern "C" {
    fn napa_family_info(family: *const c_char, info: *mut FamilyInfo) -> c_int;
    #[cfg(test)]
    fn napa_family_code(family: *const c_char, id: c_int, code: *mut u64) -> c_int;
    fn napa_create_family(family: *const c_char) -> *mut c_void;
    fn napa_destroy(det: *mut c_void);
    fn napa_detect(
        det: *mut c_void,
        img: *const u8,
        width: c_int,
        height: c_int,
        stride: c_int,
        params: *const NapaParams,
        out: *mut NapaDetection,
        max_out: c_int,
    ) -> c_int;
}

/// napa-vision's layout of a family, or None if it doesn't know the name.
pub(crate) fn family_info(name: &str) -> Option<FamilyInfo> {
    let name = CString::new(name).ok()?;
    let mut info = std::mem::MaybeUninit::<FamilyInfo>::uninit();
    // SAFETY: a NUL-terminated name, and room for the struct, which a 0 return has filled.
    (unsafe { napa_family_info(name.as_ptr(), info.as_mut_ptr()) } == 0).then(|| unsafe { info.assume_init() })
}

/// napa-vision's code `id` of a family.
#[cfg(test)]
pub(crate) fn family_code(name: &str, id: usize) -> Option<u64> {
    let name = CString::new(name).ok()?;
    let mut code = 0u64;
    // SAFETY: a NUL-terminated name and a u64 to write.
    (unsafe { napa_family_code(name.as_ptr(), c_int::try_from(id).ok()?, &mut code) } == 0).then_some(code)
}

/// One napa-vision detector, with its parameters and output buffer.
pub struct NapaCore {
    det: NonNull<c_void>,
    params: NapaParams,
    out: Vec<NapaDetection>,
    /// Luma copied out of frames with a pixel pitch over 1 (YUYV, UYVY): napa-vision reads 1 byte per pixel.
    luma: Vec<u8>,
}

// SAFETY: a napa_detector is a plain heap object with no thread affinity; it just mustn't be used from two
// threads at once, which `&mut self` on every use guarantees.
unsafe impl Send for NapaCore {}

impl NapaCore {
    pub(crate) fn new(family: &Family, s: &Settings) -> Option<Self> {
        let name = CString::new(family.name).ok()?;
        // SAFETY: a NUL-terminated name; returns NULL for an unknown family or no memory.
        let det = NonNull::new(unsafe { napa_create_family(name.as_ptr()) })?;
        let params = NapaParams {
            min_tag_px: s.min_tag_px as c_int,
            max_hamming: s.max_hamming as c_int,
            min_contrast: c_int::from(s.min_contrast),
            min_border_cells: s.min_border_cells as c_int,
            sharpening_x100: (s.sharpening * 100.0).round() as c_int,
        };
        Some(Self { det, params, out: vec![NapaDetection::default(); 64], luma: Vec::new() })
    }

    pub fn detect(&mut self, img: &ImageView, out: &mut Vec<Detection>) {
        let (width, height, pitch) = (img.width(), img.height(), img.pitch());
        let (data, stride) = if pitch == 1 {
            (img.data(), img.stride())
        } else {
            self.luma.resize(width * height, 0);
            let src = img.data();
            for (y, row) in self.luma.chunks_exact_mut(width).enumerate() {
                luma::gather_stride(&src[y * img.stride()..][..(width - 1) * pitch + 1], pitch, row);
            }
            (&self.luma[..], width)
        };
        let (Ok(w), Ok(h), Ok(stride)) = (c_int::try_from(width), c_int::try_from(height), c_int::try_from(stride))
        else {
            return;
        };
        loop {
            // SAFETY: `data` covers a height x width image at `stride` (ImageView checked its length, or
            // it is the packed copy); `out` holds `out.len()` detections; the detector is ours alone.
            let n = unsafe {
                napa_detect(
                    self.det.as_ptr(),
                    data.as_ptr(),
                    w,
                    h,
                    stride,
                    &self.params,
                    self.out.as_mut_ptr(),
                    self.out.len() as c_int,
                )
            };
            if n < 0 {
                return; // invalid input (e.g. a frame too small) or out of memory: no detections
            }
            let n = n as usize;
            if n == self.out.len() {
                // Possibly truncated: retry with room for more.
                let more = 2 * self.out.len();
                self.out.resize(more, NapaDetection::default());
                continue;
            }
            out.extend(self.out[..n].iter().map(|d| Detection {
                id: d.id as u32,
                hamming: d.hamming as u32,
                decision_margin: d.decision_margin,
                corners: [
                    [d.corners[0], d.corners[1]],
                    [d.corners[2], d.corners[3]],
                    [d.corners[4], d.corners[5]],
                    [d.corners[6], d.corners[7]],
                ],
            }));
            return;
        }
    }
}

impl Drop for NapaCore {
    fn drop(&mut self) {
        // SAFETY: created by napa_create_family and destroyed once.
        unsafe { napa_destroy(self.det.as_ptr()) };
    }
}
