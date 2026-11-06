//! Copying a packed frame's luma out to one byte per pixel (YUYV/UYVY, for napa-vision's core).
//!
//! SSE2 (x86_64) and NEON (aarch64) are baseline on their targets; SSSE3 is detected at runtime. Other
//! targets use the scalar loop.

#[cfg(target_arch = "aarch64")]
use std::arch::aarch64::*;
#[cfg(target_arch = "x86_64")]
use std::arch::x86_64::*;

/// `dst[x] = src[x * step]` for every x in `dst`; `src` holds exactly the samples' span.
pub fn gather_stride(src: &[u8], step: usize, dst: &mut [u8]) {
    let n = dst.len();
    assert!(n > 0 && src.len() == (n - 1) * step + 1, "gather of {n} at stride {step} from {}", src.len());
    if step == 1 {
        dst.copy_from_slice(src);
        return;
    }
    #[cfg(any(target_arch = "x86_64", target_arch = "aarch64"))]
    let done = gather_simd(src, step, dst);
    #[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
    let done = 0;
    for (x, d) in dst.iter_mut().enumerate().skip(done) {
        *d = src[x * step];
    }
}

/// Whole blocks of [`gather_stride`]: `chunk` outputs at a time while their `chunk * step` source bytes
/// lie within `src`. Returns how many outputs were written.
#[cfg(any(target_arch = "x86_64", target_arch = "aarch64"))]
#[inline(always)]
fn blocks(src: &[u8], step: usize, n: usize, chunk: usize, mut f: impl FnMut(usize)) -> usize {
    let mut x = 0;
    while x + chunk <= n && step * (x + chunk) <= src.len() {
        f(x);
        x += chunk;
    }
    x
}

#[cfg(target_arch = "aarch64")]
fn gather_simd(src: &[u8], step: usize, dst: &mut [u8]) -> usize {
    let n = dst.len();
    let (s, d) = (src.as_ptr(), dst.as_mut_ptr());
    // SAFETY: NEON is baseline. `blocks` only calls back for x where the load's `chunk * step` bytes at
    // `step * x` lie within src and the store's `chunk` bytes at x lie within dst. For even strides
    // (pitch-2 luma) the samples are the low bytes of u16 lanes (little-endian).
    unsafe {
        match step {
            2 => blocks(src, 2, n, 16, |x| vst1q_u8(d.add(x), vld2q_u8(s.add(2 * x)).0)),
            3 => blocks(src, 3, n, 16, |x| vst1q_u8(d.add(x), vld3q_u8(s.add(3 * x)).0)),
            4 => blocks(src, 4, n, 16, |x| vst1q_u8(d.add(x), vld4q_u8(s.add(4 * x)).0)),
            6 => blocks(src, 6, n, 8, |x| vst1_u8(d.add(x), vmovn_u16(vld3q_u16(s.add(6 * x).cast()).0))),
            8 => blocks(src, 8, n, 8, |x| vst1_u8(d.add(x), vmovn_u16(vld4q_u16(s.add(8 * x).cast()).0))),
            _ => 0,
        }
    }
}

#[cfg(target_arch = "x86_64")]
fn gather_simd(src: &[u8], step: usize, dst: &mut [u8]) -> usize {
    let n = dst.len();
    let (s, d) = (src.as_ptr(), dst.as_mut_ptr());
    match step {
        // SAFETY: SSE2 is baseline; `blocks` bounds every load (step * 16 bytes) and store (16 bytes).
        2 => unsafe {
            let lo = _mm_set1_epi16(0x00ff);
            blocks(src, 2, n, 16, |x| {
                let a = _mm_and_si128(_mm_loadu_si128(s.add(2 * x).cast()), lo);
                let b = _mm_and_si128(_mm_loadu_si128(s.add(2 * x + 16).cast()), lo);
                _mm_storeu_si128(d.add(x).cast(), _mm_packus_epi16(a, b));
            })
        },
        // SAFETY: as above. Masked to 0..=255 per 32-bit lane, so the signed pack is exact.
        4 => unsafe {
            let lo = _mm_set1_epi32(0xff);
            blocks(src, 4, n, 16, |x| {
                let v = |k: usize| _mm_and_si128(_mm_loadu_si128(s.add(4 * x + 16 * k).cast()), lo);
                let ab = _mm_packs_epi32(v(0), v(1));
                let cd = _mm_packs_epi32(v(2), v(3));
                _mm_storeu_si128(d.add(x).cast(), _mm_packus_epi16(ab, cd));
            })
        },
        // SAFETY: the SSSE3 path runs only where the CPU has it.
        3..=16 if std::arch::is_x86_feature_detected!("ssse3") => unsafe { gather_ssse3(src, step, dst) },
        _ => 0,
    }
}

/// [`gather_simd`] for any stride up to 16: each output block is `step` 16-byte loads, each shuffled
/// to the outputs it holds and OR-ed together.
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "ssse3")]
fn gather_ssse3(src: &[u8], step: usize, dst: &mut [u8]) -> usize {
    let n = dst.len();
    let (s, d) = (src.as_ptr(), dst.as_mut_ptr());
    // masks[k][i]: where output i sits in load k, or -1 (pshufb writes 0) if elsewhere.
    let mut masks = [[-1i8; 16]; 16];
    for i in 0..16 {
        masks[step * i / 16][i] = (step * i % 16) as i8;
    }
    // SAFETY: `blocks` bounds every load (`step` loads of 16 bytes at step * x) and store (16 bytes).
    unsafe {
        let m: [__m128i; 16] = std::array::from_fn(|k| _mm_loadu_si128(masks[k].as_ptr().cast()));
        blocks(src, step, n, 16, |x| {
            let mut v = _mm_setzero_si128();
            for (k, mask) in m.iter().enumerate().take(step) {
                v = _mm_or_si128(v, _mm_shuffle_epi8(_mm_loadu_si128(s.add(step * x + 16 * k).cast()), *mask));
            }
            _mm_storeu_si128(d.add(x).cast(), v);
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gather_matches_scalar() {
        // xorshift64*
        let mut state = 5u64;
        let data: Vec<u8> = (0..4096)
            .map(|_| {
                state ^= state >> 12;
                state ^= state << 25;
                state ^= state >> 27;
                (state.wrapping_mul(0x2545f4914f6cdd1d) >> 56) as u8
            })
            .collect();
        for step in 1..=17 {
            for n in [1, 7, 8, 9, 15, 16, 17, 33, 100, 200] {
                let src = &data[..(n - 1) * step + 1];
                let mut dst = vec![0u8; n];
                gather_stride(src, step, &mut dst);
                assert!(dst.iter().enumerate().all(|(x, &v)| v == src[x * step]), "step {step}, n {n}");
            }
        }
    }
}
