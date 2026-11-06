//! Tag families: the codes and cell layouts (for drawing tags; napa-vision's core decodes its own copy).

/// An AprilTag family: a square `width` cells across whose outer ring is the border, a one-cell
/// quiet ring around it, and data cells inside the square (and, for some families, outside the quiet
/// ring). Bit k of a code, most significant first, is cell (`x[k]`, `y[k]`) relative to the square's
/// top-left cell; a set bit is a white printed cell. Reversed families print the square's border white
/// inside a dark frame and are detected on the inverted image.
#[derive(Debug)]
pub struct Family {
    pub name: &'static str,
    pub width: u32,
    pub total_width: u32,
    pub reversed: bool,
    pub min_distance: u32,
    pub nbits: u32,
    pub x: &'static [i8],
    pub y: &'static [i8],
    pub codes: &'static [u64],
}

impl Family {
    pub fn by_name(name: &str) -> Option<&'static Family> {
        super::families::ALL.iter().copied().find(|f| f.name.eq_ignore_ascii_case(name))
    }

    pub fn names() -> impl Iterator<Item = &'static str> {
        super::families::ALL.iter().map(|f| f.name)
    }

    /// The most payload bits a decode may correct for this family (at most 3).
    pub fn max_hamming(&self) -> u32 {
        ((self.min_distance - 1) / 2).min(3)
    }

    /// The default correction: chance matches no likelier than tag36h11's at 3 bits (napa-vision's).
    pub fn recommended_hamming(&self) -> u32 {
        super::napa::family_info(self.name).map_or(0, |i| i.recommended_hamming as u32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::detector::families::*;

    #[test]
    fn recommended_hamming_matches_napa() {
        // napa-vision's defaults: 3 bits for tag36h11, fewer for families with less spacing.
        assert_eq!(TAG36H11.recommended_hamming(), 3);
        assert!(TAG16H5.recommended_hamming() < 2);
    }

    #[test]
    fn tables_match_napa() {
        // The Rust tables (for drawing tags) and napa-vision's (for decoding) must be the same families.
        for f in Family::names().map(|n| Family::by_name(n).unwrap()) {
            let info = crate::detector::napa::family_info(f.name).unwrap();
            assert_eq!(
                (info.ncodes as usize, info.nbits as u32, info.width as u32, info.total_width as u32),
                (f.codes.len(), f.nbits, f.width, f.total_width),
                "{}",
                f.name
            );
            assert_eq!((info.reversed != 0, info.min_distance as u32), (f.reversed, f.min_distance), "{}", f.name);
            assert_eq!(info.max_hamming as u32, f.max_hamming(), "{}", f.name);
            // The two list cells in different orders: compare each code's white cells.
            let n = f.nbits as usize;
            let cells = |xs: &[i8], ys: &[i8], code: u64| {
                let mut white: Vec<(i8, i8)> =
                    (0..n).filter(|&k| code >> (n - 1 - k) & 1 != 0).map(|k| (xs[k], ys[k])).collect();
                white.sort_unstable();
                white
            };
            for id in (0..f.codes.len()).step_by(97) {
                let napa = crate::detector::napa::family_code(f.name, id).unwrap();
                assert_eq!(cells(&info.x[..n], &info.y[..n], napa), cells(f.x, f.y, f.codes[id]), "{} {id}", f.name);
            }
        }
    }
}
