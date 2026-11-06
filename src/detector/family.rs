//! Tag families and the tables that decode them.

/// An AprilTag family: a square `width` cells across whose outer ring is the border, a one-cell
/// quiet ring around it, and data cells inside the square (and, for some families, outside the quiet
/// ring). Bit k of a code, most significant first, is cell (x[k], y[k]) relative to the square's
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
}

/// Multi-index hashing: a code within H bits of another matches at least one of H + 1 disjoint blocks
/// of bits exactly, so only codes sharing a block value are compared. With `probe`, every one-bit change
/// of each block is looked up too, so fewer, larger blocks suffice.
struct CodeIndex {
    blocks: Vec<IndexBlock>,
    probe: bool,
}

struct IndexBlock {
    shift: u32,
    bits: u32,
    /// `start[v]..start[v + 1]` indexes `list` for block value v.
    start: Vec<u32>,
    list: Vec<u32>,
}

impl CodeIndex {
    fn new(codes: &[u64], nbits: u32, nblocks: u32, probe: bool) -> Self {
        let mut shift = 0;
        let blocks = (0..nblocks)
            .map(|b| {
                // Low bits first; the larger blocks last.
                let bits = nbits / nblocks + u32::from(b >= nblocks - nbits % nblocks);
                let mask = (1u64 << bits) - 1;
                let mut start = vec![0u32; (1 << bits) + 1];
                for &c in codes {
                    start[((c >> shift) & mask) as usize + 1] += 1;
                }
                for v in 0..(1 << bits) {
                    start[v + 1] += start[v];
                }
                let mut cursor = start.clone();
                let mut list = vec![0u32; codes.len()];
                for (i, &c) in codes.iter().enumerate() {
                    let v = ((c >> shift) & mask) as usize;
                    list[cursor[v] as usize] = i as u32;
                    cursor[v] += 1;
                }
                let block = IndexBlock { shift, bits, start, list };
                shift += bits;
                block
            })
            .collect();
        Self { blocks, probe }
    }
}

/// A family laid out for decoding. Cells are in grid coordinates: the square's top-left cell is
/// (outer, outer), and the grid spans `grid` cells, the square plus any data rings outside its quiet ring.
pub(crate) struct Layout {
    pub family: &'static Family,
    pub width: usize,
    pub outer: usize,
    pub grid: usize,
    pub nbits: usize,
    pub cells: f64,
    /// Largest correction the codes' spacing allows.
    pub max_hamming: u32,
    /// Default correction: chance matches no likelier than tag36h11's at 3 bits.
    pub recommended_hamming: u32,
    /// For families whose default corrects fewer than 3 bits: the most bits a soft decision may fix.
    pub soft_hamming: u32,
    /// Largest dark fraction a blob may have of its grown quad.
    pub fill_limit: f64,
    /// Tolerance of the clipping estimate's side agreement, relative to tag36h11's.
    pub bias_slack: f64,
    /// Each bit's grid index, most significant first.
    pub cell: Vec<usize>,
    /// Bits whose cells touch each bit's cell (8-connected).
    near: Vec<u64>,
    /// Every code in 4 rotations as observed (complemented for reversed families).
    pub codes: Vec<u64>,
    pub ids: Vec<u32>,
    pub rotations: Vec<u8>,
    index3: CodeIndex,
    index5: CodeIndex,
    stamp: Vec<u32>,
    stamp_now: u32,
}

impl Layout {
    pub fn new(family: &'static Family) -> Self {
        let f = family;
        let nbits = f.nbits as usize;
        assert!(nbits <= 64 && f.x.len() == nbits && f.y.len() == nbits, "malformed family {}", f.name);
        let width = f.width as usize;
        let lo = f.x.iter().chain(f.y).copied().min().unwrap_or(0).min(0);
        let hi = f.x.iter().chain(f.y).copied().max().unwrap_or(0).max(width as i8 - 1);
        let outer = (-lo) as usize;
        let grid = (hi - lo + 1) as usize;
        assert!(outer as i8 == hi - (width as i8 - 1) && grid <= 16, "asymmetric family {}", f.name);
        let cell: Vec<usize> = (0..nbits)
            .map(|k| (f.y[k] as isize + outer as isize) as usize * grid + (f.x[k] as isize + outer as isize) as usize)
            .collect();
        let mut is_data = [false; 256];
        for &c in &cell {
            is_data[c] = true;
        }
        let bit = |k: usize| 1u64 << (nbits - 1 - k);
        let mut perm = vec![0usize; nbits];
        let mut near = vec![0u64; nbits];
        for k in 0..nbits {
            let (gx, gy) = (cell[k] % grid, cell[k] / grid);
            // Turned 90 degrees counter-clockwise, as numpy's rot90: out[y][x] = in[x][grid - 1 - y].
            let src = gx * grid + (grid - 1 - gy);
            perm[k] = cell.iter().position(|&c| c == src).expect("family layout must rotate onto itself");
            for (j, &cj) in cell.iter().enumerate() {
                let dx = (cj % grid) as isize - gx as isize;
                let dy = (cj / grid) as isize - gy as isize;
                if j != k && dx.abs() <= 1 && dy.abs() <= 1 {
                    near[k] |= bit(j);
                }
            }
        }
        let mask = if nbits == 64 { !0 } else { (1u64 << nbits) - 1 };
        let max_hamming = (f.min_distance - 1) / 2;

        // Chance that a random pattern falls within h bits of some rotated code: nrot * sum C(n, k) / 2^n,
        // counting three-bit corrections only for touching cells. Recommend the largest h whose chance
        // is at most 1e-4, about three times tag36h11's at its default of 3.
        let n = nbits as f64;
        let mut placements = [1.0, n, 0.5 * n * (n - 1.0), 0.0];
        let mut layout_probe = Self {
            family,
            width,
            outer,
            grid,
            nbits,
            cells: width as f64,
            max_hamming,
            recommended_hamming: 0,
            soft_hamming: 0,
            fill_limit: 0.0,
            bias_slack: (6.0 / (width as f64 - 2.0)).sqrt(),
            cell,
            near,
            codes: Vec::new(),
            ids: Vec::new(),
            rotations: Vec::new(),
            index3: CodeIndex { blocks: Vec::new(), probe: false },
            index5: CodeIndex { blocks: Vec::new(), probe: false },
            stamp: Vec::new(),
            stamp_now: 0,
        };
        for a in 0..nbits {
            for b in a + 1..nbits {
                for c in b + 1..nbits {
                    placements[3] += f64::from(u8::from(layout_probe.errors_connected(bit(a) | bit(b) | bit(c))));
                }
            }
        }
        let nrot = 4.0 * f.codes.len() as f64;
        let mut chance = 0.0;
        for h in 0..=max_hamming.min(3) {
            chance += nrot * placements[h as usize] / 2f64.powi(nbits as i32);
            if h == 0 || chance <= 1e-4 {
                layout_probe.recommended_hamming = h;
            }
            if chance > 1e-4 {
                break;
            }
        }
        let cap = max_hamming.min(3);
        layout_probe.soft_hamming = if layout_probe.recommended_hamming < cap { cap } else { 0 };

        let rotate = |code: u64| perm.iter().fold(0u64, |out, &p| (out << 1) | ((code >> (nbits - 1 - p)) & 1));
        let mut max_dark = 0;
        let l = &mut layout_probe;
        for (id, &printed) in f.codes.iter().enumerate() {
            // Reversed families are read on the inverted image, where a printed white cell is dark.
            let mut code = if f.reversed { !printed & mask } else { printed };
            let dark = (0..nbits)
                .filter(|&k| {
                    let gx = (l.cell[k] % grid) as isize - outer as isize;
                    let gy = (l.cell[k] / grid) as isize - outer as isize;
                    let inside = gx > 0 && gy > 0 && gx < width as isize - 1 && gy < width as isize - 1;
                    inside && code & bit(k) == 0
                })
                .count();
            max_dark = max_dark.max(dark);
            for rot in 0..4 {
                l.codes.push(code);
                l.ids.push(id as u32);
                l.rotations.push(rot);
                code = rotate(code);
            }
        }
        // Store rotation-major (all rotation 0 first), as lookups don't care and it keeps ids grouped.
        let order: Vec<usize> = (0..4).flat_map(|r| (0..f.codes.len()).map(move |i| 4 * i + r)).collect();
        l.codes = order.iter().map(|&i| l.codes[i]).collect();
        l.ids = order.iter().map(|&i| l.ids[i]).collect();
        l.rotations = order.iter().map(|&i| l.rotations[i]).collect();
        // Interior cells that aren't data (tagCustom48h12's centre) are printed white: dark when inverted.
        for gy in 1..width - 1 {
            for gx in 1..width - 1 {
                max_dark += usize::from(!is_data[(gy + outer) * grid + gx + outer]);
            }
        }
        // A tag36h11 tag is at most 54/64 dark; blobs are rejected above 0.95 of the grown quad. Other
        // families keep the same slack over their own bound.
        let dark_cells = 4 * (width - 1) + max_dark;
        l.fill_limit = 0.95 + (dark_cells as f64 / (width * width) as f64 - 54.0 / 64.0);
        l.index3 = CodeIndex::new(&l.codes, f.nbits, 4, false);
        // Up to 5 wrong bits: six exact blocks, or for large families three blocks probed with one
        // changed bit each (five errors leave some block with at most one), whose buckets are far smaller.
        let large = l.codes.len() > 4096;
        l.index5 = CodeIndex::new(&l.codes, f.nbits, if large { 3 } else { 6 }, large);
        l.stamp = vec![0; l.codes.len()];
        layout_probe
    }

    /// Index of a rotated code within `max_bits` (at most 5) of `code`, and its distance. When codes
    /// are at least 2 * max_bits + 1 apart, any match is the unique nearest.
    pub fn nearest_code(&mut self, code: u64, max_bits: u32) -> Option<(usize, u32)> {
        let ix = if max_bits <= 3 { &self.index3 } else { &self.index5 };
        self.stamp_now = self.stamp_now.wrapping_add(1);
        if self.stamp_now == 0 {
            self.stamp.fill(0);
            self.stamp_now = 1;
        }
        for b in &ix.blocks {
            let v0 = (code >> b.shift) & ((1u64 << b.bits) - 1);
            let flips = if ix.probe { b.bits as i32 } else { 0 };
            for flip in -1..flips {
                let v = if flip < 0 { v0 } else { v0 ^ (1 << flip) } as usize;
                for &i in &b.list[b.start[v] as usize..b.start[v + 1] as usize] {
                    let i = i as usize;
                    if self.stamp[i] == self.stamp_now {
                        continue;
                    }
                    self.stamp[i] = self.stamp_now;
                    let d = (code ^ self.codes[i]).count_ones();
                    if d <= max_bits {
                        return Some((i, d));
                    }
                }
            }
        }
        None
    }

    /// Whether the set bits of a code difference form one 8-connected group of cells.
    pub fn errors_connected(&self, diff: u64) -> bool {
        if diff.count_ones() <= 1 {
            return true;
        }
        let mut reached = diff & diff.wrapping_neg();
        loop {
            let last = reached;
            for k in 0..self.nbits {
                if reached >> (self.nbits - 1 - k) & 1 != 0 {
                    reached |= self.near[k] & diff;
                }
            }
            if reached == last {
                return reached == diff;
            }
        }
    }

    pub fn bit(&self, k: usize) -> u64 {
        1u64 << (self.nbits - 1 - k)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::detector::families::*;

    #[test]
    fn recommended_hamming_matches_napa() {
        // napa-vision's defaults: 3 bits for tag36h11, fewer for families with less spacing.
        assert_eq!(Layout::new(&TAG36H11).recommended_hamming, 3);
        assert!(Layout::new(&TAG16H5).recommended_hamming < 2);
    }

    #[test]
    fn every_code_decodes_to_itself() {
        let mut l = Layout::new(&TAG36H11);
        for i in (0..l.codes.len()).step_by(37) {
            let code = l.codes[i];
            let (j, d) = l.nearest_code(code ^ l.bit(5) ^ l.bit(20), 3).unwrap();
            assert_eq!((l.ids[j], l.rotations[j], d), (l.ids[i], l.rotations[i], 2));
        }
    }

    #[test]
    fn large_families_build() {
        let l = Layout::new(&TAGSTANDARD52H13);
        assert_eq!(l.codes.len(), 4 * TAGSTANDARD52H13.codes.len());
    }
}
