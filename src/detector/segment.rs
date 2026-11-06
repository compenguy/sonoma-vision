//! Adaptive thresholding of a level into runs of dark samples, joined into blobs by union-find, and
//! the extreme points of candidate blobs.

use super::level::Floors;

const TILE: usize = 4;

/// A horizontal run of dark samples in one level row.
#[derive(Clone, Copy, Debug)]
pub struct Run {
    pub x0: u16,
    pub x1: u16,
    pub y: u16,
}

impl Run {
    fn len(&self) -> i32 {
        i32::from(self.x1) - i32::from(self.x0) + 1
    }
}

/// Reusable buffers for thresholding one level.
#[derive(Default)]
pub struct Segmenter {
    tile_min: Vec<u8>,
    tile_max: Vec<u8>,
    vmin: Vec<u8>,
    vmax: Vec<u8>,
    nlo: Vec<u8>,
    nhi: Vec<u8>,
    tile_thr: Vec<u8>,
    thr_row: Vec<u8>,
    bits: Vec<u64>,
    pub runs: Vec<Run>,
    /// Union-find parents (always an earlier run), then, after `label`, `-component - 1`.
    pub parent: Vec<i32>,
    pub boxes: Vec<BlobBox>,
}

/// A blob's sample count and bounding box.
#[derive(Clone, Copy, Debug)]
pub struct BlobBox {
    pub count: i32,
    pub minx: u16,
    pub maxx: u16,
    pub miny: u16,
    pub maxy: u16,
}

fn find_root(p: &mut [i32], mut i: i32) -> i32 {
    while p[i as usize] != i {
        p[i as usize] = p[p[i as usize] as usize];
        i = p[i as usize];
    }
    i
}

/// Bit x of `bits[x / 64]` is set where `row[x] < thr[x]`. Both slices hold whole 64-sample words.
fn dark_bits(row: &[u8], thr: &[u8], bits: &mut [u64]) {
    for ((word, r), t) in bits.iter_mut().zip(row.chunks_exact(64)).zip(thr.chunks_exact(64)) {
        let mut w = 0u64;
        for k in 0..64 {
            w |= u64::from(r[k] < t[k]) << k;
        }
        *word = w;
    }
}

/// Append a row's runs of set bits. Run starts and ends are the set bits of `bits ^ (bits << 1)`, found
/// with count-trailing-zeros, so the cost is per run, not per sample.
fn extract_runs(bits: &[u64], lw: usize, y: usize, runs: &mut Vec<Run>) {
    let mut open = false;
    let mut x0 = 0;
    for (w, &word) in bits.iter().enumerate() {
        let mut edges = word ^ ((word << 1) | u64::from(open));
        while edges != 0 {
            let b = edges.trailing_zeros() as usize;
            edges &= edges - 1;
            if open {
                runs.push(Run { x0: x0 as u16, x1: (64 * w + b - 1) as u16, y: y as u16 });
            } else {
                x0 = 64 * w + b;
            }
            open = !open;
        }
    }
    if open {
        runs.push(Run { x0: x0 as u16, x1: (lw - 1) as u16, y: y as u16 });
    }
}

impl Segmenter {
    /// Threshold the level (`level` holds `lh` rows of `lw` samples and at least 63 more bytes) and label
    /// its dark samples as 8-connected runs. Each sample is compared with the midpoint of its 4×4 tile
    /// neighbourhood's (3×3 tiles) min and max; neighbourhoods with less contrast than their floor
    /// can't hold a tag edge and are skipped. `factor` is this level's stride over the floors' grid.
    pub fn threshold(&mut self, level: &[u8], lw: usize, lh: usize, floors: &Floors, factor: usize) {
        let tw = lw.div_ceil(TILE);
        let th = lh.div_ceil(TILE);
        let words = lw.div_ceil(64);
        self.tile_min.resize(tw * th, 0);
        self.tile_max.resize(tw * th, 0);
        self.vmin.resize(TILE * tw, 0);
        self.vmax.resize(TILE * tw, 0);
        self.nlo.resize(tw + 2, 0);
        self.nhi.resize(tw + 2, 0);
        self.tile_thr.resize(tw, 0);
        self.thr_row.resize(words * 64 + TILE * tw, 0);
        self.bits.resize(words, 0);
        self.runs.clear();
        self.parent.clear();

        // Tile min/max: a vertical pass over each band of TILE rows, then a horizontal pass per tile.
        for ty in 0..th {
            let y0 = ty * TILE;
            let y1 = (y0 + TILE).min(lh);
            self.vmin[..lw].copy_from_slice(&level[y0 * lw..(y0 + 1) * lw]);
            self.vmax[..lw].copy_from_slice(&level[y0 * lw..(y0 + 1) * lw]);
            for y in y0 + 1..y1 {
                let row = &level[y * lw..(y + 1) * lw];
                for ((lo, hi), &v) in self.vmin.iter_mut().zip(self.vmax.iter_mut()).zip(row) {
                    *lo = (*lo).min(v);
                    *hi = (*hi).max(v);
                }
            }
            // Pad past lw by repeating the last sample, so every tile reads four.
            let (last_lo, last_hi) = (self.vmin[lw - 1], self.vmax[lw - 1]);
            self.vmin[lw..].fill(last_lo);
            self.vmax[lw..].fill(last_hi);
            let tmin = &mut self.tile_min[ty * tw..(ty + 1) * tw];
            let tmax = &mut self.tile_max[ty * tw..(ty + 1) * tw];
            for tx in 0..tw {
                let a = &self.vmin[TILE * tx..TILE * tx + TILE];
                let b = &self.vmax[TILE * tx..TILE * tx + TILE];
                tmin[tx] = a[0].min(a[1]).min(a[2].min(a[3]));
                tmax[tx] = b[0].max(b[1]).max(b[2].max(b[3]));
            }
        }

        let mut prev: Option<(usize, usize, usize)> = None; // (y, first run, end run) of the last row
        for ty in 0..th {
            let up = ty.saturating_sub(1) * tw;
            let mid = ty * tw;
            let dn = (ty + 1).min(th - 1) * tw;
            // 3×3 tile neighbourhood, separably: vertically into a row padded by one tile at each end.
            for tx in 0..tw {
                let lo = self.tile_min[mid + tx].min(self.tile_min[up + tx]).min(self.tile_min[dn + tx]);
                let hi = self.tile_max[mid + tx].max(self.tile_max[up + tx]).max(self.tile_max[dn + tx]);
                self.nlo[tx + 1] = lo;
                self.nhi[tx + 1] = hi;
            }
            self.nlo[0] = self.nlo[1];
            self.nhi[0] = self.nhi[1];
            self.nlo[tw + 1] = self.nlo[tw];
            self.nhi[tw + 1] = self.nhi[tw];
            let uniform = floors.uniform();
            let mut any = false;
            for tx in 0..tw {
                let l = self.nlo[tx].min(self.nlo[tx + 1]).min(self.nlo[tx + 2]);
                let h = self.nhi[tx].max(self.nhi[tx + 1]).max(self.nhi[tx + 2]);
                let floor = uniform.unwrap_or_else(|| floors.at(tx, ty, factor));
                // A flat neighbourhood can't hold a tag edge: threshold 0, so no sample is dark.
                let t = if h - l >= floor { ((u16::from(l) + u16::from(h) + 1) >> 1) as u8 } else { 0 };
                self.tile_thr[tx] = t;
                any |= t != 0;
            }
            if !any {
                continue;
            }
            for (chunk, &t) in self.thr_row.chunks_exact_mut(TILE).zip(&self.tile_thr) {
                chunk.fill(t);
            }
            self.thr_row[lw..].fill(0); // nothing past the row is dark
            let y0 = ty * TILE;
            let y1 = (y0 + TILE).min(lh);
            for y in y0..y1 {
                dark_bits(&level[y * lw..y * lw + words * 64], &self.thr_row, &mut self.bits);
                let start = self.runs.len();
                extract_runs(&self.bits, lw, y, &mut self.runs);
                let end = self.runs.len();
                self.parent.extend(start as i32..end as i32);
                if let Some((py, ps, pe)) = prev
                    && py + 1 == y
                {
                    self.join_rows(ps, pe, start, end);
                }
                prev = Some((y, start, end));
            }
        }
    }

    /// Join runs of adjacent rows that overlap within one sample (8-connected).
    fn join_rows(&mut self, mut a: usize, a_end: usize, mut b: usize, b_end: usize) {
        let (r, p) = (&self.runs, &mut self.parent);
        while a < a_end && b < b_end {
            if r[a].x1 + 1 < r[b].x0 {
                a += 1;
            } else if r[b].x1 + 1 < r[a].x0 {
                b += 1;
            } else {
                // A run of this row that touches nothing yet is a singleton: hang it under a's parent,
                // which still precedes it. Runs whose parents match are already joined.
                if p[b] == b as i32 {
                    p[b] = p[a];
                } else if p[b] != p[a] {
                    let ra = find_root(p, a as i32);
                    let rb = find_root(p, b as i32);
                    if ra < rb {
                        p[rb as usize] = ra;
                    } else if rb < ra {
                        p[ra as usize] = rb;
                    }
                }
                if r[a].x1 < r[b].x1 {
                    a += 1;
                } else {
                    b += 1;
                }
            }
        }
    }

    /// Resolve runs into components with bounding boxes, in raster order of their first run. Afterwards
    /// `parent[i]` is `-component - 1` for run i.
    pub fn label(&mut self) {
        self.boxes.clear();
        let (runs, p) = (&self.runs, &mut self.parent);
        for i in 0..runs.len() {
            let r = runs[i];
            let c = if p[i] == i as i32 {
                self.boxes.push(BlobBox { count: r.len(), minx: r.x0, maxx: r.x1, miny: r.y, maxy: r.y });
                self.boxes.len() - 1
            } else {
                // A union always points to an earlier run, which already holds its component, encoded
                // negatively. Path compression may leave a link to a non-root earlier run: follow it.
                let mut q = p[i];
                while q >= 0 {
                    q = p[q as usize];
                }
                let c = (-q - 1) as usize;
                let b = &mut self.boxes[c];
                b.count += r.len();
                b.minx = b.minx.min(r.x0);
                b.maxx = b.maxx.max(r.x1);
                b.maxy = r.y;
                c
            };
            p[i] = -(c as i32) - 1;
        }
    }
}

/// Directions whose extreme points seed the rough quad, in clockwise angular order (y down), about 22.5
/// degrees apart: a corner lies within about 11 degrees of one of them.
pub const DIRS: [(i32, i32); 16] = [
    (1, 0),
    (2, 1),
    (1, 1),
    (1, 2),
    (0, 1),
    (-1, 2),
    (-1, 1),
    (-2, 1),
    (-1, 0),
    (-2, -1),
    (-1, -1),
    (-1, -2),
    (0, -1),
    (1, -2),
    (1, -1),
    (2, -1),
];

/// A candidate blob: its box and its extreme points along `DIRS`.
#[derive(Clone, Debug)]
pub struct Component {
    pub count: i32,
    pub bbox: BlobBox,
    pub ext: [(i32, i32); 16],
    extv: [i32; 16],
    row_y: i32,
    row_x0: i32,
    row_x1: i32,
}

impl Component {
    pub fn new(bbox: BlobBox) -> Self {
        Self { count: 0, bbox, ext: [(0, 0); 16], extv: [i32::MIN; 16], row_y: -1, row_x0: 0, row_x1: 0 }
    }

    /// Fold the current row into the extremes. Every direction's extreme over a row lies at the row's
    /// leftmost or rightmost dark sample, so this runs once per row, not once per run.
    pub fn flush_row(&mut self) {
        let y = self.row_y;
        for (k, &(dx, dy)) in DIRS.iter().enumerate() {
            let x = if dx > 0 { self.row_x1 } else { self.row_x0 };
            let v = dx * x + dy * y;
            if v > self.extv[k] {
                self.extv[k] = v;
                self.ext[k] = (x, y);
            }
        }
    }

    /// Runs arrive in raster order.
    pub fn add_run(&mut self, r: &Run) {
        if i32::from(r.y) != self.row_y {
            if self.row_y >= 0 {
                self.flush_row();
            }
            self.row_y = i32::from(r.y);
            self.row_x0 = i32::from(r.x0);
        }
        self.row_x1 = i32::from(r.x1);
        self.count += r.len();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn runs_from_bits() {
        let bits = [0b0111_0001u64 | (1 << 63), 0b11];
        let mut runs = Vec::new();
        extract_runs(&bits, 70, 3, &mut runs);
        let spans: Vec<_> = runs.iter().map(|r| (r.x0, r.x1)).collect();
        assert_eq!(spans, [(0, 0), (4, 6), (63, 65)]);
    }

    #[test]
    fn labels_two_blobs() {
        // An 8x8 level with a dark ring and a separate dark dot, on a bright background with contrast.
        let (lw, lh) = (16, 8);
        let mut level = vec![200u8; lw * lh + 64];
        for y in 1..6 {
            for x in 1..6 {
                if y == 1 || y == 5 || x == 1 || x == 5 {
                    level[y * lw + x] = 10;
                }
            }
        }
        level[3 * lw + 12] = 10;
        let mut floors = Floors::default();
        floors.measure(&level, lw, lh, 20, 6.0, 64);
        let mut seg = Segmenter::default();
        seg.threshold(&level, lw, lh, &floors, 1);
        seg.label();
        let mut counts: Vec<_> = seg.boxes.iter().map(|b| b.count).collect();
        counts.sort();
        assert_eq!(counts, [1, 16]);
    }
}
