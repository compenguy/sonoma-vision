//! Detection accuracy on synthetic scenes with exact ground truth.

use sonoma_vision::detector::{Config, Detection, Detector, Family, ImageView, families};
use sonoma_vision::synth::{Canvas, Rng, TruthTag, random_quad};

struct Scene {
    width: usize,
    height: usize,
    gray: Vec<u8>,
    truth: Vec<TruthTag>,
}

/// Up to one tag per quadrant, with random size, rotation, mild perspective, blur and noise.
fn scene(
    rng: &mut Rng,
    family: &Family,
    width: usize,
    height: usize,
    sizes: (f64, f64),
    blur: f64,
    noise: f64,
) -> Scene {
    let mut canvas = Canvas::new(width, height, rng.range(90.0, 200.0));
    let (qw, qh) = (width as f64 / 2.0, height as f64 / 2.0);
    let mut truth = Vec::new();
    for q in 0..4 {
        if rng.unit() < 0.2 {
            continue;
        }
        let size = rng.range(sizes.0, sizes.1);
        let reach = 0.5 * size * family.total_width as f64 / family.width as f64 * 1.45;
        let cx = (q % 2) as f64 * qw + rng.range(reach.min(qw / 2.0), (qw - reach).max(qw / 2.0));
        let cy = (q / 2) as f64 * qh + rng.range(reach.min(qh / 2.0), (qh - reach).max(qh / 2.0));
        let angle = rng.range(0.0, std::f64::consts::TAU);
        let corners = random_quad(rng, cx, cy, size, angle, 0.04);
        let id = rng.below(family.codes.len());
        let ink = rng.range(10.0, 60.0);
        let paper = rng.range(170.0, 240.0);
        truth.push(canvas.draw_tag(family, id, corners, ink, paper));
    }
    canvas.blur(blur);
    canvas.noise(noise, rng);
    Scene { width, height, gray: canvas.to_gray(), truth }
}

#[derive(Default, Debug)]
struct Score {
    tags: usize,
    found: usize,
    wrong: usize,
    errors: Vec<f64>,
}

impl Score {
    fn add(&mut self, truth: &[TruthTag], dets: &[Detection]) {
        self.tags += truth.len();
        for d in dets {
            let c = d.center();
            let hit = truth.iter().find(|t| {
                let tc = t.corners.iter().fold([0.0, 0.0], |a, p| [a[0] + p[0] / 4.0, a[1] + p[1] / 4.0]);
                (tc[0] - c[0]).hypot(tc[1] - c[1]) < 5.0 && t.id == d.id
            });
            match hit {
                Some(t) => {
                    self.found += 1;
                    let err = (0..4)
                        .map(|k| (d.corners[k][0] - t.corners[k][0]).hypot(d.corners[k][1] - t.corners[k][1]))
                        .fold(0.0, f64::max);
                    self.errors.push(err);
                }
                None => self.wrong += 1,
            }
        }
    }

    fn percentile(&mut self, p: f64) -> f64 {
        self.errors.sort_by(f64::total_cmp);
        self.errors[((self.errors.len() - 1) as f64 * p) as usize]
    }

    fn recall(&self) -> f64 {
        self.found as f64 / self.tags as f64
    }
}

fn run(
    family: &'static Family,
    frames: usize,
    seed: u64,
    sizes: (f64, f64),
    blur: (f64, f64),
    noise: (f64, f64),
) -> Score {
    let mut rng = Rng::new(seed);
    let mut det = Detector::new(family, &Config::default()).unwrap();
    let mut score = Score::default();
    for _ in 0..frames {
        let b = rng.range(blur.0, blur.1);
        let n = rng.range(noise.0, noise.1);
        let s = scene(&mut rng, family, 640, 480, sizes, b, n);
        let img = ImageView::gray(&s.gray, s.width, s.height).unwrap();
        score.add(&s.truth, &det.detect(&img));
    }
    score
}

#[test]
fn tag36h11_clean() {
    let mut s = run(&families::TAG36H11, 60, 1, (36.0, 120.0), (0.3, 0.8), (0.0, 2.0));
    let (median, p95) = (s.percentile(0.5), s.percentile(0.95));
    eprintln!("clean: {s:?} median {median:.3} p95 {p95:.3}", s = (s.tags, s.found, s.wrong));
    assert_eq!(s.wrong, 0);
    assert!(s.recall() >= 0.99, "recall {}", s.recall());
    assert!(median < 0.08, "median corner error {median}");
    assert!(p95 < 0.25, "p95 corner error {p95}");
}

#[test]
fn tag36h11_blur_and_noise() {
    let mut s = run(&families::TAG36H11, 60, 2, (40.0, 120.0), (0.8, 2.0), (3.0, 8.0));
    let (median, p95) = (s.percentile(0.5), s.percentile(0.95));
    eprintln!("hard: {s:?} median {median:.3} p95 {p95:.3}", s = (s.tags, s.found, s.wrong));
    assert_eq!(s.wrong, 0);
    assert!(s.recall() >= 0.95, "recall {}", s.recall());
    assert!(median < 0.25, "median corner error {median}");
}

#[test]
fn other_families() {
    for family in [&families::TAG16H5, &families::TAG25H9, &families::TAGSTANDARD41H12, &families::TAGCIRCLE21H7] {
        let s = run(family, 15, 3, (48.0, 110.0), (0.3, 0.8), (0.0, 2.0));
        eprintln!("{}: {:?}", family.name, (s.tags, s.found, s.wrong));
        assert_eq!(s.wrong, 0, "{}", family.name);
        assert!(s.recall() >= 0.95, "{} recall {}", family.name, s.recall());
    }
}

#[test]
fn empty_and_tiny_frames() {
    let mut det = Detector::new(&families::TAG36H11, &Config::default()).unwrap();
    let flat = vec![128u8; 320 * 240];
    assert!(det.detect(&ImageView::gray(&flat, 320, 240).unwrap()).is_empty());
    let mut rng = Rng::new(9);
    let noise: Vec<u8> = (0..320 * 240).map(|_| rng.below(256) as u8).collect();
    assert!(det.detect(&ImageView::gray(&noise, 320, 240).unwrap()).is_empty());
    assert!(det.detect(&ImageView::gray(&[0; 100], 10, 10).unwrap()).is_empty());
}

#[test]
fn packed_luma_matches_gray() {
    // A YUYV-style frame (luma at every other byte) goes to napa-vision's core through a copy: it must
    // see the same image.
    let mut det = Detector::new(&families::TAG36H11, &Config::default()).unwrap();
    let mut rng = Rng::new(21);
    let mut tags = 0;
    for i in 0..10 {
        let s = scene(&mut rng, &families::TAG36H11, 640, 480, (40.0, 120.0), 0.8, 2.0);
        let gray = det.detect(&ImageView::gray(&s.gray, s.width, s.height).unwrap());
        let yuyv: Vec<u8> = s.gray.iter().flat_map(|&y| [y, 128]).collect();
        let packed = det.detect(&ImageView::new(&yuyv, s.width, s.height, 2 * s.width, 2).unwrap());
        assert_eq!(packed, gray, "frame {i}");
        tags += gray.len();
    }
    assert!(tags > 10, "only {tags} tags");
}
