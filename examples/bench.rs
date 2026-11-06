//! Synthetic benchmark: recall, wrong IDs, corner error and latency per scenario.
//!
//!     cargo run --release --example bench -- [--frames N] [--repeat K] [--pgm-dir DIR]
//!         [--detections FILE] [--yuyv]
//!
//! The last line is a checksum of every detection (IDs and corners' exact bits), to confirm an
//! optimisation changes nothing. `--help` describes each option.

use std::fmt::Write as _;
use std::path::PathBuf;
use std::time::Instant;

use clap::Parser;

use sonoma_vision::detector::{Config, Detector, ImageView, families};
use sonoma_vision::synth::{Canvas, Rng, TruthTag, random_quad, write_pgm};

struct Scenario {
    name: &'static str,
    width: usize,
    height: usize,
    sizes: (f64, f64),
    skew: f64,
    blur: (f64, f64),
    noise: (f64, f64),
    /// Multiply the image by this before quantising (values above 1 clip the white).
    gain: f64,
    tags: usize,
}

const SCENARIOS: &[Scenario] = &[
    Scenario {
        name: "clean-640",
        width: 640,
        height: 480,
        sizes: (40.0, 160.0),
        skew: 0.04,
        blur: (0.3, 0.8),
        noise: (0.0, 2.0),
        gain: 1.0,
        tags: 4,
    },
    Scenario {
        name: "small-1280",
        width: 1280,
        height: 720,
        sizes: (32.0, 48.0),
        skew: 0.04,
        blur: (0.3, 0.8),
        noise: (1.0, 3.0),
        gain: 1.0,
        tags: 12,
    },
    Scenario {
        name: "large-1280",
        width: 1280,
        height: 720,
        sizes: (120.0, 300.0),
        skew: 0.04,
        blur: (0.5, 1.2),
        noise: (1.0, 3.0),
        gain: 1.0,
        tags: 3,
    },
    Scenario {
        name: "perspective",
        width: 1280,
        height: 720,
        sizes: (60.0, 160.0),
        skew: 0.15,
        blur: (0.3, 0.8),
        noise: (1.0, 3.0),
        gain: 1.0,
        tags: 6,
    },
    Scenario {
        name: "blur+noise",
        width: 1280,
        height: 720,
        sizes: (48.0, 160.0),
        skew: 0.04,
        blur: (1.2, 2.5),
        noise: (4.0, 10.0),
        gain: 1.0,
        tags: 6,
    },
    Scenario {
        name: "clipped",
        width: 1280,
        height: 720,
        sizes: (48.0, 160.0),
        skew: 0.04,
        blur: (0.8, 1.5),
        noise: (1.0, 3.0),
        gain: 1.6,
        tags: 6,
    },
    Scenario {
        name: "empty-1920",
        width: 1920,
        height: 1080,
        sizes: (0.0, 0.0),
        skew: 0.0,
        blur: (0.5, 0.5),
        noise: (2.0, 2.0),
        gain: 1.0,
        tags: 0,
    },
];

fn render(rng: &mut Rng, sc: &Scenario) -> (Vec<u8>, Vec<TruthTag>) {
    let family = &families::TAG36H11;
    let mut canvas = Canvas::new(sc.width, sc.height, rng.range(80.0, 180.0));
    // Background clutter: dark strokes and boxes that aren't tags.
    for _ in 0..20 {
        let (x, y) = (rng.below(sc.width), rng.below(sc.height));
        let (w, h) = (rng.below(60) + 2, rng.below(60) + 2);
        let v = rng.range(0.0, 255.0);
        for yy in y..(y + h).min(sc.height) {
            for xx in x..(x + w).min(sc.width) {
                canvas.pixels[yy * sc.width + xx] = v;
            }
        }
    }
    // Tags on a jittered grid so they don't overlap.
    let cols = (sc.tags as f64 * sc.width as f64 / sc.height as f64).sqrt().ceil().max(1.0) as usize;
    let rows = sc.tags.div_ceil(cols).max(1);
    let (cw, ch) = (sc.width as f64 / cols as f64, sc.height as f64 / rows as f64);
    let mut truth = Vec::new();
    for i in 0..sc.tags {
        let reach = |size: f64| 0.5 * size * 1.25 * (1.45 + 2.0 * sc.skew);
        let size = rng.range(sc.sizes.0, sc.sizes.1).min((cw.min(ch) / 2.0) / (0.5 * 1.25 * (1.45 + 2.0 * sc.skew)));
        let r = reach(size);
        let cx = (i % cols) as f64 * cw + rng.range(r, (cw - r).max(r));
        let cy = (i / cols) as f64 * ch + rng.range(r, (ch - r).max(r));
        let angle = rng.range(0.0, std::f64::consts::TAU);
        let corners = random_quad(rng, cx, cy, size, angle, sc.skew);
        let id = rng.below(family.codes.len());
        let (ink, paper) = (rng.range(15.0, 60.0), rng.range(170.0, 235.0));
        truth.push(canvas.draw_tag(family, id, corners, ink, paper));
    }
    let blur = rng.range(sc.blur.0, sc.blur.1);
    canvas.blur(blur);
    let noise = rng.range(sc.noise.0, sc.noise.1);
    canvas.noise(noise, rng);
    for p in &mut canvas.pixels {
        *p *= sc.gain;
    }
    (canvas.to_gray(), truth)
}

/// Synthetic benchmark: recall, wrong IDs, corner error and latency per scenario.
#[derive(Parser)]
struct Args {
    /// Frames per scenario.
    #[arg(long, default_value_t = 50)]
    frames: usize,
    /// Detect each frame this many times, timing each, so a profile is of the detector rather than of
    /// drawing frames.
    #[arg(long, default_value_t = 1)]
    repeat: usize,
    /// Also write every frame to this directory as PGM, with a truth.txt, for comparing other
    /// detectors on the same frames.
    #[arg(long, value_name = "DIR")]
    pgm_dir: Option<PathBuf>,
    /// Write every detection to this file (`frame id x0 y0 ... x3 y3`, with --pgm-dir's frame names),
    /// for comparing against other detectors.
    #[arg(long, value_name = "FILE")]
    detections: Option<PathBuf>,
    /// Hand frames over as packed YUYV (luma at every other byte).
    #[arg(long)]
    yuyv: bool,
}

fn main() {
    let args = Args::parse();
    let (frames, repeat, yuyv, dump) = (args.frames, args.repeat, args.yuyv, args.pgm_dir);
    let mut detections_txt = args.detections.map(|p| (p, String::new()));
    let mut det = Detector::new(&families::TAG36H11, &Config::default()).unwrap();
    let mut packed = Vec::new();
    let mut truth_txt = String::new();
    // FNV-1a over every detection's id and corner bits.
    let mut checksum = 0xcbf29ce484222325u64;
    let mut mix = |v: u64| {
        for b in v.to_le_bytes() {
            checksum = (checksum ^ u64::from(b)).wrapping_mul(0x100000001b3);
        }
    };
    println!(
        "{:<12} {:>5} {:>7} {:>6} {:>8} {:>8} {:>8} {:>8}",
        "scenario", "tags", "recall", "wrong", "med px", "p95 px", "med ms", "max ms"
    );
    for (si, sc) in SCENARIOS.iter().enumerate() {
        let mut rng = Rng::new(1000 + si as u64);
        let (mut tags, mut found, mut wrong) = (0, 0, 0);
        let mut errors = Vec::new();
        let mut times = Vec::new();
        let mut out = Vec::new();
        for f in 0..frames {
            let (gray, truth) = render(&mut rng, sc);
            if let Some(dir) = &dump {
                let name = format!("{}-{f:03}.pgm", sc.name);
                write_pgm(&dir.join(&name), sc.width, sc.height, &gray).expect("write pgm");
                for t in &truth {
                    let c = t.corners.iter().flatten().map(|v| format!("{v:.6}")).collect::<Vec<_>>().join(" ");
                    writeln!(truth_txt, "{name} {} {c}", t.id).unwrap();
                }
            }
            let img = if yuyv {
                packed.clear();
                packed.extend(gray.iter().flat_map(|&y| [y, 128]));
                ImageView::new(&packed, sc.width, sc.height, 2 * sc.width, 2).unwrap()
            } else {
                ImageView::gray(&gray, sc.width, sc.height).unwrap()
            };
            for _ in 0..repeat.max(1) {
                let t0 = Instant::now();
                det.detect_into(&img, &mut out);
                times.push(t0.elapsed().as_secs_f64() * 1e3);
            }
            tags += truth.len();
            for d in &out {
                if let Some((_, txt)) = &mut detections_txt {
                    let c = d.corners.iter().flatten().map(|v| format!("{v:.6}")).collect::<Vec<_>>().join(" ");
                    writeln!(txt, "{}-{f:03}.pgm {} {c}", sc.name, d.id).unwrap();
                }
                mix(u64::from(d.id));
                d.corners.iter().flatten().for_each(|c| mix(c.to_bits()));
                let c = d.center();
                let hit = truth.iter().find(|t| {
                    let tc = t.corners.iter().fold([0.0, 0.0], |a, p| [a[0] + p[0] / 4.0, a[1] + p[1] / 4.0]);
                    t.id == d.id && (tc[0] - c[0]).hypot(tc[1] - c[1]) < 5.0
                });
                match hit {
                    Some(t) => {
                        found += 1;
                        errors.push(
                            (0..4)
                                .map(|k| (d.corners[k][0] - t.corners[k][0]).hypot(d.corners[k][1] - t.corners[k][1]))
                                .fold(0.0, f64::max),
                        );
                    }
                    None => wrong += 1,
                }
            }
        }
        errors.sort_by(f64::total_cmp);
        times.sort_by(f64::total_cmp);
        let pct = |v: &[f64], p: f64| if v.is_empty() { f64::NAN } else { v[((v.len() - 1) as f64 * p) as usize] };
        println!(
            "{:<12} {:>5} {:>6.1}% {:>6} {:>8.3} {:>8.3} {:>8.3} {:>8.3}",
            sc.name,
            tags,
            if tags > 0 { 100.0 * found as f64 / tags as f64 } else { 100.0 },
            wrong,
            pct(&errors, 0.5),
            pct(&errors, 0.95),
            pct(&times, 0.5),
            pct(&times, 1.0),
        );
    }
    println!("checksum {checksum:016x}");
    if let Some((path, txt)) = detections_txt {
        std::fs::write(path, txt).expect("write detections");
    }
    if let Some(dir) = &dump {
        std::fs::write(dir.join("truth.txt"), truth_txt).expect("write truth");
    }
}
