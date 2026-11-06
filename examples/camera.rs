//! Capture from a camera and detect tags, printing per-frame timing, then a summary: median and
//! 95th-percentile luma, detect and age times.
//!
//!     cargo run --release --example camera -- [--device /dev/video0] [--width 640] [--height 480]
//!         [--fps 30] [--format mjpeg] [--seconds 5] [--quiet]

use std::path::PathBuf;
use std::time::{Duration, Instant};

use clap::Parser;
use sonoma_vision::capture::Layout;

use sonoma_vision::capture::{Camera, CameraConfig};
use sonoma_vision::clock::monotonic_now;
use sonoma_vision::detector::{Config, Detector, families};

/// Capture from a camera and detect tags, printing per-frame timing and a summary.
#[derive(Parser)]
struct Args {
    /// V4L2 capture device.
    #[arg(long, default_value = "/dev/video0")]
    device: PathBuf,
    #[arg(long, default_value_t = 640)]
    width: u32,
    #[arg(long, default_value_t = 480)]
    height: u32,
    /// Requested frame rate.
    #[arg(long, default_value_t = 30)]
    fps: u32,
    /// Pixel format (grey, nv12, yuyv, uyvy or mjpeg); default the service's choice.
    #[arg(long)]
    format: Option<Layout>,
    /// How long to capture.
    #[arg(long, default_value_t = 5.0)]
    seconds: f64,
    /// Print only the summary.
    #[arg(long)]
    quiet: bool,
    /// Log level, or an env_logger filter.
    #[arg(long, default_value = "info")]
    log_level: String,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    env_logger::Builder::new().parse_filters(&args.log_level).init();
    let cfg = CameraConfig {
        device: args.device,
        width: args.width,
        height: args.height,
        fps: Some(args.fps),
        layout: args.format,
        buffers: 3,
    };
    let mut cam = Camera::open(&cfg)?;
    let mut det = Detector::new(&families::TAG36H11, &Config::default())?;
    let (mut scratch, mut out) = (Vec::new(), Vec::new());
    let start = Instant::now();
    let (mut frames, mut bad) = (0, 0);
    let quiet = args.quiet;
    let (mut luma_ms, mut detect_ms, mut age_ms) = (Vec::new(), Vec::new(), Vec::new());
    while start.elapsed() < Duration::from_secs_f64(args.seconds) {
        let Some(frame) = cam.next_frame(Duration::from_secs(1))? else { continue };
        let t0 = Instant::now();
        // As the service does: a corrupt frame (a bad MJPEG) is skipped, not fatal.
        let img = match frame.luma(&mut scratch) {
            Ok(img) => img,
            Err(e) => {
                bad += 1;
                eprintln!("seq {:6} skipped: {e:#}", frame.sequence);
                continue;
            }
        };
        let t1 = Instant::now();
        det.detect_into(&img, &mut out);
        let age = monotonic_now().saturating_sub(frame.timestamp);
        frames += 1;
        luma_ms.push((t1 - t0).as_secs_f64() * 1e3);
        detect_ms.push(t1.elapsed().as_secs_f64() * 1e3);
        age_ms.push(age.as_secs_f64() * 1e3);
        if quiet {
            continue;
        }
        println!(
            "seq {:6} luma {:5.2} ms detect {:5.2} ms age {:6.2} ms tags {:?}",
            frame.sequence,
            (t1 - t0).as_secs_f64() * 1e3,
            t1.elapsed().as_secs_f64() * 1e3,
            age.as_secs_f64() * 1e3,
            out.iter().map(|d| d.id).collect::<Vec<_>>()
        );
    }
    let pct = |v: &mut Vec<f64>| {
        v.sort_by(f64::total_cmp);
        let at = |q: f64| v.get(((v.len() as f64 - 1.0) * q).round() as usize).copied().unwrap_or(f64::NAN);
        format!("{:.2} / {:.2} ms", at(0.5), at(0.95))
    };
    println!(
        "{frames} frames ({bad} bad skipped), {:.1} fps; median / p95: luma {}, detect {}, age {}",
        frames as f64 / start.elapsed().as_secs_f64(),
        pct(&mut luma_ms),
        pct(&mut detect_ms),
        pct(&mut age_ms)
    );
    Ok(())
}
