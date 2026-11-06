//! Capture from a camera and detect tags, printing per-frame timing.
//!
//!     cargo run --release --example camera -- /dev/video0 [width height [format]]
//!
//! `CAMERA_FPS` requests a frame rate (default 30); `CAMERA_QUIET=1` prints only the summary: median
//! and 95th-percentile luma, detect and age times.

use std::time::{Duration, Instant};

use sonoma_vision::capture::{Camera, CameraConfig};
use sonoma_vision::clock::monotonic_now;
use sonoma_vision::detector::{Config, Detector, families};

fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cfg = CameraConfig {
        device: args.first().map_or("/dev/video0".into(), Into::into),
        width: args.get(1).map_or(Ok(640), |a| a.parse())?,
        height: args.get(2).map_or(Ok(480), |a| a.parse())?,
        fps: Some(std::env::var("CAMERA_FPS").map_or(Ok(30), |f| f.parse())?),
        layout: args.get(3).map(|a| a.parse()).transpose()?,
        buffers: 3,
    };
    let mut cam = Camera::open(&cfg)?;
    let mut det = Detector::new(&families::TAG36H11, &Config::default())?;
    let (mut scratch, mut out) = (Vec::new(), Vec::new());
    let start = Instant::now();
    let (mut frames, mut bad) = (0, 0);
    let quiet = std::env::var_os("CAMERA_QUIET").is_some();
    let (mut luma_ms, mut detect_ms, mut age_ms) = (Vec::new(), Vec::new(), Vec::new());
    while start.elapsed() < Duration::from_secs(5) {
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
