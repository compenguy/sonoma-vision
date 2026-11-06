//! Capture from a camera and detect tags, printing per-frame timing.
//!
//!     cargo run --release --example camera -- /dev/video0 [width height [format]]

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
        fps: Some(30),
        layout: args.get(3).map(|a| a.parse()).transpose()?,
        buffers: 3,
    };
    let mut cam = Camera::open(&cfg)?;
    let mut det = Detector::new(&families::TAG36H11, &Config::default())?;
    let (mut scratch, mut out) = (Vec::new(), Vec::new());
    let start = Instant::now();
    let mut frames = 0;
    while start.elapsed() < Duration::from_secs(5) {
        let Some(frame) = cam.next_frame(Duration::from_secs(1))? else { continue };
        let t0 = Instant::now();
        let img = frame.luma(&mut scratch)?;
        let t1 = Instant::now();
        det.detect_into(&img, &mut out);
        let age = monotonic_now().saturating_sub(frame.timestamp);
        frames += 1;
        println!(
            "seq {:6} luma {:5.2} ms detect {:5.2} ms age {:6.2} ms tags {:?}",
            frame.sequence,
            (t1 - t0).as_secs_f64() * 1e3,
            t1.elapsed().as_secs_f64() * 1e3,
            age.as_secs_f64() * 1e3,
            out.iter().map(|d| d.id).collect::<Vec<_>>()
        );
    }
    println!("{:.1} fps", frames as f64 / start.elapsed().as_secs_f64());
    Ok(())
}
