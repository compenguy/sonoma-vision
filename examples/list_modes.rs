//! List a camera's pixel formats, frame sizes and frame rates, as the format choice sees them.
//!
//!     cargo run --release --example list_modes -- /dev/video0

use sonoma_vision::capture::{RateRange, SizeRange, frame_rates, frame_sizes};
use v4l2r::QueueType;
use v4l2r::ioctl::FormatIterator;

fn main() -> anyhow::Result<()> {
    let dev = std::env::args().nth(1).unwrap_or_else(|| "/dev/video0".into());
    let fd = std::fs::OpenOptions::new().read(true).write(true).open(&dev)?;
    for f in FormatIterator::new(&fd, QueueType::VideoCapture) {
        println!("{} ({})", f.pixelformat, f.description);
        for size in frame_sizes(&fd, f.pixelformat) {
            match size {
                SizeRange::Discrete(w, h) => {
                    let rates: Vec<String> = frame_rates(&fd, f.pixelformat, w, h).iter().map(rate).collect();
                    println!("  {w}x{h}: {}", rates.join(", "));
                }
                SizeRange::Steps([w0, w1, ws], [h0, h1, hs]) => {
                    println!("  {w0}-{w1} (step {ws}) x {h0}-{h1} (step {hs})");
                }
            }
        }
    }
    Ok(())
}

fn rate(r: &RateRange) -> String {
    let fps = format!("{:.2}", r.max_fps()).trim_end_matches('0').trim_end_matches('.').to_string();
    match r {
        RateRange::Discrete(..) => fps,
        RateRange::Shortest(..) => format!("up to {fps}"),
    }
}
