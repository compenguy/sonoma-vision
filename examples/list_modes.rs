//! List a camera's pixel formats, frame sizes and frame rates, as the format choice sees them.
//!
//!     cargo run --release --example list_modes -- [DEVICE]

use std::path::PathBuf;

use clap::Parser;
use sonoma_vision::capture::{RateRange, SizeRange, frame_rates, frame_sizes};
use v4l2r::QueueType;
use v4l2r::ioctl::FormatIterator;

/// List a camera's pixel formats, frame sizes and frame rates.
#[derive(Parser)]
struct Args {
    /// V4L2 capture device.
    #[arg(default_value = "/dev/video0")]
    device: PathBuf,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    let fd = std::fs::OpenOptions::new().read(true).write(true).open(&args.device)?;
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
