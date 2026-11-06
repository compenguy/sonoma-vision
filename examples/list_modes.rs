//! List a camera's pixel formats, frame sizes and frame rates, as the format choice sees them.
//!
//!     cargo run --release --example list_modes -- /dev/video0
use v4l2r::QueueType;
use v4l2r::bindings::{v4l2_frmivalenum, v4l2_frmsizeenum};
use v4l2r::ioctl::{self, FormatIterator, FrmIvalTypes, FrmSizeTypes};

fn main() {
    let dev = std::env::args().nth(1).unwrap();
    let fd = std::fs::OpenOptions::new().read(true).write(true).open(dev).unwrap();
    for f in FormatIterator::new(&fd, QueueType::VideoCapture) {
        println!("{} ({})", f.pixelformat, f.description);
        for i in 0.. {
            let Ok(s) = ioctl::enum_frame_sizes::<v4l2_frmsizeenum>(&fd, i, f.pixelformat) else { break };
            let (w, h) = match s.size() {
                Some(FrmSizeTypes::Discrete(d)) => (d.width, d.height),
                Some(FrmSizeTypes::StepWise(s)) => {
                    println!("  stepwise {s:?}");
                    continue;
                }
                None => continue,
            };
            let mut rates = Vec::new();
            for j in 0.. {
                let Ok(r) = ioctl::enum_frame_intervals::<v4l2_frmivalenum>(&fd, j, f.pixelformat, w, h) else { break };
                match r.intervals() {
                    Some(FrmIvalTypes::Discrete(t)) => rates.push(
                        format!("{:.4}", t.denominator as f64 / t.numerator as f64)
                            .trim_end_matches('0')
                            .trim_end_matches('.')
                            .to_string(),
                    ),
                    Some(FrmIvalTypes::StepWise(s)) => rates.push(format!("{s:?}")),
                    None => {}
                }
            }
            println!("  {w}x{h}: {}", rates.join(", "));
        }
    }
}
