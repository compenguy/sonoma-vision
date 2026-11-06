//! The clock V4L2 timestamps frames with.

use std::time::Duration;

/// Now on CLOCK_MONOTONIC, the clock of V4L2 buffer timestamps.
pub fn monotonic_now() -> Duration {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: a valid timespec for the duration of the call; CLOCK_MONOTONIC always exists on Linux.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    Duration::new(ts.tv_sec as u64, ts.tv_nsec as u32)
}
