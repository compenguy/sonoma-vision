//! V4L2 camera capture through memory-mapped driver buffers. A [`Frame`] borrows the driver's buffer
//! directly, and luma formats are handed to the detector without a copy; the buffer goes back to the
//! driver when the next frame is requested.

use std::fs::{File, OpenOptions};
use std::os::fd::AsRawFd;
use std::os::unix::fs::OpenOptionsExt;
use std::path::PathBuf;
use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use log::{info, warn};
use v4l2r::bindings::{self, v4l2_fract, v4l2_streamparm};
use v4l2r::ioctl::{
    self, BufferFlags, Capabilities, Capability, DqBufIoctlError, FormatIterator, IoctlConvertError, MemoryConsistency,
    PlaneMapping, QBufPlane, QBuffer, QueryBuffer, V4l2Buffer,
};
use v4l2r::memory::{MemoryType, MmapHandle};
use v4l2r::{Format, PixelFormat, QueueType};

use crate::detector::ImageView;

/// How a frame's bytes are laid out, and where its luma lives.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Layout {
    /// 8-bit greyscale.
    Grey,
    /// Packed 4:2:2, Y at even bytes.
    Yuyv,
    /// Packed 4:2:2, Y at odd bytes.
    Uyvy,
    /// Planar 4:2:0: a full Y plane first.
    Nv12,
    /// Motion JPEG: decoded to greyscale (a copy).
    Mjpeg,
}

impl Layout {
    /// Every layout, in order of preference: the detector reads GREY and NV12 luma as is, copies YUYV
    /// and UYVY luma out of its 2-byte pitch, and decodes MJPEG.
    pub const ALL: [Layout; 5] = [Layout::Grey, Layout::Nv12, Layout::Yuyv, Layout::Uyvy, Layout::Mjpeg];

    pub fn fourcc(self) -> &'static [u8; 4] {
        match self {
            Layout::Grey => b"GREY",
            Layout::Yuyv => b"YUYV",
            Layout::Uyvy => b"UYVY",
            Layout::Nv12 => b"NV12",
            Layout::Mjpeg => b"MJPG",
        }
    }

    fn from_fourcc(f: PixelFormat) -> Option<Self> {
        Self::ALL.into_iter().find(|l| PixelFormat::from(l.fourcc()) == f)
    }

    /// Whether the detector reads frames of this layout in place.
    pub fn zero_copy(self) -> bool {
        self != Layout::Mjpeg
    }
}

impl std::str::FromStr for Layout {
    type Err = anyhow::Error;
    fn from_str(s: &str) -> Result<Self> {
        let up = s.to_ascii_uppercase();
        Self::ALL
            .into_iter()
            .find(|l| l.fourcc() == up.as_bytes() || (up == "MJPEG" && *l == Layout::Mjpeg))
            .ok_or_else(|| anyhow!("unknown pixel format {s:?}; use grey, yuyv, uyvy, nv12 or mjpeg"))
    }
}

#[derive(Clone, Debug)]
pub struct CameraConfig {
    pub device: PathBuf,
    pub width: u32,
    pub height: u32,
    pub fps: Option<u32>,
    /// None: the first of GREY, NV12, YUYV, UYVY, MJPG that the camera offers at this size and frame
    /// rate (or, if it offers none at them, the first it offers at all).
    pub layout: Option<Layout>,
    /// Driver buffers; fewer means less queued latency, but at least 2 keep the camera streaming.
    pub buffers: u32,
}

impl CameraConfig {
    /// This configuration with `mode`'s size, rate and format.
    pub fn with_mode(&self, mode: &Mode) -> Self {
        Self { width: mode.width, height: mode.height, fps: Some(mode.fps), layout: Some(mode.layout), ..self.clone() }
    }
}

/// A capture mode a camera lists: size, frame rate and pixel format. Written as `1280x720 MJPG 120fps`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mode {
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    pub layout: Layout,
}

impl std::fmt::Display for Mode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let fourcc = std::str::from_utf8(self.layout.fourcc()).unwrap_or("?");
        write!(f, "{}x{} {fourcc} {}fps", self.width, self.height, self.fps)
    }
}

impl std::str::FromStr for Mode {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String> {
        let bad = || format!("capture mode must look like `1280x720 MJPG 120fps`, not {s:?}");
        let mut parts = s.split_whitespace();
        let (Some(size), Some(format), Some(fps), None) = (parts.next(), parts.next(), parts.next(), parts.next())
        else {
            return Err(bad());
        };
        let (w, h) = size.split_once('x').ok_or_else(bad)?;
        let num = |v: &str| v.parse::<u32>().ok().filter(|&n| n > 0).ok_or_else(bad);
        Ok(Self {
            width: num(w)?,
            height: num(h)?,
            fps: num(fps.strip_suffix("fps").unwrap_or(fps))?,
            layout: format.parse().map_err(|_| bad())?,
        })
    }
}

/// What the vision thread reports about its camera, for the capture mode chooser.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CameraState {
    /// The modes the camera lists that the detector can read, most preferred format first.
    pub modes: Vec<Mode>,
    /// The mode streaming now; None while the camera is closed.
    pub active: Option<Mode>,
    /// The mode the configuration (files and command line) opened.
    pub default: Option<Mode>,
}

/// One captured frame, borrowing the driver's buffer until the next frame is requested.
pub struct Frame<'a> {
    pub data: &'a [u8],
    pub width: usize,
    pub height: usize,
    pub stride: usize,
    pub layout: Layout,
    /// The driver's frame counter; gaps mean frames were dropped.
    pub sequence: u32,
    /// Capture time on CLOCK_MONOTONIC (start of exposure or end of transfer, as the driver reports).
    pub timestamp: Duration,
}

impl<'a> Frame<'a> {
    /// The frame's luma as a detector image: in place for raw formats; decoded into `scratch` for MJPEG.
    pub fn luma<'s>(&self, scratch: &'s mut Vec<u8>) -> Result<ImageView<'s>>
    where
        'a: 's,
    {
        let (w, h, stride) = (self.width, self.height, self.stride);
        Ok(match self.layout {
            Layout::Grey | Layout::Nv12 => ImageView::new(self.data, w, h, stride, 1)?,
            Layout::Yuyv => ImageView::new(self.data, w, h, stride, 2)?,
            Layout::Uyvy => ImageView::new(&self.data[1..], w, h, stride, 2)?,
            Layout::Mjpeg => {
                decode_mjpeg(self.data, w, h, scratch)?;
                ImageView::gray(scratch, w, h)?
            }
        })
    }
}

#[cfg(feature = "mjpeg")]
fn decode_mjpeg(data: &[u8], width: usize, height: usize, out: &mut Vec<u8>) -> Result<()> {
    use zune_jpeg::JpegDecoder;
    use zune_jpeg::zune_core::colorspace::ColorSpace;
    use zune_jpeg::zune_core::options::DecoderOptions;
    let options = DecoderOptions::default().jpeg_set_out_colorspace(ColorSpace::Luma);
    let mut dec = JpegDecoder::new_with_options(data, options);
    dec.decode_headers().context("MJPEG headers")?;
    let (dw, dh) = dec.dimensions().context("MJPEG dimensions")?;
    if (dw, dh) != (width, height) {
        bail!("MJPEG frame is {dw}x{dh}, expected {width}x{height}");
    }
    out.resize(dec.output_buffer_size().context("MJPEG size")?, 0);
    dec.decode_into(out).context("MJPEG decode")?;
    Ok(())
}

#[cfg(not(feature = "mjpeg"))]
fn decode_mjpeg(_: &[u8], _: usize, _: usize, _: &mut Vec<u8>) -> Result<()> {
    bail!("built without the mjpeg feature")
}

pub struct Camera {
    fd: File,
    mappings: Vec<PlaneMapping>,
    /// The buffer lent out as the last frame, returned to the driver on the next request.
    lent: Option<u32>,
    pub width: usize,
    pub height: usize,
    pub stride: usize,
    pub layout: Layout,
    pub fps: Option<f64>,
    /// Whether the first frame's timestamp semantics have been logged.
    reported_timestamps: bool,
}

/// What a buffer's timestamp means, from its flags: the clock, and the moment of the frame it marks.
pub fn describe_timestamp(flags: u32) -> (&'static str, &'static str) {
    let clock = match flags & bindings::V4L2_BUF_FLAG_TIMESTAMP_MASK {
        bindings::V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC => "CLOCK_MONOTONIC",
        bindings::V4L2_BUF_FLAG_TIMESTAMP_COPY => "copied from the application (not a capture time)",
        _ => "unknown",
    };
    // End of frame is 0 within the mask, so compare the masked bits rather than test for a flag.
    let source = match flags & bindings::V4L2_BUF_FLAG_TSTAMP_SRC_MASK {
        bindings::V4L2_BUF_FLAG_TSTAMP_SRC_SOE => "start of exposure",
        bindings::V4L2_BUF_FLAG_TSTAMP_SRC_EOF => "end of frame",
        _ => "unknown",
    };
    (clock, source)
}

const QUEUE: QueueType = QueueType::VideoCapture;

impl Camera {
    pub fn open(cfg: &CameraConfig) -> Result<Self> {
        let fd = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(&cfg.device)
            .with_context(|| format!("opening {}", cfg.device.display()))?;
        let caps: Capability = ioctl::querycap(&fd).context("VIDIOC_QUERYCAP")?;
        let device_caps = caps.device_caps();
        if !device_caps.contains(Capabilities::VIDEO_CAPTURE) || !device_caps.contains(Capabilities::STREAMING) {
            bail!("{} ({}) is not a streaming single-planar capture device", cfg.device.display(), caps.card);
        }
        info!("camera {}: {} ({}, {})", cfg.device.display(), caps.card, caps.driver, caps.bus_info);

        let offered: Vec<Layout> =
            FormatIterator::new(&fd, QUEUE).filter_map(|f| Layout::from_fourcc(f.pixelformat)).collect();
        let layout = match cfg.layout {
            Some(l) if offered.contains(&l) => l,
            Some(l) => bail!("camera doesn't offer {:?}; it offers {offered:?}", l),
            None => choose_layout(&fd, &offered, cfg)?,
        };
        let want = Format {
            width: cfg.width,
            height: cfg.height,
            pixelformat: PixelFormat::from(layout.fourcc()),
            plane_fmt: Vec::new(),
        };
        let mut fd = fd;
        let got: Format = ioctl::s_fmt(&mut fd, (QUEUE, &want)).context("VIDIOC_S_FMT")?;
        if got.pixelformat != want.pixelformat {
            bail!("camera switched the format to {}", got.pixelformat);
        }
        if (got.width, got.height) != (cfg.width, cfg.height) {
            warn!("asked for {}x{}, camera chose {}x{}", cfg.width, cfg.height, got.width, got.height);
        }
        let stride = got.plane_fmt.first().map_or(0, |p| p.bytesperline as usize);
        let stride = match (stride, layout) {
            (0, Layout::Yuyv | Layout::Uyvy) => 2 * got.width as usize,
            (0, _) => got.width as usize,
            (s, _) => s,
        };

        let fps = match cfg.fps {
            Some(fps) => set_fps(&fd, fps),
            None => None,
        };

        let count: usize = ioctl::reqbufs(&fd, QUEUE, MemoryType::Mmap, cfg.buffers.max(2), MemoryConsistency::empty())
            .context("VIDIOC_REQBUFS")?;
        let mut mappings = Vec::with_capacity(count);
        for i in 0..count {
            let qb: QueryBuffer = ioctl::querybuf(&fd, QUEUE, i).context("VIDIOC_QUERYBUF")?;
            let plane = qb.planes.first().ok_or_else(|| anyhow!("buffer {i} has no planes"))?;
            mappings.push(ioctl::mmap(&fd, plane.mem_offset, plane.length).context("mmap")?);
        }
        let cam = Self {
            fd,
            mappings,
            lent: None,
            width: got.width as usize,
            height: got.height as usize,
            stride,
            layout,
            fps,
            reported_timestamps: false,
        };
        for i in 0..count {
            cam.queue(i as u32)?;
        }
        ioctl::streamon(&cam.fd, QUEUE).context("VIDIOC_STREAMON")?;
        info!(
            "streaming {}x{} {:?} (stride {}), {} buffers, {}",
            cam.width,
            cam.height,
            layout,
            stride,
            count,
            fps.map_or("default frame rate".into(), |f| format!("{f:.1} fps"))
        );
        Ok(cam)
    }

    /// Manual exposure in driver units (100 µs for UVC cameras), or None for the camera's auto exposure.
    pub fn set_exposure(&self, exposure: Option<i32>) -> Result<()> {
        let mode = |m: u32| ioctl::s_ctrl(&self.fd, bindings::V4L2_CID_EXPOSURE_AUTO, m as i32);
        match exposure {
            Some(exposure) => {
                mode(bindings::v4l2_exposure_auto_type_V4L2_EXPOSURE_MANUAL)
                    .map_err(|e| anyhow!("switching to manual exposure: {e}"))?;
                let got = ioctl::s_ctrl(&self.fd, bindings::V4L2_CID_EXPOSURE_ABSOLUTE, exposure)
                    .map_err(|e| anyhow!("setting exposure {exposure}: {e}"))?;
                info!("exposure set to {got}");
            }
            None => {
                // UVC cameras offer aperture priority (auto exposure with a fixed iris); others full auto.
                mode(bindings::v4l2_exposure_auto_type_V4L2_EXPOSURE_APERTURE_PRIORITY)
                    .or_else(|_| mode(bindings::v4l2_exposure_auto_type_V4L2_EXPOSURE_AUTO))
                    .map_err(|e| anyhow!("switching to auto exposure: {e}"))?;
                info!("auto exposure");
            }
        }
        Ok(())
    }

    /// Manual focus in driver units (turning autofocus off), or None for autofocus.
    pub fn set_focus(&self, focus: Option<i32>) -> Result<()> {
        let auto = |on: bool| ioctl::s_ctrl(&self.fd, bindings::V4L2_CID_FOCUS_AUTO, i32::from(on));
        match focus {
            Some(focus) => {
                auto(false).map_err(|e| anyhow!("turning autofocus off: {e}"))?;
                let got = ioctl::s_ctrl(&self.fd, bindings::V4L2_CID_FOCUS_ABSOLUTE, focus)
                    .map_err(|e| anyhow!("setting focus {focus}: {e}"))?;
                info!("focus set to {got}");
            }
            None => {
                auto(true).map_err(|e| anyhow!("turning autofocus on: {e}"))?;
                info!("autofocus");
            }
        }
        Ok(())
    }

    /// The capture modes the camera lists in formats the detector reads: discrete sizes, each at
    /// its listed frame rates (a range's fastest), most preferred format first, then largest and
    /// fastest first.
    pub fn modes(&self) -> Vec<Mode> {
        let mut modes = Vec::new();
        let offered: Vec<Layout> =
            FormatIterator::new(&self.fd, QUEUE).filter_map(|f| Layout::from_fourcc(f.pixelformat)).collect();
        for layout in Layout::ALL.into_iter().filter(|l| offered.contains(l)) {
            let pf = PixelFormat::from(layout.fourcc());
            let sizes =
                (0..).map_while(|i| ioctl::enum_frame_sizes::<bindings::v4l2_frmsizeenum>(&self.fd, i, pf).ok());
            for size in sizes.filter_map(|e| SizeRange::from_enum(&e)) {
                let SizeRange::Discrete(width, height) = size else { continue };
                let rates = (0..).map_while(|i| {
                    ioctl::enum_frame_intervals::<bindings::v4l2_frmivalenum>(&self.fd, i, pf, width, height).ok()
                });
                for rate in rates.filter_map(|e| RateRange::from_enum(&e)) {
                    let fps = rate.max_fps().round() as u32;
                    let mode = Mode { width, height, fps, layout };
                    if fps > 0 && !modes.contains(&mode) {
                        modes.push(mode);
                    }
                }
            }
        }
        let rank = |m: &Mode| Layout::ALL.iter().position(|l| *l == m.layout);
        modes.sort_by(|a, b| {
            rank(a).cmp(&rank(b)).then((b.width * b.height).cmp(&(a.width * a.height))).then(b.fps.cmp(&a.fps))
        });
        modes
    }

    /// The mode streaming now (the frame rate rounded).
    pub fn mode(&self) -> Mode {
        Mode {
            width: self.width as u32,
            height: self.height as u32,
            fps: self.fps.map_or(0, |f| f.round() as u32),
            layout: self.layout,
        }
    }

    /// Analog gain in driver units.
    pub fn set_gain(&self, gain: i32) -> Result<()> {
        let got =
            ioctl::s_ctrl(&self.fd, bindings::V4L2_CID_GAIN, gain).map_err(|e| anyhow!("setting gain {gain}: {e}"))?;
        info!("gain set to {got}");
        Ok(())
    }

    fn queue(&self, index: u32) -> Result<()> {
        let mut qb = QBuffer::<MmapHandle>::new(QUEUE, index);
        qb.planes = vec![QBufPlane::new(0)];
        ioctl::qbuf::<_, ()>(&self.fd, qb).context("VIDIOC_QBUF")?;
        Ok(())
    }

    /// Dequeue one ready buffer, if any.
    fn try_dequeue(&self) -> Result<Option<V4l2Buffer>> {
        match ioctl::dqbuf::<V4l2Buffer>(&self.fd, QUEUE) {
            Ok(b) => Ok(Some(b)),
            Err(IoctlConvertError::IoctlError(DqBufIoctlError::NotReady)) => Ok(None),
            Err(e) => Err(anyhow!("VIDIOC_DQBUF: {e}")),
        }
    }

    /// The newest frame, waiting up to `timeout` for one. Older ready frames are returned to the driver
    /// unread, so a slow consumer always gets the latest image rather than a backlog. Returns None on
    /// timeout.
    pub fn next_frame(&mut self, timeout: Duration) -> Result<Option<Frame<'_>>> {
        if let Some(i) = self.lent.take() {
            self.queue(i)?;
        }
        let mut newest: Option<V4l2Buffer> = None;
        loop {
            match self.try_dequeue()? {
                Some(b) => {
                    if let Some(old) = newest.replace(b) {
                        self.queue(old.index())?;
                    }
                }
                None if newest.is_some() => break,
                None => {
                    let mut pfd = libc::pollfd { fd: self.fd.as_raw_fd(), events: libc::POLLIN, revents: 0 };
                    let ms = timeout.as_millis().min(i32::MAX as u128) as i32;
                    // SAFETY: one valid pollfd for the duration of the call.
                    match unsafe { libc::poll(&mut pfd, 1, ms) } {
                        0 => return Ok(None),
                        n if n < 0 => {
                            let err = std::io::Error::last_os_error();
                            if err.kind() != std::io::ErrorKind::Interrupted {
                                return Err(err).context("poll");
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
        let buf = newest.expect("loop exits with a buffer");
        let index = buf.index();
        if buf.flags().contains(BufferFlags::ERROR) {
            // A corrupt frame: give it back and report nothing this time.
            self.queue(index)?;
            return Ok(None);
        }
        self.lent = Some(index);
        let used = *buf.get_first_plane().bytesused as usize;
        let mapping = &self.mappings[index as usize];
        let data = &mapping.as_ref()[..used.min(mapping.len())];
        if !self.reported_timestamps {
            self.reported_timestamps = true;
            let flags = buf.flags().bits();
            let (clock, source) = describe_timestamp(flags);
            info!("frame timestamps: {clock}, {source} (buffer flags {flags:#x})");
            if flags & bindings::V4L2_BUF_FLAG_TIMESTAMP_MASK != bindings::V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC {
                warn!("frame timestamps are not on CLOCK_MONOTONIC: published capture times will be wrong");
            }
        }
        let tv = buf.timestamp();
        let timestamp = Duration::new(tv.tv_sec as u64, tv.tv_usec as u32 * 1000);
        if self.layout.zero_copy() && data.len() < self.stride * (self.height - 1) + self.width {
            warn!("short frame ({} bytes), skipped", data.len());
            return Ok(None);
        }
        Ok(Some(Frame {
            data,
            width: self.width,
            height: self.height,
            stride: self.stride,
            layout: self.layout,
            sequence: buf.sequence(),
            timestamp,
        }))
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        let _ = ioctl::streamoff(&self.fd, QUEUE);
    }
}

/// The most preferred offered layout that the camera lists at the configured size and frame rate. A
/// driver that doesn't list its modes is taken at its word; if none fits, the most preferred offered
/// layout, whose size or rate the driver will adjust.
fn choose_layout(fd: &File, offered: &[Layout], cfg: &CameraConfig) -> Result<Layout> {
    let chosen = pick_layout(offered, |l| mode_fits(fd, l, cfg.width, cfg.height, cfg.fps))
        .ok_or_else(|| anyhow!("camera offers none of {:?}", Layout::ALL))?;
    if !chosen.1 {
        warn!("no format offers {}x{} at the requested rate; using {:?}", cfg.width, cfg.height, chosen.0);
    }
    Ok(chosen.0)
}

/// [`choose_layout`]'s choice, given whether each layout fits: the layout, and whether it fits.
fn pick_layout(offered: &[Layout], fits: impl Fn(Layout) -> Result<(), String>) -> Option<(Layout, bool)> {
    let mut candidates = Layout::ALL.into_iter().filter(|l| offered.contains(l)).peekable();
    let first = *candidates.peek()?;
    for l in candidates {
        match fits(l) {
            Ok(()) => return Some((l, true)),
            Err(why) => info!("not using {l:?}: {why}"),
        }
    }
    Some((first, false))
}

/// Whether the camera lists `layout` at `width`x`height` (and `fps`, if given); Err says why not.
fn mode_fits(fd: &File, layout: Layout, width: u32, height: u32, fps: Option<u32>) -> Result<(), String> {
    let pf = PixelFormat::from(layout.fourcc());
    let sizes: Vec<SizeRange> = (0..)
        .map_while(|i| ioctl::enum_frame_sizes::<bindings::v4l2_frmsizeenum>(fd, i, pf).ok())
        .filter_map(|e| SizeRange::from_enum(&e))
        .collect();
    if sizes.is_empty() {
        return Ok(()); // not listed: unknown
    }
    if !sizes.iter().any(|r| r.contains(width, height)) {
        return Err(format!("not offered at {width}x{height}"));
    }
    let Some(fps) = fps else { return Ok(()) };
    let rates: Vec<RateRange> = (0..)
        .map_while(|i| ioctl::enum_frame_intervals::<bindings::v4l2_frmivalenum>(fd, i, pf, width, height).ok())
        .filter_map(|e| RateRange::from_enum(&e))
        .collect();
    if rates.is_empty() || rates.iter().any(|r| r.reaches(fps)) {
        Ok(())
    } else {
        let best = rates.iter().map(RateRange::max_fps).fold(0.0, f64::max);
        Err(format!("at most {best:.1} fps at {width}x{height}, not {fps}"))
    }
}

/// Frame sizes a format is listed at.
#[derive(Clone, Copy, Debug, PartialEq)]
enum SizeRange {
    Discrete(u32, u32),
    /// min, max and step of width, then of height.
    Steps([u32; 3], [u32; 3]),
}

impl SizeRange {
    fn from_enum(e: &bindings::v4l2_frmsizeenum) -> Option<Self> {
        use v4l2r::ioctl::FrmSizeTypes;
        Some(match e.size()? {
            FrmSizeTypes::Discrete(d) => Self::Discrete(d.width, d.height),
            FrmSizeTypes::StepWise(s) => {
                Self::Steps([s.min_width, s.max_width, s.step_width], [s.min_height, s.max_height, s.step_height])
            }
        })
    }

    fn contains(&self, width: u32, height: u32) -> bool {
        let within = |v: u32, [lo, hi, step]: [u32; 3]| (lo..=hi).contains(&v) && (v - lo).is_multiple_of(step.max(1));
        match *self {
            Self::Discrete(w, h) => (w, h) == (width, height),
            Self::Steps(ws, hs) => within(width, ws) && within(height, hs),
        }
    }
}

/// Frame intervals a format is listed at, for one size: the shortest interval gives the highest rate.
#[derive(Clone, Copy, Debug, PartialEq)]
enum RateRange {
    /// An interval of numerator/denominator seconds.
    Discrete(u32, u32),
    /// The shortest interval of a range, numerator/denominator seconds.
    Shortest(u32, u32),
}

impl RateRange {
    fn from_enum(e: &bindings::v4l2_frmivalenum) -> Option<Self> {
        use v4l2r::ioctl::FrmIvalTypes;
        Some(match e.intervals()? {
            FrmIvalTypes::Discrete(f) => Self::Discrete(f.numerator, f.denominator),
            FrmIvalTypes::StepWise(s) => Self::Shortest(s.min.numerator, s.min.denominator),
        })
    }

    fn max_fps(&self) -> f64 {
        let (Self::Discrete(n, d) | Self::Shortest(n, d)) = *self;
        if n == 0 { 0.0 } else { f64::from(d) / f64::from(n) }
    }

    /// Whether this offers `fps` or faster (within 1%: 30000/1001 counts as 30).
    fn reaches(&self, fps: u32) -> bool {
        self.max_fps() >= 0.99 * f64::from(fps)
    }
}

fn set_fps(fd: &File, fps: u32) -> Option<f64> {
    let mut parm = v4l2_streamparm { type_: QUEUE as u32, ..Default::default() };
    // Writing a Copy union field is safe; `capture` is the active member for a capture queue.
    parm.parm.capture.timeperframe = v4l2_fract { numerator: 1, denominator: fps };
    match ioctl::s_parm::<_, v4l2_streamparm>(fd, parm) {
        Ok(p) => {
            // SAFETY: as above; the driver filled in the frame interval it chose.
            let t = unsafe { p.parm.capture.timeperframe };
            (t.numerator > 0).then(|| f64::from(t.denominator) / f64::from(t.numerator))
        }
        Err(e) => {
            warn!("couldn't set {fps} fps: {e}");
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prefers_formats_the_detector_reads_in_place() {
        let all_fit = |_| Ok(());
        assert_eq!(pick_layout(&[Layout::Mjpeg, Layout::Yuyv, Layout::Nv12], all_fit), Some((Layout::Nv12, true)));
        assert_eq!(pick_layout(&[Layout::Mjpeg, Layout::Yuyv], all_fit), Some((Layout::Yuyv, true)));
        assert_eq!(pick_layout(&[], all_fit), None);
    }

    #[test]
    fn skips_formats_without_the_mode() {
        // A typical UVC camera: NV12 and YUYV only at low rates for this size, MJPEG at full rate.
        let fits = |l| if l == Layout::Mjpeg { Ok(()) } else { Err("too slow".to_string()) };
        let offered = [Layout::Yuyv, Layout::Nv12, Layout::Mjpeg];
        assert_eq!(pick_layout(&offered, fits), Some((Layout::Mjpeg, true)));
        // Nothing fits: the most preferred, for the driver to adjust.
        assert_eq!(pick_layout(&offered, |_| Err(String::new())), Some((Layout::Nv12, false)));
    }

    #[test]
    fn modes_read_back() {
        let m = Mode { width: 1280, height: 720, fps: 120, layout: Layout::Mjpeg };
        assert_eq!(m.to_string(), "1280x720 MJPG 120fps");
        assert_eq!("1280x720 MJPG 120fps".parse::<Mode>(), Ok(m));
        assert_eq!("640x480 yuyv 30".parse::<Mode>().map(|m| (m.layout, m.fps)), Ok((Layout::Yuyv, 30)));
        for bad in ["1280x720 MJPG", "1280 MJPG 30fps", "0x720 MJPG 30fps", "1280x720 H264 30fps", ""] {
            assert!(bad.parse::<Mode>().is_err(), "{bad}");
        }
    }

    #[test]
    fn size_ranges() {
        assert!(SizeRange::Discrete(1280, 720).contains(1280, 720));
        assert!(!SizeRange::Discrete(1280, 720).contains(1920, 1080));
        let steps = SizeRange::Steps([16, 1920, 16], [16, 1080, 8]);
        assert!(steps.contains(1280, 720));
        assert!(!steps.contains(1288, 720), "off the width step");
        assert!(!steps.contains(1280, 1088), "over the height range");
        // Continuous ranges report a step of 1 (or 0 from a sloppy driver).
        assert!(SizeRange::Steps([1, 4000, 0], [1, 3000, 0]).contains(1281, 721));
    }

    #[test]
    fn rate_ranges() {
        assert!(RateRange::Discrete(1, 60).reaches(60));
        assert!(RateRange::Discrete(1, 60).reaches(30));
        assert!(!RateRange::Discrete(1, 30).reaches(60));
        assert!(RateRange::Discrete(1001, 30000).reaches(30), "NTSC 29.97 counts as 30");
        assert!(RateRange::Shortest(1, 90).reaches(60));
        assert!(!RateRange::Discrete(0, 0).reaches(1));
        assert_eq!(RateRange::Discrete(1, 5).max_fps(), 5.0);
    }
}
