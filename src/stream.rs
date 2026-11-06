//! An optional MJPEG camera stream for dashboards, served the way WPILib's CameraServer serves one:
//! `multipart/x-mixed-replace` JPEG frames over HTTP at `/?action=stream` (or `/?action=snapshot` for
//! one frame), advertised in NT under `/CameraPublisher` (see [`crate::nt`]).
//!
//! The same server serves the configuration in effect, live settings included, at `/config.toml`
//! (`/config.toml?download` to save it as a file): a `sonoma-vision.toml` to install.
//!
//! Streaming never slows detection down: frames are only taken while a client is connected, at most
//! at the stream's frame rate, and the vision thread's only cost is a (downscaled) copy of the luma.
//! Image adjustments, overlays and JPEG encoding happen on a separate thread, which skips frames while
//! it's busy. The adjustments (auto-levels and gamma, from the live [`Settings`]) are for viewing only:
//! detection always reads the camera's own image.

use std::net::{Ipv4Addr, SocketAddr};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{Receiver, SyncSender};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use jpeg_encoder::{ColorType, Encoder};
use log::{debug, info, warn};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::watch;

use crate::detector::{Detection, ImageView};
use crate::settings::Settings;

/// The stream's fixed settings; its rate, size, quality and overlay are live [`Settings`].
#[derive(Clone, Debug)]
pub struct StreamConfig {
    pub port: u16,
}

/// The configuration page: renders the configuration in effect, and names the file it's saved as.
#[derive(Clone)]
pub struct ConfigPage {
    pub render: Arc<dyn Fn() -> String + Send + Sync>,
    /// The file name `?download` suggests, e.g. `cam0.toml`.
    pub filename: String,
}

/// The latest encoded JPEG, shared with every connected client.
type Jpeg = Option<Arc<[u8]>>;

/// Start the encoder thread and bind the HTTP port. The [`Tap`] goes to the vision thread; the
/// [`Server`] runs on the async runtime. `settings` supplies the stream's rate, size and look.
pub fn start(
    cfg: &StreamConfig,
    settings: watch::Receiver<Settings>,
    config: Option<ConfigPage>,
) -> Result<(Tap, Server)> {
    let listener = std::net::TcpListener::bind((Ipv4Addr::UNSPECIFIED, cfg.port))
        .with_context(|| format!("camera stream port {}", cfg.port))?;
    listener.set_nonblocking(true)?;
    let clients = Arc::new(AtomicUsize::new(0));
    let (jpeg_tx, jpeg_rx) = watch::channel(None);
    let (snap_tx, snap_rx) = std::sync::mpsc::sync_channel(1);
    let encoder_settings = settings.clone();
    std::thread::Builder::new()
        .name("stream-encode".into())
        .spawn(move || encode_loop(snap_rx, jpeg_tx, encoder_settings))?;
    let tap = Tap { clients: clients.clone(), tx: snap_tx, settings, due: Instant::now() };
    Ok((tap, Server { listener, frames: jpeg_rx, clients, config }))
}

/// The vision thread's end of the stream.
pub struct Tap {
    clients: Arc<AtomicUsize>,
    tx: SyncSender<Snapshot>,
    settings: watch::Receiver<Settings>,
    /// When the next frame is due.
    due: Instant,
}

impl Tap {
    /// Offer a processed frame. It's copied (downscaled) for the stream only if someone is watching,
    /// the stream's frame rate allows, and the encoder is free.
    pub fn offer(&mut self, img: &ImageView, detections: &[Detection]) {
        if self.clients.load(Ordering::Relaxed) == 0 {
            return;
        }
        let (fps, scale) = {
            let s = self.settings.borrow();
            (s.stream_fps, usize::from(s.stream_scale))
        };
        let interval = Duration::from_secs_f64(1.0 / fps.max(0.1));
        // A quarter interval of slack, so a frame arriving a little early isn't held to the next one.
        let now = Instant::now();
        if now + interval / 4 < self.due {
            return;
        }
        // Also catch up at once if the rate went up, rather than wait out the old interval.
        self.due = (self.due + interval).max(now + interval / 2).min(now + interval);
        // A full channel means the encoder is still busy with the last frame: skip this one.
        let _ = self.tx.try_send(Snapshot::new(img, scale.max(1), detections));
    }
}

/// A frame picked for the stream: downscaled luma, and the detections in its coordinates.
struct Snapshot {
    gray: Vec<u8>,
    width: usize,
    height: usize,
    detections: Vec<Detection>,
}

impl Snapshot {
    fn new(img: &ImageView, scale: usize, detections: &[Detection]) -> Self {
        let (gray, width, height) = downscale(img, scale);
        let s = scale as f64;
        let detections = detections
            .iter()
            .map(|d| Detection { corners: d.corners.map(|[x, y]| [(x + 0.5) / s - 0.5, (y + 0.5) / s - 0.5]), ..*d })
            .collect();
        Self { gray, width, height, detections }
    }
}

/// Average `scale`×`scale` blocks (the frame's right and bottom remainders are dropped).
fn downscale(img: &ImageView, scale: usize) -> (Vec<u8>, usize, usize) {
    let (w, h) = (img.width() / scale, img.height() / scale);
    let n = (scale * scale) as u32;
    let mut out = Vec::with_capacity(w * h);
    let mut sums = vec![0u32; w];
    for y in 0..h {
        sums.fill(0);
        for row in y * scale..(y + 1) * scale {
            for (x, v) in img.row_step(row, 1).take(w * scale).enumerate() {
                sums[x / scale] += u32::from(v);
            }
        }
        out.extend(sums.iter().map(|&t| ((t + n / 2) / n) as u8));
    }
    (out, w, h)
}

/// Encode snapshots until the vision thread drops its [`Tap`].
fn encode_loop(rx: Receiver<Snapshot>, out: watch::Sender<Jpeg>, settings: watch::Receiver<Settings>) {
    let mut buf = Vec::new();
    let mut look = Look::default();
    while let Ok(mut snap) = rx.recv() {
        let (auto_levels, gamma, quality, overlay) = {
            let s = settings.borrow();
            (s.stream_auto_levels, s.stream_gamma, s.stream_quality.clamp(1, 100), s.stream_overlay)
        };
        look.apply(&mut snap.gray, auto_levels, gamma);
        if overlay {
            draw_overlay(&mut snap);
        }
        buf.clear();
        let encoded =
            Encoder::new(&mut buf, quality).encode(&snap.gray, snap.width as u16, snap.height as u16, ColorType::Luma);
        match encoded {
            Ok(()) => {
                out.send_replace(Some(Arc::from(&buf[..])));
            }
            Err(e) => warn!("stream: JPEG encoding failed: {e}"),
        }
    }
}

/// The stream's display adjustments, applied together as one lookup table.
#[derive(Default)]
struct Look {
    /// The auto-levels black and white points, smoothed over frames so the picture doesn't flicker.
    levels: Option<(f64, f64)>,
}

impl Look {
    /// Auto-levels maps the darkest and brightest 1% of pixels to black and white; gamma above 1 then
    /// lifts the shadows.
    fn apply(&mut self, gray: &mut [u8], auto_levels: bool, gamma: f64) {
        if !auto_levels {
            self.levels = None;
            if gamma == 1.0 {
                return;
            }
        }
        let (lo, hi) = if auto_levels {
            let (lo, hi) = percentiles(gray, 0.01);
            let smoothed = match self.levels {
                Some((l, h)) => (0.7 * l + 0.3 * lo, 0.7 * h + 0.3 * hi),
                None => (lo, hi),
            };
            self.levels = Some(smoothed);
            smoothed
        } else {
            (0.0, 255.0)
        };
        // At least 16 grey levels of range, so a featureless frame isn't stretched into pure noise.
        let span = (hi - lo).max(16.0);
        let lut: [u8; 256] = std::array::from_fn(|v| {
            let t = ((v as f64 - lo) / span).clamp(0.0, 1.0);
            (255.0 * t.powf(1.0 / gamma)).round() as u8
        });
        for p in gray {
            *p = lut[usize::from(*p)];
        }
    }
}

/// The grey levels below which, and above which, `frac` of the pixels lie.
fn percentiles(gray: &[u8], frac: f64) -> (f64, f64) {
    let mut hist = [0usize; 256];
    for &p in gray {
        hist[usize::from(p)] += 1;
    }
    let k = ((gray.len() as f64 * frac) as usize).max(1);
    // The first level, in `levels` order, by which `k` pixels have been counted.
    let reach = |levels: &mut dyn Iterator<Item = usize>| {
        let mut seen = 0;
        for v in levels {
            seen += hist[v];
            if seen >= k {
                return v;
            }
        }
        0
    };
    (reach(&mut (0..256)) as f64, reach(&mut (0..256).rev()) as f64)
}

/// Outline each tag (white on black, to show on any background), mark its first corner, and label it
/// with its ID at its centre.
fn draw_overlay(snap: &mut Snapshot) {
    let mut canvas = Canvas { px: &mut snap.gray, width: snap.width, height: snap.height };
    let thick = if snap.width >= 640 { 1 } else { 0 };
    let text_scale = (snap.width / 320).max(1);
    for d in &snap.detections {
        for pass in [(thick + 1, 0u8), (thick, 255u8)] {
            for i in 0..4 {
                canvas.line(d.corners[i], d.corners[(i + 1) % 4], pass.0, pass.1);
            }
            canvas.fill_square(d.corners[0], pass.0 + 2, pass.1);
        }
        canvas.label(d.center(), d.id, text_scale);
    }
}

struct Canvas<'a> {
    px: &'a mut [u8],
    width: usize,
    height: usize,
}

impl Canvas<'_> {
    /// Fill the square of half-side `r` around pixel `(x, y)`, clipped to the image.
    fn fill_square(&mut self, [x, y]: [f64; 2], r: usize, value: u8) {
        if !(x.is_finite() && y.is_finite()) {
            return;
        }
        let (x, y, r) = (x.round() as i64, y.round() as i64, r as i64);
        self.fill_rect(x - r, y - r, x + r + 1, y + r + 1, value);
    }

    /// Fill `[x0, x1) × [y0, y1)`, clipped to the image.
    fn fill_rect(&mut self, x0: i64, y0: i64, x1: i64, y1: i64, value: u8) {
        let (w, h) = (self.width as i64, self.height as i64);
        for y in y0.clamp(0, h)..y1.clamp(0, h) {
            let row = y as usize * self.width;
            self.px[row + x0.clamp(0, w) as usize..row + x1.clamp(0, w) as usize].fill(value);
        }
    }

    /// A line `r` pixels either side of a to b.
    fn line(&mut self, a: [f64; 2], b: [f64; 2], r: usize, value: u8) {
        let steps = (b[0] - a[0]).abs().max((b[1] - a[1]).abs()).ceil();
        // Corners far outside the frame (or NaN) would be a long walk for nothing.
        if !(steps.is_finite() && steps < 8.0 * (self.width + self.height) as f64) {
            return;
        }
        let n = steps.max(1.0) as usize;
        for i in 0..=n {
            let t = i as f64 / n as f64;
            self.fill_square([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t], r, value);
        }
    }

    /// `id` in white digits on a black box centred at `at`, each font pixel `scale` pixels square.
    fn label(&mut self, at: [f64; 2], id: u32, scale: usize) {
        if !(at[0].is_finite() && at[1].is_finite()) {
            return;
        }
        let digits: Vec<usize> = id.to_string().bytes().map(|b| usize::from(b - b'0')).collect();
        let s = scale as i64;
        // Glyphs are 3×5 with a 1-pixel gap, and the box has a 1-pixel margin.
        let (tw, th) = (4 * digits.len() as i64 - 1, 5);
        let x0 = at[0].round() as i64 - (tw + 2) * s / 2;
        let y0 = at[1].round() as i64 - (th + 2) * s / 2;
        self.fill_rect(x0, y0, x0 + (tw + 2) * s, y0 + (th + 2) * s, 0);
        for (k, &digit) in digits.iter().enumerate() {
            let glyph = DIGITS[digit];
            for gy in 0..5 {
                for gx in 0..3 {
                    if glyph >> (14 - (gy * 3 + gx)) & 1 == 1 {
                        let x = x0 + (1 + 4 * k as i64 + gx) * s;
                        let y = y0 + (1 + gy) * s;
                        self.fill_rect(x, y, x + s, y + s, 255);
                    }
                }
            }
        }
    }
}

/// 3×5 digit glyphs, rows top to bottom, most significant bit first.
const DIGITS: [u16; 10] = [
    0b111_101_101_101_111,
    0b010_110_010_010_111,
    0b111_001_111_100_111,
    0b111_001_111_001_111,
    0b101_101_111_001_001,
    0b111_100_111_001_111,
    0b111_100_111_101_111,
    0b111_001_001_001_001,
    0b111_101_111_101_111,
    0b111_101_111_001_111,
];

/// The HTTP side: serves the latest JPEG to each client.
pub struct Server {
    listener: std::net::TcpListener,
    frames: watch::Receiver<Jpeg>,
    clients: Arc<AtomicUsize>,
    config: Option<ConfigPage>,
}

const BOUNDARY: &str = "sonomaframe";

impl Server {
    pub fn local_addr(&self) -> std::io::Result<SocketAddr> {
        self.listener.local_addr()
    }

    /// Accept clients until the runtime shuts down.
    pub async fn run(self) -> Result<()> {
        let listener = TcpListener::from_std(self.listener)?;
        info!("camera stream on http://{}/?action=stream", listener.local_addr()?);
        loop {
            let (sock, peer) = match listener.accept().await {
                Ok(c) => c,
                Err(e) => {
                    // Out of file descriptors and the like: back off rather than spin.
                    warn!("stream: accept: {e}");
                    tokio::time::sleep(Duration::from_millis(100)).await;
                    continue;
                }
            };
            let (frames, clients, config) = (self.frames.clone(), self.clients.clone(), self.config.clone());
            tokio::spawn(async move {
                if let Err(e) = serve_client(sock, frames, clients, config).await {
                    debug!("stream client {peer}: {e}");
                }
            });
        }
    }
}

/// Counts a connected client for as long as it lives.
struct ClientGuard(Arc<AtomicUsize>);

impl ClientGuard {
    fn new(clients: Arc<AtomicUsize>) -> Self {
        clients.fetch_add(1, Ordering::Relaxed);
        Self(clients)
    }
}

impl Drop for ClientGuard {
    fn drop(&mut self) {
        self.0.fetch_sub(1, Ordering::Relaxed);
    }
}

#[derive(Debug, PartialEq)]
enum Route {
    Stream,
    Snapshot,
    /// The configuration in effect; as a file to save with `download`.
    Config {
        download: bool,
    },
    NotFound,
    BadMethod,
}

/// Route an HTTP request line: `GET /`, `/?action=stream`, `/stream.mjpg` stream; `?action=snapshot`
/// sends one frame; `/config.toml` the configuration.
fn route(request_line: &str) -> Route {
    let mut parts = request_line.split_whitespace();
    let (method, target) = (parts.next().unwrap_or(""), parts.next().unwrap_or(""));
    if method != "GET" {
        return Route::BadMethod;
    }
    let (path, query) = target.split_once('?').unwrap_or((target, ""));
    let has =
        |key: &str| query.split('&').any(|kv| kv == key || kv.strip_prefix(key).is_some_and(|r| r.starts_with('=')));
    match path {
        "/config.toml" => Route::Config { download: has("download") },
        "/" | "/stream.mjpg" if query.split('&').any(|kv| kv == "action=snapshot") => Route::Snapshot,
        "/" | "/stream.mjpg" => Route::Stream,
        _ => Route::NotFound,
    }
}

async fn serve_client(
    mut sock: TcpStream,
    mut frames: watch::Receiver<Jpeg>,
    clients: Arc<AtomicUsize>,
    config: Option<ConfigPage>,
) -> Result<()> {
    sock.set_nodelay(true)?;
    let request_line = tokio::time::timeout(Duration::from_secs(5), read_request_line(&mut sock))
        .await
        .context("request timed out")??;
    let route = route(&request_line);
    if let (Route::Config { download }, Some(page)) = (&route, &config) {
        let body = (page.render)();
        // Shown in a browser as text; saved under the camera's name with `?download`.
        let kind = if *download {
            format!("application/toml\r\nContent-Disposition: attachment; filename=\"{}\"", page.filename)
        } else {
            "text/plain; charset=utf-8".to_string()
        };
        let head = format!(
            "HTTP/1.0 200 OK\r\nConnection: close\r\nCache-Control: no-store\r\nContent-Type: {kind}\r\n\
             Content-Length: {}\r\n\r\n",
            body.len()
        );
        sock.write_all(head.as_bytes()).await?;
        sock.write_all(body.as_bytes()).await?;
        return Ok(());
    }
    let status = match route {
        Route::Stream | Route::Snapshot => None,
        Route::Config { .. } | Route::NotFound => Some("404 Not Found"),
        Route::BadMethod => Some("405 Method Not Allowed"),
    };
    if let Some(status) = status {
        let body = format!("{status}\n");
        let resp = format!(
            "HTTP/1.0 {status}\r\nContent-Type: text/plain\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
            body.len()
        );
        sock.write_all(resp.as_bytes()).await?;
        return Ok(());
    }
    // Being counted is what makes the vision thread start producing frames.
    let _client = ClientGuard::new(clients);
    let snapshot = matches!(route, Route::Snapshot);
    if !snapshot {
        let header = format!(
            "HTTP/1.0 200 OK\r\nConnection: close\r\nCache-Control: no-store, no-cache\r\nPragma: no-cache\r\n\
             Access-Control-Allow-Origin: *\r\nContent-Type: multipart/x-mixed-replace;boundary={BOUNDARY}\r\n\r\n"
        );
        sock.write_all(header.as_bytes()).await?;
    }
    loop {
        // Only frames encoded after the client arrived: whatever's stored may be from long ago.
        frames.changed().await.context("stream stopped")?;
        let Some(jpeg) = frames.borrow_and_update().clone() else { continue };
        if snapshot {
            let header = format!(
                "HTTP/1.0 200 OK\r\nConnection: close\r\nCache-Control: no-store\r\nContent-Type: image/jpeg\r\n\
                 Content-Length: {}\r\n\r\n",
                jpeg.len()
            );
            sock.write_all(header.as_bytes()).await?;
            sock.write_all(&jpeg).await?;
            return Ok(());
        }
        let part = format!("--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n", jpeg.len());
        sock.write_all(part.as_bytes()).await?;
        sock.write_all(&jpeg).await?;
        sock.write_all(b"\r\n").await?;
    }
}

/// Read the request headers (discarding all but the request line).
async fn read_request_line(sock: &mut TcpStream) -> Result<String> {
    let mut buf = Vec::with_capacity(1024);
    let mut chunk = [0u8; 1024];
    while !buf.windows(4).any(|w| w == b"\r\n\r\n") {
        anyhow::ensure!(buf.len() < 16 * 1024, "request headers too long");
        let n = sock.read(&mut chunk).await?;
        anyhow::ensure!(n > 0, "connection closed before the request ended");
        buf.extend_from_slice(&chunk[..n]);
    }
    let line = buf.split(|&b| b == b'\r').next().unwrap_or_default();
    Ok(String::from_utf8_lossy(line).into_owned())
}

/// This machine's IPv4 addresses, loopback last: the hosts a dashboard might reach the stream on.
pub fn local_ipv4() -> Vec<Ipv4Addr> {
    let mut addrs = Vec::new();
    let mut ifs: *mut libc::ifaddrs = std::ptr::null_mut();
    // SAFETY: getifaddrs fills `ifs` with a list we walk read-only and free once.
    unsafe {
        if libc::getifaddrs(&mut ifs) != 0 {
            return addrs;
        }
        let mut p = ifs;
        while let Some(ifa) = p.as_ref() {
            if let Some(sa) = ifa.ifa_addr.as_ref()
                && i32::from(sa.sa_family) == libc::AF_INET
            {
                let sin = &*(ifa.ifa_addr as *const libc::sockaddr_in);
                let ip = Ipv4Addr::from(u32::from_be(sin.sin_addr.s_addr));
                if !addrs.contains(&ip) {
                    addrs.push(ip);
                }
            }
            p = ifa.ifa_next;
        }
        libc::freeifaddrs(ifs);
    }
    addrs.sort_by_key(|ip| ip.is_loopback());
    addrs
}

/// The stream's URLs in CameraServer's form (`mjpg:http://host:port/?action=stream`), one per address.
pub fn stream_urls(port: u16) -> Vec<String> {
    local_ipv4().iter().map(|ip| format!("mjpg:http://{ip}:{port}/?action=stream")).collect()
}

/// The configuration page's URL on this machine's first address (loopback only if there's no other).
pub fn config_url(port: u16) -> String {
    let ip = local_ipv4().first().copied().unwrap_or(Ipv4Addr::LOCALHOST);
    format!("http://{ip}:{port}/config.toml")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn downscale_averages_blocks() {
        // 5×3, pitch 2 (like YUYV): the odd column and row are dropped.
        let luma = [0u8, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140];
        let data: Vec<u8> = luma.iter().flat_map(|&y| [y, 128]).collect();
        let img = ImageView::new(&data, 5, 3, 10, 2).unwrap();
        let (out, w, h) = downscale(&img, 2);
        assert_eq!((w, h), (2, 1));
        // (0 + 10 + 50 + 60 + 2) / 4 and (20 + 30 + 70 + 80 + 2) / 4, rounded.
        assert_eq!(out, [30, 50]);
        let (same, w, h) = downscale(&img, 1);
        assert_eq!((same, w, h), (luma.to_vec(), 5, 3));
    }

    #[test]
    fn look_stretches_levels_and_applies_gamma() {
        // A dim, flat frame: levels 40-100.
        let frame: Vec<u8> = (0..1000).map(|i| 40 + (i % 61) as u8).collect();
        let mut gray = frame.clone();
        Look::default().apply(&mut gray, true, 1.0);
        assert_eq!((gray.iter().min(), gray.iter().max()), (Some(&0), Some(&255)), "stretched to full range");
        // Off with gamma 1: untouched.
        let mut gray = frame.clone();
        Look::default().apply(&mut gray, false, 1.0);
        assert_eq!(gray, frame);
        // A featureless frame (one level) isn't stretched into noise.
        let mut gray = vec![50u8; 100];
        Look::default().apply(&mut gray, true, 1.0);
        assert!(gray.iter().all(|&v| v == gray[0]));
        // Gamma 2 alone lifts mid-grey: 64 -> 255 * (64/255)^0.5 = 128.
        let mut gray = vec![64u8, 0, 255];
        Look::default().apply(&mut gray, false, 2.0);
        assert_eq!(gray, [128, 0, 255]);
    }

    #[test]
    fn routes() {
        assert!(matches!(route("GET / HTTP/1.1"), Route::Stream));
        assert!(matches!(route("GET /?action=stream HTTP/1.1"), Route::Stream));
        assert!(matches!(route("GET /stream.mjpg HTTP/1.0"), Route::Stream));
        assert!(matches!(route("GET /?action=snapshot HTTP/1.1"), Route::Snapshot));
        assert!(matches!(route("GET /favicon.ico HTTP/1.1"), Route::NotFound));
        assert!(matches!(route("POST / HTTP/1.1"), Route::BadMethod));
        assert_eq!(route("GET /config.toml HTTP/1.1"), Route::Config { download: false });
        assert_eq!(route("GET /config.toml?download HTTP/1.1"), Route::Config { download: true });
        assert_eq!(route("GET /config.toml?download=1 HTTP/1.1"), Route::Config { download: true });
        assert_eq!(route("GET /config.toml?downloads HTTP/1.1"), Route::Config { download: false });
    }

    #[test]
    fn overlay_draws_and_clips() {
        let det = |id, corners| Detection { id, hamming: 0, decision_margin: 50.0, corners };
        let mut snap = Snapshot {
            gray: vec![128; 64 * 48],
            width: 64,
            height: 48,
            detections: vec![
                det(7, [[10.0, 10.0], [30.0, 10.0], [30.0, 30.0], [10.0, 30.0]]),
                // Off the edge, huge, and NaN: drawn where visible, never out of bounds.
                det(123, [[-5.0, 40.0], [70.0, 40.0], [70.0, 60.0], [-5.0, 60.0]]),
                det(1, [[f64::NAN, 0.0], [1e12, 0.0], [5.0, 5.0], [0.0, 5.0]]),
            ],
        };
        draw_overlay(&mut snap);
        let at = |x: usize, y: usize| snap.gray[y * 64 + x];
        assert_eq!(at(20, 10), 255, "outline");
        assert_eq!(at(20, 9), 0, "outline border");
        assert_eq!(at(20, 15), 128, "inside untouched");
        // "7" centred at (20, 20): its top row is white, with a black box around it.
        assert_eq!(at(19, 18), 255);
        assert_eq!(at(18, 17), 0);
    }

    #[test]
    fn urls_have_camera_server_form() {
        let urls = stream_urls(1190);
        assert!(urls.iter().any(|u| u == "mjpg:http://127.0.0.1:1190/?action=stream"), "{urls:?}");
        assert!(urls.last().unwrap().contains("127.0.0.1"), "loopback last: {urls:?}");
    }
}
