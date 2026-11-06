//! sonoma-vision: capture frames from a V4L2 camera, detect AprilTags, publish them over NT4.

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::Parser;
use log::{error, info, warn};
use tokio::sync::watch;

use sonoma_vision::capture::{Camera, CameraConfig, Layout};
use sonoma_vision::detector::{Config, Detector, Family};
use sonoma_vision::nt::{self, CameraAdvert, FrameResult, NtConfig};
use sonoma_vision::settings::Settings;
use sonoma_vision::stream::{self, StreamConfig, Tap};

#[derive(Parser, Debug)]
#[command(version, about)]
struct Args {
    /// V4L2 capture device.
    #[arg(long, env = "SONOMA_DEVICE", default_value = "/dev/video0")]
    device: PathBuf,
    #[arg(long, default_value_t = 1280)]
    width: u32,
    #[arg(long, default_value_t = 720)]
    height: u32,
    /// Requested frame rate; the camera may choose another.
    #[arg(long)]
    fps: Option<u32>,
    /// Pixel format: grey, yuyv, uyvy or nv12 (read in place) or mjpeg (decoded). Default: the first
    /// of these the camera offers.
    #[arg(long)]
    format: Option<Layout>,
    /// Driver buffers to cycle (2-4). Fewer means less queued latency.
    #[arg(long, default_value_t = 3)]
    buffers: u32,
    /// Manual exposure in driver units (100 µs for UVC cameras); default auto. Short exposures limit
    /// motion blur. Live setting: NT values win (see the README).
    #[arg(long)]
    exposure: Option<i32>,
    /// Analog gain in driver units; default the camera's own. Brightens without blur, adds noise.
    /// Live setting.
    #[arg(long)]
    gain: Option<i32>,

    /// Tag family.
    #[arg(long, default_value = "tag36h11")]
    family: String,
    /// Smallest tag side (black square, pixels) to guarantee; smaller is slower.
    #[arg(long, default_value_t = 32)]
    min_tag_px: u32,
    /// Grey levels required between tag black and white. Live setting.
    #[arg(long, default_value_t = 20, value_parser = clap::value_parser!(u8).range(1..))]
    min_contrast: u8,
    /// Payload bits to correct (default: the family's recommendation; 3 for tag36h11).
    #[arg(long)]
    max_hamming: Option<u32>,

    /// NT server: `local` (the robot program on this controller), a team number, or an IPv4 address.
    #[arg(long, env = "SONOMA_NT_SERVER", default_value = "local")]
    nt_server: String,
    #[arg(long, default_value_t = 5810)]
    nt_port: u16,
    /// NT client name, as the server lists it.
    #[arg(long, env = "SONOMA_NT_NAME", default_value = "sonoma-vision")]
    nt_name: String,
    /// NT table to publish under.
    #[arg(long, env = "SONOMA_TABLE", default_value = "/sonoma/cam0")]
    table: String,

    /// Serve an MJPEG camera stream for dashboards, advertised in NT as WPILib's CameraServer does.
    #[arg(long)]
    stream: bool,
    /// Camera stream HTTP port. The field network allows 1180-1190 for cameras; CameraServer starts
    /// at 1181, so 1190 stays clear of a robot program's own cameras.
    #[arg(long, default_value_t = 1190)]
    stream_port: u16,
    /// Most frames per second to stream; the camera's rate is the upper limit.
    #[arg(long, default_value_t = 15.0)]
    stream_fps: f64,
    /// JPEG quality of the stream, 1-100.
    #[arg(long, default_value_t = 50, value_parser = clap::value_parser!(u8).range(1..=100))]
    stream_quality: u8,
    /// Divide the stream's width and height by this (1-8): 2 streams a 640x480 camera at 320x240.
    #[arg(long, default_value_t = 1, value_parser = clap::value_parser!(u8).range(1..=8))]
    stream_scale: u8,
    /// Draw each detected tag's outline and ID on the stream.
    #[arg(long)]
    stream_overlay: bool,
    /// Stretch each streamed frame to the full grey range (viewing only, not detection). Live setting.
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    stream_auto_levels: bool,
    /// Stream gamma, 0.1-10: above 1 lifts shadows (viewing only). Live setting.
    #[arg(long, default_value_t = 1.0)]
    stream_gamma: f64,
    /// Camera name dashboards show (default: sonoma- and the table's last part, e.g. sonoma-cam0).
    #[arg(long)]
    stream_name: Option<String>,
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let args = Args::parse();
    let family = Family::by_name(&args.family).with_context(|| {
        format!("unknown family {:?}; one of {}", args.family, Family::names().collect::<Vec<_>>().join(", "))
    })?;
    let config = Config {
        min_tag_px: args.min_tag_px,
        max_hamming: args.max_hamming,
        min_contrast: args.min_contrast,
        ..Config::default()
    };
    // Check the configuration now; the vision thread builds the detector (and rebuilds it when a live
    // setting changes it).
    Detector::new(family, &config)?;
    anyhow::ensure!((0.1..=10.0).contains(&args.stream_gamma), "--stream-gamma must be from 0.1 to 10");
    let settings = Settings {
        exposure: args.exposure.filter(|&e| e > 0),
        gain: args.gain.filter(|&g| g >= 0),
        min_contrast: args.min_contrast,
        stream_auto_levels: args.stream_auto_levels,
        stream_gamma: args.stream_gamma,
    };
    let (settings_tx, settings_rx) = watch::channel(settings);
    let camera = CameraConfig {
        device: args.device.clone(),
        width: args.width,
        height: args.height,
        fps: args.fps,
        layout: args.format,
        buffers: args.buffers,
    };
    let (tap, server, camera_advert) = if args.stream {
        let cfg = StreamConfig {
            port: args.stream_port,
            fps: args.stream_fps,
            quality: args.stream_quality,
            scale: usize::from(args.stream_scale),
            overlay: args.stream_overlay,
        };
        let (tap, server) = stream::start(&cfg, settings_rx.clone())?;
        let name = args.stream_name.clone().unwrap_or_else(|| {
            format!("sonoma-{}", args.table.trim_end_matches('/').rsplit('/').next().unwrap_or("cam"))
        });
        let advert = CameraAdvert {
            name,
            port: server.local_addr()?.port(),
            source: format!("usb:{}", args.device.display()),
            mode: format!(
                "{}x{} MJPEG {} fps",
                args.width / u32::from(args.stream_scale),
                args.height / u32::from(args.stream_scale),
                args.stream_fps.round()
            ),
        };
        (Some(tap), Some(server), Some(advert))
    } else {
        (None, None, None)
    };
    let nt_config = NtConfig {
        server: nt::parse_server(&args.nt_server)?,
        port: args.nt_port,
        client_name: args.nt_name,
        table: args.table,
        camera: camera_advert,
    };

    let (tx, rx) = watch::channel(None);
    let stop = Arc::new(AtomicBool::new(false));
    let vision = {
        let stop = stop.clone();
        std::thread::Builder::new()
            .name("vision".into())
            .spawn(move || vision_loop(camera, family, config, settings_rx, tx, tap, &stop))?
    };

    let runtime = tokio::runtime::Builder::new_multi_thread().worker_threads(1).enable_all().build()?;
    runtime.block_on(async {
        let mut term = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
        let stream = async {
            match server {
                Some(server) => server.run().await,
                None => std::future::pending().await,
            }
        };
        tokio::select! {
            res = nt::run(nt_config, rx, settings_tx) => res,
            res = stream => res,
            _ = tokio::signal::ctrl_c() => Ok(()),
            _ = term.recv() => Ok(()),
        }
    })?;
    info!("shutting down");
    stop.store(true, Ordering::Relaxed);
    let _ = vision.join();
    Ok(())
}

/// Capture and detect until `stop`, reopening the camera after errors (e.g. a USB unplug), and apply
/// live settings as they change.
fn vision_loop(
    cfg: CameraConfig,
    family: &'static Family,
    mut config: Config,
    mut settings: watch::Receiver<Settings>,
    tx: watch::Sender<Option<Arc<FrameResult>>>,
    mut tap: Option<Tap>,
    stop: &AtomicBool,
) {
    let mut detector = Detector::new(family, &config).expect("checked at startup");
    let mut scratch = Vec::new();
    let mut detections = Vec::new();
    let mut dropped = 0u64;
    while !stop.load(Ordering::Relaxed) {
        let mut camera = match Camera::open(&cfg) {
            Ok(c) => c,
            Err(e) => {
                error!("camera: {e:#}");
                std::thread::sleep(Duration::from_secs(1));
                continue;
            }
        };
        // A newly opened camera gets every control; after that, only changes.
        let mut applied = None;
        if !camera.layout.zero_copy() {
            warn!("{:?} frames are decoded (one copy per frame); a raw format avoids it", camera.layout);
        }
        let mut last_seq: Option<u32> = None;
        let mut fps = 0.0;
        let mut last_frame: Option<Instant> = None;
        while !stop.load(Ordering::Relaxed) {
            if applied.is_none() || settings.has_changed().unwrap_or(false) {
                let new = settings.borrow_and_update().clone();
                apply_settings(&camera, &mut detector, &mut config, applied.as_ref(), &new);
                applied = Some(new);
            }
            let frame = match camera.next_frame(Duration::from_millis(500)) {
                Ok(Some(f)) => f,
                Ok(None) => continue,
                Err(e) => {
                    error!("capture: {e:#}");
                    break;
                }
            };
            let t0 = Instant::now();
            let img = match frame.luma(&mut scratch) {
                Ok(img) => img,
                Err(e) => {
                    warn!("frame {}: {e:#}", frame.sequence);
                    continue;
                }
            };
            detector.detect_into(&img, &mut detections);
            let detect_time = t0.elapsed();
            if let Some(tap) = &mut tap {
                tap.offer(&img, &detections);
            }
            if let Some(prev) = last_seq {
                dropped += u64::from(frame.sequence.wrapping_sub(prev).saturating_sub(1));
            }
            last_seq = Some(frame.sequence);
            if let Some(prev) = last_frame {
                let dt = t0.duration_since(prev).as_secs_f64();
                fps = if fps == 0.0 { 1.0 / dt } else { 0.9 * fps + 0.1 / dt };
            }
            last_frame = Some(t0);
            let result = FrameResult {
                sequence: frame.sequence,
                capture: frame.timestamp,
                detections: detections.clone(),
                detect_time,
                fps,
                dropped,
            };
            // Only the publisher holds a receiver; if it's gone, the service is shutting down.
            if tx.send(Some(Arc::new(result))).is_err() {
                return;
            }
        }
    }
}

/// Apply the live settings in `new` that differ from `old` (every camera control when `old` is None).
fn apply_settings(
    camera: &Camera,
    detector: &mut Detector,
    config: &mut Config,
    old: Option<&Settings>,
    new: &Settings,
) {
    if old.is_none_or(|o| o.exposure != new.exposure)
        && let Err(e) = camera.set_exposure(new.exposure)
    {
        warn!("{e:#}");
    }
    // Gain None leaves the camera's own, so there's nothing to set.
    if let Some(gain) = new.gain
        && old.is_none_or(|o| o.gain != new.gain)
        && let Err(e) = camera.set_gain(gain)
    {
        warn!("{e:#}");
    }
    if new.min_contrast != config.min_contrast {
        config.min_contrast = new.min_contrast;
        match Detector::new(detector.family(), config) {
            Ok(d) => {
                *detector = d;
                info!("min_contrast set to {}", new.min_contrast);
            }
            Err(e) => warn!("min_contrast {}: {e}", new.min_contrast),
        }
    }
}
