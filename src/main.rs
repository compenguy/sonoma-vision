//! sonoma-vision: capture frames from a V4L2 camera, detect AprilTags, publish them over NT4.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::Parser;
use log::{error, info, warn};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use tokio::sync::watch;

use sonoma_vision::capture::{Camera, CameraConfig, Layout};
use sonoma_vision::detector::{Config, Detector, Family};
use sonoma_vision::nt::{self, CameraAdvert, FrameResult, NtConfig};
use sonoma_vision::pose::camera::Intrinsics;
use sonoma_vision::pose::field::{self, FieldLayout};
use sonoma_vision::pose::{Estimator, FRC_TAG_SIZE, PoseConfig};
use sonoma_vision::settings::{NO_LAYOUT, Settings};
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
    /// Pixel format: grey or nv12 (read in place), yuyv or uyvy (luma copied out), or mjpeg (decoded).
    /// Default: the first of these, in that order, the camera offers at the requested size and rate.
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

    /// PhotonVision calibration JSON for this camera (any resolution of the same aspect ratio). Without
    /// one, the camera is approximated from --fov, which is fine for trying things but not for
    /// accurate poses.
    #[arg(long, env = "SONOMA_CALIBRATION")]
    calibration: Option<PathBuf>,
    /// Diagonal field of view in degrees, for the approximation used without --calibration.
    #[arg(long, default_value_t = 70.0)]
    fov: f64,
    /// Side of the tag's black square, in metres (FRC: 6.5 in).
    #[arg(long, default_value_t = FRC_TAG_SIZE)]
    tag_size: f64,
    /// Field layout for poses: the name of a layout in a `fields` directory (searched in
    /// /usr/share/sonoma-vision, /usr/local/share/sonoma-vision, the working directory, then beside
    /// the executable), a WPILib layout JSON file, or `none` for no poses. Without it,
    /// 2026-rebuilt-welded if it's found, else none. A selection persisted in NT wins over this.
    #[arg(long, env = "SONOMA_FIELD_LAYOUT")]
    field_layout: Option<String>,
    /// The camera's pose on the robot, WPILib's robot-to-camera transform: x,y,z in metres (forward,
    /// left, up from the robot's origin), then roll,pitch,yaw in degrees.
    #[arg(long, default_value = "0,0,0,0,0,0", value_parser = parse_transform, allow_hyphen_values = true)]
    robot_to_camera: Isometry3<f64>,
    /// Single-tag robot poses more ambiguous than this (0-1) aren't published.
    #[arg(long, default_value_t = 0.2)]
    max_ambiguity: f64,
}

/// `x,y,z,roll,pitch,yaw` (metres, degrees) as a transform.
fn parse_transform(s: &str) -> Result<Isometry3<f64>, String> {
    let v: Vec<f64> =
        s.split(',').map(|p| p.trim().parse::<f64>()).collect::<Result<_, _>>().map_err(|e| e.to_string())?;
    let [x, y, z, roll, pitch, yaw] = v[..] else {
        return Err(format!("expected x,y,z,roll,pitch,yaw (6 numbers), got {}", v.len()));
    };
    Ok(Isometry3::from_parts(
        Translation3::new(x, y, z),
        UnitQuaternion::from_euler_angles(roll.to_radians(), pitch.to_radians(), yaw.to_radians()),
    ))
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
        field_layout: None,
    };
    let (layouts, field_layout) = field_layouts(args.field_layout.as_deref())?;
    let settings = Settings { field_layout, ..settings };
    let (settings_tx, settings_rx) = watch::channel(settings);
    let pose = PoseConfig {
        calibration: match &args.calibration {
            Some(path) => {
                let json = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
                Some(Intrinsics::from_photonvision(&json).with_context(|| format!("calibration {}", path.display()))?)
            }
            None => None,
        },
        fov_deg: args.fov,
        tag_size: args.tag_size,
        // Chosen live, from `layouts`.
        layout: None,
        robot_to_camera: args.robot_to_camera,
        max_ambiguity: args.max_ambiguity,
    };
    anyhow::ensure!(args.tag_size > 0.0, "--tag-size must be positive");
    if pose.calibration.is_none() {
        warn!("no --calibration: approximating the camera from a {}° field of view; poses are rough", args.fov);
    }
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
        field_layouts: layouts.keys().cloned().collect(),
    };

    let (tx, rx) = watch::channel(None);
    let stop = Arc::new(AtomicBool::new(false));
    let vision = {
        let stop = stop.clone();
        std::thread::Builder::new()
            .name("vision".into())
            .spawn(move || vision_loop(Setup { camera, family, config, pose, layouts }, settings_rx, tx, tap, &stop))?
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
/// What the vision thread works with, fixed at startup.
struct Setup {
    camera: CameraConfig,
    family: &'static Family,
    config: Config,
    pose: PoseConfig,
    /// The field layouts that can be selected, by name.
    layouts: BTreeMap<String, FieldLayout>,
}

/// The default layout, when it's found.
const DEFAULT_LAYOUT: &str = "2026-rebuilt-welded";

/// The layouts that can be selected (those in the `fields` directories, and `spec` if it's a file),
/// and which to start with: `spec`'s, or without one, [`DEFAULT_LAYOUT`] if it's there.
fn field_layouts(spec: Option<&str>) -> Result<(BTreeMap<String, FieldLayout>, Option<String>)> {
    let catalog = field::Catalog::discover();
    for (name, (_, path)) in &catalog.layouts {
        info!("field layout {name}: {}", path.display());
    }
    let mut layouts: BTreeMap<String, FieldLayout> =
        catalog.layouts.into_iter().map(|(name, (layout, _))| (name, layout)).collect();
    let selected = match spec {
        Some(NO_LAYOUT) => None,
        Some(spec) if layouts.contains_key(spec) => Some(spec.to_string()),
        Some(spec) => {
            let path = std::path::Path::new(spec);
            anyhow::ensure!(
                path.is_file(),
                "--field-layout {spec:?} is neither a file nor a layout found in {} (found: {})",
                catalog.dirs.iter().map(|d| d.display().to_string()).collect::<Vec<_>>().join(", "),
                if layouts.is_empty() { "none".into() } else { layouts.keys().cloned().collect::<Vec<_>>().join(", ") },
            );
            layouts.insert(spec.to_string(), FieldLayout::from_file(path)?);
            Some(spec.to_string())
        }
        None if layouts.contains_key(DEFAULT_LAYOUT) => Some(DEFAULT_LAYOUT.to_string()),
        None => {
            warn!("field layout {DEFAULT_LAYOUT} not found; no poses until a layout is selected");
            None
        }
    };
    if layouts.is_empty() {
        warn!(
            "no field layouts found in {}: poses are off",
            catalog.dirs.iter().map(|d| d.display().to_string()).collect::<Vec<_>>().join(", ")
        );
    }
    Ok((layouts, selected))
}

/// An estimator for `width` × `height` frames against the layout `name`; None (no pose solving) with
/// no layout.
fn build_estimator(
    pose: &PoseConfig,
    layouts: &BTreeMap<String, FieldLayout>,
    name: Option<&str>,
    width: usize,
    height: usize,
) -> Option<Estimator> {
    let Some(name) = name else {
        info!("no field layout: poses off");
        return None;
    };
    let Some(layout) = layouts.get(name) else {
        warn!("no field layout {name:?}: poses off");
        return None;
    };
    match Estimator::new(&PoseConfig { layout: Some(layout.clone()), ..pose.clone() }, width, height) {
        Ok(e) => {
            let c = &e.camera;
            info!(
                "poses against {name} ({} tags); camera model: fx {:.1} fy {:.1} cx {:.1} cy {:.1}",
                layout.tags.len(),
                c.fx,
                c.fy,
                c.cx,
                c.cy
            );
            Some(e)
        }
        Err(e) => {
            error!("no poses: {e:#}");
            None
        }
    }
}

fn vision_loop(
    setup: Setup,
    mut settings: watch::Receiver<Settings>,
    tx: watch::Sender<Option<Arc<FrameResult>>>,
    mut tap: Option<Tap>,
    stop: &AtomicBool,
) {
    let Setup { camera: cfg, family, mut config, pose, layouts } = setup;
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
        let mut applied: Option<Settings> = None;
        // Built with the settings below, and rebuilt when the field layout changes.
        let mut estimator = None;
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
                if applied.as_ref().is_none_or(|old| old.field_layout != new.field_layout) {
                    let name = new.field_layout.as_deref();
                    estimator = build_estimator(&pose, &layouts, name, camera.width, camera.height);
                }
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
            // With no field layout, no pose solving at all.
            let (tag_poses, field_pose) = match &estimator {
                Some(e) => {
                    let tags: Vec<_> = detections.iter().map(|d| e.tag(d)).collect();
                    let field = e.field(&detections, &tags);
                    (tags, field)
                }
                None => (vec![None; detections.len()], None),
            };
            let result = FrameResult {
                sequence: frame.sequence,
                capture: frame.timestamp,
                detections: detections.clone(),
                detect_time,
                fps,
                dropped,
                tag_poses,
                field_pose,
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
