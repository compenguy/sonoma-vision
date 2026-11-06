//! sonoma-vision: capture frames from a V4L2 camera, detect AprilTags, publish them over NT4.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::{CommandFactory, FromArgMatches};
use log::{error, info, warn};
use tokio::sync::watch;

use sonoma_vision::capture::{Camera, CameraConfig, CameraState, Layout};
use sonoma_vision::config::{self, FileValues};
use sonoma_vision::detector::{Config, Detector, Family};
use sonoma_vision::nt::{self, CameraAdvert, FrameResult, NtConfig};
use sonoma_vision::pose::camera::Intrinsics;
use sonoma_vision::pose::field::{self, FieldLayout};
use sonoma_vision::pose::{Estimator, FRC_TAG_SIZE, MountPose, PoseConfig};
use sonoma_vision::settings::{NO_LAYOUT, Settings};
use sonoma_vision::stream::{self, ConfigPage, StreamConfig, Tap};

#[derive(clap::Parser, Debug)]
#[command(version, about)]
struct Args {
    /// Configuration file; repeat to layer several (a shared file, then a camera's), later ones
    /// winning key by key. Without it, sonoma-vision.toml is read from /usr/share/sonoma-vision,
    /// /usr/local/share/sonoma-vision, the working directory and the executable's directory, in that
    /// order. Command line options override the files.
    #[arg(long, value_name = "FILE", action = clap::ArgAction::Append)]
    config: Vec<PathBuf>,
    /// Write the configuration in effect (the files' values with the command line's over them) to
    /// stdout as a sonoma-vision.toml, and exit.
    #[arg(long)]
    print_config: bool,

    /// V4L2 capture device.
    #[arg(long, default_value = "/dev/video0")]
    device: PathBuf,
    /// Requested frame width; the camera may choose another. Width, height, fps and format are
    /// chosen live together as the capture mode.
    #[arg(long, default_value_t = 1280)]
    width: u32,
    /// Requested frame height.
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
    /// motion blur. Live setting.
    #[arg(long)]
    exposure: Option<i32>,
    /// Analog gain in driver units; default the camera's own. Brightens without blur, adds noise.
    /// Live setting.
    #[arg(long)]
    gain: Option<i32>,
    /// Manual focus in driver units, turning autofocus off; default the camera's own. Calibrated
    /// poses need a fixed focus: autofocus changes the lens's focal length. Live setting.
    #[arg(long)]
    focus: Option<i32>,

    /// Tag family.
    #[arg(long, default_value = "tag36h11")]
    family: String,
    /// Smallest tag side (black square, pixels) to guarantee; smaller is slower. Live setting.
    #[arg(long, default_value_t = 32)]
    min_tag_px: u32,
    /// Grey levels required between tag black and white. Live setting.
    #[arg(long, default_value_t = 20, value_parser = clap::value_parser!(u8).range(1..))]
    min_contrast: u8,
    /// Payload bits to correct (default: the family's recommendation; 3 for tag36h11). Live setting.
    #[arg(long)]
    max_hamming: Option<u32>,
    /// Dark border cells required for a tag, out of tag36h11's 28 (0-28); fewer accepts partly
    /// occluded or glared tags, and more false ones. Live setting.
    #[arg(long, default_value_t = 22, value_parser = clap::value_parser!(u32).range(0..=28))]
    min_border_cells: u32,
    /// Cell sharpening strength, 0-1, for blurred tags; upstream AprilTag uses 0.25. Live setting.
    #[arg(long, default_value_t = 0.25)]
    sharpening: f64,

    /// NT server: `local` (the robot program on this controller), a team number, or an IPv4 address.
    #[arg(long, default_value = "local")]
    nt_server: String,
    #[arg(long, default_value_t = 5810)]
    nt_port: u16,
    /// NT client name, as the server lists it.
    #[arg(long, default_value = "sonoma-vision")]
    nt_name: String,
    /// NT table to publish under.
    #[arg(long, default_value = "/sonoma/cam0")]
    table: String,

    /// Serve an MJPEG camera stream for dashboards, advertised in NT as WPILib's CameraServer does.
    #[arg(long, value_name = "BOOL", num_args = 0..=1, default_value_t = false, default_missing_value = "true",
          action = clap::ArgAction::Set)]
    stream: bool,
    /// Camera stream HTTP port. The field network allows 1180-1190 for cameras; CameraServer starts
    /// at 1181, so 1190 stays clear of a robot program's own cameras.
    #[arg(long, default_value_t = 1190)]
    stream_port: u16,
    /// Most frames per second to stream (0.1-1000); the camera's rate is the upper limit. Live setting.
    #[arg(long, default_value_t = 15.0)]
    stream_fps: f64,
    /// JPEG quality of the stream, 1-100. Live setting.
    #[arg(long, default_value_t = 50, value_parser = clap::value_parser!(u8).range(1..=100))]
    stream_quality: u8,
    /// Divide the stream's width and height by this (1-8): 2 streams a 640x480 camera at 320x240.
    /// Live setting.
    #[arg(long, default_value_t = 1, value_parser = clap::value_parser!(u8).range(1..=8))]
    stream_scale: u8,
    /// Draw each detected tag's outline and ID on the stream. Live setting.
    #[arg(long, value_name = "BOOL", num_args = 0..=1, default_value_t = false, default_missing_value = "true",
          action = clap::ArgAction::Set)]
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
    #[arg(long)]
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
    /// 2026-rebuilt-welded if it's found, else none. Live setting.
    #[arg(long)]
    field_layout: Option<String>,
    /// The camera's pose on the robot, WPILib's robot-to-camera transform: x,y,z in metres (forward,
    /// left, up from the robot's origin), then roll,pitch,yaw in degrees. Live setting.
    #[arg(long, default_value = "0,0,0,0,0,0", allow_hyphen_values = true)]
    robot_to_camera: MountPose,
    /// Single-tag robot poses more ambiguous than this (0-1) aren't published. Live setting.
    #[arg(long, default_value_t = 0.2)]
    max_ambiguity: f64,

    /// Log level, or an env_logger filter such as `info,sonoma_vision::nt=debug`.
    #[arg(long, default_value = "info")]
    log_level: String,
}

/// The arguments, with configuration files' values as their defaults, and the files read.
fn parse_args() -> Result<(Args, clap::ArgMatches, FileValues)> {
    let explicit = config::explicit_paths(std::env::args_os());
    let files = FileValues::load(&Args::command(), &explicit, &config::search_dirs())?;
    let matches = files.apply(Args::command()).get_matches();
    let args = Args::from_arg_matches(&matches).unwrap_or_else(|e| e.exit());
    Ok((args, matches, files))
}

fn main() -> Result<()> {
    let (args, matches, files) = parse_args()?;
    if args.print_config {
        let note = "sonoma-vision configuration, written by `sonoma-vision --print-config`.";
        print!("{}", config::render(&Args::command(), &matches, &files, &[], note));
        return Ok(());
    }
    env_logger::Builder::new().parse_filters(&args.log_level).init();
    for f in &files.files {
        info!("configuration: {}", f.display());
    }
    let family = Family::by_name(&args.family).with_context(|| {
        format!("unknown family {:?}; one of {}", args.family, Family::names().collect::<Vec<_>>().join(", "))
    })?;
    anyhow::ensure!((0.1..=10.0).contains(&args.stream_gamma), "--stream-gamma must be from 0.1 to 10");
    anyhow::ensure!((0.1..=1000.0).contains(&args.stream_fps), "--stream-fps must be from 0.1 to 1000");
    anyhow::ensure!((0.0..=1.0).contains(&args.max_ambiguity), "--max-ambiguity must be from 0 to 1");
    anyhow::ensure!(args.max_hamming.is_none_or(|h| h <= 3), "--max-hamming must be from 0 to 3");
    let (layouts, field_layout) = field_layouts(args.field_layout.as_deref())?;
    let settings = Settings {
        exposure: args.exposure.filter(|&e| e > 0),
        gain: args.gain.filter(|&g| g >= 0),
        focus: args.focus.filter(|&f| f >= 0),
        min_contrast: args.min_contrast,
        min_tag_px: args.min_tag_px,
        max_hamming: args.max_hamming,
        min_border_cells: args.min_border_cells,
        sharpening: args.sharpening,
        max_ambiguity: args.max_ambiguity,
        robot_to_camera: args.robot_to_camera,
        stream_auto_levels: args.stream_auto_levels,
        stream_gamma: args.stream_gamma,
        stream_overlay: args.stream_overlay,
        stream_quality: args.stream_quality,
        stream_fps: args.stream_fps,
        stream_scale: args.stream_scale,
        field_layout,
        capture_mode: None,
    };
    // Check the detector settings now; the vision thread builds the detector (and rebuilds it when a
    // live setting changes it).
    Detector::new(family, &detector_config(&settings))?;
    let initial = settings.clone();
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
        // Live settings.
        layout: None,
        robot_to_camera: args.robot_to_camera.isometry(),
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
    let instance = args.table.trim_end_matches('/').rsplit('/').next().unwrap_or("cam").to_string();
    let (tap, server, camera_advert) = if args.stream {
        let page = config_page(matches, files, initial, settings_rx.clone(), &args.table, &instance);
        let (tap, server) = stream::start(&StreamConfig { port: args.stream_port }, settings_rx.clone(), Some(page))?;
        let name = args.stream_name.clone().unwrap_or_else(|| format!("sonoma-{instance}"));
        let advert =
            CameraAdvert { name, port: server.local_addr()?.port(), source: format!("usb:{}", args.device.display()) };
        info!("configuration in effect at {}", stream::config_url(advert.port));
        (Some(tap), Some(server), Some(advert))
    } else {
        (None, None, None)
    };
    let (state_tx, state_rx) = watch::channel(CameraState::default());
    let nt_config = NtConfig {
        server: nt::parse_server(&args.nt_server)?,
        port: args.nt_port,
        client_name: args.nt_name,
        table: args.table,
        camera: camera_advert,
        field_layouts: layouts.keys().cloned().collect(),
        camera_state: state_rx,
    };

    let (tx, rx) = watch::channel(None);
    let stop = Arc::new(AtomicBool::new(false));
    let vision = {
        let stop = stop.clone();
        std::thread::Builder::new().name("vision".into()).spawn(move || {
            vision_loop(Setup { camera, family, pose, layouts }, settings_rx, state_tx, tx, tap, &stop)
        })?
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
    state: watch::Sender<CameraState>,
    tx: watch::Sender<Option<Arc<FrameResult>>>,
    mut tap: Option<Tap>,
    stop: &AtomicBool,
) {
    let Setup { camera: cfg, family, pose, layouts } = setup;
    let mut config = detector_config(&settings.borrow());
    let mut detector = Detector::new(family, &config).expect("checked at startup");
    let mut scratch = Vec::new();
    let mut detections = Vec::new();
    let mut dropped = 0u64;
    // The mode the configuration opened, once it has been.
    let mut default_mode = None;
    while !stop.load(Ordering::Relaxed) {
        // A capture mode chosen live replaces the configuration's size, rate and format.
        let mode = settings.borrow().capture_mode;
        let opened = match mode {
            Some(m) => Camera::open(&cfg.with_mode(&m)),
            None => Camera::open(&cfg),
        };
        let mut camera = match opened {
            Ok(c) => c,
            Err(e) => {
                error!("camera: {e:#}");
                state.send_modify(|s| s.active = None);
                std::thread::sleep(Duration::from_secs(1));
                continue;
            }
        };
        if mode.is_none() {
            default_mode = Some(camera.mode());
        }
        let modes = camera.modes();
        info!("capture modes: {}", modes.iter().map(|m| m.to_string()).collect::<Vec<_>>().join(", "));
        state.send_replace(CameraState { modes, active: Some(camera.mode()), default: default_mode });
        // A newly opened camera gets every control; after that, only changes.
        let mut applied: Option<Settings> = None;
        // Built with the settings below, and rebuilt when the pose settings change.
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
                if new.capture_mode != mode {
                    info!(
                        "capture mode {}: reopening the camera",
                        new.capture_mode.map_or("default".into(), |m| m.to_string())
                    );
                    break;
                }
                apply_settings(&camera, family, &mut detector, &mut config, applied.as_ref(), &new);
                let pose_changed = |old: &Settings| {
                    (&old.field_layout, old.robot_to_camera, old.max_ambiguity)
                        != (&new.field_layout, new.robot_to_camera, new.max_ambiguity)
                };
                if applied.as_ref().is_none_or(pose_changed) {
                    let live = PoseConfig {
                        robot_to_camera: new.robot_to_camera.isometry(),
                        max_ambiguity: new.max_ambiguity,
                        ..pose.clone()
                    };
                    let name = new.field_layout.as_deref();
                    estimator = build_estimator(&live, &layouts, name, camera.width, camera.height);
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
        state.send_modify(|s| s.active = None);
    }
}

/// The detector's settings, from the live ones.
fn detector_config(s: &Settings) -> Config {
    Config {
        min_tag_px: s.min_tag_px,
        max_hamming: s.max_hamming,
        min_contrast: s.min_contrast,
        min_border_cells: s.min_border_cells,
        sharpening: s.sharpening,
    }
}

/// Apply the live settings in `new` that differ from `old` (every camera control when `old` is None).
fn apply_settings(
    camera: &Camera,
    family: &'static Family,
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
    // Focus None leaves a newly opened camera's own; changed to None live, it's autofocus again.
    let focus_changed = match old {
        None => new.focus.is_some(),
        Some(o) => o.focus != new.focus,
    };
    if focus_changed && let Err(e) = camera.set_focus(new.focus) {
        warn!("{e:#}");
    }
    let wanted = detector_config(new);
    if wanted != *config {
        match Detector::new(family, &wanted) {
            Ok(d) => {
                *detector = d;
                info!("detector: {wanted:?}");
                *config = wanted;
            }
            Err(e) => warn!("detector settings {wanted:?}: {e}"),
        }
    }
}

/// The `/config.toml` page: the configuration the service started with (`matches`, from `files`
/// and the command line), with the live settings that differ from `initial` over it.
fn config_page(
    matches: clap::ArgMatches,
    files: FileValues,
    initial: Settings,
    settings: watch::Receiver<Settings>,
    table: &str,
    instance: &str,
) -> ConfigPage {
    let note = format!(
        "sonoma-vision configuration for {table}, as it's running.\n\
         To keep it, install it as /etc/sonoma-vision/{instance}.toml (it includes the shared file's\n\
         values too) and restart the service."
    );
    let render = move || {
        let live = settings.borrow().config_overrides(&initial);
        config::render(&Args::command(), &matches, &files, &live, &note)
    };
    ConfigPage { render: Arc::new(render), filename: format!("{instance}.toml") }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Options that only make sense on the command line.
    const COMMAND_LINE_ONLY: &[&str] = &["config", "print_config", "help", "version"];

    #[test]
    fn every_option_has_a_config_key() {
        let cmd = Args::command();
        let ids: Vec<&str> = cmd.get_arguments().map(|a| a.get_id().as_str()).collect();
        for key in config::KEYS {
            assert!(ids.contains(&key.arg), "[{}] {} sets no option {}", key.section, key.name, key.arg);
        }
        for id in ids {
            assert!(
                COMMAND_LINE_ONLY.contains(&id) || config::KEYS.iter().any(|k| k.arg == id),
                "--{id} has no key in config::KEYS (or add it to COMMAND_LINE_ONLY)"
            );
        }
    }

    #[test]
    fn printed_config_reads_back_to_the_same_arguments() {
        let dir = std::env::temp_dir().join(format!("sonoma-config-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("sonoma-vision.toml");
        std::fs::write(
            &file,
            "[camera]\nfps = 120\n[stream]\nenabled = true\n[pose]\nrobot_to_camera = [0.3, 0, 0.5, 0, -15, 0]\n",
        )
        .unwrap();
        let parse = |files: &FileValues, argv: &[&str]| {
            let m =
                files.apply(Args::command()).try_get_matches_from(std::iter::once("sv").chain(argv.iter().copied()));
            let m = m.unwrap();
            (Args::from_arg_matches(&m).unwrap(), m)
        };
        let files = FileValues::load(&Args::command(), std::slice::from_ref(&file), &[]).unwrap();
        let (args, m) = parse(&files, &["--width", "640", "--stream-overlay"]);
        assert_eq!((args.fps, args.width, args.stream, args.stream_overlay), (Some(120), 640, true, true));
        let text = config::render(&Args::command(), &m, &files, &[], "test");
        std::fs::write(&file, &text).unwrap();
        let again = FileValues::load(&Args::command(), std::slice::from_ref(&file), &[]).unwrap();
        let (back, _) = parse(&again, &[]);
        std::fs::remove_dir_all(&dir).unwrap();
        assert_eq!(format!("{back:?}"), format!("{args:?}"), "{text}");
    }

    #[test]
    fn live_settings_write_back_as_options() {
        use sonoma_vision::capture::{Layout, Mode};
        use sonoma_vision::settings::Key;
        let m = Args::command().try_get_matches_from(["sv"]).unwrap();
        let initial = Settings::default();
        // Every live setting changed.
        let live = Settings {
            exposure: Some(30),
            gain: Some(4),
            focus: Some(0),
            min_contrast: 12,
            min_tag_px: 24,
            max_hamming: Some(1),
            min_border_cells: 20,
            sharpening: 0.5,
            max_ambiguity: 0.1,
            robot_to_camera: MountPose([0.3, 0.0, 0.5, 0.0, -15.0, 0.0]),
            stream_auto_levels: false,
            stream_gamma: 1.5,
            stream_overlay: true,
            stream_quality: 70,
            stream_fps: 30.0,
            stream_scale: 2,
            field_layout: Some("2026-rebuilt-welded".into()),
            capture_mode: Some(Mode { width: 640, height: 480, fps: 120, layout: Layout::Mjpeg }),
        };
        let overrides = live.config_overrides(&initial);
        for key in Key::ALL {
            assert!(overrides.iter().any(|(id, _)| *id == key.name()), "{key:?} not written back");
        }
        let text = config::render(&Args::command(), &m, &FileValues::default(), &overrides, "test");
        let dir = std::env::temp_dir().join(format!("sonoma-live-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("cam0.toml");
        std::fs::write(&file, &text).unwrap();
        let files = FileValues::load(&Args::command(), std::slice::from_ref(&file), &[]).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        let back = Args::from_arg_matches(&files.apply(Args::command()).try_get_matches_from(["sv"]).unwrap()).unwrap();
        assert_eq!(
            (back.exposure, back.gain, back.focus, back.min_contrast, back.min_tag_px, back.max_hamming),
            (Some(30), Some(4), Some(0), 12, 24, Some(1)),
            "{text}"
        );
        assert_eq!((back.min_border_cells, back.sharpening, back.max_ambiguity), (20, 0.5, 0.1));
        assert_eq!(back.robot_to_camera, live.robot_to_camera);
        assert_eq!((back.stream_auto_levels, back.stream_gamma, back.stream_overlay), (false, 1.5, true));
        assert_eq!((back.stream_quality, back.stream_fps, back.stream_scale), (70, 30.0, 2));
        assert_eq!(back.field_layout.as_deref(), Some("2026-rebuilt-welded"));
        assert_eq!((back.width, back.height, back.fps, back.format), (640, 480, Some(120), Some(Layout::Mjpeg)));
    }
}
