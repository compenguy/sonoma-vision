//! sonoma-vision: capture frames from a V4L2 camera, detect AprilTags, publish them over NT4.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use anyhow::{Context, Result};
use clap::{CommandFactory, FromArgMatches};
use log::{info, warn};
use tokio::sync::watch;

use sonoma_vision::capture::{CameraConfig, CameraState, Layout};
use sonoma_vision::config::{self, FileValues};
use sonoma_vision::detector::{Detector, Family};
use sonoma_vision::nt::{self, CameraAdvert, NtConfig};
use sonoma_vision::pose::camera::Intrinsics;
use sonoma_vision::pose::field::{self, FieldLayout};
use sonoma_vision::pose::{FRC_TAG_SIZE, MountPose, PoseBounds, PoseConfig};
use sonoma_vision::settings::{NO_LAYOUT, Settings, limits};
use sonoma_vision::stream::{self, ConfigPage, StreamConfig};
use sonoma_vision::vision::{self, Setup};

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
    #[arg(long, default_value_t = Settings::DEFAULT.min_tag_px, value_parser = clap::value_parser!(u32).range(limits::MIN_TAG_PX))]
    min_tag_px: u32,
    /// Grey levels required between tag black and white. Live setting.
    #[arg(long, default_value_t = Settings::DEFAULT.min_contrast, value_parser = clap::value_parser!(u8).range(limits::MIN_CONTRAST))]
    min_contrast: u8,
    /// Payload bits to correct (default: the family's recommendation; 3 for tag36h11). Live setting.
    #[arg(long, value_parser = clap::value_parser!(u32).range(limits::MAX_HAMMING))]
    max_hamming: Option<u32>,
    /// Dark border cells required for a tag, out of tag36h11's 28 (0-28); fewer accepts partly
    /// occluded or glared tags, and more false ones. Live setting.
    #[arg(long, default_value_t = Settings::DEFAULT.min_border_cells, value_parser = clap::value_parser!(u32).range(limits::MIN_BORDER_CELLS))]
    min_border_cells: u32,
    /// Cell sharpening strength, 0-1, for blurred tags; upstream AprilTag uses 0.25. Live setting.
    #[arg(long, default_value_t = Settings::DEFAULT.sharpening, value_parser = limits::decimal(limits::SHARPENING))]
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
    #[arg(long, default_value_t = Settings::DEFAULT.stream_fps, value_parser = limits::decimal(limits::STREAM_FPS))]
    stream_fps: f64,
    /// JPEG quality of the stream, 1-100. Live setting.
    #[arg(long, default_value_t = Settings::DEFAULT.stream_quality, value_parser = clap::value_parser!(u8).range(limits::STREAM_QUALITY))]
    stream_quality: u8,
    /// Divide the stream's width and height by this (1-8): 2 streams a 640x480 camera at 320x240.
    /// Live setting.
    #[arg(long, default_value_t = Settings::DEFAULT.stream_scale, value_parser = clap::value_parser!(u8).range(limits::STREAM_SCALE))]
    stream_scale: u8,
    /// Draw each detected tag's outline and ID on the stream. Live setting.
    #[arg(long, value_name = "BOOL", num_args = 0..=1, default_value_t = Settings::DEFAULT.stream_overlay,
          default_missing_value = "true", action = clap::ArgAction::Set)]
    stream_overlay: bool,
    /// Stretch each streamed frame to the full grey range (viewing only, not detection). Live setting.
    #[arg(long, default_value_t = Settings::DEFAULT.stream_auto_levels, action = clap::ArgAction::Set)]
    stream_auto_levels: bool,
    /// Stream gamma, 0.1-10: above 1 lifts shadows (viewing only). Live setting.
    #[arg(long, default_value_t = Settings::DEFAULT.stream_gamma, value_parser = limits::decimal(limits::STREAM_GAMMA))]
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
    #[arg(long, default_value_t = FRC_TAG_SIZE, value_parser = positive)]
    tag_size: f64,
    /// Field layout for poses: the name of a layout in a `fields` directory (searched in
    /// /usr/share/sonoma-vision, /usr/local/share/sonoma-vision, the working directory, then beside
    /// the executable), a WPILib layout JSON file, or `none` for no poses. Without it,
    /// 2026-rebuilt-welded if it's found, else none. Live setting.
    #[arg(long)]
    field_layout: Option<String>,
    /// The camera's pose on the robot, WPILib's robot-to-camera transform: x,y,z in metres (forward,
    /// left, up from the robot's origin), then roll,pitch,yaw in degrees. Live setting.
    #[arg(long, default_value_t = Settings::DEFAULT.robot_to_camera, allow_hyphen_values = true)]
    robot_to_camera: MountPose,
    /// Single-tag robot poses more ambiguous than this (0-1) aren't published. Live setting.
    #[arg(long, default_value_t = Settings::DEFAULT.max_ambiguity, value_parser = limits::decimal(limits::MAX_AMBIGUITY))]
    max_ambiguity: f64,
    /// Each corner coordinate's noise, in pixels, that pose covariances (and the published standard
    /// deviations) assume. Live setting.
    #[arg(long, default_value_t = Settings::DEFAULT.corner_noise_px, value_parser = limits::decimal(limits::CORNER_NOISE_PX))]
    corner_noise_px: f64,
    /// Robot poses more than this many metres off the floor aren't published. Live setting.
    #[arg(long, default_value_t = Settings::DEFAULT.max_height, value_parser = limits::decimal(limits::MAX_HEIGHT))]
    max_height: f64,
    /// Robot poses tilted more than this many degrees from level aren't published. Live setting.
    #[arg(long, default_value_t = Settings::DEFAULT.max_tilt, value_parser = limits::decimal(limits::MAX_TILT))]
    max_tilt: f64,

    /// Log level, or an env_logger filter such as `info,sonoma_vision::nt=debug`.
    #[arg(long, default_value = "info")]
    log_level: String,
}

/// A command line parser for a positive number.
fn positive(s: &str) -> Result<f64, String> {
    match s.parse::<f64>() {
        Ok(v) if v > 0.0 && v.is_finite() => Ok(v),
        Ok(_) => Err(format!("{s} is not positive")),
        Err(e) => Err(e.to_string()),
    }
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
        corner_noise_px: args.corner_noise_px,
        max_height: args.max_height,
        max_tilt: args.max_tilt,
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
    Detector::new(family, &vision::detector_config(&settings))?;
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
        // Live settings, which the vision thread fills in.
        layout: None,
        robot_to_camera: Default::default(),
        max_ambiguity: Settings::DEFAULT.max_ambiguity,
        corner_noise_px: Settings::DEFAULT.corner_noise_px,
        bounds: PoseBounds::DEFAULT,
    };
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
            vision::run(Setup { camera, family, pose, layouts }, settings_rx, state_tx, tx, tap, &stop)
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
    fn out_of_range_file_values_name_the_file_and_key() {
        let dir = std::env::temp_dir().join(format!("sonoma-range-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("cam0.toml");
        let cases = [
            ("[stream]\ngamma = 20\n", "[stream] gamma", "20 is not in 0.1..=10"),
            ("[stream]\nquality = 0\n", "[stream] quality", "0 is not in 1..=100"),
            ("[detector]\nsharpening = 3\n", "[detector] sharpening", "3 is not in 0..=1"),
            ("[detector]\nmax_hamming = 4\n", "[detector] max_hamming", "4 is not in 0..=3"),
            ("[pose]\ntag_size = 0\n", "[pose] tag_size", "0 is not positive"),
        ];
        for (text, key, why) in cases {
            std::fs::write(&file, text).unwrap();
            let err = FileValues::load(&Args::command(), std::slice::from_ref(&file), &[]).unwrap_err();
            let err = format!("{err:#}");
            assert!(err.contains("cam0.toml") && err.contains(key) && err.contains(why), "{err}");
        }
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn option_defaults_are_the_settings_defaults() {
        let args = Args::from_arg_matches(&Args::command().try_get_matches_from(["sv"]).unwrap()).unwrap();
        let d = Settings::DEFAULT;
        assert_eq!(
            (args.min_contrast, args.min_tag_px, args.max_hamming),
            (d.min_contrast, d.min_tag_px, d.max_hamming)
        );
        assert_eq!(
            (args.min_border_cells, args.sharpening, args.max_ambiguity),
            (d.min_border_cells, d.sharpening, d.max_ambiguity)
        );
        assert_eq!(args.robot_to_camera, d.robot_to_camera);
        assert_eq!(
            (
                args.stream_auto_levels,
                args.stream_gamma,
                args.stream_overlay,
                args.stream_quality,
                args.stream_fps,
                args.stream_scale
            ),
            (d.stream_auto_levels, d.stream_gamma, d.stream_overlay, d.stream_quality, d.stream_fps, d.stream_scale)
        );
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
            corner_noise_px: 0.5,
            max_height: 1.0,
            max_tilt: 10.0,
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
        assert_eq!((back.corner_noise_px, back.max_height, back.max_tilt), (0.5, 1.0, 10.0));
        assert_eq!(back.robot_to_camera, live.robot_to_camera);
        assert_eq!((back.stream_auto_levels, back.stream_gamma, back.stream_overlay), (false, 1.5, true));
        assert_eq!((back.stream_quality, back.stream_fps, back.stream_scale), (70, 30.0, 2));
        assert_eq!(back.field_layout.as_deref(), Some("2026-rebuilt-welded"));
        assert_eq!((back.width, back.height, back.fps, back.format), (640, 480, Some(120), Some(Layout::Mjpeg)));
    }
}
