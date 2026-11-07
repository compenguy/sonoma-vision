//! The vision thread: capture frames, detect tags, solve poses and hand the results to the publisher,
//! applying live settings as they change and reopening the camera when it fails or the capture mode
//! changes.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use log::{error, info, warn};
use tokio::sync::watch;

use crate::capture::{Camera, CameraConfig, CameraState};
use crate::detector::{Config, Detector, Family};
use crate::nt::FrameResult;
use crate::pose::field::FieldLayout;
use crate::pose::{Estimator, PoseBounds, PoseConfig};
use crate::settings::Settings;
use crate::stream::Tap;

/// What the vision thread works with, fixed at startup.
pub struct Setup {
    pub camera: CameraConfig,
    pub family: &'static Family,
    pub pose: PoseConfig,
    /// The field layouts that can be selected, by name.
    pub layouts: BTreeMap<String, FieldLayout>,
}

/// The live settings the pose estimator is built from.
fn pose_settings(s: &Settings) -> impl PartialEq + '_ {
    (&s.field_layout, s.robot_to_camera, s.max_ambiguity, s.corner_noise_px, s.max_height, s.max_tilt)
}

/// The startup pose configuration with the live settings applied (all but the layout, which
/// [`build_estimator`] looks up).
fn pose_config(pose: &PoseConfig, s: &Settings) -> PoseConfig {
    PoseConfig {
        robot_to_camera: s.robot_to_camera.isometry(),
        max_ambiguity: s.max_ambiguity,
        corner_noise_px: s.corner_noise_px,
        bounds: PoseBounds { max_height: s.max_height, max_tilt_deg: s.max_tilt, ..pose.bounds },
        ..pose.clone()
    }
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

/// Capture and detect until `stop`, reopening the camera after errors (e.g. a USB unplug) or to change
/// the capture mode, and apply live settings as they change.
pub fn run(
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
        if camera.layout.decoded() {
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
                let pose_changed = |old: &Settings| pose_settings(old) != pose_settings(&new);
                if applied.as_ref().is_none_or(pose_changed) {
                    let live = pose_config(&pose, &new);
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
            let (tag_poses, field_pose, pose_rejected) = match &estimator {
                Some(e) => {
                    let tags: Vec<_> = detections.iter().map(|d| e.tag(d)).collect();
                    match e.field(&detections, &tags) {
                        Ok(field) => (tags, field, None),
                        Err(why) => (tags, None, Some(why)),
                    }
                }
                None => (vec![None; detections.len()], None, None),
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
                pose_rejected,
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
pub fn detector_config(s: &Settings) -> Config {
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
