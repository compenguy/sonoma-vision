//! Publishing detections to NetworkTables 4.
//!
//! Topics, under the configured table (e.g. `/sonoma/cam0`):
//!
//! - `frame` (double[]): one atomic record per processed frame,
//!   `[capture_time_us, sequence, latency_ms, n, then per tag: id, hamming, margin, x0, y0, x1, y1, x2, y2, x3, y3]`.
//!   `capture_time_us` is the frame's capture time on the NT server's clock (the robot's FPGA time on
//!   SystemCore), for `addVisionMeasurement`; `latency_ms` is capture to publish. The NT value itself
//!   is also timestamped with the capture time.
//! - `ids` (int[]): the tag IDs in the last frame (timestamped likewise), for dashboards.
//! - `fps` (double), `detect_ms` (double), `dropped` (int, frames the camera skipped since start).
//! - `tags/<id>/` for each tag ID seen, for dashboards: `visible` (boolean), `margin` (double, one
//!   decimal), `hamming` (int), `center_x` and `center_y` (double, pixels, one decimal), and `corners`
//!   (double[8], x0, y0 ... x3, y3). Values are timestamped with the capture time and kept when the
//!   tag leaves view; `visible` turns false. If a frame has a tag ID twice, the clearer one is used.
//!   With its pose solved, also `distance` (m), `bearing` (degrees, left positive), `ambiguity`, and
//!   `camera_to_tag` (struct Transform3d: the tag's pose in the camera's frame).
//! - With a field layout, the robot's pose when tags from it are in view: `robot_pose` (struct Pose3d,
//!   timestamped with the capture time, for `addVisionMeasurement`), `pose_tags` (int[], the tags used),
//!   `pose_error_px` and `pose_ambiguity` (double), and `field`, a WPILib Field2d (`Robot` is
//!   `[x, y, degrees]`) for dashboards' field widgets.
//!
//! With the camera stream on, it's also advertised as WPILib's CameraServer advertises streams, under
//! `/CameraPublisher/<name>` (`streams`, `source`, `description`, `connected`, `mode`, `modes`), so
//! dashboards list it. (Elastic only recognises a camera with all of them.)
//!
//! Settings that can be changed while running live under `<table>/settings/` (see
//! [`crate::settings::Key`]): camera exposure, gain and focus, the detector's thresholds, the pose
//! solver's mount and ambiguity limit, and the stream's look, rate and size. They're persistent topics,
//! so the robot's NT server keeps them across restarts. On connecting, values already on the server
//! win over the configuration's; settings the server doesn't have are published from ours. Edits from
//! a dashboard or the robot program apply immediately.
//!
//! The field layout and the capture mode are chosen under `<table>/settings/field_layout/` and
//! `<table>/settings/capture_mode/`, laid out as WPILib's `SendableChooser` (`.type` "String
//! Chooser", `options`, `default`, `active`, `selected`) so dashboards show them as drop-downs.
//! Writing an option to `selected` switches to it, and `active` is the one in use. `field_layout`
//! offers the layouts found and `none`, which turns pose solving off. `capture_mode` offers the
//! camera's modes (from the vision thread's [`CameraState`]); an empty selection is the
//! configuration's mode. `selected` is persistent and, like the other settings, the server's value
//! wins on connecting.
//!
//! With the camera stream on, `<table>/config_url` is where the configuration in effect is served.

use std::collections::hash_map::Entry;
use std::collections::{BTreeMap, HashMap, HashSet};
use std::net::Ipv4Addr;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Result, bail};
use log::{info, warn};
use nalgebra::Isometry3;
use nt_client::data::DataType;
use nt_client::error::ReconnectError;
use nt_client::math::{Pose3d, Quaternion, Rotation3d, Transform3d, Translation3d};
use nt_client::publish::{GenericPublisher, Publisher};
use nt_client::r#struct::{Struct, StructData, StructSchema};
use nt_client::subscribe::{ReceivedMessage, SubscriptionOptions};
use nt_client::topic::Properties;
use nt_client::{ClientHandle, NTAddr, NewClientOptions};
use tokio::sync::watch;

use crate::capture::{CameraState, Mode};
use crate::clock::monotonic_now;
use crate::detector::Detection;
use crate::pose::{FieldPose, TagPose};
use crate::settings::{Key, NO_LAYOUT, NtValue, Settings};

/// Values per tag in the `frame` record.
pub const TAG_FIELDS: usize = 11;
/// Values before the first tag in the `frame` record.
pub const HEADER_FIELDS: usize = 4;

/// What the vision thread reports for each processed frame.
#[derive(Clone, Debug, Default)]
pub struct FrameResult {
    pub sequence: u32,
    /// Capture time on CLOCK_MONOTONIC.
    pub capture: Duration,
    pub detections: Vec<Detection>,
    pub detect_time: Duration,
    pub fps: f64,
    pub dropped: u64,
    /// Each detection's pose relative to the camera, where it could be solved.
    pub tag_poses: Vec<Option<TagPose>>,
    /// The robot's pose on the field, if tags from the layout were in view.
    pub field_pose: Option<FieldPose>,
}

/// Where the NT server is: `local` (127.0.0.1, the robot program on the same controller), a team
/// number (10.TE.AM.2), or an IPv4 address.
pub fn parse_server(s: &str) -> Result<NTAddr> {
    if s.eq_ignore_ascii_case("local") || s == "localhost" {
        return Ok(NTAddr::Local);
    }
    if let Ok(team) = s.parse::<u16>() {
        return Ok(NTAddr::TeamNumber(team));
    }
    match s.parse::<Ipv4Addr>() {
        Ok(ip) => Ok(NTAddr::Custom(ip)),
        Err(_) => bail!("NT server must be `local`, a team number or an IPv4 address, not {s:?}"),
    }
}

pub struct NtConfig {
    pub server: NTAddr,
    pub port: u16,
    pub client_name: String,
    pub table: String,
    /// A camera stream to advertise, if one is running.
    pub camera: Option<CameraAdvert>,
    /// The field layouts that can be selected, by name.
    pub field_layouts: Vec<String>,
    /// The camera's modes and the one streaming, from the vision thread.
    pub camera_state: watch::Receiver<CameraState>,
}

/// A camera stream to advertise under `/CameraPublisher/<name>`.
#[derive(Clone, Debug)]
pub struct CameraAdvert {
    pub name: String,
    /// The stream's HTTP port; its URLs are built from this machine's addresses on each connection.
    pub port: u16,
    /// What the stream shows, e.g. `usb:/dev/video0`.
    pub source: String,
}

type BoxError = Box<dyn std::error::Error + Send + Sync>;

/// Publish every frame result from `results` until the channel closes, reconnecting as needed, and
/// keep `settings` in step with the settings in NT.
pub async fn run(
    cfg: NtConfig,
    results: watch::Receiver<Option<Arc<FrameResult>>>,
    settings: watch::Sender<Settings>,
) -> Result<()> {
    let settings = Arc::new(settings);
    let options = NewClientOptions {
        addr: cfg.server,
        unsecure_port: cfg.port,
        secure_port: None,
        name: cfg.client_name,
        ..Default::default()
    };
    let table = cfg.table.trim_end_matches('/').to_string();
    let camera = cfg.camera;
    let camera_state = cfg.camera_state;
    let chooser =
        Arc::new(Chooser { options: cfg.field_layouts, default: settings.borrow().field_layout_name().into() });
    let mut attempt = 0u32;
    let outcome = nt_client::reconnect(options, |client| {
        let table = table.clone();
        let camera = camera.clone();
        let camera_state = camera_state.clone();
        let results = results.clone();
        let settings = settings.clone();
        let chooser = chooser.clone();
        attempt += 1;
        let delay = if attempt > 1 { Duration::from_secs(1) } else { Duration::ZERO };
        async move {
            tokio::time::sleep(delay).await;
            let handle = client.handle().clone();
            let advertise = camera.map(|c| {
                let (settings, state) = (settings.subscribe(), camera_state.clone());
                tokio::spawn(advertise_loop(handle.clone(), table.clone(), c, settings, state))
            });
            let sync = tokio::spawn(sync_settings(
                handle.clone(),
                format!("{table}/settings"),
                settings,
                chooser,
                camera_state,
            ));
            let publish = tokio::spawn(publish_loop(handle, table, results));
            // Dropping a spawned task's handle doesn't stop it: abort the others with the connection.
            let _sync = AbortOnDrop(sync.abort_handle());
            let _advertise = advertise.as_ref().map(|a| AbortOnDrop(a.abort_handle()));
            let advertised = async {
                match advertise {
                    Some(a) => a.await,
                    None => std::future::pending().await,
                }
            };
            tokio::select! {
                res = client.connect() => {
                    warn!("NetworkTables connection ended: {res:?}");
                    Err(ReconnectError::Nonfatal(match res {
                        Ok(()) => "server closed the connection".into(),
                        Err(e) => e.into(),
                    }))
                }
                res = publish => match res {
                    // The vision thread has stopped: shut down.
                    Ok(Ok(())) => Ok(()),
                    Ok(Err(e)) => Err(ReconnectError::Nonfatal(e)),
                    Err(e) => Err(ReconnectError::Fatal(e.into())),
                },
                res = sync => match res {
                    Ok(Err(e)) => Err(ReconnectError::Nonfatal(e)),
                    Ok(Ok(())) => Err(ReconnectError::Nonfatal("settings subscription ended".into())),
                    Err(e) => Err(ReconnectError::Fatal(e.into())),
                },
                res = advertised => match res {
                    Ok(Err(e)) => Err(ReconnectError::Nonfatal(e)),
                    Ok(Ok(())) => Err(ReconnectError::Nonfatal("camera advertisement ended".into())),
                    Err(e) => Err(ReconnectError::Fatal(e.into())),
                },
            }
        }
    })
    .await;
    outcome.map_err(|e| anyhow::anyhow!(e))
}

struct Publishers {
    frame: Publisher<Vec<f64>>,
    ids: Publisher<Vec<i64>>,
    fps: Publisher<f64>,
    detect_ms: Publisher<f64>,
    dropped: Publisher<i64>,
    robot_pose: Publisher<Struct<Pose3d>>,
    pose_tags: Publisher<Vec<i64>>,
    pose_error_px: Publisher<f64>,
    pose_ambiguity: Publisher<f64>,
    field_type: Publisher<String>,
    field_robot: Publisher<Vec<f64>>,
}

/// Publish results as they arrive. Returns Ok when the vision thread has gone away.
async fn publish_loop(
    client: ClientHandle,
    table: String,
    mut results: watch::Receiver<Option<Arc<FrameResult>>>,
) -> Result<(), BoxError> {
    let topic = |name: &str| client.topic(format!("{table}/{name}"));
    // Struct values need their schemas on the server first.
    tokio::try_join!(
        publish_schema::<Translation3d>(&client),
        publish_schema::<Quaternion>(&client),
        publish_schema::<Rotation3d>(&client),
        publish_schema::<Pose3d>(&client),
        publish_schema::<Transform3d>(&client),
    )?;
    let p = Publishers {
        frame: topic("frame").publish_bypass(Default::default()).await?,
        ids: topic("ids").publish_bypass(Default::default()).await?,
        fps: topic("fps").publish_bypass(Default::default()).await?,
        detect_ms: topic("detect_ms").publish_bypass(Default::default()).await?,
        dropped: topic("dropped").publish_bypass(Default::default()).await?,
        robot_pose: topic("robot_pose").publish_bypass(Default::default()).await?,
        pose_tags: topic("pose_tags").publish_bypass(Default::default()).await?,
        pose_error_px: topic("pose_error_px").publish_bypass(Default::default()).await?,
        pose_ambiguity: topic("pose_ambiguity").publish_bypass(Default::default()).await?,
        field_type: topic("field/.type").publish_bypass(Default::default()).await?,
        field_robot: topic("field/Robot").publish_bypass(Default::default()).await?,
    };
    p.field_type.set("Field2d".to_string()).await?;
    info!("publishing to {table}");
    let time = client.time();
    let mut tags: HashMap<u32, TagPublishers> = HashMap::new();
    loop {
        if results.changed().await.is_err() {
            return Ok(());
        }
        let Some(r) = results.borrow_and_update().clone() else { continue };
        // Map the capture time onto the server's clock: now on the server, minus how long ago the
        // frame was captured on ours.
        let server_now = time.read().await.server_time();
        let age = monotonic_now().saturating_sub(r.capture);
        let Some(capture_server) = server_now.checked_sub(age) else { continue };
        // Stamped with the capture time, so a robot reading the value's timestamp gets it directly.
        p.frame.set_at(frame_record(&r, capture_server, age), capture_server).await?;
        p.ids.set_at(r.detections.iter().map(|d| i64::from(d.id)).collect(), capture_server).await?;
        p.fps.set(r.fps).await?;
        p.detect_ms.set(r.detect_time.as_secs_f64() * 1e3).await?;
        p.dropped.set(r.dropped as i64).await?;

        if let Some(f) = &r.field_pose {
            p.robot_pose.set_at(pose3d(&f.robot).into_struct_data(), capture_server).await?;
            p.pose_tags.set_at(f.tags.iter().map(|&id| i64::from(id)).collect(), capture_server).await?;
            p.pose_error_px.set_at(f.error_px, capture_server).await?;
            p.pose_ambiguity.set_at(f.ambiguity, capture_server).await?;
            let (_, _, yaw) = f.robot.rotation.euler_angles();
            let t = f.robot.translation.vector;
            p.field_robot.set_at(vec![t.x, t.y, yaw.to_degrees()], capture_server).await?;
        }

        // Per tag ID, the clearest sighting.
        let mut seen: BTreeMap<u32, usize> = BTreeMap::new();
        for (i, d) in r.detections.iter().enumerate() {
            let best = seen.entry(d.id).or_insert(i);
            if d.decision_margin > r.detections[*best].decision_margin {
                *best = i;
            }
        }
        for (&id, &i) in &seen {
            let tag = match tags.entry(id) {
                Entry::Occupied(e) => e.into_mut(),
                Entry::Vacant(e) => e.insert(TagPublishers::new(&client, &table, id).await?),
            };
            let pose = r.tag_poses.get(i).copied().flatten();
            tag.show(&r.detections[i], pose.as_ref(), capture_server).await?;
        }
        for (id, t) in &mut tags {
            if t.visible && !seen.contains_key(id) {
                t.visible = false;
                t.p_visible.set_at(false, capture_server).await?;
            }
        }
    }
}

/// One tag ID's topics, under `<table>/tags/<id>/`.
struct TagPublishers {
    /// Whether the tag was in the last frame.
    visible: bool,
    p_visible: Publisher<bool>,
    margin: Publisher<f64>,
    hamming: Publisher<i64>,
    center_x: Publisher<f64>,
    center_y: Publisher<f64>,
    corners: Publisher<Vec<f64>>,
    distance: Publisher<f64>,
    bearing: Publisher<f64>,
    ambiguity: Publisher<f64>,
    camera_to_tag: Publisher<Struct<Transform3d>>,
}

impl TagPublishers {
    async fn new(client: &ClientHandle, table: &str, id: u32) -> Result<Self, BoxError> {
        let topic = |name: &str| client.topic(format!("{table}/tags/{id}/{name}"));
        let names = ["visible", "margin", "hamming", "center_x", "center_y", "corners"];
        let [t_visible, t_margin, t_hamming, t_x, t_y, t_corners] = names.map(topic);
        let [t_distance, t_bearing, t_ambiguity, t_camera_to_tag] =
            ["distance", "bearing", "ambiguity", "camera_to_tag"].map(topic);
        // Together, so a new tag holds up publishing for one round trip, not ten.
        let ((p_visible, margin, hamming, center_x, center_y, corners), (distance, bearing, ambiguity, camera_to_tag)) =
            tokio::try_join!(
                async {
                    tokio::try_join!(
                        t_visible.publish(Default::default()),
                        t_margin.publish(Default::default()),
                        t_hamming.publish(Default::default()),
                        t_x.publish(Default::default()),
                        t_y.publish(Default::default()),
                        t_corners.publish(Default::default()),
                    )
                },
                async {
                    tokio::try_join!(
                        t_distance.publish(Default::default()),
                        t_bearing.publish(Default::default()),
                        t_ambiguity.publish(Default::default()),
                        t_camera_to_tag.publish(Default::default()),
                    )
                },
            )?;
        Ok(Self {
            visible: false,
            p_visible,
            margin,
            hamming,
            center_x,
            center_y,
            corners,
            distance,
            bearing,
            ambiguity,
            camera_to_tag,
        })
    }

    async fn show(&mut self, d: &Detection, pose: Option<&TagPose>, at: Duration) -> Result<(), BoxError> {
        // One decimal: steadier to read on a dashboard, and finer than the detector's accuracy matters.
        let tenth = |v: f64| (v * 10.0).round() / 10.0;
        let [cx, cy] = d.center();
        self.margin.set_at(tenth(f64::from(d.decision_margin)), at).await?;
        self.hamming.set_at(i64::from(d.hamming), at).await?;
        self.center_x.set_at(tenth(cx), at).await?;
        self.center_y.set_at(tenth(cy), at).await?;
        self.corners.set_at(d.corners.iter().flatten().copied().collect(), at).await?;
        if let Some(pose) = pose {
            let t = pose.camera_to_tag.translation.vector;
            self.distance.set_at((t.norm() * 100.0).round() / 100.0, at).await?;
            self.bearing.set_at(tenth(t.y.atan2(t.x).to_degrees()), at).await?;
            self.ambiguity.set_at((pose.ambiguity * 100.0).round() / 100.0, at).await?;
            self.camera_to_tag.set_at(transform3d(&pose.camera_to_tag).into_struct_data(), at).await?;
        }
        if !self.visible {
            self.visible = true;
            self.p_visible.set_at(true, at).await?;
        }
        Ok(())
    }
}

/// Publish `T`'s struct schema (not its nested structs': each is published on its own). nt_client's
/// schema manager republishes shared nested schemas, and a server announces a topic to a client only
/// once per connection, so its second publish of one would wait forever for the announcement.
async fn publish_schema<T: StructData>(client: &ClientHandle) -> Result<(), BoxError> {
    let properties = Properties { retained: Some(true), ..Default::default() };
    let publisher = client.struct_schema_topic::<T>().publish_bypass::<StructSchema>(properties).await?;
    publisher.set_default(T::schema()).await?;
    Ok(())
}

fn translation3d(p: &Isometry3<f64>) -> Translation3d {
    let t = p.translation.vector;
    Translation3d { x: t.x, y: t.y, z: t.z }
}

fn rotation3d(p: &Isometry3<f64>) -> Rotation3d {
    let q = p.rotation.quaternion();
    Rotation3d { quaternion: Quaternion { w: q.w, x: q.i, y: q.j, z: q.k } }
}

fn pose3d(p: &Isometry3<f64>) -> Pose3d {
    Pose3d { translation: translation3d(p), rotation: rotation3d(p) }
}

fn transform3d(p: &Isometry3<f64>) -> Transform3d {
    Transform3d { translation: translation3d(p), rotation: rotation3d(p) }
}

struct CameraPublishers {
    streams: Publisher<Vec<String>>,
    source: Publisher<String>,
    description: Publisher<String>,
    connected: Publisher<bool>,
    mode: Publisher<String>,
    modes: Publisher<Vec<String>>,
}

/// The stream's video mode as CameraServer writes it (`320x240 MJPEG 15 fps`): the camera's mode
/// scaled down, at the stream's rate if that's lower. None while the camera is closed.
pub fn stream_mode(state: &CameraState, s: &Settings) -> Option<String> {
    let m = state.active?;
    let scale = u32::from(s.stream_scale.max(1));
    let fps = if m.fps == 0 { s.stream_fps } else { s.stream_fps.min(f64::from(m.fps)) };
    Some(format!("{}x{} MJPEG {} fps", m.width / scale, m.height / scale, fps.round()))
}

/// Advertise the camera stream for one connection, keeping its mode in step with the camera and the
/// stream settings, and publish `<table>/config_url`.
async fn advertise_loop(
    client: ClientHandle,
    table: String,
    c: CameraAdvert,
    mut settings: watch::Receiver<Settings>,
    mut state: watch::Receiver<CameraState>,
) -> Result<(), BoxError> {
    let topic = |name: &str| client.topic(format!("/CameraPublisher/{}/{name}", c.name));
    let urls = crate::stream::stream_urls(c.port);
    info!("advertising camera stream {:?}: {}", c.name, urls.join(", "));
    let p = CameraPublishers {
        streams: topic("streams").publish_bypass(Default::default()).await?,
        source: topic("source").publish_bypass(Default::default()).await?,
        description: topic("description").publish_bypass(Default::default()).await?,
        connected: topic("connected").publish_bypass(Default::default()).await?,
        mode: topic("mode").publish_bypass(Default::default()).await?,
        modes: topic("modes").publish_bypass(Default::default()).await?,
    };
    let config_url: Publisher<String> =
        client.topic(format!("{table}/config_url")).publish_bypass(Default::default()).await?;
    p.streams.set(urls).await?;
    p.source.set(c.source.clone()).await?;
    p.description.set("sonoma-vision AprilTag camera".to_string()).await?;
    p.connected.set(true).await?;
    config_url.set(crate::stream::config_url(c.port)).await?;
    let mut shown = None;
    let (mut settings_open, mut state_open) = (true, true);
    loop {
        let mode = stream_mode(&state.borrow_and_update(), &settings.borrow_and_update());
        if mode.is_some() && mode != shown {
            let m = mode.clone().unwrap_or_default();
            p.mode.set(m.clone()).await?;
            p.modes.set(vec![m]).await?;
            shown = mode;
        }
        tokio::select! {
            r = settings.changed(), if settings_open => settings_open = r.is_ok(),
            r = state.changed(), if state_open => state_open = r.is_ok(),
            // Both gone: the service is shutting down; hold the advertisement until it does.
            () = std::future::pending(), if !settings_open && !state_open => {}
        }
    }
}

struct AbortOnDrop(tokio::task::AbortHandle);

impl Drop for AbortOnDrop {
    fn drop(&mut self) {
        self.0.abort();
    }
}

/// The field layout chooser's fixed parts.
struct Chooser {
    /// The layouts that can be selected (without `none`).
    options: Vec<String>,
    /// What the configuration chose.
    default: String,
}

/// A `SendableChooser`'s topics, under `<settings>/<name>/`.
struct ChooserPublishers {
    _type: Publisher<String>,
    _controllable: Publisher<bool>,
    options: Publisher<Vec<String>>,
    default: Publisher<String>,
    active: Publisher<String>,
    selected: Publisher<String>,
}

impl ChooserPublishers {
    async fn new(client: &ClientHandle, prefix: &str, options: Vec<String>, default: String) -> Result<Self, BoxError> {
        let topic = |name: &str| client.topic(format!("{prefix}/{name}"));
        let persistent = Properties { persistent: Some(true), ..Default::default() };
        let p = Self {
            _type: topic(".type").publish_bypass(Default::default()).await?,
            _controllable: topic(".controllable").publish_bypass(Default::default()).await?,
            options: topic("options").publish_bypass(Default::default()).await?,
            default: topic("default").publish_bypass(Default::default()).await?,
            active: topic("active").publish_bypass(Default::default()).await?,
            selected: topic("selected").publish_bypass(persistent).await?,
        };
        p._type.set("String Chooser".to_string()).await?;
        p._controllable.set(true).await?;
        p.options.set(options).await?;
        p.default.set(default).await?;
        Ok(p)
    }
}

/// The capture mode chooser's options, default and active mode, as published.
fn mode_chooser(state: &CameraState) -> (Vec<String>, String, String) {
    let name = |m: Option<Mode>| m.map(|m| m.to_string()).unwrap_or_default();
    (state.modes.iter().map(Mode::to_string).collect(), name(state.default), name(state.active))
}

/// What a message under the settings prefix was.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Setting {
    Key(Key),
    FieldLayout,
    CaptureMode,
}

/// The choosers' `selected` topics.
struct Selected {
    field_layout: String,
    capture_mode: String,
}

/// Keep `settings` and the persistent topics under `prefix` in step for one connection: adopt what
/// the server already has, publish the rest, then apply every change made in NT.
async fn sync_settings(
    client: ClientHandle,
    prefix: String,
    settings: Arc<watch::Sender<Settings>>,
    chooser: Arc<Chooser>,
    mut camera: watch::Receiver<CameraState>,
) -> Result<(), BoxError> {
    // Subscribe first, so values the server already has arrive while we publish.
    let options = SubscriptionOptions { prefix: Some(true), ..Default::default() };
    let mut sub = client.topic(format!("{prefix}/")).subscribe(options).await?;
    let properties = Properties { persistent: Some(true), ..Default::default() };
    let mut publishers = Vec::with_capacity(Key::ALL.len());
    for key in Key::ALL {
        let data_type = match settings.borrow().get(key) {
            NtValue::Int(_) => DataType::Int,
            NtValue::Bool(_) => DataType::Boolean,
            NtValue::Double(_) => DataType::Double,
            NtValue::DoubleArray(_) => DataType::DoubleArray,
        };
        let topic = client.topic(format!("{prefix}/{}", key.name()));
        publishers.push((key, topic.generic_publish_bypass(data_type, properties.clone()).await?));
    }
    let mut layouts = chooser.options.clone();
    layouts.push(NO_LAYOUT.to_string());
    let layout =
        ChooserPublishers::new(&client, &format!("{prefix}/field_layout"), layouts, chooser.default.clone()).await?;
    let (modes, default_mode, active_mode) = mode_chooser(&camera.borrow_and_update());
    let capture = ChooserPublishers::new(&client, &format!("{prefix}/capture_mode"), modes, default_mode).await?;
    let selected = Selected {
        field_layout: format!("{prefix}/field_layout/selected"),
        capture_mode: format!("{prefix}/capture_mode/selected"),
    };
    let apply = |msg| apply_setting(&settings, &selected, &chooser.options, msg);

    // Take in what the server sent while we were publishing (its values arrive right after we
    // subscribe), then publish ours for the settings it doesn't have.
    let mut adopted = HashSet::new();
    while let Ok(msg) = tokio::time::timeout(Duration::from_millis(100), sub.recv()).await {
        if let Some(setting) = apply(msg?) {
            adopted.insert(setting);
        }
    }
    let current = settings.borrow().clone();
    for (key, publisher) in &publishers {
        if !adopted.contains(&Setting::Key(*key)) {
            set_generic(publisher, current.get(*key)).await?;
        }
    }
    if !adopted.contains(&Setting::FieldLayout) {
        layout.selected.set(current.field_layout_name().to_string()).await?;
    }
    if !adopted.contains(&Setting::CaptureMode) {
        // Empty: the configuration's mode. A mode chosen live persists until it's cleared.
        let mode = current.capture_mode.map(|m| m.to_string()).unwrap_or_default();
        capture.selected.set(mode).await?;
    }
    layout.active.set(current.field_layout_name().to_string()).await?;
    capture.active.set(active_mode).await?;
    info!("settings under {prefix}: {current:?}");

    let mut camera_open = true;
    loop {
        tokio::select! {
            msg = sub.recv() => match msg {
                Ok(msg) => {
                    if apply(msg) == Some(Setting::FieldLayout) {
                        let active = settings.borrow().field_layout_name().to_string();
                        layout.active.set(active).await?;
                    }
                }
                // Only the latest value of each setting matters.
                Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                Err(e) => return Err(e.into()),
            },
            // The capture mode's `active` follows the camera, which reopens in the chosen mode.
            r = camera.changed(), if camera_open => {
                camera_open = r.is_ok();
                let (modes, default, active) = mode_chooser(&camera.borrow_and_update());
                capture.options.set(modes).await?;
                capture.default.set(default).await?;
                capture.active.set(active).await?;
            }
        }
    }
}

/// Apply a setting update received from NT (`selected` being the choosers' selections; field layouts
/// one of `layouts`), returning which setting it was.
fn apply_setting(
    settings: &watch::Sender<Settings>,
    selected: &Selected,
    layouts: &[String],
    msg: ReceivedMessage,
) -> Option<Setting> {
    let ReceivedMessage::Updated((topic, value)) = msg else { return None };
    let (setting, name) = if topic.name() == selected.field_layout {
        (Setting::FieldLayout, "field_layout")
    } else if topic.name() == selected.capture_mode {
        (Setting::CaptureMode, "capture_mode")
    } else {
        let key = Key::from_name(topic.name().rsplit('/').next()?)?;
        (Setting::Key(key), key.name())
    };
    let mut result = Ok(());
    settings.send_if_modified(|s| {
        let before = s.clone();
        result = match (setting, value.as_str()) {
            (Setting::Key(key), _) => s.set(key, &value),
            (Setting::FieldLayout, Some(v)) => s.set_field_layout(v, layouts),
            (Setting::CaptureMode, Some(v)) => s.set_capture_mode(v),
            (_, None) => Err(format!("{name} must be a string, not {value}")),
        };
        *s != before
    });
    match result {
        Ok(()) => info!("setting {name} = {value}"),
        Err(e) => warn!("ignoring setting: {e}"),
    }
    Some(setting)
}

async fn set_generic(publisher: &GenericPublisher, value: NtValue) -> Result<(), BoxError> {
    match value {
        NtValue::Int(v) => publisher.set(v).await?,
        NtValue::Bool(v) => publisher.set(v).await?,
        NtValue::Double(v) => publisher.set(v).await?,
        NtValue::DoubleArray(v) => publisher.set(v).await?,
    }
    Ok(())
}

/// The packed `frame` record (see the module docs).
pub fn frame_record(r: &FrameResult, capture_server: Duration, age: Duration) -> Vec<f64> {
    let mut v = Vec::with_capacity(HEADER_FIELDS + TAG_FIELDS * r.detections.len());
    v.extend([
        capture_server.as_micros() as f64,
        f64::from(r.sequence),
        age.as_secs_f64() * 1e3,
        r.detections.len() as f64,
    ]);
    for d in &r.detections {
        v.extend([f64::from(d.id), f64::from(d.hamming), f64::from(d.decision_margin)]);
        v.extend(d.corners.iter().flatten());
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn record_layout() {
        let d = Detection {
            id: 7,
            hamming: 1,
            decision_margin: 50.0,
            corners: [[1.0, 2.0], [3.0, 4.0], [5.0, 6.0], [7.0, 8.0]],
        };
        let r = FrameResult { sequence: 9, detections: vec![d], ..Default::default() };
        let v = frame_record(&r, Duration::from_micros(1_234_567), Duration::from_millis(12));
        assert_eq!(v.len(), HEADER_FIELDS + TAG_FIELDS);
        assert_eq!(&v[..4], &[1_234_567.0, 9.0, 12.0, 1.0]);
        assert_eq!(&v[4..], &[7.0, 1.0, 50.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]);
    }

    #[test]
    fn servers() {
        assert!(matches!(parse_server("local").unwrap(), NTAddr::Local));
        assert!(matches!(parse_server("9567").unwrap(), NTAddr::TeamNumber(9567)));
        assert!(matches!(parse_server("10.95.67.2").unwrap(), NTAddr::Custom(_)));
        assert!(parse_server("robot.local").is_err());
    }
}
