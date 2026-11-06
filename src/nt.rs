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
//!
//! With the camera stream on, it's also advertised as WPILib's CameraServer advertises streams, under
//! `/CameraPublisher/<name>` (`streams`, `source`, `description`, `connected`, `mode`, `modes`), so
//! dashboards list it. (Elastic only recognises a camera with all of them.)
//!
//! Settings that can be changed while running live under `<table>/settings/` (see
//! [`crate::settings::Key`]): `exposure`, `gain`, `min_contrast`, `stream_auto_levels`, `stream_gamma`.
//! They're persistent topics, so the robot's NT server keeps them across restarts. On connecting,
//! values already on the server win over the command line's; settings the server doesn't have are
//! published from ours. Edits from a dashboard or the robot program apply immediately.

use std::collections::hash_map::Entry;
use std::collections::{BTreeMap, HashMap, HashSet};
use std::net::Ipv4Addr;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Result, bail};
use log::{info, warn};
use nt_client::data::DataType;
use nt_client::error::ReconnectError;
use nt_client::publish::{GenericPublisher, Publisher};
use nt_client::subscribe::{ReceivedMessage, SubscriptionOptions};
use nt_client::topic::Properties;
use nt_client::{ClientHandle, NTAddr, NewClientOptions};
use tokio::sync::watch;

use crate::clock::monotonic_now;
use crate::detector::Detection;
use crate::settings::{Key, NtValue, Settings};

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
}

/// A camera stream to advertise under `/CameraPublisher/<name>`.
#[derive(Clone, Debug)]
pub struct CameraAdvert {
    pub name: String,
    /// The stream's HTTP port; its URLs are built from this machine's addresses on each connection.
    pub port: u16,
    /// What the stream shows, e.g. `usb:/dev/video0`.
    pub source: String,
    /// The stream's video mode, as CameraServer writes it: `320x240 MJPEG 15 fps`.
    pub mode: String,
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
    let mut attempt = 0u32;
    let outcome = nt_client::reconnect(options, |client| {
        let table = table.clone();
        let camera = camera.clone();
        let results = results.clone();
        let settings = settings.clone();
        attempt += 1;
        let delay = if attempt > 1 { Duration::from_secs(1) } else { Duration::ZERO };
        async move {
            tokio::time::sleep(delay).await;
            let sync = tokio::spawn(sync_settings(client.handle().clone(), format!("{table}/settings"), settings));
            let publish = tokio::spawn(publish_loop(client.handle().clone(), table, camera, results));
            // Dropping a spawned task's handle doesn't stop it: abort the settings task with the connection.
            let _sync = AbortOnDrop(sync.abort_handle());
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
}

/// Publish results as they arrive. Returns Ok when the vision thread has gone away.
async fn publish_loop(
    client: ClientHandle,
    table: String,
    camera: Option<CameraAdvert>,
    mut results: watch::Receiver<Option<Arc<FrameResult>>>,
) -> Result<(), BoxError> {
    let topic = |name: &str| client.topic(format!("{table}/{name}"));
    let p = Publishers {
        frame: topic("frame").publish_bypass(Default::default()).await?,
        ids: topic("ids").publish_bypass(Default::default()).await?,
        fps: topic("fps").publish_bypass(Default::default()).await?,
        detect_ms: topic("detect_ms").publish_bypass(Default::default()).await?,
        dropped: topic("dropped").publish_bypass(Default::default()).await?,
    };
    info!("publishing to {table}");
    // Held for the connection: the advertisement lasts as long as its publishers.
    let _camera = match &camera {
        Some(c) => Some(advertise_camera(&client, c).await?),
        None => None,
    };
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

        // Per tag ID, the clearest sighting.
        let mut seen: BTreeMap<u32, &Detection> = BTreeMap::new();
        for d in &r.detections {
            let best = seen.entry(d.id).or_insert(d);
            if d.decision_margin > best.decision_margin {
                *best = d;
            }
        }
        for (&id, d) in &seen {
            let tag = match tags.entry(id) {
                Entry::Occupied(e) => e.into_mut(),
                Entry::Vacant(e) => e.insert(TagPublishers::new(&client, &table, id).await?),
            };
            tag.show(d, capture_server).await?;
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
}

impl TagPublishers {
    async fn new(client: &ClientHandle, table: &str, id: u32) -> Result<Self, BoxError> {
        let topic = |name: &str| client.topic(format!("{table}/tags/{id}/{name}"));
        let (t_visible, t_margin, t_hamming, t_x, t_y, t_corners) = (
            topic("visible"),
            topic("margin"),
            topic("hamming"),
            topic("center_x"),
            topic("center_y"),
            topic("corners"),
        );
        // Together, so a new tag holds up publishing for one round trip, not six.
        let (p_visible, margin, hamming, center_x, center_y, corners) = tokio::try_join!(
            t_visible.publish(Default::default()),
            t_margin.publish(Default::default()),
            t_hamming.publish(Default::default()),
            t_x.publish(Default::default()),
            t_y.publish(Default::default()),
            t_corners.publish(Default::default()),
        )?;
        Ok(Self { visible: false, p_visible, margin, hamming, center_x, center_y, corners })
    }

    async fn show(&mut self, d: &Detection, at: Duration) -> Result<(), BoxError> {
        // One decimal: steadier to read on a dashboard, and finer than the detector's accuracy matters.
        let tenth = |v: f64| (v * 10.0).round() / 10.0;
        let [cx, cy] = d.center();
        self.margin.set_at(tenth(f64::from(d.decision_margin)), at).await?;
        self.hamming.set_at(i64::from(d.hamming), at).await?;
        self.center_x.set_at(tenth(cx), at).await?;
        self.center_y.set_at(tenth(cy), at).await?;
        self.corners.set_at(d.corners.iter().flatten().copied().collect(), at).await?;
        if !self.visible {
            self.visible = true;
            self.p_visible.set_at(true, at).await?;
        }
        Ok(())
    }
}

struct CameraPublishers {
    streams: Publisher<Vec<String>>,
    source: Publisher<String>,
    description: Publisher<String>,
    connected: Publisher<bool>,
    mode: Publisher<String>,
    modes: Publisher<Vec<String>>,
}

async fn advertise_camera(client: &ClientHandle, c: &CameraAdvert) -> Result<CameraPublishers, BoxError> {
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
    p.streams.set(urls).await?;
    p.source.set(c.source.clone()).await?;
    p.description.set("sonoma-vision AprilTag camera".to_string()).await?;
    p.connected.set(true).await?;
    p.mode.set(c.mode.clone()).await?;
    p.modes.set(vec![c.mode.clone()]).await?;
    Ok(p)
}

struct AbortOnDrop(tokio::task::AbortHandle);

impl Drop for AbortOnDrop {
    fn drop(&mut self) {
        self.0.abort();
    }
}

/// Keep `settings` and the persistent topics under `prefix` in step for one connection: adopt what
/// the server already has, publish the rest, then apply every change made in NT.
async fn sync_settings(
    client: ClientHandle,
    prefix: String,
    settings: Arc<watch::Sender<Settings>>,
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
        };
        let topic = client.topic(format!("{prefix}/{}", key.name()));
        publishers.push((key, topic.generic_publish_bypass(data_type, properties.clone()).await?));
    }

    // Take in what the server sent while we were publishing (its values arrive right after we
    // subscribe), then publish ours for the settings it doesn't have.
    let mut adopted = HashSet::new();
    while let Ok(msg) = tokio::time::timeout(Duration::from_millis(100), sub.recv()).await {
        if let Some(key) = apply_setting(&settings, msg?) {
            adopted.insert(key);
        }
    }
    let current = settings.borrow().clone();
    for (key, publisher) in &publishers {
        if !adopted.contains(key) {
            set_generic(publisher, current.get(*key)).await?;
        }
    }
    info!("settings under {prefix}: {current:?}");

    loop {
        match sub.recv().await {
            Ok(msg) => {
                apply_setting(&settings, msg);
            }
            // Only the latest value of each setting matters.
            Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
            Err(e) => return Err(e.into()),
        }
    }
}

/// Apply a setting update received from NT, returning which setting it was.
fn apply_setting(settings: &watch::Sender<Settings>, msg: ReceivedMessage) -> Option<Key> {
    let ReceivedMessage::Updated((topic, value)) = msg else { return None };
    let key = Key::from_name(topic.name().rsplit('/').next()?)?;
    let mut result = Ok(());
    settings.send_if_modified(|s| {
        let before = s.clone();
        result = s.set(key, &value);
        *s != before
    });
    match result {
        Ok(()) => info!("setting {} = {value}", key.name()),
        Err(e) => warn!("ignoring setting: {e}"),
    }
    Some(key)
}

async fn set_generic(publisher: &GenericPublisher, value: NtValue) -> Result<(), BoxError> {
    match value {
        NtValue::Int(v) => publisher.set(v).await?,
        NtValue::Bool(v) => publisher.set(v).await?,
        NtValue::Double(v) => publisher.set(v).await?,
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
