//! End to end: publish a frame result to a minimal in-process NT4 server and check what arrives.

use std::collections::HashMap;
use std::net::Ipv4Addr;
use std::sync::Arc;
use std::time::{Duration, Instant};

use futures_util::{SinkExt, StreamExt};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use rmpv::Value;
use sonoma_vision::capture::{CameraState, Layout, Mode};
use sonoma_vision::clock::monotonic_now;
use sonoma_vision::detector::Detection;
use sonoma_vision::nt::{self, CameraAdvert, FrameResult, NtConfig};
use sonoma_vision::pose::{FieldPose, TagPose};
use tokio::net::TcpListener;
use tokio::sync::{mpsc, watch};
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::tungstenite::handshake::server::{Request, Response};

/// The fake server's clock runs this far ahead of its start, so a missing time sync shows up.
const SERVER_EPOCH_US: u64 = 5_000_000_000;

#[allow(clippy::result_large_err)] // the handshake callback's error type is tungstenite's
/// Accept one NT4 client: answer time sync, record values as (topic, receipt time, value timestamp, value).
async fn fake_server(listener: TcpListener, values: mpsc::UnboundedSender<(String, u64, u64, Value)>) {
    let (stream, _) = listener.accept().await.unwrap();
    stream.set_nodelay(true).unwrap();
    let ws = tokio_tungstenite::accept_hdr_async(stream, |req: &Request, mut resp: Response| {
        let proto = req.headers().get("Sec-WebSocket-Protocol").and_then(|v| v.to_str().ok()).unwrap_or("");
        let first = proto.split(',').next().unwrap().trim().to_string();
        resp.headers_mut().insert("Sec-WebSocket-Protocol", first.parse().unwrap());
        Ok(resp)
    })
    .await
    .unwrap();
    let (mut tx, mut rx) = ws.split();
    let start = Instant::now();
    let now = || SERVER_EPOCH_US + start.elapsed().as_micros() as u64;
    let mut topics: HashMap<i64, String> = HashMap::new();
    let mut announced = std::collections::HashSet::new();
    while let Some(Ok(msg)) = rx.next().await {
        match msg {
            Message::Text(text) => {
                let msgs: Vec<serde_json::Value> = serde_json::from_str(&text).unwrap();
                for m in msgs {
                    if m["method"] == "publish" {
                        let p = &m["params"];
                        let pubuid = p["pubuid"].as_i64().unwrap();
                        let name = p["name"].as_str().unwrap().to_string();
                        topics.insert(pubuid, name.clone());
                        // Announce it back to its publisher (topic id = pubuid), only once per topic as
                        // WPILib's server does: publishing a topic again gets no second announcement.
                        if !announced.insert(name) {
                            continue;
                        }
                        let announce = serde_json::json!([{"method": "announce", "params": {
                            "name": p["name"], "id": pubuid, "type": p["type"], "pubuid": pubuid, "properties": {},
                        }}]);
                        tx.send(Message::Text(announce.to_string().into())).await.unwrap();
                    }
                }
            }
            Message::Binary(data) => {
                let mut cur = &data[..];
                while !cur.is_empty() {
                    let v = rmpv::decode::read_value(&mut cur).unwrap();
                    let a = v.as_array().unwrap();
                    let id = a[0].as_i64().unwrap();
                    if id == -1 {
                        // Time sync: reply with our time and the client's.
                        let reply = Value::Array(vec![Value::from(-1), Value::from(now()), a[2].clone(), a[3].clone()]);
                        let mut buf = Vec::new();
                        rmpv::encode::write_value(&mut buf, &reply).unwrap();
                        tx.send(Message::Binary(buf.into())).await.unwrap();
                    } else if let Some(name) = topics.get(&id) {
                        values.send((name.clone(), now(), a[1].as_u64().unwrap(), a[3].clone())).unwrap();
                    }
                }
            }
            _ => {}
        }
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn publishes_frames_with_server_capture_time() {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let (vtx, mut vrx) = mpsc::unbounded_channel();
    tokio::spawn(fake_server(listener, vtx));

    let (tx, rx) = watch::channel(None);
    // A 640x480 camera at 30 fps, streamed at half size and 15 fps.
    let mode = Mode { width: 640, height: 480, fps: 30, layout: Layout::Mjpeg };
    let (_state, state) = watch::channel(CameraState { modes: vec![mode], active: Some(mode), default: Some(mode) });
    let cfg = NtConfig {
        server: nt::parse_server("127.0.0.1").unwrap(),
        port,
        client_name: "test".into(),
        table: "/sonoma/test".into(),
        camera: Some(CameraAdvert { name: "test-cam".into(), port: 1190, source: "usb:/dev/video9".into() }),
        field_layouts: vec![],
        camera_state: state,
    };
    let (settings, _) =
        watch::channel(sonoma_vision::settings::Settings { stream_scale: 2, stream_fps: 15.0, ..Default::default() });
    let client = tokio::spawn(nt::run(cfg, rx, settings));

    let det = Detection {
        id: 3,
        hamming: 0,
        decision_margin: 80.0,
        corners: [[1.0, 2.0], [3.0, 4.0], [5.0, 6.0], [7.0, 8.0]],
    };
    // Keep sending (as the vision thread would) until a frame record arrives after time sync.
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut seq = 0;
    let mut camera = HashMap::new();
    let mut tag = HashMap::new();
    let mut pose = HashMap::new();
    // The tag 2 m ahead and 0.5 m left, facing the camera; the robot at (1, 2) facing +Y.
    let camera_to_tag = Isometry3::from_parts(
        Translation3::new(2.0, 0.5, 0.0),
        UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI),
    );
    let robot = Isometry3::from_parts(
        Translation3::new(1.0, 2.0, 0.0),
        UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2),
    );
    let tag_pose = TagPose { camera_to_tag, alternate: None, error_px: 0.1, ambiguity: 0.05 };
    let field_pose = FieldPose { robot, camera: robot, tags: vec![3], error_px: 0.1, ambiguity: 0.05 };
    let record_pose = |pose: &mut HashMap<String, Value>, name: &str, v: &Value| {
        if let Some(key) = name.strip_prefix("/sonoma/test/")
            && ["robot_pose", "pose_tags", "field/.type", "field/Robot"].contains(&key)
        {
            pose.insert(key.to_string(), v.clone());
        }
    };
    let (server_rx_time, record) = loop {
        assert!(Instant::now() < deadline, "no frame record arrived");
        seq += 1;
        let result = FrameResult {
            sequence: seq,
            capture: monotonic_now() - Duration::from_millis(20),
            detections: vec![det],
            tag_poses: vec![Some(tag_pose)],
            field_pose: Some(field_pose.clone()),
            ..Default::default()
        };
        tx.send(Some(Arc::new(result))).unwrap();
        let got = tokio::time::timeout(Duration::from_millis(200), async {
            while let Some((name, t, stamp, v)) = vrx.recv().await {
                if name == "/sonoma/test/frame" {
                    return Some((t, stamp, v));
                }
                if let Some(key) = name.strip_prefix("/CameraPublisher/test-cam/") {
                    camera.insert(key.to_string(), v);
                } else if let Some(key) = name.strip_prefix("/sonoma/test/tags/3/") {
                    tag.insert(key.to_string(), v);
                } else {
                    record_pose(&mut pose, &name, &v);
                }
            }
            None
        })
        .await;
        if let Ok(Some((t, stamp, v))) = got {
            let rec: Vec<f64> = v.as_array().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
            // Before the first time sync the client's clock is unsynchronised: wait for a synced record.
            if rec[0] > SERVER_EPOCH_US as f64 {
                assert_eq!(stamp as f64, rec[0], "value timestamp is the capture time");
                break (t, rec);
            }
        }
    };
    // On the server's clock, the frame arrives its age at publish (at least the 20 ms we gave it, more if
    // it waited for the connection) plus a small transmission delay: the clocks agree.
    let behind_ms = (server_rx_time as f64 - record[0]) / 1e3;
    let age_ms = record[2];
    assert!(age_ms >= 20.0, "age at publish {age_ms} ms");
    assert!(
        (0.0..3.0).contains(&(behind_ms - age_ms)),
        "arrived {behind_ms} ms after capture, {age_ms} ms old when sent"
    );
    assert_eq!(record[3], 1.0);
    assert_eq!(&record[4..], &[3.0, 0.0, 80.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]);

    // Each tag ID gets its own named topics.
    let keys = [
        "ambiguity",
        "bearing",
        "camera_to_tag",
        "center_x",
        "center_y",
        "corners",
        "distance",
        "hamming",
        "margin",
        "visible",
    ];
    let pose_keys = ["robot_pose", "pose_tags", "field/.type", "field/Robot"];
    let collect = async {
        while !keys.iter().all(|k| tag.contains_key(*k)) || !pose_keys.iter().all(|k| pose.contains_key(*k)) {
            let (name, _, _, v) = vrx.recv().await.unwrap();
            if let Some(key) = name.strip_prefix("/sonoma/test/tags/3/") {
                tag.insert(key.to_string(), v);
            } else {
                record_pose(&mut pose, &name, &v);
            }
        }
    };
    let got = tokio::time::timeout(Duration::from_secs(5), collect).await;
    assert!(got.is_ok(), "per-tag and pose topics: got {tag:?} and {pose:?}");
    assert_eq!(tag["visible"], Value::from(true));
    assert_eq!(tag["margin"], Value::from(80.0));
    assert_eq!(tag["hamming"], Value::from(0));
    assert_eq!((tag["center_x"].as_f64(), tag["center_y"].as_f64()), (Some(4.0), Some(5.0)));
    let corners: Vec<f64> = tag["corners"].as_array().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
    assert_eq!(corners, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]);
    // 2 m ahead and 0.5 m left: 2.06 m away, 14.0° to the left.
    assert_eq!(tag["distance"], Value::from(2.06));
    assert_eq!(tag["bearing"], Value::from(14.0));
    assert_eq!(tag["ambiguity"], Value::from(0.05));

    // The robot's pose: a WPILib Pose3d struct (x, y, z, then quaternion w, x, y, z, as little-endian
    // doubles), and a Field2d for dashboards.
    let bytes = match &pose["robot_pose"] {
        Value::Binary(b) => b.clone(),
        other => panic!("robot_pose is {other:?}"),
    };
    let doubles: Vec<f64> = bytes.chunks(8).map(|c| f64::from_le_bytes(c.try_into().unwrap())).collect();
    let half = std::f64::consts::FRAC_1_SQRT_2;
    let expected = [1.0, 2.0, 0.0, half, 0.0, 0.0, half];
    assert!(doubles.iter().zip(expected).all(|(a, b)| (a - b).abs() < 1e-12), "{doubles:?}");
    assert_eq!(pose["pose_tags"], Value::Array(vec![Value::from(3)]));
    assert_eq!(pose["field/.type"], Value::from("Field2d"));
    let robot2d: Vec<f64> = pose["field/Robot"].as_array().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
    assert!((robot2d[0] - 1.0).abs() < 1e-12 && (robot2d[1] - 2.0).abs() < 1e-12 && (robot2d[2] - 90.0).abs() < 1e-9);

    // The camera stream is advertised as CameraServer does, before any frame, with every topic
    // Elastic needs to recognise a camera.
    let mut keys: Vec<&str> = camera.keys().map(String::as_str).collect();
    keys.sort();
    assert_eq!(keys, ["connected", "description", "mode", "modes", "source", "streams"]);
    assert_eq!(camera["mode"].as_str(), Some("320x240 MJPEG 15 fps"));
    let streams: Vec<String> =
        camera["streams"].as_array().unwrap().iter().map(|u| u.as_str().unwrap().into()).collect();
    assert!(streams.contains(&"mjpg:http://127.0.0.1:1190/?action=stream".to_string()), "{streams:?}");

    // Closing the results channel shuts the publisher down cleanly.
    drop(tx);
    tokio::time::timeout(Duration::from_secs(5), client).await.unwrap().unwrap().unwrap();
}
