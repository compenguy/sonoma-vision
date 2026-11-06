//! End to end: live settings against a minimal in-process NT4 server that already holds a persisted
//! value, and later pushes a dashboard's edits.

use std::collections::HashMap;
use std::net::Ipv4Addr;
use std::time::{Duration, Instant};

use futures_util::{SinkExt, StreamExt};
use rmpv::Value;
use serde_json::json;
use sonoma_vision::nt::{self, NtConfig};
use sonoma_vision::settings::Settings;
use tokio::net::TcpListener;
use tokio::sync::{mpsc, watch};
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::tungstenite::handshake::server::{Request, Response};

/// What the server saw: a publish (name, properties) or a value set (name, value).
#[derive(Debug)]
enum Seen {
    Publish(String, serde_json::Value),
    Value(String, Value),
}

fn binary(id: i64, value: Value) -> Message {
    let type_idx = match value {
        Value::Boolean(_) => 0,
        Value::F64(_) => 1,
        Value::String(_) => 4,
        _ => 2,
    };
    let mut buf = Vec::new();
    let msg = Value::Array(vec![Value::from(id), Value::from(1_000_000u64), Value::from(type_idx), value]);
    rmpv::encode::write_value(&mut buf, &msg).unwrap();
    Message::Binary(buf.into())
}

#[allow(clippy::result_large_err)] // the handshake callback's error type is tungstenite's
/// Accept one client. `persisted` topics exist on the server before it connects; each command in
/// `edits` sets a topic's value as a dashboard would.
async fn fake_server(
    listener: TcpListener,
    persisted: Vec<(String, &'static str, Value)>,
    seen: mpsc::UnboundedSender<Seen>,
    mut edits: mpsc::UnboundedReceiver<(String, Value)>,
) {
    let (stream, _) = listener.accept().await.unwrap();
    let ws = tokio_tungstenite::accept_hdr_async(stream, |req: &Request, mut resp: Response| {
        let proto = req.headers().get("Sec-WebSocket-Protocol").and_then(|v| v.to_str().ok()).unwrap_or("");
        let first = proto.split(',').next().unwrap().trim().to_string();
        resp.headers_mut().insert("Sec-WebSocket-Protocol", first.parse().unwrap());
        Ok(resp)
    })
    .await
    .unwrap();
    let (mut tx, mut rx) = ws.split();
    // Topic name -> (server id, type), and publisher id -> topic name.
    let mut topics: HashMap<String, (i64, String)> = HashMap::new();
    let mut publishers: HashMap<i64, String> = HashMap::new();
    let mut next_id = 100;
    let announce = |name: &str, id: i64, ty: &str, pubuid: Option<i64>| {
        let mut params = json!({"name": name, "id": id, "type": ty, "properties": {"persistent": true}});
        if let Some(p) = pubuid {
            params["pubuid"] = json!(p);
        }
        Message::Text(json!([{"method": "announce", "params": params}]).to_string().into())
    };
    loop {
        tokio::select! {
            Some((name, value)) = edits.recv() => {
                let (id, _) = topics[&name];
                tx.send(binary(id, value)).await.unwrap();
            }
            msg = rx.next() => {
                let Some(Ok(msg)) = msg else { return };
                match msg {
                    Message::Text(text) => {
                        let msgs: Vec<serde_json::Value> = serde_json::from_str(&text).unwrap();
                        for m in msgs {
                            let p = &m["params"];
                            match m["method"].as_str().unwrap() {
                                "publish" => {
                                    let name = p["name"].as_str().unwrap().to_string();
                                    let pubuid = p["pubuid"].as_i64().unwrap();
                                    publishers.insert(pubuid, name.clone());
                                    let ty = p["type"].as_str().unwrap().to_string();
                                    let (id, ty) = topics.entry(name.clone()).or_insert_with(|| {
                                        next_id += 1;
                                        (next_id, ty)
                                    }).clone();
                                    tx.send(announce(&name, id, &ty, Some(pubuid))).await.unwrap();
                                    seen.send(Seen::Publish(name, p["properties"].clone())).unwrap();
                                }
                                "subscribe" => {
                                    // Like a real server: announce the persisted topics, then their values.
                                    for (name, ty, value) in &persisted {
                                        next_id += 1;
                                        topics.insert(name.clone(), (next_id, ty.to_string()));
                                        tx.send(announce(name, next_id, ty, None)).await.unwrap();
                                        tx.send(binary(next_id, value.clone())).await.unwrap();
                                    }
                                }
                                _ => {}
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
                                let reply = Value::Array(vec![Value::from(-1), Value::from(1_000_000u64), a[2].clone(), a[3].clone()]);
                                let mut buf = Vec::new();
                                rmpv::encode::write_value(&mut buf, &reply).unwrap();
                                tx.send(Message::Binary(buf.into())).await.unwrap();
                            } else if let Some(name) = publishers.get(&id) {
                                seen.send(Seen::Value(name.clone(), a[3].clone())).unwrap();
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}

/// Wait until `settings` satisfies `pred`.
async fn wait_for(settings: &mut watch::Receiver<Settings>, what: &str, pred: impl Fn(&Settings) -> bool) {
    let ok = tokio::time::timeout(Duration::from_secs(5), settings.wait_for(|s| pred(s))).await.is_ok();
    assert!(ok, "timed out waiting for {what}: {:?}", *settings.borrow());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn settings_adopt_persisted_values_publish_the_rest_and_follow_edits() {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let (seen_tx, mut seen) = mpsc::unbounded_channel();
    let (edit, edits) = mpsc::unbounded_channel();
    // The server kept gain = 120 and the field layout b from an earlier run.
    let persisted = vec![
        ("/sonoma/test/settings/gain".to_string(), "int", Value::from(120)),
        ("/sonoma/test/settings/field_layout/selected".to_string(), "string", Value::from("b")),
    ];
    tokio::spawn(fake_server(listener, persisted, seen_tx, edits));

    let (_results_tx, results) = watch::channel(None);
    let (settings_tx, mut settings) =
        watch::channel(Settings { exposure: Some(50), field_layout: Some("a".into()), ..Settings::default() });
    let cfg = NtConfig {
        server: nt::parse_server("127.0.0.1").unwrap(),
        port,
        client_name: "test".into(),
        table: "/sonoma/test".into(),
        camera: None,
        field_layouts: vec!["a".into(), "b".into()],
    };
    tokio::spawn(nt::run(cfg, results, settings_tx));

    // The persisted value wins over the command line's (gain: the camera's own).
    wait_for(&mut settings, "the persisted gain", |s| s.gain == Some(120)).await;
    wait_for(&mut settings, "the persisted field layout", |s| s.field_layout.as_deref() == Some("b")).await;

    // Every setting is published as persistent, and ours are set for the settings the server lacked.
    let mut published = HashMap::new();
    let mut values = HashMap::new();
    let deadline = Instant::now() + Duration::from_secs(5);
    let chooser = "/sonoma/test/settings/field_layout/";
    let wanted = ["exposure", "min_contrast", "stream_auto_levels", "stream_gamma"];
    let chooser_wanted = [".type", "options", "default", "active"];
    let have_all = |v: &HashMap<String, Value>| {
        wanted.iter().all(|k| v.contains_key(*k))
            && chooser_wanted.iter().all(|k| v.contains_key(&format!("field_layout/{k}")))
    };
    while !have_all(&values) {
        let remaining = deadline.saturating_duration_since(Instant::now());
        match tokio::time::timeout(remaining, seen.recv()).await.expect("settings weren't all published").unwrap() {
            Seen::Publish(name, props) => {
                published.insert(name, props);
            }
            Seen::Value(name, v) => {
                if let Some(key) = name.strip_prefix("/sonoma/test/settings/") {
                    values.insert(key.to_string(), v);
                }
            }
        }
    }
    for key in ["exposure", "gain", "min_contrast", "stream_auto_levels", "stream_gamma"] {
        let props = &published[&format!("/sonoma/test/settings/{key}")];
        assert_eq!(props["persistent"], true, "{key}: {props}");
    }
    assert_eq!(values["exposure"], Value::from(50));
    assert_eq!(values["min_contrast"], Value::from(20));
    assert_eq!(values["stream_auto_levels"], Value::from(true));
    assert_eq!(values["stream_gamma"], Value::from(1.0));
    assert!(!values.contains_key("gain"), "the server's gain was overwritten");
    // The field layout chooser, as a SendableChooser: the server's selection was adopted.
    assert_eq!(values["field_layout/.type"], Value::from("String Chooser"));
    assert_eq!(values["field_layout/options"], Value::Array(vec!["a".into(), "b".into(), "none".into()]));
    assert_eq!(values["field_layout/default"], Value::from("a"));
    assert_eq!(values["field_layout/active"], Value::from("b"));
    assert!(!values.contains_key("field_layout/selected"), "the server's selection was overwritten");
    assert_eq!(published[&format!("{chooser}selected")]["persistent"], true);

    // A dashboard's edits apply; invalid ones are ignored.
    edit.send(("/sonoma/test/settings/stream_gamma".into(), Value::from(0.0))).unwrap();
    edit.send(("/sonoma/test/settings/min_contrast".into(), Value::from(12))).unwrap();
    wait_for(&mut settings, "the edited min_contrast", |s| s.min_contrast == 12).await;
    assert_eq!(settings.borrow().stream_gamma, 1.0);
    edit.send(("/sonoma/test/settings/exposure".into(), Value::from(-1))).unwrap();
    wait_for(&mut settings, "auto exposure", |s| s.exposure.is_none()).await;

    // Selecting a layout that isn't there is ignored (`active` stays b); `none` turns poses off, and
    // `active` follows.
    edit.send((format!("{chooser}selected"), Value::from("c"))).unwrap();
    edit.send((format!("{chooser}selected"), Value::from("none"))).unwrap();
    wait_for(&mut settings, "no field layout", |s| s.field_layout.is_none()).await;
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        let msg = tokio::time::timeout(remaining, seen.recv()).await.expect("active wasn't updated").unwrap();
        if let Seen::Value(name, v) = msg
            && name == format!("{chooser}active")
        {
            if v == Value::from("none") {
                break;
            }
            assert_eq!(v, Value::from("b"), "active only follows valid selections");
        }
    }
}
