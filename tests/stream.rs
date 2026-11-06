//! End to end: serve a synthetic frame with a tag over the MJPEG stream and read it back over HTTP.
#![cfg(feature = "mjpeg")]

use std::net::Ipv4Addr;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use sonoma_vision::detector::{Config, Detector, ImageView};
use sonoma_vision::settings::Settings;
use sonoma_vision::stream::{self, ConfigPage, StreamConfig};
use sonoma_vision::synth::Canvas;
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::net::TcpStream;

/// Read header lines up to the blank line.
async fn headers(r: &mut BufReader<TcpStream>) -> Vec<String> {
    let mut lines = Vec::new();
    loop {
        let mut line = String::new();
        assert!(r.read_line(&mut line).await.unwrap() > 0, "connection closed in headers");
        let line = line.trim_end().to_string();
        if line.is_empty() {
            return lines;
        }
        lines.push(line);
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn streams_jpeg_frames_with_overlay() {
    let family = sonoma_vision::detector::Family::by_name("tag36h11").unwrap();
    let mut canvas = Canvas::new(320, 240, 200.0);
    canvas.draw_tag(family, 2, [[100.0, 60.0], [220.0, 60.0], [220.0, 180.0], [100.0, 180.0]], 20.0, 230.0);
    let gray = canvas.to_gray();
    let detections =
        Detector::new(family, &Config::default()).unwrap().detect(&ImageView::gray(&gray, 320, 240).unwrap());
    assert_eq!(detections.len(), 1);

    let live = Settings {
        stream_fps: 100.0,
        stream_quality: 80,
        stream_scale: 2,
        stream_overlay: true,
        ..Settings::default()
    };
    let (settings, look) = tokio::sync::watch::channel(live);
    // The page renders whatever the live settings say, each time it's asked.
    let page_settings = look.clone();
    let page = ConfigPage {
        render: Arc::new(move || format!("[stream]\nscale = {}\n", page_settings.borrow().stream_scale)),
        filename: "cam0.toml".into(),
    };
    let (mut tap, server) = stream::start(&StreamConfig { port: 0 }, look, Some(page)).unwrap();
    let port = server.local_addr().unwrap().port();
    tokio::spawn(server.run());
    // The vision thread's part: offer frames continuously; they're only taken once a client is connected.
    let done = Arc::new(AtomicBool::new(false));
    let feeding = done.clone();
    let feeder = std::thread::spawn(move || {
        while !feeding.load(Ordering::Relaxed) {
            tap.offer(&ImageView::gray(&gray, 320, 240).unwrap(), &detections);
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    let mut sock = BufReader::new(TcpStream::connect((Ipv4Addr::LOCALHOST, port)).await.unwrap());
    sock.get_mut().write_all(b"GET /?action=stream HTTP/1.1\r\nHost: x\r\n\r\n").await.unwrap();
    let head = tokio::time::timeout(Duration::from_secs(5), headers(&mut sock)).await.unwrap();
    assert_eq!(head[0], "HTTP/1.0 200 OK");
    assert!(head.iter().any(|h| h == "Content-Type: multipart/x-mixed-replace;boundary=sonomaframe"), "{head:?}");

    for _ in 0..2 {
        let part = tokio::time::timeout(Duration::from_secs(5), headers(&mut sock)).await.unwrap();
        assert_eq!(part[0], "--sonomaframe");
        let len: usize = part.iter().find_map(|h| h.strip_prefix("Content-Length: ")).unwrap().parse().unwrap();
        let mut jpeg = vec![0; len + 2];
        sock.read_exact(&mut jpeg).await.unwrap();
        assert_eq!(&jpeg[len..], b"\r\n");
        let mut dec = zune_jpeg::JpegDecoder::new(&jpeg[..len]);
        let pixels = dec.decode().unwrap();
        assert_eq!(dec.dimensions().unwrap(), (160, 120), "half size");
        // The overlay's white outline runs along the tag's top edge (y = 30 at half size), over what
        // was the dark border.
        let channels = pixels.len() / (160 * 120);
        let at = |x: usize, y: usize| pixels[(y * 160 + x) * channels];
        assert!(at(80, 30) > 200, "outline: {}", at(80, 30));
        assert!(at(80, 34) < 80, "tag border under it: {}", at(80, 34));
    }

    // Anything but the stream paths is a 404.
    let mut other = BufReader::new(TcpStream::connect((Ipv4Addr::LOCALHOST, port)).await.unwrap());
    other.get_mut().write_all(b"GET /favicon.ico HTTP/1.1\r\n\r\n").await.unwrap();
    assert_eq!(headers(&mut other).await[0], "HTTP/1.0 404 Not Found");

    // The configuration page, after a live change; as text, and as a file to save.
    settings.send_modify(|s| s.stream_scale = 4);
    for (query, disposition) in
        [("", None), ("?download", Some("Content-Disposition: attachment; filename=\"cam0.toml\""))]
    {
        let mut page = BufReader::new(TcpStream::connect((Ipv4Addr::LOCALHOST, port)).await.unwrap());
        page.get_mut().write_all(format!("GET /config.toml{query} HTTP/1.1\r\n\r\n").as_bytes()).await.unwrap();
        let head = headers(&mut page).await;
        assert_eq!(head[0], "HTTP/1.0 200 OK");
        assert_eq!(head.iter().any(|h| Some(h.as_str()) == disposition), disposition.is_some(), "{head:?}");
        let mut body = String::new();
        page.read_to_string(&mut body).await.unwrap();
        assert_eq!(body, "[stream]\nscale = 4\n");
    }

    done.store(true, Ordering::Relaxed);
    feeder.join().unwrap();
}
