# sonoma-vision

A standalone AprilTag processor service for the FRC SystemCore controller, in pure Rust. It captures
frames from a V4L2 (USB/UVC) camera, detects AprilTags, and publishes them over NetworkTables 4 to the
robot program running on the same controller.

The detector is a Rust implementation of [napa-vision](https://github.com/zwatx/NAPA)'s sparse-sampling design
by Zachary Wilson: sample a sparse grid, skip regions without contrast, group dark runs into blobs,
reject non-tags with a cheap early decode, and fit edges and decode only the tags, at full
resolution. See [NOTICE](NOTICE) for credits and licensing (GPL-3.0-only).

## Layout

| Path | What |
| --- | --- |
| `src/detector/` | The detector: levels, thresholding and blobs, edge fitting, decoding, tag families |
| `src/capture.rs` | V4L2 capture via [v4l2r](https://crates.io/crates/v4l2r), memory-mapped buffers |
| `src/nt.rs` | NT4 publishing via [nt_client](https://crates.io/crates/nt_client) (patched, `vendor/`) |
| `src/stream.rs` | Optional MJPEG camera stream for dashboards, with tag overlays |
| `src/settings.rs` | Settings that can be changed while running, over NT |
| `src/main.rs` | The service: a capture/detect thread feeding the NT publisher |
| `src/synth.rs` | Synthetic frames with exact ground truth, for tests and benchmarks |
| `deploy/` | systemd unit (one instance per camera) and an example config |
| `vendor/apriltag/` | Upstream AprilTag family sources (unmodified), which the generator reads |
| `tools/` | Family table generator, SystemCore cross-build script |

## How frames flow

- **Zero copy.** The camera's driver buffers are memory-mapped, and each `Frame` borrows one in place.
  GREY, YUYV, UYVY and NV12 frames are read by the detector directly: packed YUYV is a luma view with a
  2-byte pixel pitch. MJPEG has to be decoded (one greyscale copy, about 0.4 ms at 720p on a desktop).
- **Newest frame first.** Each request drains the driver's queue and returns the most recent frame,
  handing older ones back unread, so a slow frame never builds a backlog. Use 2–3 buffers.
- **Capture timestamps.** Frames carry the driver's CLOCK_MONOTONIC capture time. The publisher maps it
  onto the NT server's clock using NT4 time sync, and stamps the published values with it.

## NetworkTables topics

Under the table (default `/sonoma/cam0`):

| Topic | Type | Contents |
| --- | --- | --- |
| `frame` | double[] | `[capture_time_us, sequence, latency_ms, n, then per tag: id, hamming, margin, x0, y0, x1, y1, x2, y2, x3, y3]` |
| `ids` | int[] | Tag IDs in the last frame |
| `fps`, `detect_ms` | double | Processing rate and per-frame detection time |
| `dropped` | int | Frames the camera skipped since start |
| `tags/<id>/visible` | boolean | Whether tag `<id>` was in the last frame |
| `tags/<id>/margin`, `hamming` | double, int | How clearly it was read (see below) |
| `tags/<id>/center_x`, `center_y` | double | Its centre, in pixels |
| `tags/<id>/corners` | double[] | `[x0, y0, x1, y1, x2, y2, x3, y3]` |

The `tags/<id>/` topics are for dashboards: a group per tag ID seen, rounded to one decimal so they're
readable, kept when the tag leaves view (`visible` turns false). Robot code should read `frame`, the
one atomic record per frame.

`frame` is one atomic record per frame. Its NT timestamp, and `capture_time_us`, are the capture time
on the NT server's clock, the clock the robot program's own NT timestamps use. Corners are in
pixel-centre coordinates, in napa-vision's (OpenCV's AprilTag) corner order. Pose solving is not implemented
yet: robot code gets corners.

Reading it in robot code (Java):

```java
DoubleArraySubscriber frames = NetworkTableInstance.getDefault()
    .getDoubleArrayTopic("/sonoma/cam0/frame").subscribe(new double[0]);

for (TimestampedDoubleArray f : frames.readQueue()) {
  double[] v = f.value;
  double captureSeconds = f.timestamp / 1e6; // capture time
  for (int i = 0; i < (int) v[3]; i++) {
    int base = 4 + 11 * i;
    int id = (int) v[base];
    double x0 = v[base + 3], y0 = v[base + 4]; // ... through v[base + 10]
  }
}
```

## Live settings

These can be changed while running, from a dashboard or the robot program, under `<table>/settings/`:

| Topic | Type | Command line | |
| --- | --- | --- | --- |
| `exposure` | int | `--exposure` | Manual exposure in driver units (100 µs for UVC cameras); -1 for auto |
| `gain` | int | `--gain` | Analog gain in driver units; -1 leaves the camera's own |
| `min_contrast` | int | `--min-contrast` | Grey levels required between tag black and white (1-255) |
| `stream_auto_levels` | boolean | `--stream-auto-levels` | Stretch the stream to the full grey range (default true) |
| `stream_gamma` | double | `--stream-gamma` | Stream gamma, 0.1-10; above 1 lifts shadows (default 1) |

They're persistent topics, so the robot's NT server saves them across restarts. **Values the server
already has win over the command line's**: the command line only sets settings the server doesn't
have yet. To start over from the command line, delete the topics on the server (in OutlineViewer,
for example). Invalid values are logged and ignored.

Exposure and gain change what the camera captures, so they affect detection. The stream settings
are for viewing only. For detection in dim light, raise gain (or exposure, at the cost of motion blur)
rather than adjusting the image afterwards: the detector's thresholds are already local and relative.

### Reading decision margins

Each tag's `margin` (`tags/<id>/margin`, and the third value of the tag's block in `frame`) is how
far, on average, the tag's payload cells sit from the black/white threshold, in grey levels (the smaller of the white cells' and black cells' averages). It is
roughly half the tag's black-to-white contrast as captured, so it says how clearly the tag was read:

| Margin | |
| --- | --- |
| below ~10 | Barely readable: dim, blurred, far, or glare. Expect dropouts and occasional misreads |
| ~10-30 | Usable; tuning exposure or gain will help |
| above ~30 | Clear |

Watch it while tuning exposure and gain, together with `hamming` (bits corrected, usually 0; a tag
often needing corrections is damaged or poorly lit). A margin can't exceed about 127 (black at 0 and
white at 255).

## Camera stream

Off unless `--stream` is given. The stream is MJPEG over HTTP, served and advertised the way WPILib's
CameraServer does it: `http://<controller>:1190/?action=stream` (or `?action=snapshot` for one JPEG),
listed in NT under `/CameraPublisher/<name>` (`streams`, `mode`, `modes`, `source`, `description`,
`connected`, all of which Elastic needs to recognise a camera), so its camera stream widget finds it.

| Option | Default | |
| --- | --- | --- |
| `--stream-port` | 1190 | The field network allows 1180-1190 for cameras; CameraServer starts at 1181 |
| `--stream-fps` | 15 | At most this rate; the camera's is the upper limit |
| `--stream-quality` | 50 | JPEG quality, 1-100 |
| `--stream-scale` | 1 | Divide width and height by this; 2 streams a 640×480 camera at 320×240 |
| `--stream-overlay` | off | Outline detected tags, mark their first corner, and label their IDs |
| `--stream-name` | `sonoma-cam0` | The camera's name in dashboards: `sonoma-` and the table's last part |

The stream is greyscale (the detector's luma), stretched to the full grey range by default
(`stream_auto_levels`, a live setting, as is `stream_gamma`). It costs detection nothing while no one is watching;
while someone is, the vision thread copies (and downscales) a frame at the stream's rate, and a
separate thread draws overlays and encodes. Mind the field's bandwidth cap: 320×240 at 15 fps is
plenty for drivers.

## Building and running

```sh
cargo build --release
./target/release/sonoma-vision --device /dev/video0 --width 1280 --height 720 --nt-server local
```

`--help` lists every option: `--format`, `--fps`, `--exposure` (manual exposure limits motion blur),
`--family`, `--min-tag-px`, `--nt-server` (`local`, a team number, or an IPv4 address), `--table`,
and the `--stream` options above.

Building needs libclang: v4l2r generates its V4L2 bindings with bindgen.

### SystemCore

`tools/build-systemcore.sh` cross-compiles for aarch64. It needs `gcc-aarch64-linux-gnu` and
`libc6-dev-arm64-cross` (see the script). The binary needs glibc 2.35 or newer. Then copy the binary to
`/usr/local/bin/`, the unit to `/etc/systemd/system/`, and a config per camera:

```sh
sudo install -m 755 sonoma-vision /usr/local/bin/
sudo install -m 644 deploy/sonoma-vision@.service /etc/systemd/system/
sudo install -D -m 644 deploy/cam0.env /etc/sonoma-vision/cam0.env
sudo systemctl daemon-reload && sudo systemctl enable --now sonoma-vision@cam0
```

## Testing

```sh
cargo test                                       # unit tests, synthetic accuracy, NT4 and stream end to end
cargo run --release --example bench -- 50        # recall, corner error and latency per scenario
cargo run --release --example camera -- /dev/video0 640 480   # live capture and detection
```

`tests/nt_publish.rs` runs the publisher against a minimal in-process NT4 server, and checks that the
capture timestamps survive time sync; `tests/nt_settings.rs` checks live settings against one holding
persisted values.

Synthetic benchmark (`examples/bench.rs`, 40 frames per scenario, one desktop core; napa-vision's C detector
on the same frames for reference):

| Scenario | Recall | Wrong IDs | Median / p95 corner error | Median time (NAPA C) |
| --- | ---: | ---: | ---: | ---: |
| clean-640 | 100% | 0 | 0.016 / 0.045 px | 0.27 ms (0.11) |
| small-1280 (32–48 px tags) | 100% | 0 | 0.032 / 0.094 px | 0.65 ms (0.31) |
| large-1280 | 100% | 0 | 0.021 / 0.044 px | 0.40 ms (0.14) |
| perspective | 100% | 0 | 0.021 / 0.058 px | 0.53 ms (0.20) |
| blur+noise | 100% | 0 | 0.116 / 0.301 px | 1.53 ms (0.41) |
| clipped exposure | 100% | 0 | 0.038 / 0.129 px | 0.58 ms (0.24) |
| empty-1920 | — | 0 | — | 0.38 ms (0.06) |

Accuracy matches NAPA's C detector; it is 2–6× slower, without hand-written SIMD.

## Regenerating the tag families

`src/detector/families/` is generated from upstream AprilTag's family sources, vendored unmodified
in `vendor/apriltag/` (see its README for the commit):

```sh
python3 -I tools/gen_families.py
```
