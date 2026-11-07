# sonoma-vision

A standalone AprilTag processor service for the FRC SystemCore controller, implemented as a Rust
service built around [napa-vision's C core](https://github.com/zwatx/NAPA/blob/master/src/napa_vision/_native/napa_core.c),
providing highly optimized SIMD code for the tag detection.

It captures frames from a V4L2 (USB/UVC) camera, detects AprilTags, and publishes them over
NetworkTables 4 to the robot program running on the same controller.

This absolutely would not be possible without Zachary Wilson's brilliant work on
[napa-vision](https://github.com/zwatx/NAPA). Its sparse-sampling design samples a sparse grid,
skips regions without contrast, groups dark runs into blobs, rejects non-tags with a cheap early
decode, and fits edges and decodes only the tags, at full resolution.

## Compared with napa-vision

napa-vision is a library: give it a frame, and it returns tags, and poses if you also give it a
calibration. sonoma-vision uses the same detector, so detections are identical. What it adds is a
running service for an FRC robot, built around that detector.

**sonoma-vision only:**

- **Camera capture (V4L2).** The camera driver's buffers are read in place. The format is chosen so the
  detector reads it without a copy where possible, and checked against the camera's sizes and frame rates.
  Each read takes the newest frame, so a slow frame never builds a backlog.
- **Capture timestamps on the robot's clock.** The driver's capture time is mapped onto the NT server's
  clock through NT4 time sync, ready for `addVisionMeasurement`.
- **NetworkTables 4 publishing.** An atomic per-frame record, `robot_pose` (Pose3d) and `camera_to_tag`
  (Transform3d) as WPILib structs, the `addVisionMeasurement` arguments with standard deviations in one
  atomic value, a Field2d, and per-tag topics for dashboards.
- **Live settings over NT.** Detection, exposure, gain, focus, stream and mounting settings, plus
  drop-downs for the field layout and the camera mode. The server keeps them across restarts, and
  `/config.toml` saves them to a file.
- **A CameraServer-compatible MJPEG stream,** with tag overlays. Elastic finds it, and it costs
  nothing while no one is watching.
- **FRC inputs:** WPILib field layout JSON, PhotonVision calibration exports, and the camera's pose on
  the robot.
- **Deployment.** One native binary needing only glibc 2.35, with layered toml configuration, a
  per-camera systemd unit, and release bundles for SystemCore.

**napa-vision only:**

- **Covariances for single tags and tag bearings,** for aiming at a tag rather than localizing.
  sonoma-vision publishes uncertainty for the robot's pose only.
- **Stereo:** one pose from two calibrated cameras.
- **Tools:** camera calibration from checkerboard photos (`napa-calibrate`), printable tags
  (`napa-tag`), scoring on your own labeled footage (`napa-benchmark`), and a drop-in replacement for
  `pupil_apriltags`.
- **Other platforms:** Python wheels for Windows, macOS and Android, and a C library for Android and
  Meta Quest.

## License

sonoma-vision's own code is under your choice of the [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE)
license. napa-vision's core, and the files derived from it, are under the [GPL-3.0-only](LICENSE-GPL),
and so is the compiled program as a whole. [NOTICE](NOTICE) lists which files are which, with the
credits and the licenses of other vendored code.

## Layout

| Path | What |
| --- | --- |
| `src/detector/` | The detector: napa-vision's core behind a Rust API, luma views of frames, tag families |
| `src/capture.rs` | V4L2 capture via [v4l2r](https://crates.io/crates/v4l2r), memory-mapped buffers |
| `src/nt.rs` | NT4 publishing via [nt_client](https://crates.io/crates/nt_client) (patched, `vendor/`) |
| `src/stream.rs` | Optional MJPEG camera stream for dashboards, with tag overlays, and `/config.toml` |
| `src/settings.rs` | Settings that can be changed while running, over NT |
| `src/pose/` | Pose solving: camera model, IPPE and refinement, uncertainty, field layouts |
| `src/config.rs` | `sonoma-vision.toml`: finding, reading and writing configuration files |
| `src/vision.rs` | The vision thread: capture, detect and solve poses, applying live settings |
| `src/main.rs` | The service: options, startup, and the vision thread feeding the NT publisher |
| `src/synth.rs` | Synthetic frames with exact ground truth, for tests and benchmarks |
| `fields/` | Field layouts (WPILib JSON), read at startup |
| `deploy/` | systemd unit (one instance per camera) and example configuration files |
| `vendor/napa-vision/` | napa-vision's C core (unmodified): the detector |
| `vendor/apriltag/` | Upstream AprilTag family sources (unmodified), which the family generator reads |
| `tools/` | Family table generator, SystemCore cross-build and release packaging scripts |
| `.github/workflows/` | Release builds: bundles for SystemCore and desktop Linux |

## How frames flow

- **Few copies.** The camera's driver buffers are memory-mapped, and each `Frame` borrows one in place.
  GREY and NV12 frames are read by the detector directly. Packed YUYV and UYVY are luma views with a
  2-byte pixel pitch, which the detector copies out to one byte per pixel (SIMD, about 0.05 ms at
  1080p). MJPEG has to be decoded (one greyscale copy, about 0.4 ms at 720p on a desktop).
- **Format choice.** Without `--format`, the camera gets the first of GREY, NV12, YUYV, UYVY and MJPEG
  that it lists at the requested size and `--fps`. Many UVC cameras list YUYV at large sizes only at
  a few frames per second, where MJPEG reaches full rate; the log says why a format was passed over.
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
| `tags/<id>/distance`, `bearing` | double | Metres from the camera; degrees off its axis, left positive |
| `tags/<id>/ambiguity` | double | How uncertain its orientation is (see Poses) |
| `tags/<id>/camera_to_tag` | struct Transform3d | The tag's pose in the camera's frame |
| `robot_pose` | struct Pose3d | The robot's pose on the field, timestamped with the capture time |
| `pose_tags` | int[] | The tags `robot_pose` came from |
| `robot_pose_measurement` | double[6] | `[x, y, yaw, std_x, std_y, std_yaw]`: what `addVisionMeasurement` takes, in one value (see Poses) |
| `robot_pose_covariance` | double[36] | The robot pose's 6×6 covariance, row-major, over x, y, z, rx, ry, rz in the field's axes |
| `pose_error_px`, `pose_ambiguity` | double | Its RMS reprojection error, and how nearly a second pose fits as well |
| `pose_outliers` | int[] | Tags left out of `robot_pose` because they disagreed with the rest |
| `pose_outlier_reasons` | string[] | Why each was left out, in the same order |
| `pose_rejected` | string | Why tags from the layout were in view but gave no pose; empty while poses are published |
| `field` | Field2d | The robot's pose for dashboards' field widgets |

The `tags/<id>/` topics are for dashboards: a group per tag ID seen, rounded to one decimal so they're
readable, kept when the tag leaves view (`visible` turns false). Robot code should read `frame`, the
one atomic record per frame.

`frame` is one atomic record per frame. Its NT timestamp, and `capture_time_us`, are the capture time
on the NT server's clock, the clock the robot program's own NT timestamps use. Corners are in
pixel-centre coordinates, in napa-vision's (OpenCV's AprilTag) corner order: the printed tag's top-left,
top-right, bottom-right, bottom-left.

Feeding the robot's pose to the drivetrain's pose estimator (Java), with its standard deviations:

```java
DoubleArraySubscriber measurements = NetworkTableInstance.getDefault()
    .getDoubleArrayTopic("/sonoma/cam0/robot_pose_measurement").subscribe(new double[0]);

for (TimestampedDoubleArray m : measurements.readQueue()) {
  double[] v = m.value; // x, y, yaw (radians), then the standard deviations of each
  poseEstimator.addVisionMeasurement(
      new Pose2d(v[0], v[1], new Rotation2d(v[2])), m.timestamp / 1e6, VecBuilder.fill(v[3], v[4], v[5]));
}
```

Reading corners from `frame`:

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

## Poses

Each tag's pose relative to the camera is solved from its four corners (IPPE, then a least-squares
refinement). From the tags the field layout knows, the robot's pose on the field is fitted to every
corner in view at once. Each tag's two solutions seed that fit, and so does the wall's when several
tags are on one wall. The best fit with the robot somewhere it can be is the pose. Poses use WPILib's
conventions: the camera frame is X forward, Y left, Z up, and tags face out along their +X. With no
field layout selected, no poses are solved at all (tags are still detected and published).

- **Camera model.** `--calibration` loads a PhotonVision calibration export (JSON) for the camera, at
  any resolution scaling to the one streamed. Without one, the camera is approximated from a diagonal
  field of view (`--fov`, default 70°) with no lens distortion. That's fine for trying things out,
  but distances and angles will be off by a few percent; calibrate for real use.
- **Field.** `--field-layout` is the name of a known layout, a WPILib field layout JSON file, or
  `none` for no poses. Without it, `2026-rebuilt-welded` is used if it's found, else none. It can be
  changed while running (see Live settings). Known layouts are read at startup from every `fields`
  directory found among `/usr/share/sonoma-vision/fields`, `/usr/local/share/sonoma-vision/fields`,
  `./fields` (the working directory) and `fields` beside the executable; each `*.json` is named by its
  file stem. If two directories have the same name, the last in that order wins. The repository's
  `fields/` has `2026-rebuilt-welded` and `2026-rebuilt-andymark`.
- **Mounting.** `--robot-to-camera x,y,z,roll,pitch,yaw` is the camera's pose on the robot (metres
  forward, left and up from the robot's origin; degrees), as WPILib's robot-to-camera transform.
- **Tag size.** `--tag-size`, the black square's side in metres (FRC: 0.1651, 6.5 in).
- **Ambiguity.** A single tag seen nearly face-on has two poses that fit almost equally well (the
  second is mirrored about the line of sight), and so can several tags on one wall seen from afar.
  `ambiguity` is the better fit's error over the other's: near 0 it's clear, near 1 it's a coin toss.
  Single-tag robot poses above `--max-ambiguity` (default 0.2) aren't published. Tags at different
  depths settle it.
- **Bounds.** Poses with the robot more than `--max-height` metres off the floor (default 2), tilted
  more than `--max-tilt` degrees (default 30), or more than 1 m off the field aren't published. The
  mirror-image pose usually breaks one of these, so the bounds also settle many ambiguities. They
  assume `--robot-to-camera` is right: a pitched camera with no mount configured looks like a tilted
  robot.
- **Fit.** A pose whose corners fit worse than `--corner-noise-px` explains (a 1-in-10,000 chance) isn't
  published: a tag isn't where the layout says, or the calibration is off.
- **Outliers.** When several tags don't fit together, the largest group that agree on a pose is used,
  and the rest are left out and listed in `pose_outliers`, with why in `pose_outlier_reasons`. Each
  tag's two solutions are a hypothesis. The tags whose corners are where a hypothesis predicts them,
  allowing for that hypothesis's own uncertainty, join it, and the group is refitted until it settles.
  A lone distant tag predicts the others only loosely, and the test allows for that. With two tags that
  disagree, or two equal groups that disagree, there's no telling which tags are where the layout says,
  so no pose is published.

`pose_rejected` says which of these dropped a pose.

### Uncertainty

`robot_pose_measurement` carries, with the robot's x, y and yaw, the standard deviations of x and y
along the robot's own forward and left axes, and of yaw, in radians. These are what WPILib's pose
estimators expect: they apply vision standard deviations in the robot's frame. `robot_pose_covariance`
has the full 6×6 covariance, along the field's axes, for filters that take one.

They're computed from each frame's geometry: how many corners there are, how far away, how steeply
seen and how widely spread. So a close pose from several tags is trusted far more than a lone distant
tag, without a hand-tuned table. Each corner coordinate is assumed to have `--corner-noise-px` of noise
(default 1, a live setting). Real cameras add blur, rolling shutter and calibration error, which all
look like corner noise, so 1 px is deliberately cautious. To tune it, log poses while the robot stands
still, and compare their scatter with the published standard deviations.

The covariance starts from the fit's own (first-order, Gauss-Newton) covariance, then makes two
adjustments where a single Gaussian at the best fit would be overconfident:

- **Valleys.** Tags seen nearly head-on, or from far away, leave the fit a long, nearly flat valley,
  for example trading the camera's height against its pitch. The valley through each candidate fit is
  traced along its least-constrained direction, re-fitting the other five directions at each step, out
  to where the corners fit clearly worse or the robot leaves the bounds. The covariance covers the
  traced poses, each weighted by how well it fits. A mirror-image solution that fits nearly as well
  widens it the same way.
- **Slack.** 10% of each variance is added back onto itself. This caps every correlation at 0.91, so the
  covariance can't claim that two errors cancel (a turn and the sideways shift that mimics it) more
  precisely than they really do.

`tests/uncertainty.rs` checks this by Monte Carlo. It projects tags exactly, adds Gaussian corner noise
of the assumed size, and counts how often the truth falls outside each published standard deviation's
95% range (5% would be exact):

| Scene (1 px noise) | x / y / yaw outside | Time per frame |
| --- | ---: | ---: |
| One tag, 1-4 m, up to 45° off its normal | 5.8 / 5.5 / 4.8% | 0.11 ms |
| One tag, 3-6 m, nearly head-on | 4.8 / 6.8 / 6.3% | 0.16 ms |
| Three tags on a wall, 2-6 m | 3.2 / 3.8 / 3.8% | 0.14 ms |
| Two tags 40 cm apart, 4-7 m, nearly head-on | 1.2 / 8.8 / 8.0% | 0.25 ms |
| Tags at two depths, 3.5-6 m | 5.8 / 3.8 / 4.8% | 0.06 ms |

The times are for every tag's pose and the robot's, on one desktop core. The hardest scene, two small
tags side by side seen head-on from afar, is the one no Gaussian describes well. In none of these
clean scenes was a tag wrongly left out as an outlier.

With the layout wrong about one tag (the solver told it's elsewhere along the wall), 2.5-5 m away:

| Scene (1 px noise) | Wrong tag left out | Another tag left out | x / y / yaw outside |
| --- | ---: | ---: | ---: |
| Four tags, one 0.5 m off | 100% of frames | 0 | 2.7 / 4.0 / 4.0% |
| Three tags, one 0.5 m off | 99.7% | 0 | 2.7 / 3.3 / 3.3% |
| Three tags, one 0.15 m off | 99.3% | 2 of 300 frames | 3.7 / 4.7 / 5.0% |
| Two tags, one 0.5 m off | No pose in 300 of 300 frames | | |

On synthetic frames with an exact camera model, tag positions come back within about 0.02% of the
distance and orientations within a degree (`tests/pose.rs`).

## Live settings

These can be changed while running, from a dashboard or the robot program, under `<table>/settings/`:

| Topic | Type | Option | |
| --- | --- | --- | --- |
| `exposure` | int | `--exposure` | Manual exposure in driver units (100 µs for UVC cameras); -1 for auto |
| `gain` | int | `--gain` | Analog gain in driver units; -1 leaves the camera's own |
| `focus` | int | `--focus` | Manual focus in driver units; -1 for autofocus |
| `min_contrast` | int | `--min-contrast` | Grey levels required between tag black and white (1-255) |
| `min_tag_px` | int | `--min-tag-px` | Smallest tag side to guarantee, in pixels (8-30000); smaller is slower |
| `max_hamming` | int | `--max-hamming` | Payload bits to correct (0-3); -1 for the family's recommendation |
| `min_border_cells` | int | `--min-border-cells` | Dark border cells required, out of 28 (0-28) |
| `sharpening` | double | `--sharpening` | Cell sharpening for blurred tags (0-1) |
| `max_ambiguity` | double | `--max-ambiguity` | Single-tag robot poses more ambiguous than this aren't published (0-1) |
| `corner_noise_px` | double | `--corner-noise-px` | Each corner coordinate's noise, in pixels, that the standard deviations assume (0.01-20) |
| `max_height` | double | `--max-height` | Robot poses more than this many metres off the floor aren't published (0-20) |
| `max_tilt` | double | `--max-tilt` | Robot poses tilted more than this many degrees from level aren't published (0-180) |
| `robot_to_camera` | double[6] | `--robot-to-camera` | The camera's pose on the robot: x, y, z (m), roll, pitch, yaw (degrees) |
| `stream_auto_levels` | boolean | `--stream-auto-levels` | Stretch the stream to the full grey range (default true) |
| `stream_gamma` | double | `--stream-gamma` | Stream gamma, 0.1-10; above 1 lifts shadows (default 1) |
| `stream_overlay` | boolean | `--stream-overlay` | Draw detected tags on the stream |
| `stream_quality` | int | `--stream-quality` | The stream's JPEG quality (1-100) |
| `stream_fps` | double | `--stream-fps` | Most frames per second to stream (0.1-1000) |
| `stream_scale` | int | `--stream-scale` | Divide the stream's size by this (1-8) |
| `field_layout/` | String Chooser | `--field-layout` | The field layout poses are solved against, or `none` |
| `capture_mode/` | String Chooser | `--width`, `--height`, `--fps`, `--format` | The camera's mode, e.g. `1280x720 MJPG 120fps` |

The choosers are laid out like WPILib's `SendableChooser`, so Elastic and Shuffleboard show them as
drop-downs: `options` (string[]), `default` (the configuration's), `active` (the one in use), and
`selected`, which switches when written. `field_layout/options` lists the layouts found plus `none`.
`capture_mode/options` lists the modes the camera offers in formats the detector reads (logged at
startup as `capture modes:`); choosing one reopens the camera, with a gap of about half a second, and
`active` follows once it's streaming. An empty `capture_mode/selected` means the configuration's mode.
Only `selected` is persistent. The stream's advertised mode follows the capture mode and the stream
settings.

They're persistent topics, so the robot's NT server saves them across restarts. **Values the server
already has win over the command line's and the configuration file's**: those only set settings the
server doesn't have yet. To start over from the command line, delete the topics on the server (in OutlineViewer,
for example). Invalid values are logged and ignored.

With the camera stream on, its server also serves the configuration in effect at
`http://<controller>:<stream port>/config.toml` (also published as `<table>/config_url`): the
configuration the service started with, plus the settings changed live. In a browser it's shown as
text; `/config.toml?download` saves it as `<camera>.toml` (from the table's last part, e.g.
`cam0.toml`). It's a complete `sonoma-vision.toml`, the shared file's values included, so installing it
as `/etc/sonoma-vision/cam0.toml` and restarting keeps the changes:

```sh
curl -o cam0.toml http://10.95.67.2:1190/config.toml
```

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
`--focus` (fixed focus, for calibrated poses), `--family`, `--min-tag-px`, `--nt-server` (`local`, a
team number, or an IPv4 address), `--table`, `--log-level`, and the `--stream` options above.

### Configuration file

Options can also be set in a `sonoma-vision.toml`. Each option is a key in its section, named without
the section's prefix (`--stream-port` is `port` in `[stream]`, `--nt-server` is `server` in `[nt]`, and
`--stream` itself is `enabled`):

```toml
[camera]
device = "/dev/v4l/by-id/usb-Generic_USB_Camera_0001-video-index0"
width = 1280
height = 720
fps = 120

[stream]
enabled = true

[pose]
calibration = "cam0-calibration.json"    # relative paths are relative to this file
robot_to_camera = [0.3, 0.0, 0.5, 0.0, -15.0, 0.0]
```

Without `--config`, `sonoma-vision.toml` is read from each of `/usr/share/sonoma-vision`,
`/usr/local/share/sonoma-vision`, the working directory and the executable's directory that has one
(where field layouts are looked for), a later directory's value for a key winning. `--config <file>`
reads only the files given, in order, a later file's value for a key winning: repeat it to layer a
camera's file over a shared one. Priority, lowest first: built-in defaults, the files, the command
line, then live settings persisted on the NT server. Unknown keys and invalid values are errors.
Configuration doesn't come from environment variables; the log level is `--log-level` (or `[log]
level`), which takes an env_logger filter such as `info,sonoma_vision::nt=debug`.

`--print-config` writes the configuration in effect, the files' values with the command line's
over them, to stdout as a `sonoma-vision.toml`: every key with its description, those at their
built-in default commented out. `--stream` and `--stream-overlay` take an optional value, so
`--stream=false` turns off a stream the file turns on.

Building needs libclang (v4l2r generates its V4L2 bindings with bindgen) and a C compiler, for
napa-vision's core.

### Releases

[Releases](https://github.com/compenguy/sonoma-vision/releases) carry a bundle per platform, each with
the binary, this README, `NOTICE` and the license texts (`LICENSE-*`), the field layouts (`fields/`)
and the licenses of vendored code built in (`licenses/`):

| Bundle | For |
| --- | --- |
| `sonoma-vision-<version>-systemcore.tar.gz` | SystemCore: tuned for its Cortex-A76, with `deploy/` (the systemd unit and example configuration) |
| `sonoma-vision-<version>-linux-aarch64.tar.gz` | Other 64-bit Arm Linux |
| `sonoma-vision-<version>-linux-x86_64.tar.gz` | x86-64 Linux |

Each needs glibc 2.35 or newer (Ubuntu 22.04, Debian 12, SystemCore). Pushing a tag `v<version>`
matching `Cargo.toml`'s version runs `.github/workflows/release.yml`, which tests, builds the bundles
(`tools/package.sh`) and publishes the release; running the workflow by hand only builds them.

### SystemCore

The SystemCore release bundle has everything below. To build it yourself instead,
`tools/build-systemcore.sh` cross-compiles for aarch64 (it needs `gcc-aarch64-linux-gnu` and
`libc6-dev-arm64-cross`; see the script), and `tools/package.sh aarch64-unknown-linux-gnu systemcore
--deploy` bundles it. The binary needs glibc 2.35 or newer. Then copy the binary to
`/usr/local/bin/`, the field layouts to `/usr/local/share/sonoma-vision/fields/`, the unit to `/etc/systemd/system/`, and the configuration to
`/etc/sonoma-vision/`: `sonoma-vision.toml` with the settings every camera shares, and a file per
camera (`cam0.toml`, with its device, table, stream port, calibration and pose on the robot) that the
unit reads after it:

```sh
sudo install -m 755 sonoma-vision /usr/local/bin/
sudo install -D -m 644 -t /usr/local/share/sonoma-vision/fields fields/*.json
sudo install -m 644 deploy/sonoma-vision@.service /etc/systemd/system/
sudo install -D -m 644 -t /etc/sonoma-vision deploy/sonoma-vision.toml deploy/cam0.toml
sudo systemctl daemon-reload && sudo systemctl enable --now sonoma-vision@cam0
```

## Testing

```sh
cargo test                                       # unit tests, synthetic accuracy, NT4 and stream end to end
cargo run --release --example bench -- --frames 50           # recall, corner error and latency per scenario
cargo run --release --example camera -- --device /dev/video0  # live capture and detection
cargo run --release --example list_modes -- /dev/video0       # a camera's formats, sizes and rates
```

`tests/nt_publish.rs` runs the publisher against a minimal in-process NT4 server, and checks that the
capture timestamps survive time sync; `tests/nt_settings.rs` checks live settings against one holding
persisted values.

Synthetic benchmark (`examples/bench.rs`, 40 frames per scenario, each detected 5 times on one pinned
desktop core, `taskset -c 2 … bench -- --frames 40 --repeat 5`), with napa-vision built standalone by its own CMake and
timed the same way on the same frames:

| Scenario | Recall | Wrong IDs | Median / p95 corner error | Median time (standalone NAPA) |
| --- | ---: | ---: | ---: | ---: |
| clean-640 | 100% | 0 | 0.016 / 0.045 px | 0.109 ms (0.100) |
| small-1280 (32–48 px tags) | 100% | 0 | 0.032 / 0.094 px | 0.301 ms (0.286) |
| large-1280 | 100% | 0 | 0.021 / 0.044 px | 0.134 ms (0.126) |
| perspective | 100% | 0 | 0.021 / 0.058 px | 0.195 ms (0.182) |
| blur+noise | 100% | 0 | 0.115 / 0.301 px | 0.437 ms (0.423) |
| clipped exposure | 100% | 0 | 0.038 / 0.130 px | 0.259 ms (0.239) |
| empty-1920 | — | 0 | — | 0.066 ms (0.062) |

Detections are identical to standalone NAPA's (every ID and corner, to six decimals). Timed in the
same loop as standalone NAPA (load a frame, detect it repeatedly), the Rust wrapper costs nothing
measurable (within ±2%); the gap above comes from `bench.rs` drawing each frame between detects,
which leaves the detector's buffers out of cache. Packed YUYV (`--yuyv`) costs a further 10–60 µs a frame, copying out the luma.
`examples/bench.rs` prints a checksum of every detection, to confirm a change alters nothing, and
`--detections <file>` writes them out for comparing against other detectors.

## Regenerating the tag families

`src/detector/families/` (the codes `src/synth.rs` draws tags from; a test checks they match
napa-vision's own tables) is generated from upstream AprilTag's family sources, vendored unmodified
in `vendor/apriltag/` (see its README for the commit):

```sh
python3 -I tools/gen_families.py
```
