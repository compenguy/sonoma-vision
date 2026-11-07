//! Settings that can be changed while running, over NT (see [`crate::nt`]): camera exposure, gain and
//! focus, the detector's thresholds, the pose solver's mount, limits and noise model, the camera
//! stream's look, rate and size, the field layout, and the capture mode.
//!
//! Each starts from the configuration (files and command line) and each has the same name as its
//! command line option, so the configuration in effect can be written back out with the live values
//! (see [`Settings::config_overrides`]).

use std::ops::RangeInclusive;

use rmpv::Value;

use crate::capture::Mode;
use crate::detector;
use crate::pose::{MountPose, PoseBounds};

/// Each numeric setting's valid values, for the command line and NT alike.
pub mod limits {
    use std::ops::RangeInclusive;

    pub const MIN_CONTRAST: RangeInclusive<i64> = 1..=255;
    pub const MIN_TAG_PX: RangeInclusive<i64> = 8..=30000;
    pub const MAX_HAMMING: RangeInclusive<i64> = 0..=3;
    pub const MIN_BORDER_CELLS: RangeInclusive<i64> = 0..=28;
    pub const STREAM_QUALITY: RangeInclusive<i64> = 1..=100;
    pub const STREAM_SCALE: RangeInclusive<i64> = 1..=8;
    pub const SHARPENING: RangeInclusive<f64> = 0.0..=1.0;
    pub const MAX_AMBIGUITY: RangeInclusive<f64> = 0.0..=1.0;
    pub const CORNER_NOISE_PX: RangeInclusive<f64> = 0.01..=20.0;
    pub const MAX_HEIGHT: RangeInclusive<f64> = 0.0..=20.0;
    pub const MAX_TILT: RangeInclusive<f64> = 0.0..=180.0;
    pub const STREAM_GAMMA: RangeInclusive<f64> = 0.1..=10.0;
    pub const STREAM_FPS: RangeInclusive<f64> = 0.1..=1000.0;

    /// A command line parser for a number within `range`.
    pub fn decimal(range: RangeInclusive<f64>) -> impl Fn(&str) -> Result<f64, String> + Clone + Send + Sync {
        move |s| match s.parse::<f64>() {
            Ok(v) if range.contains(&v) => Ok(v),
            Ok(_) => Err(format!("{s} is not in {}..={}", range.start(), range.end())),
            Err(e) => Err(e.to_string()),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
    /// Manual exposure in driver units (100 µs for UVC cameras); None for the camera's auto exposure.
    pub exposure: Option<i32>,
    /// Analog gain in driver units; None leaves the camera's own setting.
    pub gain: Option<i32>,
    /// Manual focus in driver units; None for the camera's own (autofocus, once changed live).
    pub focus: Option<i32>,
    /// Grey levels required between tag black and white.
    pub min_contrast: u8,
    /// Smallest tag side, in pixels, the detector guarantees to find.
    pub min_tag_px: u32,
    /// Payload bits to correct; None for the family's recommendation.
    pub max_hamming: Option<u32>,
    /// Dark border cells required, out of tag36h11's 28.
    pub min_border_cells: u32,
    /// Cell sharpening strength, 0-1.
    pub sharpening: f64,
    /// Single-tag robot poses more ambiguous than this (0-1) aren't published.
    pub max_ambiguity: f64,
    /// Each corner coordinate's noise, in pixels, that pose covariances assume.
    pub corner_noise_px: f64,
    /// Most metres the robot can be off the floor; poses higher are dropped.
    pub max_height: f64,
    /// Most degrees the robot can tilt from level; poses tilted more are dropped.
    pub max_tilt: f64,
    /// The camera's pose on the robot.
    pub robot_to_camera: MountPose,
    /// Stretch each streamed frame's levels to the full range (the stream only, not detection).
    pub stream_auto_levels: bool,
    /// Gamma applied to the stream: above 1 lifts shadows, below 1 deepens them.
    pub stream_gamma: f64,
    /// Draw each detected tag on the stream.
    pub stream_overlay: bool,
    /// The stream's JPEG quality, 1-100.
    pub stream_quality: u8,
    /// Most frames per second to stream.
    pub stream_fps: f64,
    /// Divide the stream's width and height by this, 1-8.
    pub stream_scale: u8,
    /// The field layout poses are solved against, by name; None turns pose solving off.
    pub field_layout: Option<String>,
    /// A capture mode chosen live; None for the configuration's.
    pub capture_mode: Option<Mode>,
}

impl Settings {
    /// The built-in defaults, which the command line's options default to as well.
    pub const DEFAULT: Settings = Settings {
        exposure: None,
        gain: None,
        focus: None,
        min_contrast: detector::Config::DEFAULT.min_contrast,
        min_tag_px: detector::Config::DEFAULT.min_tag_px,
        max_hamming: detector::Config::DEFAULT.max_hamming,
        min_border_cells: detector::Config::DEFAULT.min_border_cells,
        sharpening: detector::Config::DEFAULT.sharpening,
        max_ambiguity: 0.2,
        corner_noise_px: 1.0,
        max_height: PoseBounds::DEFAULT.max_height,
        max_tilt: PoseBounds::DEFAULT.max_tilt_deg,
        robot_to_camera: MountPose([0.0; 6]),
        stream_auto_levels: true,
        stream_gamma: 1.0,
        stream_overlay: false,
        stream_quality: 50,
        stream_fps: 15.0,
        stream_scale: 1,
        field_layout: None,
        capture_mode: None,
    };
}

impl Default for Settings {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Each setting's NT name (under `<table>/settings/`), which is also its command line option's.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Key {
    /// int; -1 for auto exposure.
    Exposure,
    /// int; -1 to leave the camera's gain alone.
    Gain,
    /// int; -1 for autofocus.
    Focus,
    /// int, 1-255.
    MinContrast,
    /// int, 8-30000.
    MinTagPx,
    /// int, 0-3; -1 for the family's recommendation.
    MaxHamming,
    /// int, 0-28.
    MinBorderCells,
    /// double, 0-1.
    Sharpening,
    /// double, 0-1.
    MaxAmbiguity,
    /// double, 0.01-20.
    CornerNoisePx,
    /// double, 0-20.
    MaxHeight,
    /// double, 0-180.
    MaxTilt,
    /// `double[6]`: x, y, z (metres), roll, pitch, yaw (degrees).
    RobotToCamera,
    /// boolean.
    StreamAutoLevels,
    /// double, 0.1-10.
    StreamGamma,
    /// boolean.
    StreamOverlay,
    /// int, 1-100.
    StreamQuality,
    /// double, 0.1-1000.
    StreamFps,
    /// int, 1-8.
    StreamScale,
}

impl Key {
    pub const ALL: [Key; 19] = [
        Key::Exposure,
        Key::Gain,
        Key::Focus,
        Key::MinContrast,
        Key::MinTagPx,
        Key::MaxHamming,
        Key::MinBorderCells,
        Key::Sharpening,
        Key::MaxAmbiguity,
        Key::CornerNoisePx,
        Key::MaxHeight,
        Key::MaxTilt,
        Key::RobotToCamera,
        Key::StreamAutoLevels,
        Key::StreamGamma,
        Key::StreamOverlay,
        Key::StreamQuality,
        Key::StreamFps,
        Key::StreamScale,
    ];

    /// The NT topic's name, and the command line option's (argument id).
    pub fn name(self) -> &'static str {
        match self {
            Key::Exposure => "exposure",
            Key::Gain => "gain",
            Key::Focus => "focus",
            Key::MinContrast => "min_contrast",
            Key::MinTagPx => "min_tag_px",
            Key::MaxHamming => "max_hamming",
            Key::MinBorderCells => "min_border_cells",
            Key::Sharpening => "sharpening",
            Key::MaxAmbiguity => "max_ambiguity",
            Key::CornerNoisePx => "corner_noise_px",
            Key::MaxHeight => "max_height",
            Key::MaxTilt => "max_tilt",
            Key::RobotToCamera => "robot_to_camera",
            Key::StreamAutoLevels => "stream_auto_levels",
            Key::StreamGamma => "stream_gamma",
            Key::StreamOverlay => "stream_overlay",
            Key::StreamQuality => "stream_quality",
            Key::StreamFps => "stream_fps",
            Key::StreamScale => "stream_scale",
        }
    }

    pub fn from_name(name: &str) -> Option<Key> {
        Self::ALL.into_iter().find(|k| k.name() == name)
    }
}

/// A setting's value as published to NT.
#[derive(Clone, Debug, PartialEq)]
pub enum NtValue {
    Int(i64),
    Bool(bool),
    Double(f64),
    DoubleArray(Vec<f64>),
}

impl Settings {
    pub fn get(&self, key: Key) -> NtValue {
        let opt = |v: Option<i32>| NtValue::Int(v.map_or(-1, i64::from));
        match key {
            Key::Exposure => opt(self.exposure),
            Key::Gain => opt(self.gain),
            Key::Focus => opt(self.focus),
            Key::MinContrast => NtValue::Int(i64::from(self.min_contrast)),
            Key::MinTagPx => NtValue::Int(i64::from(self.min_tag_px)),
            Key::MaxHamming => NtValue::Int(self.max_hamming.map_or(-1, i64::from)),
            Key::MinBorderCells => NtValue::Int(i64::from(self.min_border_cells)),
            Key::Sharpening => NtValue::Double(self.sharpening),
            Key::MaxAmbiguity => NtValue::Double(self.max_ambiguity),
            Key::CornerNoisePx => NtValue::Double(self.corner_noise_px),
            Key::MaxHeight => NtValue::Double(self.max_height),
            Key::MaxTilt => NtValue::Double(self.max_tilt),
            Key::RobotToCamera => NtValue::DoubleArray(self.robot_to_camera.0.to_vec()),
            Key::StreamAutoLevels => NtValue::Bool(self.stream_auto_levels),
            Key::StreamGamma => NtValue::Double(self.stream_gamma),
            Key::StreamOverlay => NtValue::Bool(self.stream_overlay),
            Key::StreamQuality => NtValue::Int(i64::from(self.stream_quality)),
            Key::StreamFps => NtValue::Double(self.stream_fps),
            Key::StreamScale => NtValue::Int(i64::from(self.stream_scale)),
        }
    }

    /// Set `key` from a value received over NT (a number of either type for numeric settings).
    /// Returns an error describing why a value was rejected; the setting is then unchanged.
    pub fn set(&mut self, key: Key, value: &Value) -> Result<(), String> {
        let name = key.name();
        let as_number = |v: &Value| v.as_f64().or_else(|| v.as_i64().map(|i| i as f64));
        let number = || as_number(value);
        let int = |range: RangeInclusive<i64>| match number() {
            Some(v) if v.fract() == 0.0 && (*range.start() as f64..=*range.end() as f64).contains(&v) => Ok(v as i64),
            _ => Err(format!("{name} must be a whole number from {} to {}, not {value}", range.start(), range.end())),
        };
        let real = |range: RangeInclusive<f64>| match number() {
            Some(v) if range.contains(&v) => Ok(v),
            _ => Err(format!("{name} must be from {} to {}, not {value}", range.start(), range.end())),
        };
        let boolean = || value.as_bool().ok_or_else(|| format!("{name} must be a boolean, not {value}"));
        // -1 (and, for exposure, 0, which no camera means as an exposure) is "the camera's own".
        let camera_int = |min: i64| int(-1..=i64::from(i32::MAX)).map(|v| Some(v as i32).filter(|&v| v >= min as i32));
        match key {
            Key::Exposure => self.exposure = camera_int(1)?,
            Key::Gain => self.gain = camera_int(0)?,
            Key::Focus => self.focus = camera_int(0)?,
            Key::MinContrast => self.min_contrast = int(limits::MIN_CONTRAST)? as u8,
            Key::MinTagPx => self.min_tag_px = int(limits::MIN_TAG_PX)? as u32,
            // -1: the family's recommendation.
            Key::MaxHamming => self.max_hamming = u32::try_from(int(-1..=*limits::MAX_HAMMING.end())?).ok(),
            Key::MinBorderCells => self.min_border_cells = int(limits::MIN_BORDER_CELLS)? as u32,
            Key::Sharpening => self.sharpening = real(limits::SHARPENING)?,
            Key::MaxAmbiguity => self.max_ambiguity = real(limits::MAX_AMBIGUITY)?,
            Key::CornerNoisePx => self.corner_noise_px = real(limits::CORNER_NOISE_PX)?,
            Key::MaxHeight => self.max_height = real(limits::MAX_HEIGHT)?,
            Key::MaxTilt => self.max_tilt = real(limits::MAX_TILT)?,
            Key::RobotToCamera => {
                let v: Option<Vec<f64>> = value.as_array().and_then(|a| a.iter().map(as_number).collect());
                let v = v.ok_or_else(|| format!("{name} must be a double[6], not {value}"))?;
                self.robot_to_camera = MountPose::from_slice(&v).map_err(|e| format!("{name}: {e}"))?;
            }
            Key::StreamAutoLevels => self.stream_auto_levels = boolean()?,
            Key::StreamGamma => self.stream_gamma = real(limits::STREAM_GAMMA)?,
            Key::StreamOverlay => self.stream_overlay = boolean()?,
            Key::StreamQuality => self.stream_quality = int(limits::STREAM_QUALITY)? as u8,
            Key::StreamFps => self.stream_fps = real(limits::STREAM_FPS)?,
            Key::StreamScale => self.stream_scale = int(limits::STREAM_SCALE)? as u8,
        }
        Ok(())
    }

    /// Select the field layout `name`, one of `available`, or `none` (or empty) for no poses.
    pub fn set_field_layout(&mut self, name: &str, available: &[String]) -> Result<(), String> {
        self.field_layout = match name {
            "" | NO_LAYOUT => None,
            name if available.iter().any(|a| a == name) => Some(name.to_string()),
            name => return Err(format!("no field layout {name:?}; one of {}, or {NO_LAYOUT}", available.join(", "))),
        };
        Ok(())
    }

    /// The field layout's name as published to NT: `none` for no layout.
    pub fn field_layout_name(&self) -> &str {
        self.field_layout.as_deref().unwrap_or(NO_LAYOUT)
    }

    /// Select a capture mode by its name (`1280x720 MJPG 120fps`); empty for the configuration's.
    pub fn set_capture_mode(&mut self, name: &str) -> Result<(), String> {
        self.capture_mode = if name.is_empty() { None } else { Some(name.parse()?) };
        Ok(())
    }

    /// The command line options (argument ids) whose values here differ from `initial`'s, with the
    /// values as the command line would give them; None for an option back at its built-in default.
    /// Writing these over the configuration gives the configuration in effect.
    pub fn config_overrides(&self, initial: &Settings) -> Vec<(&'static str, Option<String>)> {
        let mut out = Vec::new();
        for key in Key::ALL {
            if self.get(key) != initial.get(key) {
                out.push((key.name(), self.arg_value(key)));
            }
        }
        if self.field_layout != initial.field_layout {
            out.push(("field_layout", Some(self.field_layout_name().to_string())));
        }
        if self.capture_mode != initial.capture_mode
            && let Some(m) = self.capture_mode
        {
            let fourcc = std::str::from_utf8(m.layout.fourcc()).unwrap_or_default().to_ascii_lowercase();
            out.extend([
                ("width", Some(m.width.to_string())),
                ("height", Some(m.height.to_string())),
                ("fps", Some(m.fps.to_string())),
                ("format", Some(fourcc)),
            ]);
        }
        out
    }

    /// `key`'s value as its command line option takes it; None where the option is left unset.
    fn arg_value(&self, key: Key) -> Option<String> {
        match key {
            Key::Exposure => self.exposure.map(|v| v.to_string()),
            Key::Gain => self.gain.map(|v| v.to_string()),
            Key::Focus => self.focus.map(|v| v.to_string()),
            Key::MaxHamming => self.max_hamming.map(|v| v.to_string()),
            Key::RobotToCamera => Some(self.robot_to_camera.to_string()),
            _ => Some(match self.get(key) {
                NtValue::Int(v) => v.to_string(),
                NtValue::Bool(v) => v.to_string(),
                NtValue::Double(v) => v.to_string(),
                NtValue::DoubleArray(v) => v.iter().map(f64::to_string).collect::<Vec<_>>().join(","),
            }),
        }
    }
}

/// The field layout name that turns pose solving off.
pub const NO_LAYOUT: &str = "none";

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::Layout;

    #[test]
    fn round_trips_and_validates() {
        let mut s = Settings::default();
        s.set(Key::Exposure, &Value::from(50)).unwrap();
        assert_eq!(s.exposure, Some(50));
        assert_eq!(s.get(Key::Exposure), NtValue::Int(50));
        // -1 (and 0, which no camera means as an exposure) is auto.
        s.set(Key::Exposure, &Value::from(-1)).unwrap();
        assert_eq!(s.exposure, None);
        assert_eq!(s.get(Key::Exposure), NtValue::Int(-1));
        // Whole doubles are accepted for integer settings, as a dashboard may send them.
        s.set(Key::Gain, &Value::from(120.0)).unwrap();
        assert_eq!(s.gain, Some(120));
        assert!(s.set(Key::Gain, &Value::from(1.5)).is_err());
        s.set(Key::Focus, &Value::from(0)).unwrap();
        assert_eq!(s.focus, Some(0), "focus 0 is a focus (infinity, often), not autofocus");
        s.set(Key::Focus, &Value::from(-1)).unwrap();
        assert_eq!(s.focus, None);
        assert!(s.set(Key::MinContrast, &Value::from(0)).is_err());
        assert!(s.set(Key::MinContrast, &Value::from(300)).is_err());
        assert!(s.set(Key::StreamAutoLevels, &Value::from(1)).is_err());
        s.set(Key::StreamGamma, &Value::from(2)).unwrap();
        assert_eq!(s.stream_gamma, 2.0);
        assert!(s.set(Key::StreamGamma, &Value::from(0.0)).is_err());
        assert_eq!(s.min_contrast, 20, "rejected values leave the setting alone");
        s.set(Key::MaxHamming, &Value::from(1)).unwrap();
        assert_eq!(s.max_hamming, Some(1));
        s.set(Key::MaxHamming, &Value::from(-1)).unwrap();
        assert_eq!((s.max_hamming, s.get(Key::MaxHamming)), (None, NtValue::Int(-1)));
        assert!(s.set(Key::MaxHamming, &Value::from(4)).is_err());
        assert!(s.set(Key::MinTagPx, &Value::from(7)).is_err());
        assert!(s.set(Key::Sharpening, &Value::from(1.5)).is_err());
        assert!(s.set(Key::CornerNoisePx, &Value::from(0.0)).is_err());
        s.set(Key::MaxTilt, &Value::from(45)).unwrap();
        assert_eq!(s.max_tilt, 45.0);
        assert!(s.set(Key::StreamScale, &Value::from(9)).is_err());
        let mount = Value::Array([0.3, 0.0, 0.5, 0.0, -15.0, 0.0].map(Value::from).to_vec());
        s.set(Key::RobotToCamera, &mount).unwrap();
        assert_eq!(s.robot_to_camera, MountPose([0.3, 0.0, 0.5, 0.0, -15.0, 0.0]));
        assert_eq!(s.get(Key::RobotToCamera), NtValue::DoubleArray(vec![0.3, 0.0, 0.5, 0.0, -15.0, 0.0]));
        assert!(s.set(Key::RobotToCamera, &Value::Array(vec![Value::from(1.0)])).is_err());
        assert!(s.set(Key::RobotToCamera, &Value::from("0,0,0,0,0,0")).is_err());
        let available = vec!["a".to_string(), "b".to_string()];
        s.set_field_layout("b", &available).unwrap();
        assert_eq!(s.field_layout.as_deref(), Some("b"));
        assert!(s.set_field_layout("c", &available).is_err());
        assert_eq!(s.field_layout_name(), "b");
        s.set_field_layout("none", &available).unwrap();
        assert_eq!(s.field_layout, None);
        assert_eq!(s.field_layout_name(), "none");
        s.set_capture_mode("640x480 MJPG 120fps").unwrap();
        assert_eq!(s.capture_mode, Some(Mode { width: 640, height: 480, fps: 120, layout: Layout::Mjpeg }));
        assert!(s.set_capture_mode("fast please").is_err());
        s.set_capture_mode("").unwrap();
        assert_eq!(s.capture_mode, None);
        for k in Key::ALL {
            assert_eq!(Key::from_name(k.name()), Some(k));
        }
    }

    #[test]
    fn overrides_are_the_changed_settings_as_options() {
        let initial = Settings { exposure: Some(50), field_layout: Some("a".into()), ..Settings::default() };
        assert!(initial.config_overrides(&initial).is_empty());
        let mut live = initial.clone();
        live.exposure = None;
        live.min_tag_px = 24;
        live.robot_to_camera = MountPose([0.3, 0.0, 0.5, 0.0, -15.0, 0.0]);
        live.stream_overlay = true;
        live.field_layout = None;
        live.capture_mode = Some(Mode { width: 640, height: 480, fps: 120, layout: Layout::Mjpeg });
        let o: std::collections::HashMap<_, _> = live.config_overrides(&initial).into_iter().collect();
        assert_eq!(o["exposure"], None, "back to auto: the option unset");
        assert_eq!(o["min_tag_px"].as_deref(), Some("24"));
        assert_eq!(o["robot_to_camera"].as_deref(), Some("0.3,0,0.5,0,-15,0"));
        assert_eq!(o["stream_overlay"].as_deref(), Some("true"));
        assert_eq!(o["field_layout"].as_deref(), Some("none"));
        assert_eq!(
            ["width", "height", "fps", "format"].map(|k| o[k].clone().unwrap()),
            ["640", "480", "120", "mjpg"].map(String::from)
        );
        assert_eq!(o.len(), 9, "only what changed: {o:?}");
    }
}
