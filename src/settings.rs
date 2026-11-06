//! Settings that can be changed while running, over NT (see [`crate::nt`]): camera exposure and gain,
//! the detector's contrast threshold, and the stream's image adjustments.

use rmpv::Value;

#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
    /// Manual exposure in driver units (100 µs for UVC cameras); None for the camera's auto exposure.
    pub exposure: Option<i32>,
    /// Analog gain in driver units; None leaves the camera's own setting.
    pub gain: Option<i32>,
    /// Grey levels required between tag black and white.
    pub min_contrast: u8,
    /// Stretch each streamed frame's levels to the full range (the stream only, not detection).
    pub stream_auto_levels: bool,
    /// Gamma applied to the stream: above 1 lifts shadows, below 1 deepens them.
    pub stream_gamma: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Self { exposure: None, gain: None, min_contrast: 20, stream_auto_levels: true, stream_gamma: 1.0 }
    }
}

/// Each setting's NT name (under `<table>/settings/`) and type.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Key {
    /// int; -1 for auto exposure.
    Exposure,
    /// int; -1 to leave the camera's gain alone.
    Gain,
    /// int, 1-255.
    MinContrast,
    /// boolean.
    StreamAutoLevels,
    /// double, 0.1-10.
    StreamGamma,
}

impl Key {
    pub const ALL: [Key; 5] = [Key::Exposure, Key::Gain, Key::MinContrast, Key::StreamAutoLevels, Key::StreamGamma];

    pub fn name(self) -> &'static str {
        match self {
            Key::Exposure => "exposure",
            Key::Gain => "gain",
            Key::MinContrast => "min_contrast",
            Key::StreamAutoLevels => "stream_auto_levels",
            Key::StreamGamma => "stream_gamma",
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
}

impl Settings {
    pub fn get(&self, key: Key) -> NtValue {
        match key {
            Key::Exposure => NtValue::Int(self.exposure.map_or(-1, i64::from)),
            Key::Gain => NtValue::Int(self.gain.map_or(-1, i64::from)),
            Key::MinContrast => NtValue::Int(i64::from(self.min_contrast)),
            Key::StreamAutoLevels => NtValue::Bool(self.stream_auto_levels),
            Key::StreamGamma => NtValue::Double(self.stream_gamma),
        }
    }

    /// Set `key` from a value received over NT (a number of either type for numeric settings).
    /// Returns an error describing why a value was rejected; the setting is then unchanged.
    pub fn set(&mut self, key: Key, value: &Value) -> Result<(), String> {
        let number = || value.as_f64().or_else(|| value.as_i64().map(|v| v as f64));
        let int = |lo: i64, hi: i64| match number() {
            Some(v) if v.fract() == 0.0 && (lo as f64..=hi as f64).contains(&v) => Ok(v as i64),
            _ => Err(format!("{} must be a whole number from {lo} to {hi}, not {value}", key.name())),
        };
        match key {
            Key::Exposure => self.exposure = Some(int(-1, i64::from(i32::MAX))? as i32).filter(|&v| v > 0),
            Key::Gain => self.gain = Some(int(-1, i64::from(i32::MAX))? as i32).filter(|&v| v >= 0),
            Key::MinContrast => self.min_contrast = int(1, 255)? as u8,
            Key::StreamAutoLevels => {
                self.stream_auto_levels =
                    value.as_bool().ok_or_else(|| format!("stream_auto_levels must be a boolean, not {value}"))?
            }
            Key::StreamGamma => match number() {
                Some(g) if (0.1..=10.0).contains(&g) => self.stream_gamma = g,
                _ => return Err(format!("stream_gamma must be from 0.1 to 10, not {value}")),
            },
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        assert!(s.set(Key::MinContrast, &Value::from(0)).is_err());
        assert!(s.set(Key::MinContrast, &Value::from(300)).is_err());
        assert!(s.set(Key::StreamAutoLevels, &Value::from(1)).is_err());
        s.set(Key::StreamGamma, &Value::from(2)).unwrap();
        assert_eq!(s.stream_gamma, 2.0);
        assert!(s.set(Key::StreamGamma, &Value::from(0.0)).is_err());
        assert_eq!(s.min_contrast, 20, "rejected values leave the setting alone");
        for k in Key::ALL {
            assert_eq!(Key::from_name(k.name()), Some(k));
        }
    }
}
