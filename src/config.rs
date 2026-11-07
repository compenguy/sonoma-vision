//! The service's configuration file, `sonoma-vision.toml`.
//!
//! Values come, lowest priority first, from: built-in defaults, configuration files, the command
//! line, then (for live settings) values persisted on the NT server. The files' values become the
//! command line's defaults, so the command line parser validates them and anything given on the
//! command line wins.
//!
//! Without `--config`, `sonoma-vision.toml` is read from each of [`search_dirs`] that has one, as
//! field layouts are; a later directory's value for a key wins. With `--config <file>` (repeatable),
//! only those files are read, a later file's value for a key winning: a shared file, then a camera's.
//!
//! The file is in sections, and each key is its command line option without the section's prefix:
//!
//! ```toml
//! [camera]
//! device = "/dev/v4l/by-id/usb-Generic_USB_Camera_0001-video-index0"
//! fps = 120
//!
//! [stream]
//! enabled = true    # --stream
//! port = 1190       # --stream-port
//!
//! [pose]
//! calibration = "cam0-calibration.json"         # relative to this file
//! robot_to_camera = [0.3, 0.0, 0.5, 0.0, -15.0, 0.0]
//! ```

use std::collections::BTreeMap;
use std::ffi::OsString;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use clap::parser::ValueSource;
use clap::{ArgMatches, Command};

/// The configuration file's name, in each of [`search_dirs`].
pub const FILE_NAME: &str = "sonoma-vision.toml";

/// Where shared data and configuration are looked for, in order: the system and local share
/// directories, the working directory, then the executable's directory. Field layouts are in a
/// `fields` directory in each.
pub fn search_dirs() -> Vec<PathBuf> {
    let mut dirs = vec![PathBuf::from("/usr/share/sonoma-vision"), PathBuf::from("/usr/local/share/sonoma-vision")];
    if let Ok(cwd) = std::env::current_dir() {
        dirs.push(cwd);
    }
    if let Some(dir) = std::env::current_exe().ok().as_deref().and_then(Path::parent) {
        dirs.push(dir.to_path_buf());
    }
    dirs.dedup();
    dirs
}

/// How a key's value is written in the file.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Int,
    /// A number; integers are accepted.
    Float,
    Bool,
    Str,
    /// A file path; a relative one is relative to the configuration file.
    Path,
    /// A layout name, `none`, or a JSON file (ending in `.json` or containing a `/`), which, if
    /// relative, is relative to the configuration file.
    FieldLayout,
    /// x, y, z, roll, pitch, yaw: an array of 6 numbers, or the command line's comma-separated string.
    Transform,
}

/// A key of the file, and the command line option (clap argument id) it sets.
#[derive(Clone, Copy, Debug)]
pub struct Key {
    pub section: &'static str,
    pub name: &'static str,
    pub arg: &'static str,
    pub kind: Kind,
}

const fn key(section: &'static str, name: &'static str, arg: &'static str, kind: Kind) -> Key {
    Key { section, name, arg, kind }
}

/// Every key, in the order `--print-config` writes them.
pub const KEYS: &[Key] = &[
    key("camera", "device", "device", Kind::Str),
    key("camera", "width", "width", Kind::Int),
    key("camera", "height", "height", Kind::Int),
    key("camera", "fps", "fps", Kind::Int),
    key("camera", "format", "format", Kind::Str),
    key("camera", "buffers", "buffers", Kind::Int),
    key("camera", "exposure", "exposure", Kind::Int),
    key("camera", "gain", "gain", Kind::Int),
    key("camera", "focus", "focus", Kind::Int),
    key("detector", "family", "family", Kind::Str),
    key("detector", "min_tag_px", "min_tag_px", Kind::Int),
    key("detector", "min_contrast", "min_contrast", Kind::Int),
    key("detector", "max_hamming", "max_hamming", Kind::Int),
    key("detector", "min_border_cells", "min_border_cells", Kind::Int),
    key("detector", "sharpening", "sharpening", Kind::Float),
    key("nt", "server", "nt_server", Kind::Str),
    key("nt", "port", "nt_port", Kind::Int),
    key("nt", "name", "nt_name", Kind::Str),
    key("nt", "table", "table", Kind::Str),
    key("stream", "enabled", "stream", Kind::Bool),
    key("stream", "port", "stream_port", Kind::Int),
    key("stream", "fps", "stream_fps", Kind::Float),
    key("stream", "quality", "stream_quality", Kind::Int),
    key("stream", "scale", "stream_scale", Kind::Int),
    key("stream", "overlay", "stream_overlay", Kind::Bool),
    key("stream", "auto_levels", "stream_auto_levels", Kind::Bool),
    key("stream", "gamma", "stream_gamma", Kind::Float),
    key("stream", "name", "stream_name", Kind::Str),
    key("pose", "calibration", "calibration", Kind::Path),
    key("pose", "fov", "fov", Kind::Float),
    key("pose", "tag_size", "tag_size", Kind::Float),
    key("pose", "field_layout", "field_layout", Kind::FieldLayout),
    key("pose", "robot_to_camera", "robot_to_camera", Kind::Transform),
    key("pose", "max_ambiguity", "max_ambiguity", Kind::Float),
    key("pose", "corner_noise_px", "corner_noise_px", Kind::Float),
    key("pose", "max_height", "max_height", Kind::Float),
    key("pose", "max_tilt", "max_tilt", Kind::Float),
    key("log", "level", "log_level", Kind::Str),
];

const SECTIONS: &[(&str, &str)] = &[
    ("camera", "The V4L2 camera and its capture mode."),
    ("detector", "Tag detection."),
    ("nt", "NetworkTables publishing."),
    ("stream", "The MJPEG camera stream for dashboards."),
    ("pose", "Pose solving."),
    ("log", "Logging."),
];

/// The `--config` paths, in order, from the arguments (`--config <path>` or `--config=<path>`, before
/// any `--`). Read before the full parse, which needs the files' values as defaults.
pub fn explicit_paths(args: impl IntoIterator<Item = OsString>) -> Vec<PathBuf> {
    let mut args = args.into_iter().skip(1);
    let mut found = Vec::new();
    while let Some(a) = args.next() {
        if a == "--" {
            break;
        } else if a == "--config" {
            found.extend(args.next().map(PathBuf::from));
        } else if let Some(p) = a.to_str().and_then(|s| s.strip_prefix("--config=")) {
            found.push(PathBuf::from(p));
        }
    }
    found
}

/// Values read from configuration files, by argument id, as command line strings.
#[derive(Clone, Debug, Default)]
pub struct FileValues {
    /// The files read, in order.
    pub files: Vec<PathBuf>,
    /// Each set argument's value, and the file it came from.
    pub values: BTreeMap<&'static str, (String, PathBuf)>,
}

impl FileValues {
    /// The `explicit` files if any (they must exist), else every `sonoma-vision.toml` in `dirs`.
    pub fn load(cmd: &Command, explicit: &[PathBuf], dirs: &[PathBuf]) -> Result<Self> {
        let paths = if explicit.is_empty() {
            dirs.iter().map(|d| d.join(FILE_NAME)).filter(|p| p.is_file()).collect()
        } else {
            explicit.to_vec()
        };
        let mut out = Self::default();
        for path in paths {
            let text = std::fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?;
            out.add(cmd, &text, &path)?;
        }
        Ok(out)
    }

    /// Add one file's values over those already read.
    pub fn add(&mut self, cmd: &Command, text: &str, path: &Path) -> Result<()> {
        let at = || path.display().to_string();
        let table: toml::Table = text.parse().with_context(|| format!("parsing {}", at()))?;
        let base = path.parent().unwrap_or(Path::new("."));
        for (section, body) in &table {
            let Some(body) = body.as_table() else {
                bail!("{}: `{section}` must be a section ([{section}]); sections are {}", at(), section_names());
            };
            if !SECTIONS.iter().any(|(s, _)| s == section) {
                bail!("{}: unknown section [{section}]; sections are {}", at(), section_names());
            }
            for (name, value) in body {
                let Some(key) = KEYS.iter().find(|k| k.section == section && k.name == name) else {
                    let names: Vec<_> = KEYS.iter().filter(|k| k.section == section).map(|k| k.name).collect();
                    bail!("{}: unknown key `{name}` in [{section}]; keys are {}", at(), names.join(", "));
                };
                let raw = to_arg(key, value, base).with_context(|| format!("{}: [{section}] {name}", at()))?;
                check(cmd, key, &raw).with_context(|| format!("{}: [{section}] {name}", at()))?;
                self.values.insert(key.arg, (raw, path.to_path_buf()));
            }
        }
        self.files.push(path.to_path_buf());
        Ok(())
    }

    /// `cmd` with the files' values as its defaults.
    pub fn apply(&self, mut cmd: Command) -> Command {
        for (arg, (raw, _)) in &self.values {
            cmd = cmd.mut_arg(*arg, |a| a.default_value(raw.clone()));
        }
        cmd
    }
}

fn section_names() -> String {
    SECTIONS.iter().map(|(s, _)| format!("[{s}]")).collect::<Vec<_>>().join(", ")
}

/// A file value as the command line would give it.
fn to_arg(key: &Key, value: &toml::Value, base: &Path) -> Result<String> {
    use toml::Value as V;
    let path = |s: &str| base.join(s).to_string_lossy().into_owned();
    Ok(match (key.kind, value) {
        (Kind::Int, V::Integer(i)) => i.to_string(),
        (Kind::Float, V::Float(f)) => f.to_string(),
        (Kind::Float, V::Integer(i)) => i.to_string(),
        (Kind::Bool, V::Boolean(b)) => b.to_string(),
        (Kind::Str, V::String(s)) => s.clone(),
        (Kind::Path, V::String(s)) => path(s),
        (Kind::FieldLayout, V::String(s)) if is_layout_file(s) => path(s),
        (Kind::FieldLayout, V::String(s)) => s.clone(),
        (Kind::Transform, V::String(s)) => s.clone(),
        (Kind::Transform, V::Array(a)) => {
            let n: Option<Vec<String>> = a
                .iter()
                .map(|v| match v {
                    V::Float(f) => Some(f.to_string()),
                    V::Integer(i) => Some(i.to_string()),
                    _ => None,
                })
                .collect();
            match n {
                Some(n) if n.len() == 6 => n.join(","),
                _ => bail!("expected [x, y, z, roll, pitch, yaw]: 6 numbers (metres, degrees)"),
            }
        }
        (kind, v) => bail!("expected {}, not {}", kind_name(kind), v.type_str()),
    })
}

fn kind_name(kind: Kind) -> &'static str {
    match kind {
        Kind::Int => "an integer",
        Kind::Float => "a number",
        Kind::Bool => "true or false",
        Kind::Str | Kind::Path | Kind::FieldLayout => "a string",
        Kind::Transform => "[x, y, z, roll, pitch, yaw]",
    }
}

fn is_layout_file(s: &str) -> bool {
    s.ends_with(".json") || s.contains('/')
}

/// Validate a value with the option's own parser.
fn check(cmd: &Command, key: &Key, raw: &str) -> Result<()> {
    let flag = long_flag(cmd, key.arg);
    match cmd.clone().try_get_matches_from([OsString::from("sonoma-vision"), format!("--{flag}={raw}").into()]) {
        Ok(_) => Ok(()),
        Err(e) => {
            let msg = e.to_string();
            let first = msg.lines().next().unwrap_or_default();
            bail!("{}", first.strip_prefix("error: ").unwrap_or(first))
        }
    }
}

fn long_flag(cmd: &Command, arg: &str) -> String {
    cmd.get_arguments()
        .find(|a| a.get_id() == arg)
        .and_then(|a| a.get_long())
        .unwrap_or_else(|| panic!("no option {arg}"))
        .to_string()
}

/// The configuration in effect as a `sonoma-vision.toml`: each key set by the files, the command
/// line or `live`, and every other key commented out with its built-in default. `defaults` is the
/// command without the files' values; `matches` are the arguments parsed with them. `live` holds
/// options changed while running (see [`crate::settings::Settings::config_overrides`]), None for one
/// back at its built-in default. `note` opens the file.
pub fn render(
    defaults: &Command,
    matches: &ArgMatches,
    files: &FileValues,
    live: &[(&str, Option<String>)],
    note: &str,
) -> String {
    use std::fmt::Write as _;
    let mut out = String::new();
    for line in note.lines() {
        let _ = writeln!(out, "# {line}");
    }
    if files.files.is_empty() {
        out.push_str("# From the command line");
    } else {
        out.push_str("# From:\n");
        for f in &files.files {
            let _ = writeln!(out, "#   {}", f.display());
        }
        out.push_str("# with the command line's options over them");
    }
    out.push_str(if live.is_empty() { ".\n" } else { ", and the settings changed live over NT.\n" });
    out.push_str(
        "# Commented-out keys are at their built-in defaults. The command line overrides this file, and\n\
         # live settings persisted on the NT server override both.\n",
    );
    for (section, about) in SECTIONS {
        let _ = write!(out, "\n# {about}\n[{section}]\n");
        for key in KEYS.iter().filter(|k| k.section == *section) {
            let Some(arg) = defaults.get_arguments().find(|a| a.get_id() == key.arg) else { continue };
            let help = arg.get_long_help().or(arg.get_help()).map(|h| h.to_string()).unwrap_or_default();
            out.push('\n');
            for line in wrap(&format!("{help} (--{})", long_flag(defaults, key.arg)), 98) {
                let _ = writeln!(out, "# {line}");
            }
            let source = matches.value_source(key.arg);
            let raw = matches.get_raw(key.arg).and_then(|mut v| v.next()).map(|v| v.to_string_lossy().into_owned());
            let set = match source {
                Some(ValueSource::CommandLine) => true,
                Some(ValueSource::DefaultValue) => files.values.contains_key(key.arg),
                _ => false,
            };
            // The command line's relative paths are relative to the working directory.
            let configured = match raw {
                Some(raw) if set && source != Some(ValueSource::DefaultValue) => Some(absolute(key, raw)),
                Some(raw) if set => Some(raw),
                _ => None,
            };
            let value = match live.iter().find(|(id, _)| *id == key.arg) {
                Some((_, v)) => v.clone(),
                None => configured,
            };
            match value {
                Some(raw) => {
                    let _ = writeln!(out, "{} = {}", key.name, to_toml(key, &raw));
                }
                _ => match arg.get_default_values().first() {
                    Some(d) => {
                        let _ = writeln!(out, "# {} = {}", key.name, to_toml(key, &d.to_string_lossy()));
                    }
                    None => {
                        let _ = writeln!(out, "# {} =", key.name);
                    }
                },
            }
        }
    }
    out
}

fn absolute(key: &Key, raw: String) -> String {
    let file = key.kind == Kind::Path || (key.kind == Kind::FieldLayout && is_layout_file(&raw));
    match std::env::current_dir() {
        Ok(cwd) if file && Path::new(&raw).is_relative() => cwd.join(&raw).to_string_lossy().into_owned(),
        _ => raw,
    }
}

/// A command line value as a TOML value.
fn to_toml(key: &Key, raw: &str) -> String {
    use toml::Value as V;
    let value = match key.kind {
        Kind::Int => raw.parse().map(V::Integer).ok(),
        Kind::Float => raw.parse().map(V::Float).ok(),
        Kind::Bool => raw.parse().map(V::Boolean).ok(),
        Kind::Transform => raw
            .split(',')
            .map(|p| p.trim().parse().map(V::Float))
            .collect::<Result<Vec<_>, _>>()
            .ok()
            .filter(|v| v.len() == 6)
            .map(V::Array),
        Kind::Str | Kind::Path | Kind::FieldLayout => None,
    };
    value.unwrap_or_else(|| V::String(raw.to_string())).to_string()
}

/// `text` in lines of at most `width` characters, at spaces.
fn wrap(text: &str, width: usize) -> Vec<String> {
    let mut lines = Vec::new();
    for para in text.split('\n') {
        let mut line = String::new();
        for word in para.split_whitespace() {
            if !line.is_empty() && line.len() + 1 + word.len() > width {
                lines.push(std::mem::take(&mut line));
            }
            if !line.is_empty() {
                line.push(' ');
            }
            line.push_str(word);
        }
        lines.push(line);
    }
    lines
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::{Arg, ArgAction};

    fn cmd() -> Command {
        Command::new("t")
            .arg(Arg::new("device").long("device").default_value("/dev/video0"))
            .arg(Arg::new("width").long("width").value_parser(clap::value_parser!(u32)).default_value("1280"))
            .arg(Arg::new("fps").long("fps").value_parser(clap::value_parser!(u32)))
            .arg(Arg::new("table").long("table").default_value("/sonoma/cam0"))
            .arg(
                Arg::new("stream")
                    .long("stream")
                    .value_parser(clap::value_parser!(bool))
                    .num_args(0..=1)
                    .default_value("false")
                    .default_missing_value("true")
                    .action(ArgAction::Set),
            )
            .arg(
                Arg::new("stream_gamma").long("stream-gamma").value_parser(clap::value_parser!(f64)).default_value("1"),
            )
            .arg(Arg::new("calibration").long("calibration"))
            .arg(Arg::new("field_layout").long("field-layout"))
            .arg(
                Arg::new("robot_to_camera")
                    .long("robot-to-camera")
                    .allow_hyphen_values(true)
                    .default_value("0,0,0,0,0,0"),
            )
            .arg(Arg::new("config").long("config"))
    }

    fn file(text: &str) -> Result<FileValues> {
        let mut f = FileValues::default();
        f.add(&cmd(), text, Path::new("/etc/sonoma-vision/cam0.toml"))?;
        Ok(f)
    }

    fn parse(f: &FileValues, argv: &[&str]) -> ArgMatches {
        f.apply(cmd()).try_get_matches_from(std::iter::once("t").chain(argv.iter().copied())).unwrap()
    }

    const FILE: &str = r#"
[camera]
width = 640
fps = 120
[nt]
table = "/sonoma/front"
[stream]
enabled = true
gamma = 2
[pose]
calibration = "cal/cam0.json"
field_layout = "2026-rebuilt-welded"
robot_to_camera = [0.3, 0, 0.5, 0, -15, 0]
"#;

    #[test]
    fn file_values_are_defaults_the_command_line_overrides() {
        let f = file(FILE).unwrap();
        let m = parse(&f, &[]);
        assert_eq!(m.get_one::<u32>("width"), Some(&640));
        assert_eq!(m.get_one::<u32>("fps"), Some(&120));
        assert_eq!(m.get_one::<bool>("stream"), Some(&true));
        assert_eq!(m.get_one::<f64>("stream_gamma"), Some(&2.0));
        assert_eq!(m.get_one::<String>("calibration").unwrap(), "/etc/sonoma-vision/cal/cam0.json");
        assert_eq!(m.get_one::<String>("field_layout").unwrap(), "2026-rebuilt-welded");
        assert_eq!(m.get_one::<String>("robot_to_camera").unwrap(), "0.3,0,0.5,0,-15,0");
        // Not in the file: the built-in default.
        assert_eq!(m.get_one::<String>("device").unwrap(), "/dev/video0");
        // The command line wins, and can turn a flag off.
        let m = parse(&f, &["--width", "320", "--stream=false", "--calibration", "x.json"]);
        assert_eq!(m.get_one::<u32>("width"), Some(&320));
        assert_eq!(m.get_one::<bool>("stream"), Some(&false));
        assert_eq!(m.get_one::<String>("calibration").unwrap(), "x.json");
        // A bare flag is still true.
        assert_eq!(parse(&FileValues::default(), &["--stream"]).get_one::<bool>("stream"), Some(&true));
    }

    #[test]
    fn later_files_win_per_key() {
        let mut f = FileValues::default();
        f.add(&cmd(), "[camera]\nwidth = 640\nfps = 30\n", Path::new("/usr/share/sonoma-vision/sonoma-vision.toml"))
            .unwrap();
        f.add(&cmd(), "[camera]\nfps = 60\n", Path::new("/srv/sonoma-vision.toml")).unwrap();
        let m = parse(&f, &[]);
        assert_eq!((m.get_one::<u32>("width"), m.get_one::<u32>("fps")), (Some(&640), Some(&60)));
    }

    #[test]
    fn mistakes_name_the_file_and_key() {
        let err = |text: &str| format!("{:#}", file(text).unwrap_err());
        assert!(err("[camera]\nwidht = 1\n").contains("unknown key `widht` in [camera]"));
        assert!(err("[cam]\n").contains("unknown section [cam]"));
        assert!(err("width = 1\n").contains("must be a section"));
        let e = err("[camera]\nwidth = \"wide\"\n");
        assert!(e.contains("cam0.toml: [camera] width") && e.contains("expected an integer, not string"), "{e}");
        // The option's own parser checks the range.
        let e = err("[camera]\nwidth = -5\n");
        assert!(e.contains("[camera] width") && e.contains("invalid value"), "{e}");
        assert!(err("[pose]\nrobot_to_camera = [1, 2]\n").contains("6 numbers"));
        assert!(err("[camera]\nwidth = 1\n[camera\n").contains("parsing"));
    }

    #[test]
    fn explicit_paths_from_arguments() {
        let a = |v: &[&str]| v.iter().map(OsString::from).collect::<Vec<_>>();
        let p = |v: &[&str]| v.iter().map(PathBuf::from).collect::<Vec<_>>();
        assert_eq!(explicit_paths(a(&["sv", "--config", "a.toml"])), p(&["a.toml"]));
        assert_eq!(
            explicit_paths(a(&["sv", "--config=b.toml", "--fps", "3", "--config", "c.toml"])),
            p(&["b.toml", "c.toml"])
        );
        assert_eq!(explicit_paths(a(&["sv", "--", "--config", "c"])), p(&[]));
        assert_eq!(explicit_paths(a(&["sv"])), p(&[]));
    }

    #[test]
    fn print_config_round_trips() {
        let f = file(FILE).unwrap();
        let m = parse(&f, &["--fps", "60"]);
        let text = render(&cmd(), &m, &f, &[], "test");
        // The test command has only some of the keys; render skips the rest.
        let t: toml::Table = text.parse().unwrap_or_else(|e| panic!("{e}\n{text}"));
        assert_eq!(t["camera"]["fps"].as_integer(), Some(60), "the command line's value");
        assert_eq!(t["camera"]["width"].as_integer(), Some(640), "the file's value");
        assert_eq!(t["stream"]["gamma"].as_float(), Some(2.0));
        assert_eq!(t["pose"]["calibration"].as_str(), Some("/etc/sonoma-vision/cal/cam0.json"));
        assert_eq!(t["pose"]["robot_to_camera"].as_array().map(Vec::len), Some(6));
        assert!(t["camera"].get("device").is_none() && text.contains("# device = \"/dev/video0\""));
        // Reading it back gives the same values.
        let mut again = FileValues::default();
        again.add(&cmd(), &text, Path::new("/tmp/x/sonoma-vision.toml")).unwrap();
        let m2 = parse(&again, &[]);
        for id in ["width", "fps"] {
            assert_eq!(m2.get_one::<u32>(id), m.get_one::<u32>(id), "{id}");
        }
        assert_eq!(m2.get_one::<String>("calibration"), m.get_one::<String>("calibration"));
    }

    #[test]
    fn live_values_override_and_unset() {
        let f = file(FILE).unwrap();
        let m = parse(&f, &[]);
        let live = [("width", Some("320".to_string())), ("fps", None), ("stream_gamma", Some("1.5".to_string()))];
        let text = render(&cmd(), &m, &f, &live, "Live configuration of cam0");
        assert!(text.starts_with("# Live configuration of cam0\n"), "{text}");
        assert!(text.contains("changed live over NT"));
        let t: toml::Table = text.parse().unwrap();
        assert_eq!(t["camera"]["width"].as_integer(), Some(320));
        assert!(t["camera"].get("fps").is_none(), "unset live: commented out");
        assert_eq!(t["stream"]["gamma"].as_float(), Some(1.5));
        assert_eq!(t["stream"]["enabled"].as_bool(), Some(true), "untouched: the file's");
    }
}
