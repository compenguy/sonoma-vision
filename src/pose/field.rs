//! AprilTag field layouts in WPILib's JSON format (`AprilTagFieldLayout`'s `tags`/`field`, or its
//! newer `field-tags`/`field-dimensions`).
//!
//! Named layouts are read at startup from every `fields` directory in [`search_dirs`]; each `*.json`
//! there is a layout named by its file stem (`2026-rebuilt-welded.json` is `2026-rebuilt-welded`).
//! When several directories have a layout of the same name, the last directory searched wins.

use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use anyhow::{Context, Result, ensure};
use log::{debug, info, warn};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use serde_json::Value;

/// Where named layouts are looked for, in order: `fields` in each of [`crate::config::search_dirs`] (the
/// system and local share directories, the working directory, then the executable's directory).
pub fn search_dirs() -> Vec<PathBuf> {
    crate::config::search_dirs().into_iter().map(|d| d.join("fields")).collect()
}

/// The named layouts found in a set of directories.
#[derive(Clone, Debug, Default)]
pub struct Catalog {
    /// Each layout and the file it came from, by name.
    pub layouts: BTreeMap<String, (FieldLayout, PathBuf)>,
    /// The directories searched, in order.
    pub dirs: Vec<PathBuf>,
}

impl Catalog {
    /// Every layout in [`search_dirs`].
    pub fn discover() -> Self {
        Self::from_dirs(&search_dirs())
    }

    /// Every `*.json` layout in `dirs`; on a name clash the later directory wins. Directories that
    /// don't exist are skipped, and files that don't parse are skipped with a warning.
    pub fn from_dirs(dirs: &[PathBuf]) -> Self {
        let mut catalog = Self { layouts: BTreeMap::new(), dirs: Vec::new() };
        for dir in dirs {
            // The working directory and the executable's may be the same place.
            if catalog.dirs.contains(dir) {
                continue;
            }
            catalog.dirs.push(dir.clone());
            let entries = match std::fs::read_dir(dir) {
                Ok(entries) => entries,
                Err(e) => {
                    debug!("field layouts: skipping {}: {e}", dir.display());
                    continue;
                }
            };
            let mut paths: Vec<PathBuf> = entries
                .filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| p.extension().is_some_and(|x| x == "json") && p.is_file())
                .collect();
            paths.sort();
            for path in paths {
                let Some(name) = path.file_stem().and_then(|s| s.to_str()).map(str::to_string) else { continue };
                match FieldLayout::from_file(&path) {
                    Ok(layout) => {
                        debug!("field layout {name}: {}", path.display());
                        if let Some((_, earlier)) = catalog.layouts.insert(name.clone(), (layout, path.clone())) {
                            info!("field layout {name}: {} overrides {}", path.display(), earlier.display());
                        }
                    }
                    Err(e) => warn!("field layout {}: {e:#}", path.display()),
                }
            }
        }
        catalog
    }

    pub fn get(&self, name: &str) -> Option<&FieldLayout> {
        self.layouts.get(name).map(|(layout, _)| layout)
    }

    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.layouts.keys().map(String::as_str)
    }
}

/// The layouts in [`search_dirs`], read on first use.
pub fn catalog() -> &'static Catalog {
    static CATALOG: OnceLock<Catalog> = OnceLock::new();
    CATALOG.get_or_init(Catalog::discover)
}

#[derive(Clone, Debug)]
pub struct FieldLayout {
    pub name: String,
    /// Each tag's pose on the field (WPILib field coordinates; the tag's +X points out of its face).
    pub tags: HashMap<u32, Isometry3<f64>>,
    pub length: f64,
    pub width: f64,
}

impl FieldLayout {
    /// A layout from the [`catalog`] by name, or a layout file by path.
    pub fn load(spec: &str) -> Result<Self> {
        let catalog = catalog();
        if let Some(layout) = catalog.get(spec) {
            return Ok(layout.clone());
        }
        if Path::new(spec).is_file() {
            return Self::from_file(Path::new(spec));
        }
        let names: Vec<_> = catalog.names().collect();
        let dirs: Vec<_> = catalog.dirs.iter().map(|d| d.display().to_string()).collect();
        anyhow::bail!(
            "field layout {spec:?} is neither a file nor a known layout (found: {}; searched {})",
            if names.is_empty() { "none".to_string() } else { names.join(", ") },
            dirs.join(", ")
        )
    }

    pub fn from_file(path: &Path) -> Result<Self> {
        let json = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        Self::parse(&json).with_context(|| format!("field layout {}", path.display()))
    }

    pub fn parse(json: &str) -> Result<Self> {
        let v: Value = serde_json::from_str(json).context("not JSON")?;
        let tags = v.get("tags").or_else(|| v.get("field-tags")).and_then(Value::as_array).context("no tags")?;
        let field = v.get("field").or_else(|| v.get("field-dimensions")).context("no field dimensions")?;
        let num = |v: &Value, what: &str| v.as_f64().with_context(|| format!("{what} is not a number"));
        let mut poses = HashMap::new();
        for tag in tags {
            let id = tag["ID"].as_u64().context("tag without an ID")? as u32;
            let p = &tag["pose"];
            let t = &p["translation"];
            let q = &p["rotation"]["quaternion"];
            let translation = Translation3::new(num(&t["x"], "x")?, num(&t["y"], "y")?, num(&t["z"], "z")?);
            let q = Quaternion::new(num(&q["W"], "W")?, num(&q["X"], "X")?, num(&q["Y"], "Y")?, num(&q["Z"], "Z")?);
            ensure!(q.norm() > 0.5, "tag {id} has a degenerate rotation");
            poses.insert(id, Isometry3::from_parts(translation, UnitQuaternion::from_quaternion(q)));
        }
        let name = v["name"].as_str().unwrap_or("custom").to_string();
        Ok(Self { name, tags: poses, length: num(&field["length"], "length")?, width: num(&field["width"], "width")? })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shipped_layouts_load() {
        // Tests run in the crate root, so ./fields is the repository's.
        for name in ["2026-rebuilt-welded", "2026-rebuilt-andymark"] {
            let layout = FieldLayout::load(name).unwrap();
            assert_eq!(layout.tags.len(), 32, "{name}");
            assert!((16.0..17.0).contains(&layout.length) && (8.0..8.2).contains(&layout.width));
        }
        // Tag 1 on the welded field faces -X (rotated 180° about Z).
        let tag1 = FieldLayout::load("2026-rebuilt-welded").unwrap().tags[&1];
        let out = tag1.rotation * nalgebra::Vector3::x();
        assert!((out - -nalgebra::Vector3::x()).norm() < 1e-9);
        assert!(FieldLayout::load("no-such-field").is_err());
    }

    #[test]
    fn catalog_accumulates_and_later_directories_win() {
        let root = std::env::temp_dir().join(format!("sonoma-fields-{}", std::process::id()));
        let (a, b) = (root.join("a"), root.join("b"));
        std::fs::create_dir_all(&a).unwrap();
        std::fs::create_dir_all(&b).unwrap();
        let layout = |len: f64| format!(r#"{{"tags": [], "field": {{"length": {len}, "width": 8.0}}}}"#);
        std::fs::write(a.join("shared.json"), layout(1.0)).unwrap();
        std::fs::write(a.join("only-a.json"), layout(2.0)).unwrap();
        std::fs::write(b.join("shared.json"), layout(3.0)).unwrap();
        std::fs::write(b.join("only-b.json"), layout(4.0)).unwrap();
        std::fs::write(b.join("broken.json"), "not json").unwrap();
        std::fs::write(b.join("notes.txt"), "ignored").unwrap();
        let catalog = Catalog::from_dirs(&[a.clone(), root.join("missing"), b, a]);
        std::fs::remove_dir_all(&root).unwrap();
        assert_eq!(catalog.names().collect::<Vec<_>>(), ["only-a", "only-b", "shared"]);
        assert_eq!(catalog.get("shared").unwrap().length, 3.0);
        assert_eq!(catalog.get("only-b").unwrap().length, 4.0);
        assert_eq!(catalog.dirs.len(), 3);
    }

    #[test]
    fn parses_the_classic_format() {
        let json = r#"{"tags": [{"ID": 7, "pose": {"translation": {"x": 1, "y": 2, "z": 0.5},
            "rotation": {"quaternion": {"W": 1, "X": 0, "Y": 0, "Z": 0}}}}],
            "field": {"length": 16.0, "width": 8.0}}"#;
        let layout = FieldLayout::parse(json).unwrap();
        assert_eq!(layout.tags[&7].translation.vector, nalgebra::Vector3::new(1.0, 2.0, 0.5));
    }
}
