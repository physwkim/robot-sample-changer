//! Loading and comment-preserving saving of `taught_waypoints.yaml`.
//!
//! Saving edits the file textually — the value after a scalar `key:` or
//! one entry of a flow list — so every comment and untouched line
//! survives, unlike a parse-and-dump. The edited text is parsed back and
//! the touched slots compared before it replaces the original
//! (temp + rename), the same discipline as the sequencer's holder-map
//! trim persist, which writes this file too.

use std::collections::BTreeMap;
use std::path::Path;

use serde_yaml::Value;

/// One editable location in the file.
#[derive(Clone, Debug, PartialEq)]
pub enum Slot {
    Scalar(&'static str),
    List(&'static str, usize),
}

/// The parameter map under `/**: ros__parameters:` (with fallbacks for
/// files that drop the wrapping, like the sequencer's loader).
pub fn load(path: &Path) -> Result<BTreeMap<String, Value>, String> {
    let text =
        std::fs::read_to_string(path).map_err(|e| format!("read {}: {e}", path.display()))?;
    let root: Value =
        serde_yaml::from_str(&text).map_err(|e| format!("parse {}: {e}", path.display()))?;
    let params = root
        .get("/**")
        .and_then(|v| v.get("ros__parameters"))
        .or_else(|| root.get("ros__parameters"))
        .unwrap_or(&root);
    let map = params
        .as_mapping()
        .ok_or_else(|| format!("{}: parameters are not a mapping", path.display()))?;
    Ok(map
        .iter()
        .filter_map(|(k, v)| Some((k.as_str()?.to_string(), v.clone())))
        .collect())
}

pub fn f64_at(params: &BTreeMap<String, Value>, key: &str) -> f64 {
    params.get(key).and_then(Value::as_f64).unwrap_or(0.0)
}

pub fn vec_at(params: &BTreeMap<String, Value>, key: &str, len: usize) -> Vec<f64> {
    match params.get(key).and_then(Value::as_sequence) {
        Some(list) => {
            let mut v: Vec<f64> = list.iter().filter_map(Value::as_f64).collect();
            v.resize(len, 0.0);
            v
        }
        None => vec![0.0; len],
    }
}

/// Rounds like the sequencer's persist: 7 decimals, 0.1 um in metres.
fn round7(v: f64) -> f64 {
    (v * 1e7).round() / 1e7
}

fn set_scalar_line(text: &str, key: &str, value: f64) -> Result<String, String> {
    let prefix = format!("{key}:");
    let mut hit = false;
    let mut lines: Vec<String> = Vec::new();
    for line in text.lines() {
        let trimmed = line.trim_start();
        if let Some(rest) = trimmed.strip_prefix(&prefix) {
            if hit {
                return Err(format!("'{key}' appears more than once"));
            }
            rest.trim()
                .parse::<f64>()
                .map_err(|e| format!("cannot parse '{key}' value: {e}"))?;
            let indent = &line[..line.len() - trimmed.len()];
            lines.push(format!("{indent}{key}: {value:?}"));
            hit = true;
            continue;
        }
        lines.push(line.to_string());
    }
    if !hit {
        return Err(format!("key '{key}' not found"));
    }
    Ok(lines.join("\n") + "\n")
}

fn set_list_entry(text: &str, key: &str, index: usize, value: f64) -> Result<String, String> {
    let prefix = format!("{key}:");
    let src: Vec<&str> = text.lines().collect();
    let mut out: Vec<String> = Vec::new();
    let mut hit = false;
    let mut i = 0;
    while i < src.len() {
        let line = src[i];
        let trimmed = line.trim_start();
        let Some(rest) = trimmed.strip_prefix(&prefix) else {
            out.push(line.to_string());
            i += 1;
            continue;
        };
        if hit {
            return Err(format!("'{key}' appears more than once"));
        }
        let indent = &line[..line.len() - trimmed.len()];
        let mut body = rest.trim().to_string();
        while !body.ends_with(']') {
            i += 1;
            let Some(cont) = src.get(i) else {
                return Err(format!("list '{key}' is not closed with ']'"));
            };
            body.push(' ');
            body.push_str(cont.trim());
        }
        let inner = body
            .strip_prefix('[')
            .and_then(|b| b.strip_suffix(']'))
            .ok_or_else(|| format!("'{key}' is not a flow list"))?;
        let mut values: Vec<f64> = Vec::new();
        for part in inner.split(',') {
            values.push(
                part.trim()
                    .parse()
                    .map_err(|e| format!("cannot parse '{key}' entry: {e}"))?,
            );
        }
        if index >= values.len() {
            return Err(format!(
                "'{key}' has {} entries, wanted index {index}",
                values.len()
            ));
        }
        values[index] = value;
        let joined = values
            .iter()
            .map(|v| format!("{v:?}"))
            .collect::<Vec<_>>()
            .join(", ");
        out.push(format!("{indent}{key}: [{joined}]"));
        hit = true;
        i += 1;
    }
    if !hit {
        return Err(format!("key '{key}' not found"));
    }
    Ok(out.join("\n") + "\n")
}

/// Writes the given absolute values into their slots. The whole batch
/// lands atomically or not at all.
pub fn apply_edits(path: &Path, edits: &[(Slot, f64)]) -> Result<(), String> {
    if edits.is_empty() {
        return Ok(());
    }
    let mut text =
        std::fs::read_to_string(path).map_err(|e| format!("read {}: {e}", path.display()))?;
    for (slot, value) in edits {
        let value = round7(*value);
        text = match slot {
            Slot::Scalar(key) => set_scalar_line(&text, key, value)?,
            Slot::List(key, index) => set_list_entry(&text, key, *index, value)?,
        };
    }
    let tmp = path.with_extension("yaml.new");
    std::fs::write(&tmp, &text).map_err(|e| format!("write {}: {e}", tmp.display()))?;
    let reread = load(&tmp).inspect_err(|_| {
        let _ = std::fs::remove_file(&tmp);
    })?;
    for (slot, value) in edits {
        let expected = round7(*value);
        let got = match slot {
            Slot::Scalar(key) => reread.get(*key).and_then(Value::as_f64),
            Slot::List(key, index) => reread
                .get(*key)
                .and_then(Value::as_sequence)
                .and_then(|l| l.get(*index))
                .and_then(Value::as_f64),
        };
        if got != Some(expected) {
            let _ = std::fs::remove_file(&tmp);
            return Err(format!(
                "verify failed for {slot:?}: wrote {expected:?}, read back {got:?}; \
                 the original file is untouched"
            ));
        }
    }
    std::fs::rename(&tmp, path).map_err(|e| format!("replace {}: {e}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn temp_copy(tag: &str) -> PathBuf {
        let src = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/taught_waypoints.yaml"
        ));
        let dst = std::env::temp_dir().join(format!(
            "robot_gui_yamledit_{tag}_{}.yaml",
            std::process::id()
        ));
        std::fs::copy(src, &dst).expect("copy");
        dst
    }

    #[test]
    fn edits_scalar_and_wrapped_list_preserving_comments() {
        let path = temp_copy("mix");
        apply_edits(
            &path,
            &[
                (Slot::Scalar("holder_on_position_tilt_z_deg"), 0.1),
                (Slot::List("holder_multi_z_offsets", 3), 0.00025),
            ],
        )
        .expect("apply");
        let params = load(&path).expect("reload");
        assert_eq!(f64_at(&params, "holder_on_position_tilt_z_deg"), 0.1);
        let z = vec_at(&params, "holder_multi_z_offsets", 9);
        assert_eq!(z[3], 0.00025);
        assert_eq!(z[4], 0.00025);
        let text = std::fs::read_to_string(&path).expect("read");
        assert!(text.contains("# Insertion-depth trim"));
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn refuses_unknown_keys_without_touching_the_file() {
        let path = temp_copy("bad");
        let before = std::fs::read_to_string(&path).expect("read");
        let err = apply_edits(&path, &[(Slot::Scalar("no_such_key"), 1.0)]);
        assert!(err.is_err());
        assert_eq!(std::fs::read_to_string(&path).expect("read"), before);
        std::fs::remove_file(&path).ok();
    }
}
