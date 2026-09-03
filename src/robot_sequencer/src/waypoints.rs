//! Taught-waypoint file loader.
//!
//! Same file the ROS node consumed (`taught_waypoints.yaml`, ROS parameter
//! layout kept so calibrated values carry over unchanged): values live
//! under `/**: ros__parameters:`, with the same fallbacks the C++ loader
//! had (`ros__parameters` at root, then root itself). Reloaded before
//! every sequence, so operators can re-teach between runs.

use std::path::{Path, PathBuf};

use serde_yaml::Value;

use crate::error::SequencerError;

/// Joint-value vectors are 7 wide in the file:
/// `[gripper_finger, shoulder_pan, wrist_3, wrist_2, wrist_1, elbow,
/// shoulder_lift]` (the teach tool recorded /joint_states order). Index 0
/// is the gripper finger and is not an arm joint.
pub const WAYPOINT_JOINT_ORDER: [&str; 6] = [
    "shoulder_pan_joint",
    "wrist_3_joint",
    "wrist_2_joint",
    "wrist_1_joint",
    "elbow_joint",
    "shoulder_lift_joint",
];

/// Seats on the rack. Every per-holder list is exactly this long and
/// holder N lives at index N-1 — holder 1 included, which is the whole
/// point: it is a seat like the other nine, not the rack's origin.
pub const HOLDERS: usize = 10;
const HOLDERS_I32: i32 = HOLDERS as i32;

#[derive(Debug, Clone)]
pub struct WaypointData {
    pub holder1_standby: Vec<f64>,
    pub holder1_on_position: Vec<f64>,
    pub sample_holder_standby: Vec<f64>,
    pub sample_holder_on_position: Vec<f64>,
    pub above_y_offset: f64,
    pub retreat_z_offset: f64,
    /// How far the holder seat sits above the taught pose, m.
    ///
    /// Its own field and not part of the shared trim above, because that
    /// trim is applied to the standby pose as well and the standby pose
    /// cannot be planned to: it reads as a collision against the convex
    /// stage, and the sequence only gets away with it by being already
    /// there. Move it by a tenth of a millimetre and step 1 stops being
    /// a no-op and starts being a planning failure (measured
    /// 2026-08-18).
    pub holder_on_lift: f64,
    /// How far the grip pose is turned about base +x before the fingers
    /// close, deg, positive toward the direction the tilt scan found
    /// soft.
    ///
    /// The scan that motivated it (2026-08-18, gentle grip, raised
    /// seat): from the seated grip, -0.05 deg about base x met 0.386 Nm
    /// at once — 7.7 Nm/deg — while +0.30 deg swept clean at a ninth of
    /// that rate. The taught orientation holds the puck pitched against
    /// one side of its seat, and closing the fingers on that pitch is
    /// what loads base y at every height the bracket can reach.
    pub holder_on_tilt_x_deg: f64,
    /// The other lean axis: how far the grip pose is turned about tool
    /// z (base +y), deg. Together with the x angle this fixes the full
    /// seat lean; spin about the puck's own axis (tool y) is not a
    /// lean and stays with [`Self::wrist3_rotation_offset`].
    pub holder_on_tilt_z_deg: f64,
    /// The rack's own base correction, m, applied to BOTH holder poses
    /// before the per-holder step and before the rack pitch/roll.
    ///
    /// This is rack geometry, not holder 1's seat: holder 1 has its own
    /// entry in the multi lists like every other holder. The two used to
    /// be the same number, which meant trimming holder 1 walked all ten
    /// seats sideways.
    pub rack_x_offset: f64,
    pub rack_y_offset: f64,
    pub rack_z_offset: f64,
    pub sample_holder_on_x_offset: f64,
    pub sample_holder_on_y_offset: f64,
    pub sample_holder_on_z_offset: f64,
    pub holder_multi_x_offsets: Vec<f64>,
    /// Per-holder insertion-depth trim, tool-frame y (base −z), meters,
    /// holder N at index N-1: positive is deeper. The rail step itself is
    /// exact 30 mm; this carries each seat's own depth error, which the
    /// x/z trims cannot reach.
    pub holder_multi_y_offsets: Vec<f64>,
    pub holder_multi_z_offsets: Vec<f64>,
    /// Per-holder trim added to [`Self::holder_on_tilt_x_deg`], deg,
    /// holder N at index N-1 like the multi offsets. Each holder's seat
    /// leans by its own manufacturing error; the shared angle carries
    /// what the puck geometry needs and this carries the rest.
    pub holder_multi_tilt_x_deg: Vec<f64>,
    /// Per-holder trim added to [`Self::holder_on_tilt_z_deg`], deg,
    /// holder N at index N-1, same shape as the x list.
    pub holder_multi_tilt_z_deg: Vec<f64>,
    pub wrist3_rotation_offset: f64,
}

impl WaypointData {
    /// One holder's own seat-lean trim, deg. The shared
    /// [`Self::holder_on_tilt_x_deg`] is the rack's rigid-body pitch
    /// and is applied to the base waypoints; this is only what that
    /// holder's seat adds on top.
    pub fn holder_tilt_x_trim_deg(&self, holder: i32) -> f64 {
        if (1..=HOLDERS_I32).contains(&holder) {
            self.holder_multi_tilt_x_deg
                .get((holder - 1) as usize)
                .copied()
                .unwrap_or(0.0)
        } else {
            0.0
        }
    }

    /// The tool-z twin of [`Self::holder_tilt_x_trim_deg`].
    pub fn holder_tilt_z_trim_deg(&self, holder: i32) -> f64 {
        if (1..=HOLDERS_I32).contains(&holder) {
            self.holder_multi_tilt_z_deg
                .get((holder - 1) as usize)
                .copied()
                .unwrap_or(0.0)
        } else {
            0.0
        }
    }
}

/// Rounds to 7 decimals (0.1 um in metres): enough for any trim, short
/// enough that the file stays readable.
fn round7(v: f64) -> f64 {
    (v * 1e7).round() / 1e7
}

/// Replaces one entry of a top-level flow-list `key: [..]`, consuming a
/// list wrapped over several lines and re-emitting it on one. Everything
/// outside the list block is preserved byte for byte.
fn edit_list_entry(
    text: &str,
    key: &str,
    index: usize,
    delta: f64,
) -> Result<(String, f64, f64), SequencerError> {
    let prefix = format!("{key}:");
    let src: Vec<&str> = text.lines().collect();
    let mut out: Vec<String> = Vec::new();
    let mut hit = None;
    let mut i = 0;
    while i < src.len() {
        let line = src[i];
        let trimmed = line.trim_start();
        let Some(rest) = trimmed.strip_prefix(&prefix) else {
            out.push(line.to_string());
            i += 1;
            continue;
        };
        if hit.is_some() {
            return Err(SequencerError(format!(
                "waypoints: '{key}' appears more than once; refusing to edit"
            )));
        }
        let indent = &line[..line.len() - trimmed.len()];
        let mut body = rest.trim().to_string();
        while !body.ends_with(']') {
            i += 1;
            let Some(cont) = src.get(i) else {
                return Err(SequencerError(format!(
                    "waypoints: list '{key}' is not closed with ']'"
                )));
            };
            body.push(' ');
            body.push_str(cont.trim());
        }
        let inner = body
            .strip_prefix('[')
            .and_then(|b| b.strip_suffix(']'))
            .ok_or_else(|| SequencerError(format!("waypoints: '{key}' is not a flow list")))?;
        let mut values: Vec<f64> = Vec::new();
        for part in inner.split(',') {
            values.push(part.trim().parse().map_err(|e| {
                SequencerError(format!("waypoints: cannot parse '{key}' entry: {e}"))
            })?);
        }
        let old = *values.get(index).ok_or_else(|| {
            SequencerError(format!(
                "waypoints: '{key}' has {} entries, wanted index {index}",
                values.len()
            ))
        })?;
        let new = round7(old + delta);
        values[index] = new;
        let joined = values
            .iter()
            .map(|v| format!("{v:?}"))
            .collect::<Vec<_>>()
            .join(", ");
        out.push(format!("{indent}{key}: [{joined}]"));
        hit = Some((old, new));
        i += 1;
    }
    let (old, new) = hit.ok_or_else(|| {
        SequencerError(format!(
            "waypoints: key '{key}' not found; refusing to edit"
        ))
    })?;
    Ok((out.join("\n") + "\n", old, new))
}

/// The scalar twin of [`edit_list_entry`]: replaces the number on a
/// `key: <n>` line, wherever it is indented. Same one-hit rule, so a key
/// that appears twice is refused rather than half-edited.
fn edit_scalar(text: &str, key: &str, delta: f64) -> Result<(String, f64, f64), SequencerError> {
    let prefix = format!("{key}:");
    let mut out: Vec<String> = Vec::new();
    let mut hit = None;
    for line in text.lines() {
        let trimmed = line.trim_start();
        let Some(rest) = trimmed.strip_prefix(&prefix) else {
            out.push(line.to_string());
            continue;
        };
        if hit.is_some() {
            return Err(SequencerError(format!(
                "waypoints: '{key}' appears more than once; refusing to edit"
            )));
        }
        let indent = &line[..line.len() - trimmed.len()];
        let old: f64 = rest.trim().parse().map_err(|e| {
            SequencerError(format!("waypoints: cannot parse '{key}' as a number: {e}"))
        })?;
        let new = round7(old + delta);
        out.push(format!("{indent}{key}: {new:?}"));
        hit = Some((old, new));
    }
    let (old, new) = hit.ok_or_else(|| {
        SequencerError(format!(
            "waypoints: key '{key}' not found; refusing to edit"
        ))
    })?;
    Ok((out.join("\n") + "\n", old, new))
}

/// Where one trim lives in the file. The rack keeps ten of each in a
/// flow list; the stage, being one seat, keeps three scalars. Nothing
/// else about writing a trim differs between them, so this is the only
/// place the difference is spelt out.
#[derive(Clone, Copy)]
enum SlotRef {
    Scalar(&'static str),
    Entry(&'static str, usize),
}

impl SlotRef {
    fn edit(self, text: &str, delta: f64) -> Result<(String, f64, f64), SequencerError> {
        match self {
            Self::Scalar(key) => edit_scalar(text, key, delta),
            Self::Entry(key, index) => edit_list_entry(text, key, index, delta),
        }
    }

    fn name(self) -> String {
        match self {
            Self::Scalar(key) => key.to_string(),
            Self::Entry(key, index) => format!("{key}[{index}]"),
        }
    }
}

/// Where a write is staged before the rename that commits it: this
/// file's name plus this process's id.
///
/// The name has to be the writer's own. The daemon rewrites trims from
/// a grip null or a jog apply, the GUI's Teach page rewrites the same
/// file from the same editor, and both stage under the file's own
/// directory so the rename is atomic. With one shared name they
/// truncate each other's staging file, verify text the other wrote, or
/// rename a half-written one over the original. Only the staging is
/// private; the rename is still the commit, and both writers re-read
/// the file before editing it, so the later one wins whole.
fn staging_name(path: &Path) -> PathBuf {
    path.with_extension(format!("yaml.new.{}", std::process::id()))
}

/// Adds measured deltas (metres) to one seat's three trim slots in the
/// taught-waypoints file, editing the text in place so the comments and
/// every untouched line survive (unlike a parse-and-dump). The edited
/// text is parsed back and the touched fields compared to what was
/// intended before it replaces the original (temp + rename), so a write
/// that cannot be read back never lands. Returns one line per slot
/// written, for the caller's log.
fn persist_trims(
    path: &Path,
    seat: &str,
    slots: [(&str, Option<f64>, SlotRef, ReadBack); 3],
) -> Result<Vec<String>, SequencerError> {
    let mut text = std::fs::read_to_string(path)
        .map_err(|e| SequencerError(format!("cannot read waypoints {}: {e}", path.display())))?;
    let mut report = Vec::new();
    let mut checks: Vec<(String, SlotRef, f64, ReadBack)> = Vec::new();
    for (axis, delta, slot, read_back) in slots {
        let Some(delta) = delta else { continue };
        let (new_text, old, new) = slot.edit(&text, delta)?;
        let slot_name = slot.name();
        text = new_text;
        report.push(format!(
            "{seat} {axis} trim ({slot_name}): {old:?} -> {new:?} ({:+.3} mm)",
            delta * 1000.0
        ));
        checks.push((slot_name, slot, new, read_back));
    }
    if checks.is_empty() {
        return Ok(report);
    }
    let tmp = staging_name(path);
    std::fs::write(&tmp, &text)
        .map_err(|e| SequencerError(format!("cannot write {}: {e}", tmp.display())))?;
    let reread = WaypointData::load(&tmp).inspect_err(|_| {
        let _ = std::fs::remove_file(&tmp);
    })?;
    for (slot_name, slot, expected, read_back) in &checks {
        let index = match slot {
            SlotRef::Scalar(_) => 0,
            SlotRef::Entry(_, index) => *index,
        };
        let got = read_back(&reread, index);
        if got != Some(*expected) {
            let _ = std::fs::remove_file(&tmp);
            return Err(SequencerError(format!(
                "waypoints: verify failed for {slot_name}: wrote {expected:?}, read back {got:?}; \
                 the original file is untouched"
            )));
        }
    }
    std::fs::rename(&tmp, path)
        .map_err(|e| SequencerError(format!("cannot replace {}: {e}", path.display())))?;
    Ok(report)
}

/// Reads a trim back out of a reloaded file. The index is the list
/// position and is ignored by a scalar slot, which has only the one.
type ReadBack = fn(&WaypointData, usize) -> Option<f64>;

/// One holder's three rack trims. One shape for all ten holders: holder
/// 1 used to be written to a scalar that every other holder also read,
/// so mapping it moved the whole rack.
pub fn persist_holder_trims(
    path: &Path,
    holder: i32,
    dx_m: Option<f64>,
    dy_m: Option<f64>,
    dz_m: Option<f64>,
) -> Result<Vec<String>, SequencerError> {
    if !(1..=HOLDERS_I32).contains(&holder) {
        return Err(SequencerError(format!(
            "waypoints: holder {holder} has no trim slots"
        )));
    }
    let i = (holder - 1) as usize;
    persist_trims(
        path,
        &format!("holder {holder}"),
        [
            (
                "x",
                dx_m,
                SlotRef::Entry("holder_multi_x_offsets", i),
                |w, i| w.holder_multi_x_offsets.get(i).copied(),
            ),
            (
                "y",
                dy_m,
                SlotRef::Entry("holder_multi_y_offsets", i),
                |w, i| w.holder_multi_y_offsets.get(i).copied(),
            ),
            (
                "z",
                dz_m,
                SlotRef::Entry("holder_multi_z_offsets", i),
                |w, i| w.holder_multi_z_offsets.get(i).copied(),
            ),
        ],
    )
}

/// The stage's three trims. Scalars rather than a list because there is
/// one stage bore, but the same slots in the same tool frame as a
/// holder's: `sample_holder_on_position` is offset by them the way a
/// rack seat is offset by its list entries.
pub fn persist_stage_trims(
    path: &Path,
    dx_m: Option<f64>,
    dy_m: Option<f64>,
    dz_m: Option<f64>,
) -> Result<Vec<String>, SequencerError> {
    persist_trims(
        path,
        "stage",
        [
            (
                "x",
                dx_m,
                SlotRef::Scalar("sample_holder_on_position_x_offset"),
                |w, _| Some(w.sample_holder_on_x_offset),
            ),
            (
                "y",
                dy_m,
                SlotRef::Scalar("sample_holder_on_position_y_offset"),
                |w, _| Some(w.sample_holder_on_y_offset),
            ),
            (
                "z",
                dz_m,
                SlotRef::Scalar("sample_holder_on_position_z_offset"),
                |w, _| Some(w.sample_holder_on_z_offset),
            ),
        ],
    )
}

fn f64_at(params: &Value, key: &str, default: f64) -> f64 {
    params.get(key).and_then(Value::as_f64).unwrap_or(default)
}

fn vec_at(params: &Value, key: &str) -> Result<Vec<f64>, SequencerError> {
    let list = params
        .get(key)
        .and_then(Value::as_sequence)
        .ok_or_else(|| SequencerError(format!("waypoints: missing or non-list '{key}'")))?;
    list.iter()
        .map(|v| {
            v.as_f64()
                .ok_or_else(|| SequencerError(format!("waypoints: non-number in '{key}'")))
        })
        .collect()
}

/// One per-holder list, holder N at index N-1.
///
/// Length is checked rather than padded. These lists used to run from
/// holder 2 and be read with `get(N-2).unwrap_or(0.0)`, so a file still
/// written that way would now hand holder 1 the trim taught for holder 2
/// and every seat after it the one belonging to its neighbour — nine
/// wrong poses driven into a rack, with nothing said. Absent is fine and
/// means untrimmed; present and short is a file from before the move.
fn seats_at(params: &Value, key: &str) -> Result<Vec<f64>, SequencerError> {
    let Some(list) = params.get(key).and_then(Value::as_sequence) else {
        return Ok(vec![0.0; HOLDERS]);
    };
    if list.len() != HOLDERS {
        return Err(SequencerError(format!(
            "waypoints: '{key}' has {} entries, expected {HOLDERS} (holder N at index N-1). \
             A {} entry list is from before holder 1 had a seat of its own; give it a \
             leading entry for holder 1 and move the rack-wide part to holder_rack_*_offset",
            list.len(),
            HOLDERS - 1
        )));
    }
    list.iter()
        .map(|v| {
            v.as_f64()
                .ok_or_else(|| SequencerError(format!("waypoints: non-number in '{key}'")))
        })
        .collect()
}

impl WaypointData {
    pub fn load(path: &Path) -> Result<Self, SequencerError> {
        let text = std::fs::read_to_string(path).map_err(|e| {
            SequencerError(format!("cannot read waypoints {}: {e}", path.display()))
        })?;
        let root: Value = serde_yaml::from_str(&text).map_err(|e| {
            SequencerError(format!("cannot parse waypoints {}: {e}", path.display()))
        })?;
        let params = root
            .get("/**")
            .and_then(|v| v.get("ros__parameters"))
            .or_else(|| root.get("ros__parameters"))
            .unwrap_or(&root);

        // The rack base used to be spelled holder1_on_position_*_offset
        // and doubled as holder 1's own trim. Reading a file that still
        // spells it that way would silently zero the rack base — every
        // seat off by the same tenths of a millimetre — so say so.
        for axis in ["x", "y", "z"] {
            let old_key = format!("holder1_on_position_{axis}_offset");
            if params.get(old_key.as_str()).is_some() {
                return Err(SequencerError(format!(
                    "waypoints: '{old_key}' is the rack base under its old name, which also \
                     served as holder 1's own trim; rename it to 'holder_rack_{axis}_offset' \
                     and give holder 1 its own leading entry in the holder_multi_* lists"
                )));
            }
        }

        let data = Self {
            holder1_standby: vec_at(params, "holder1_standby")?,
            holder1_on_position: vec_at(params, "holder1_on_position")?,
            sample_holder_standby: vec_at(params, "sample_holder_standby")?,
            sample_holder_on_position: vec_at(params, "sample_holder_on_position")?,
            above_y_offset: f64_at(params, "above_y_offset", -0.005),
            retreat_z_offset: f64_at(params, "retreat_z_offset", -0.05),
            holder_on_lift: f64_at(params, "holder_on_position_lift", 0.0),
            holder_on_tilt_x_deg: f64_at(params, "holder_on_position_tilt_x_deg", 0.0),
            holder_on_tilt_z_deg: f64_at(params, "holder_on_position_tilt_z_deg", 0.0),
            rack_x_offset: f64_at(params, "holder_rack_x_offset", 0.0),
            rack_y_offset: f64_at(params, "holder_rack_y_offset", 0.0),
            rack_z_offset: f64_at(params, "holder_rack_z_offset", 0.0),
            sample_holder_on_x_offset: f64_at(params, "sample_holder_on_position_x_offset", 0.0),
            sample_holder_on_y_offset: f64_at(params, "sample_holder_on_position_y_offset", 0.0),
            sample_holder_on_z_offset: f64_at(params, "sample_holder_on_position_z_offset", 0.0),
            holder_multi_x_offsets: seats_at(params, "holder_multi_x_offsets")?,
            holder_multi_y_offsets: seats_at(params, "holder_multi_y_offsets")?,
            holder_multi_z_offsets: seats_at(params, "holder_multi_z_offsets")?,
            holder_multi_tilt_x_deg: seats_at(params, "holder_multi_tilt_x_deg")?,
            holder_multi_tilt_z_deg: seats_at(params, "holder_multi_tilt_z_deg")?,
            wrist3_rotation_offset: f64_at(params, "wrist3_rotation_offset", 0.0),
        };

        for (key, list) in [
            ("holder1_standby", &data.holder1_standby),
            ("holder1_on_position", &data.holder1_on_position),
            ("sample_holder_standby", &data.sample_holder_standby),
            ("sample_holder_on_position", &data.sample_holder_on_position),
        ] {
            if list.len() != 7 {
                return Err(SequencerError(format!(
                    "waypoints: '{key}' has {} values, expected 7 (gripper + 6 arm joints)",
                    list.len()
                )));
            }
        }
        Ok(data)
    }

    /// Arm joints of a 7-wide taught vector as `(name, value)` pairs
    /// (drops the gripper finger at index 0).
    pub fn arm_joints(values: &[f64]) -> Vec<(String, f64)> {
        WAYPOINT_JOINT_ORDER
            .iter()
            .zip(&values[1..])
            .map(|(name, value)| (name.to_string(), *value))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loads_the_production_file() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/taught_waypoints.yaml"
        ));
        let data = WaypointData::load(path).expect("load");
        assert_eq!(data.holder1_standby.len(), 7);
        assert_eq!(data.above_y_offset, -0.005);
        assert_eq!(data.retreat_z_offset, -0.05);
        assert_eq!(data.rack_y_offset, 0.0005);
        assert_eq!(data.holder_on_lift, 0.00015);
        assert_eq!(data.holder_on_tilt_x_deg, 0.3);
        assert_eq!(data.holder_multi_x_offsets.len(), HOLDERS);
        assert_eq!(data.holder_multi_tilt_x_deg.len(), HOLDERS);
        assert_eq!(data.holder_on_tilt_z_deg, 0.0);
        assert_eq!(data.holder_multi_tilt_z_deg.len(), HOLDERS);
        assert_eq!(data.holder_tilt_x_trim_deg(1), 0.0);
        assert_eq!(data.holder_tilt_z_trim_deg(2), 0.0);
        assert_eq!(data.wrist3_rotation_offset, 0.0);
    }

    fn temp_copy(tag: &str) -> std::path::PathBuf {
        let src = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/taught_waypoints.yaml"
        ));
        let dst = std::env::temp_dir().join(format!(
            "taught_waypoints_persist_{tag}_{}.yaml",
            std::process::id()
        ));
        std::fs::copy(src, &dst).expect("copy");
        dst
    }

    /// Holder 1 is a seat like the other nine: its trim lands in the
    /// list at index 0 and the rack base it used to share does not move.
    /// That sharing is what made a holder-1 trim walk all ten seats.
    #[test]
    fn persist_edits_holder1_without_moving_the_rack() {
        let path = temp_copy("h1");
        let before = WaypointData::load(&path).expect("load");
        let report = persist_holder_trims(&path, 1, Some(0.0001), None, None).expect("persist");
        assert_eq!(report.len(), 1, "{report:?}");
        let data = WaypointData::load(&path).expect("reload");
        assert_eq!(
            data.holder_multi_x_offsets[0],
            round7(before.holder_multi_x_offsets[0] + 0.0001)
        );
        assert_eq!(data.rack_x_offset, before.rack_x_offset, "rack base held");
        assert_eq!(data.rack_y_offset, before.rack_y_offset, "rack base held");
        assert_eq!(data.rack_z_offset, before.rack_z_offset, "rack base held");
        assert_eq!(
            data.holder_multi_x_offsets[1], before.holder_multi_x_offsets[1],
            "holder 2 untouched"
        );
        let text = std::fs::read_to_string(&path).expect("read");
        assert!(text.contains("# Insertion-depth trim"), "comments survive");
        assert!(text.contains("holder_on_position_tilt_x_deg: 0.3"));
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn persist_edits_a_wrapped_multi_list_entry() {
        let path = temp_copy("multi");
        // holder 4 -> index 3; both slots at once. The z list wraps two
        // lines in the production file, which is the case this exercises.
        let before = WaypointData::load(&path).expect("load");
        let report =
            persist_holder_trims(&path, 4, Some(-0.0005), None, Some(0.0002)).expect("persist");
        assert_eq!(report.len(), 2, "{report:?}");
        let data = WaypointData::load(&path).expect("reload");
        assert_eq!(
            data.holder_multi_x_offsets[3],
            round7(before.holder_multi_x_offsets[3] - 0.0005)
        );
        assert_eq!(
            data.holder_multi_z_offsets[3],
            round7(before.holder_multi_z_offsets[3] + 0.0002)
        );
        // neighbours untouched, including the far end of the wrapped list
        assert_eq!(
            data.holder_multi_x_offsets[2],
            before.holder_multi_x_offsets[2]
        );
        assert_eq!(
            data.holder_multi_z_offsets[4],
            before.holder_multi_z_offsets[4]
        );
        assert_eq!(
            data.holder_multi_z_offsets[9],
            before.holder_multi_z_offsets[9]
        );
        std::fs::remove_file(&path).ok();
    }

    /// The depth slot, which the grip null writes from the close's base
    /// z force. Read
    /// relative to what the file already holds: these are live trims an
    /// operator edits, so an absolute expectation is a fixture that goes
    /// stale rather than a property of the writer.
    #[test]
    fn persist_edits_the_depth_slot_beside_the_lateral_ones() {
        let path = temp_copy("depth");
        let before = WaypointData::load(&path).expect("load");
        let report =
            persist_holder_trims(&path, 7, Some(0.0001), Some(-0.00002), None).expect("persist");
        assert_eq!(report.len(), 2, "{report:?}");
        let after = WaypointData::load(&path).expect("reload");
        // Holder 7 -> index 6.
        assert_eq!(
            after.holder_multi_y_offsets[6],
            round7(before.holder_multi_y_offsets[6] - 0.00002)
        );
        assert_eq!(
            after.holder_multi_x_offsets[6],
            round7(before.holder_multi_x_offsets[6] + 0.0001)
        );
        // The axis that was not measured leaves its slot alone.
        assert_eq!(
            after.holder_multi_z_offsets[6],
            before.holder_multi_z_offsets[6]
        );
        assert_eq!(
            after.holder_multi_y_offsets[5], before.holder_multi_y_offsets[5],
            "neighbour untouched"
        );
        std::fs::remove_file(&path).ok();
    }

    /// The stage keeps its three trims as scalars, so it exercises the
    /// other slot shape: the number on the line is replaced in place,
    /// the rack lists beside it do not move, and an axis with no
    /// measurement keeps its slot.
    #[test]
    fn persist_edits_the_stage_scalars_without_moving_the_rack() {
        let path = temp_copy("stage");
        let before = WaypointData::load(&path).expect("load");
        let report =
            persist_stage_trims(&path, Some(0.0001), None, Some(-0.00002)).expect("persist");
        assert_eq!(report.len(), 2, "{report:?}");
        let after = WaypointData::load(&path).expect("reload");
        assert_eq!(
            after.sample_holder_on_x_offset,
            round7(before.sample_holder_on_x_offset + 0.0001)
        );
        assert_eq!(
            after.sample_holder_on_z_offset,
            round7(before.sample_holder_on_z_offset - 0.00002)
        );
        assert_eq!(
            after.sample_holder_on_y_offset, before.sample_holder_on_y_offset,
            "the axis with no measurement keeps its slot"
        );
        assert_eq!(
            after.holder_multi_x_offsets, before.holder_multi_x_offsets,
            "the rack is a different seat"
        );
        let text = std::fs::read_to_string(&path).expect("read");
        assert!(text.contains("# Insertion-depth trim"), "comments survive");
        std::fs::remove_file(&path).ok();
    }

    /// A scalar key that is not there is refused for the reason a list
    /// key is: half a correction written is worse than none, and the
    /// stage's three offsets are optional in the file's schema, so a
    /// file without them must fail loudly rather than default to zero
    /// and add to it.
    #[test]
    fn persist_refuses_a_missing_stage_scalar() {
        let path = temp_copy("stage_missing");
        let text = std::fs::read_to_string(&path)
            .expect("read")
            .replace("sample_holder_on_position_y_offset", "renamed_away");
        std::fs::write(&path, &text).expect("write");
        let err = persist_stage_trims(&path, None, Some(0.0002), None);
        assert!(err.is_err(), "{err:?}");
        assert_eq!(std::fs::read_to_string(&path).expect("read"), text);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn persist_refuses_a_missing_key_and_leaves_the_file() {
        let path = temp_copy("missing");
        let mut text = std::fs::read_to_string(&path).expect("read");
        text = text.replace("holder_multi_z_offsets", "renamed_away");
        std::fs::write(&path, &text).expect("write");
        let err = persist_holder_trims(&path, 4, None, None, Some(0.0002));
        assert!(err.is_err());
        assert_eq!(std::fs::read_to_string(&path).expect("read"), text);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn arm_joints_drops_the_gripper_slot() {
        let values = [9.9, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6];
        let joints = WaypointData::arm_joints(&values);
        assert_eq!(joints.len(), 6);
        assert_eq!(joints[0], ("shoulder_pan_joint".to_string(), 0.1));
        assert_eq!(joints[5], ("shoulder_lift_joint".to_string(), 0.6));
    }
}
