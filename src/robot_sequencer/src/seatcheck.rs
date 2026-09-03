//! Is the seat the arm is about to use in the state the run assumes —
//! occupied for a pick, empty for a place — asked of the D405 depth
//! stream before anything moves.
//!
//! This is deliberately the only question put to the camera. The errors
//! that jam this rig are 0.03-0.5 mm (a rack well's play is +-0.032 mm
//! at holder 10), and from a holder standby one pixel is 0.344 mm with
//! `T_ee_cam` itself uncertain by 0.45 mm 1σ — a tenth of a pixel is not
//! a measurement. Occupancy is a different scale entirely: a seated
//! puck's cap stands ~8.7 mm proud of the empty recess rim against
//! +-1 mm of temporal noise, which is where the 14/14 validation of
//! 2026-08-18 came from. Clearance stays the force channel's job, where
//! 0.2 mm reads as 4.4 N.
//!
//! The rule is stated geometrically rather than as the pixel window that
//! validation used. A hardcoded ROI is pinned to one camera mount AND
//! one taught standby, and it fails **silently** when either moves: a
//! shifted camera still returns a median, just of the wrong patch. Here
//! the window is projected from the seat's own pose through `T_ee_cam`
//! each time it is asked, so re-teaching a standby costs nothing and
//! remounting the camera costs a `CalibMode = 3` capture and a re-solve
//! — which the daemon already collects.

use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion};
use serde::Deserialize;

use crate::error::SequencerError;

/// A pinhole camera: the intrinsic matrix and the Brown-Conrady
/// coefficients `solvePnP` and `rs2_project_point_to_pixel` agree on
/// (tools/handeye/check_distortion_model.py settles which model the
/// IOC's coefficients are in — they go in as-is).
#[derive(Debug, Clone, Copy)]
pub struct Lens {
    pub k: [f64; 9],
    pub dist: [f64; 5],
}

impl Lens {
    /// Where a point in the camera's own frame lands on the sensor, and
    /// how far away it is along the optical axis. `None` behind the
    /// camera, where the projection is meaningless rather than merely
    /// off-frame.
    pub fn project(&self, p: &Point3<f64>) -> Option<(f64, f64, f64)> {
        if p.z <= 0.0 {
            return None;
        }
        let (x, y) = (p.x / p.z, p.y / p.z);
        let r2 = x * x + y * y;
        let [k1, k2, p1, p2, k3] = self.dist;
        let radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        let xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        Some((self.k[0] * xd + self.k[2], self.k[4] * yd + self.k[5], p.z))
    }
}

/// The hand-eye result: where the camera sits on the tool, and the lens
/// it was solved under.
///
/// Loaded from `T_ee_cam.yaml` — the file `tools/handeye/solve_joint.py`
/// writes and `doc/handeye_calibration.md` records — rather than from
/// constants, because a remounted camera changes this file and nothing
/// else should have to change with it.
pub struct HandEye {
    /// `ik_frame` -> camera. The capture's `base_t_ee` is the `ik_frame`
    /// pose, so this composes straight onto `Model::fk`.
    pub ee_t_cam: Isometry3<f64>,
    /// The colour lens the pose was solved under. The seat check reads
    /// the depth stream and so projects through
    /// [`DepthCamera::lens`], not this one; it is kept because it is
    /// what identifies the solve — a `T_ee_cam.yaml` whose focal length
    /// does not match the camera now on the tool is the failure that
    /// leaves every projection plausible and wrong.
    pub colour: Lens,
}

#[derive(Deserialize)]
struct HandEyeFile {
    #[serde(rename = "T_ee_cam")]
    t_ee_cam: Pose,
    camera_matrix: Vec<f64>,
    dist_coeffs: Vec<f64>,
}

#[derive(Deserialize)]
struct Pose {
    translation_m: Vec<f64>,
    rotation_matrix: Vec<Vec<f64>>,
}

impl HandEye {
    pub fn load(path: &std::path::Path) -> Result<Self, SequencerError> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| SequencerError(format!("cannot read hand-eye {}: {e}", path.display())))?;
        let file: HandEyeFile = serde_yaml::from_str(&text).map_err(|e| {
            SequencerError(format!("cannot parse hand-eye {}: {e}", path.display()))
        })?;
        let t = &file.t_ee_cam.translation_m;
        let r = &file.t_ee_cam.rotation_matrix;
        if t.len() != 3 || r.len() != 3 || r.iter().any(|row| row.len() != 3) {
            return Err(SequencerError(format!(
                "hand-eye {}: T_ee_cam needs 3 translation and 3x3 rotation entries",
                path.display()
            )));
        }
        if file.camera_matrix.len() != 9 || file.dist_coeffs.len() != 5 {
            return Err(SequencerError(format!(
                "hand-eye {}: camera_matrix needs 9 entries and dist_coeffs 5",
                path.display()
            )));
        }
        let rotation = Rotation3::from_matrix_unchecked(Matrix3::new(
            r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2],
        ));
        let mut k = [0.0; 9];
        k.copy_from_slice(&file.camera_matrix);
        let mut dist = [0.0; 5];
        dist.copy_from_slice(&file.dist_coeffs);
        Ok(Self {
            ee_t_cam: Isometry3::from_parts(
                Translation3::new(t[0], t[1], t[2]),
                UnitQuaternion::from_rotation_matrix(&rotation),
            ),
            colour: Lens { k, dist },
        })
    }

    /// Where `point_base` lands when the tool is at `base_t_ee`: pixel
    /// column, pixel row, and range along the optical axis in metres.
    pub fn look(
        &self,
        base_t_ee: &Isometry3<f64>,
        point_base: &Point3<f64>,
        lens: &Lens,
    ) -> Option<(f64, f64, f64)> {
        let cam = (base_t_ee * self.ee_t_cam).inverse();
        lens.project(&(cam * point_base))
    }
}

/// Half-width of the sampling window along tool x, metres.
///
/// The neck the fingers close on is 11.4 mm across at holder 4, so
/// +-5 mm stays on the puck and off the well rim either side.
const WINDOW_HALF_X_M: f64 = 0.005;

/// How far the window reaches out of the seat (tool -y) and back into it
/// (tool +y), metres.
///
/// Both ends are measured, not chosen for symmetry. Out of the seat is
/// where the discrimination lives: at holder 2 the empty well reads
/// 175-178 mm across the rows 6 mm above the grasp point while the
/// seated puck reads 130-133 mm there. Back into the seat it dies —
/// 4 mm below the grasp point empty is 138 mm against 128 mm seated,
/// and 8 mm below the two are the same holder body at 126 mm. So the
/// window sits mostly above the grasp point and stops just under it.
/// Both numbers are the *rack's*, and where a seat presents its puck
/// differently the difference is a bias on the window rather than a
/// second pair of constants — see `seat_check.*_window_bias_mm` and
/// `Seat::window_bias`. The stage earned one when it was moved on
/// 2026-09-03: its puck top now shows itself below the grasp point,
/// where this window reaches only 2 mm.
const WINDOW_UP_M: f64 = 0.006;
const WINDOW_DOWN_M: f64 = 0.002;

/// A seat is occupied when the window's median range is no further than
/// this behind the grasp point's own range, metres.
///
/// Every seat this rig has was read on 2026-08-19, most of them both
/// ways. Occupied: -3.3 (h1), -4.5 (h4), -4.5 (h5), -5.2 (h6), -5.0
/// (the stage), against the archived -4.8/-4.3/-3.6 of the 2026-08-18
/// A/B. Empty: +40.3 (h2), +41.2 (h3), +41.1 (h4), +40.2 (h7), +40.5
/// (h8), +38.8 (h9) — and then the two that decide this number, +16.9
/// (h1) and +17.2 (h10), whose wells show a floor 17 mm behind the
/// grasp point where the others see past the well entirely.
///
/// So the gap to split is -3.3 to +16.9 and 7 mm sits in the middle of
/// it: 10.3 mm of margin on the occupied side, 9.9 mm on the empty one,
/// against the +-1 mm the depth stream wanders frame to frame. 15 mm,
/// which the rack majority would have supported, would have left h1 and
/// h10 with 2 mm.
///
/// Stated as a difference from the seat's own projected range rather
/// than as an absolute distance so that it survives what an absolute
/// number does not: a re-taught standby, a different holder in the rack
/// pitch, and the stage, whose seat is 71 mm further from the camera
/// than the rack's.
const PRESENT_MAX_DELTA_M: f64 = 0.007;

/// Below this fraction of pixels carrying a range, the window is not a
/// measurement.
///
/// Low, because the fraction turns out not to measure how good the
/// reading is. Holder 7's window is 21-29% valid over six consecutive
/// frames and returns +40.4 to +42.0 mm every time; holder 1's is 18%
/// on one frame and 52% on the next, and the two answers differ by
/// 1.9 mm. The invalid pixels are the same ones frame after frame — a
/// fixed patch of the well the stereo pair cannot triangulate, not
/// noise — so pooling frames buys nothing (measured: six frames pooled
/// at h7 stay at 24%). What the floor is left to catch is a window with
/// too few samples to take a median of at all: 10% of a ~660-pixel
/// window is still 66.
const MIN_VALID_FRACTION: f64 = 0.10;

/// The depth stream's own pinhole and scale, read from the D405 IOC
/// rather than pinned here: `RSDepthUnits_RBV` is 0.0001 m on the D405
/// and 0.001 on the D435i, and the intrinsics change with the profile.
///
/// These are NOT [`HandEye::colour`] — the depth imager has its own
/// focal length and principal point (389.555 against 396.993,
/// (322.806, 244.102) against (325.748, 254.637)). What is shared is
/// the pose: the depth stream is registered to the colour frame to
/// within the few pixels doc/handeye_calibration.md bounds, which is
/// well inside a window whose two classes are 45 mm apart.
#[derive(Debug, Clone)]
pub struct DepthCamera {
    pub lens: Lens,
    pub unit_m: f64,
    pub width: usize,
    pub height: usize,
}

/// One depth frame, raw counts, row-major, `width * height` of them.
pub struct DepthFrame {
    pub counts: Vec<u16>,
}

/// The pixel rectangle the seat's own sampling window projects to,
/// inclusive of both bounds and clamped to the frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Window {
    pub c0: usize,
    pub c1: usize,
    pub r0: usize,
    pub r1: usize,
}

/// What the seat looks like, and the numbers behind it.
#[derive(Debug, Clone)]
pub struct Reading {
    pub verdict: Verdict,
    /// Median range over the valid pixels of the window, metres.
    pub median_m: f64,
    /// Where the seat's grasp point itself projects to, metres.
    pub predicted_m: f64,
    /// `median_m - predicted_m`, the statistic the verdict reads.
    pub delta_m: f64,
    pub valid_fraction: f64,
    pub window: Window,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// Something is standing where a puck in this seat would stand.
    Occupied,
    /// The window sees past the seat, into the well.
    Empty,
    /// Too few pixels carried a range to say either way.
    Unreadable,
}

impl Verdict {
    pub fn label(self) -> &'static str {
        match self {
            Self::Occupied => "occupied",
            Self::Empty => "empty",
            Self::Unreadable => "unreadable",
        }
    }
}

impl HandEye {
    /// The pixel rectangle to sample for a seat at `seat_pose`, seen
    /// from `base_t_ee`.
    ///
    /// The window is a rectangle in the seat's own tool frame, and its
    /// four corners are projected rather than a centre plus a pixel
    /// half-size: the rack and the stage are turned 92 degrees from each
    /// other about the approach axis, so the same rectangle lands with
    /// its axes swapped at the stage and a pixel half-size taken from
    /// the rack would sample the wrong shape there.
    /// `bias` moves the window in the seat's own tool frame, metres, and
    /// nothing else: the grasp point the reading is measured against
    /// stays where the seat is. That separation is the point — the bias
    /// says where this seat shows its puck to the camera, not where the
    /// seat is, so biasing the window never moves the arm and never
    /// changes what "occupied" means.
    pub fn window(
        &self,
        base_t_ee: &Isometry3<f64>,
        seat_pose: &Isometry3<f64>,
        cam: &DepthCamera,
        bias: [f64; 3],
    ) -> Option<Window> {
        let (mut c0, mut c1, mut r0, mut r1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
        for x in [-WINDOW_HALF_X_M, WINDOW_HALF_X_M] {
            for y in [-WINDOW_UP_M, WINDOW_DOWN_M] {
                let corner = Point3::from(
                    (seat_pose * Translation3::new(x + bias[0], y + bias[1], bias[2]))
                        .translation
                        .vector,
                );
                let (u, v, _) = self.look(base_t_ee, &corner, &cam.lens)?;
                c0 = c0.min(u);
                c1 = c1.max(u);
                r0 = r0.min(v);
                r1 = r1.max(v);
            }
        }
        let clamp = |v: f64, hi: usize| v.round().clamp(0.0, hi as f64 - 1.0) as usize;
        let w = Window {
            c0: clamp(c0, cam.width),
            c1: clamp(c1, cam.width),
            r0: clamp(r0, cam.height),
            r1: clamp(r1, cam.height),
        };
        (w.c1 > w.c0 && w.r1 > w.r0).then_some(w)
    }

    /// Is a puck standing in the seat at `seat_pose`?
    ///
    /// `None` when the seat does not project into the frame at all,
    /// which is a caller error (the wrong observation pose) rather than
    /// a camera problem, and so is not one of the verdicts.
    pub fn read_seat(
        &self,
        frame: &DepthFrame,
        cam: &DepthCamera,
        base_t_ee: &Isometry3<f64>,
        seat_pose: &Isometry3<f64>,
        bias: [f64; 3],
    ) -> Option<Reading> {
        let window = self.window(base_t_ee, seat_pose, cam, bias)?;
        let grasp = Point3::from(seat_pose.translation.vector);
        let (_, _, predicted_m) = self.look(base_t_ee, &grasp, &cam.lens)?;

        let mut ranges =
            Vec::with_capacity((window.c1 - window.c0 + 1) * (window.r1 - window.r0 + 1));
        let mut total = 0usize;
        for row in window.r0..=window.r1 {
            for col in window.c0..=window.c1 {
                total += 1;
                match frame.counts.get(row * cam.width + col) {
                    // Zero is the driver's "no range here", not a
                    // surface on the lens.
                    Some(0) | None => {}
                    Some(count) => ranges.push(f64::from(*count) * cam.unit_m),
                }
            }
        }
        let valid_fraction = ranges.len() as f64 / total as f64;
        if valid_fraction < MIN_VALID_FRACTION {
            return Some(Reading {
                verdict: Verdict::Unreadable,
                median_m: f64::NAN,
                predicted_m,
                delta_m: f64::NAN,
                valid_fraction,
                window,
            });
        }
        ranges.sort_by(f64::total_cmp);
        let median_m = ranges[ranges.len() / 2];
        let delta_m = median_m - predicted_m;
        Some(Reading {
            verdict: if delta_m < PRESENT_MAX_DELTA_M {
                Verdict::Occupied
            } else {
                Verdict::Empty
            },
            median_m,
            predicted_m,
            delta_m,
            valid_fraction,
            window,
        })
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;
    use crate::config::Config;
    use crate::model::{JointMap, Model};
    use crate::waypoints::WaypointData;

    fn handeye() -> HandEye {
        HandEye::load(Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../T_ee_cam.yaml"
        )))
        .expect("hand-eye")
    }

    /// The D405's depth stream as the IOC reported it on 2026-08-19,
    /// which is what the archived frames below were taken through.
    fn depth_camera() -> DepthCamera {
        DepthCamera {
            lens: Lens {
                k: [389.555, 0.0, 322.806, 0.0, 389.555, 244.102, 0.0, 0.0, 1.0],
                dist: [0.0; 5],
            },
            unit_m: 0.0001,
            width: 640,
            height: 480,
        }
    }

    /// Where the arm stands to look at holder `holder`, and where that
    /// holder's grasp point is — the two poses every reading needs,
    /// built the way `compute_run_waypoints` builds them.
    fn rack_seat(holder: i32) -> (Isometry3<f64>, Isometry3<f64>) {
        let config = Config::load(Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer.yaml"
        )))
        .expect("config");
        let model = Model::load(&config).expect("model");
        let w = WaypointData::load(&config.sequence.waypoints_yaml).expect("waypoints");
        let taught = |v: &[f64]| -> JointMap { WaypointData::arm_joints(v).into_iter().collect() };
        let rack = [w.rack_x_offset, w.rack_y_offset, w.rack_z_offset];
        let i = (holder - 1) as usize;
        let pitch = f64::from(holder - 1) * config.sequence.holder_offset;
        let trim = |list: &[f64]| list.get(i).copied().unwrap_or(0.0);
        let offset = [
            trim(&w.holder_multi_x_offsets),
            pitch + trim(&w.holder_multi_y_offsets),
            trim(&w.holder_multi_z_offsets),
        ];
        let standby0 = model
            .apply_cartesian_offset(&taught(&w.holder1_standby), rack, false, "standby0")
            .expect("standby0");
        let standby = model
            .apply_cartesian_offset(&standby0, offset, false, "standby")
            .expect("standby");
        let on0 = model
            .apply_cartesian_offset(&taught(&w.holder1_on_position), rack, false, "on0")
            .expect("on0");
        let on = model
            .apply_cartesian_offset(&on0, offset, false, "on")
            .expect("on");
        (
            model.fk(&standby).expect("fk standby"),
            model.fk(&on).expect("fk on"),
        )
    }

    /// A frame where every pixel reads the same range.
    fn flat_frame(mm: f64, cam: &DepthCamera) -> DepthFrame {
        DepthFrame {
            counts: vec![(mm / 1000.0 / cam.unit_m) as u16; cam.width * cam.height],
        }
    }

    /// The projection chain has one independently-checked answer in this
    /// repo: doc/vision_correction_plan.md §13.1 computed holder 3's
    /// grasp point at (306, 330) and 136.8 mm from the holder standby,
    /// and the same section reports the photograph it was validated
    /// against (136.0 mm, 72 px against a computed 73). Reproducing it
    /// here is what says `Model::fk * T_ee_cam * K` is composed the right
    /// way round — a transposed rotation or an inverted transform still
    /// yields a plausible pixel.
    #[test]
    fn the_projection_reproduces_the_measured_holder_3_view() {
        let eye = handeye();
        let (base_t_ee, seat) = rack_seat(3);
        let grasp = Point3::from(seat.translation.vector);
        let (px, py, range) = eye
            .look(&base_t_ee, &grasp, &eye.colour)
            .expect("grasp point is in front of the camera");
        assert!(
            (range - 0.1368).abs() < 0.002,
            "axial range {:.1} mm, doc says 136.8",
            range * 1000.0
        );
        assert!(
            (px - 306.0).abs() < 6.0 && (py - 330.0).abs() < 6.0,
            "grasp projects to ({px:.0}, {py:.0}), doc says (306, 330)"
        );
    }

    /// The window has to land where the discrimination was measured, or
    /// the thresholds below are calibrated against a different patch of
    /// the scene than the one they will be asked about.
    ///
    /// Holder 2 is the seat the 2026-08-18 A/B was taken at (loaded,
    /// then twice with its puck carried away). Reading that pair patch
    /// by patch, the rows that separate the two run 300-329 and the
    /// columns 285-340; outside that the empty well and the seated puck
    /// return the same holder body to within 1 mm.
    /// The rack's window, unmoved -- what every fingerprint below was
    /// measured with.
    const NO_BIAS: [f64; 3] = [0.0; 3];

    /// A bias moves the window and nothing else. The pair matters more
    /// than either half: if it moved the grasp point too, a biased
    /// window would keep reading zero delta wherever it was pointed and
    /// the check would answer "occupied" about anything it happened to
    /// land on.
    #[test]
    fn a_window_bias_moves_the_window_and_leaves_the_grasp_point() {
        let cam = depth_camera();
        let (base_t_ee, seat) = rack_seat(2);
        let eye = handeye();
        let plain = eye
            .window(&base_t_ee, &seat, &cam, NO_BIAS)
            .expect("window");
        let moved = eye
            .window(&base_t_ee, &seat, &cam, [0.0, 0.008, 0.0])
            .expect("window");
        assert_ne!(plain, moved, "an 8 mm bias left the window where it was");

        let frame = flat_frame(0.2, &cam);
        let a = eye
            .read_seat(&frame, &cam, &base_t_ee, &seat, NO_BIAS)
            .expect("reading");
        let b = eye
            .read_seat(&frame, &cam, &base_t_ee, &seat, [0.0, 0.008, 0.0])
            .expect("reading");
        assert!(
            (a.predicted_m - b.predicted_m).abs() < 1e-9,
            "the bias moved the grasp point: {} vs {}",
            a.predicted_m,
            b.predicted_m
        );
    }

    #[test]
    fn the_window_lands_on_the_rows_that_separate_loaded_from_empty() {
        let cam = depth_camera();
        let (base_t_ee, seat) = rack_seat(2);
        let w = handeye()
            .window(&base_t_ee, &seat, &cam, NO_BIAS)
            .expect("window");
        assert!(
            (300..=305).contains(&w.r0) && (322..=329).contains(&w.r1),
            "rows {}-{} are outside the measured separating band 300-329",
            w.r0,
            w.r1
        );
        assert!(
            (285..=295).contains(&w.c0) && (313..=340).contains(&w.c1),
            "cols {}-{} are outside the measured separating band 285-340",
            w.c0,
            w.c1
        );
    }

    /// The classes as they were measured, put through the decision.
    ///
    /// Three seats and not one, because the rack does not read the same
    /// everywhere. Holder 2 is the archived A/B — 131.6 mm loaded
    /// against a grasp point at 136.4, 177.5 mm emptied. Holder 1 is the
    /// seat that decides `PRESENT_MAX_DELTA_M`: its empty well shows a
    /// floor at 152.9 mm against a grasp point at 136.0, a +16.9 mm
    /// signal where holder 2 gives +41. Holder 10 reads the same way
    /// (+17.2). A flat frame is enough because the verdict reads one
    /// median.
    #[test]
    fn the_verdict_splits_the_two_measured_clusters() {
        let eye = handeye();
        let cam = depth_camera();
        let read = |holder: i32, mm: f64| {
            let (base_t_ee, seat) = rack_seat(holder);
            eye.read_seat(&flat_frame(mm, &cam), &cam, &base_t_ee, &seat, NO_BIAS)
                .expect("seat is in view")
        };
        let loaded = read(2, 131.6);
        assert_eq!(loaded.verdict, Verdict::Occupied, "{loaded:?}");
        assert!(
            (loaded.delta_m * 1000.0 + 4.8).abs() < 1.0,
            "loaded delta {:.1} mm, measured -4.8",
            loaded.delta_m * 1000.0
        );
        let emptied = read(2, 177.5);
        assert_eq!(emptied.verdict, Verdict::Empty, "{emptied:?}");
        assert!(
            (emptied.delta_m * 1000.0 - 41.1).abs() < 1.0,
            "emptied delta {:.1} mm, measured +41.1",
            emptied.delta_m * 1000.0
        );

        // The narrow one. A threshold set from the rack majority alone
        // would read this as a puck and let a place drop a second one
        // into it.
        let shallow = read(1, 152.9);
        assert_eq!(shallow.verdict, Verdict::Empty, "{shallow:?}");
        assert!(
            (shallow.delta_m * 1000.0 - 16.9).abs() < 1.0,
            "holder 1 empty delta {:.1} mm, measured +16.9",
            shallow.delta_m * 1000.0
        );
        assert_eq!(read(1, 132.7).verdict, Verdict::Occupied);

        // A frame carrying no ranges is neither, and must not read as
        // the class whose side of the threshold zero happens to fall on.
        let blank = DepthFrame {
            counts: vec![0; cam.width * cam.height],
        };
        let (base_t_ee, seat) = rack_seat(2);
        let dark = eye
            .read_seat(&blank, &cam, &base_t_ee, &seat, NO_BIAS)
            .expect("seat is in view");
        assert_eq!(dark.verdict, Verdict::Unreadable, "{dark:?}");
    }
}
