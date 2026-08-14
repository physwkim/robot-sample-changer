//! Eye-in-hand calibration capture: the camera side.
//!
//! What lives here is everything the daemon needs to turn a pose into a
//! sample — the pose schedule, the AprilTag detector child, and the
//! samples writer. The motion loop that drives it stays in
//! [`crate::sequence`], where `Stop` and the trigger loop already have an
//! owner.
//!
//! Why rotations and not an orbit around the tag: `calibrateHandEye`
//! wants rotational diversity, not translation, and the cell is tight.
//! A 100 mm tag at ~290 mm subtends about ±10 deg inside a 78×63 deg
//! view, so the tool can turn tens of degrees before the tag leaves the
//! frame — and turning the tool moves the camera only by the mount
//! offset, tens of mm, instead of the ~100 mm an orbit of the same angle
//! would cost.
//!
//! The tag's pose in the base frame is never needed. `calibrateHandEye`
//! solves AX = XB, which eliminates it; the tag only has to stay still
//! while the capture runs.

use std::io::{BufRead, BufReader, Write};
use std::path::Path;
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};

use nalgebra::{Isometry3, Unit, UnitQuaternion, Vector3};

use crate::config::HandEyeConfig;
use crate::error::SequencerError;
use crate::log;
use crate::model::{JointMap, Model};

/// Tool-local rotation axes the schedule turns about. Three distinct axes
/// because two relative motions sharing a rotation axis are degenerate for
/// AX = XB — the classic way to get a confident-looking wrong answer.
const AXES: [(&str, [f64; 3]); 3] = [
    ("x", [1.0, 0.0, 0.0]),
    ("y", [0.0, 1.0, 0.0]),
    ("z", [0.0, 0.0, 1.0]),
];

/// Fractions of the configured angle used on each axis, both signs.
const FRACTIONS: [f64; 2] = [0.5, 1.0];

/// Gap left between the tag's image bounding box and the frame edge. The
/// detector needs all four corners, so a pose that lands a millimetre
/// past its goal must still have the whole tag inside.
const SWEEP_MARGIN_PX: f64 = 8.0;

/// How far out along each sweep direction to stop, as a fraction of the
/// way from the tag's home position to the frame edge. Two rings rather
/// than only the extreme: the radial terms of the lens model are r^3, r^5
/// and r^7, which over a short span of radius are near enough to each
/// other that the fit trades them freely — telling them apart needs the
/// far radii sampled at more than one place.
const SWEEP_FRACTIONS: [f64; 2] = [0.6, 1.0];

/// Smallest image shift a probe may produce and still be fitted. A 20 mm
/// step walks the tag some 27 px at the working distance; a few px means
/// the arm did not move or the frame was stale, and a Jacobian fitted to
/// that points the sweep in an arbitrary direction.
const MIN_PROBE_SHIFT_PX: f64 = 5.0;

/// Smallest ratio of |det| to the product of the column norms that still
/// counts as two independent directions — 30 degrees apart. Below it the
/// two probes say almost the same thing and the inverse is noise.
const MIN_PROBE_INDEPENDENCE: f64 = 0.5;

/// The tag's plane as the depth stream measured it, `n · X = d` in the
/// depth camera's frame, metres.
///
/// The one quantity in a sample that the corners cannot supply. In an
/// image, the lens focal length and the tag's true edge length trade
/// against each other exactly — a tag 1 % small and an fx 1 % long
/// project identically — so no number of corners separates them, and the
/// capture's ranges came out 1.8 % longer than the robot's own travel
/// with no way to say which was at fault. Depth measures the range
/// without reference to either.
///
/// A plane rather than a depth per corner: see `tag_plane` in the
/// detector for why the alignment between the two streams makes the
/// plane the trustworthy form.
#[derive(Clone)]
pub struct DepthPlane {
    /// Unit normal, pointing away from the camera.
    pub normal: [f64; 3],
    /// Perpendicular distance from the camera's origin (m).
    pub offset_m: f64,
    /// Where the plane cuts the ray through the tag's centre (m).
    pub range_m: f64,
    /// Residual of the plane fit (m), and how many pixels it used.
    /// Together these say whether the patch was a plane at all.
    pub rms_m: f64,
    pub pixels: u32,
}

/// What the camera saw at one pose.
#[derive(Clone)]
pub struct Detection {
    /// Tag pose in the camera frame, from the detector's solvePnP.
    pub cam_t_tag: Isometry3<f64>,
    pub reproj_px: f64,
    pub side_px: f64,
    pub center_px: [f64; 2],
    /// The four tag corners in pixels, in the detector's object-point
    /// order. This is the raw measurement `cam_t_tag` was derived from,
    /// and the only part of a sample that survives a change of lens
    /// model — keeping it is what lets a recalibrated camera re-solve
    /// the file instead of the robot re-running the capture.
    pub corners_px: [[f64; 2]; 4],
    /// `None` when this camera serves no depth, or when the patch was
    /// too broken to call a plane. Absent for a pose, not for the run.
    pub depth: Option<DepthPlane>,
}

impl Detection {
    /// The one-line summary the capture log prints per pose.
    pub fn summary(&self) -> String {
        let base = format!(
            "{:.1} mm, {:.1} px, centre ({:.0}, {:.0}), reproj {:.3} px",
            self.cam_t_tag.translation.vector.norm() * 1000.0,
            self.side_px,
            self.center_px[0],
            self.center_px[1],
            self.reproj_px
        );
        // The depth range next to the tag's own is the check worth having
        // in the log: the two disagreeing by more than the plane's
        // residual is the scale error showing itself live.
        match &self.depth {
            Some(d) => format!(
                "{base}, depth {:.1} mm (rms {:.2} mm, {} px)",
                d.range_m * 1000.0,
                d.rms_m * 1000.0,
                d.pixels
            ),
            None => base,
        }
    }
}

/// One captured pose: the arm's own kinematics and what the camera saw.
pub struct Sample {
    pub label: String,
    pub joints: JointMap,
    /// `ik_frame` pose in the base frame, from FK at `joints`.
    pub base_t_ee: Isometry3<f64>,
    pub seen: Detection,
}

/// The tool rotations to visit, as joint goals the arm can both solve for
/// and reach in a straight line from home.
pub struct Schedule {
    pub poses: Vec<(String, JointMap)>,
    /// Labels dropped before anything moved, each with why. Reported, not
    /// fatal — a schedule that loses a couple of extremes in a tight cell
    /// still calibrates.
    pub dropped: Vec<(String, &'static str)>,
}

/// Which way the tag walks in the image when the tool steps in its own
/// xy plane, in px per metre.
///
/// Measured at capture time, not derived from `T_ee_cam`: that is the
/// unknown this whole mode exists to find, and the part of it that
/// decides the answer here — the camera's roll on the bracket — is
/// exactly what a remount changes. Two probe steps pin it, so nothing
/// about how the camera is bolted on has to be assumed, not even which
/// way is up.
pub struct ImageJacobian {
    /// `[[dcol/dx, dcol/dy], [drow/dx, drow/dy]]`, px per metre of
    /// tool-frame step.
    m: [[f64; 2]; 2],
}

impl ImageJacobian {
    /// Fits the map from `(tool step in m, image shift in px)` pairs —
    /// exactly two, far enough apart to span the plane.
    ///
    /// Every way the fit could be meaningless is refused here rather than
    /// at the inverse, so an `ImageJacobian` that exists is one that can
    /// be inverted and the sweep needs no fallback for a degenerate map.
    pub fn from_probes(probes: &[([f64; 2], [f64; 2])]) -> Result<Self, SequencerError> {
        let [(s0, d0), (s1, d1)] = probes else {
            return Err(SequencerError(format!(
                "the image Jacobian takes exactly 2 probes, got {}",
                probes.len()
            )));
        };
        for (i, d) in [d0, d1].into_iter().enumerate() {
            let moved = (d[0] * d[0] + d[1] * d[1]).sqrt();
            if moved < MIN_PROBE_SHIFT_PX {
                return Err(SequencerError(format!(
                    "probe {i} moved the tag {moved:.1} px, want {MIN_PROBE_SHIFT_PX:.0} px or more"
                )));
            }
        }
        let step_det = s0[0] * s1[1] - s1[0] * s0[1];
        let step_norms =
            (s0[0] * s0[0] + s0[1] * s0[1]).sqrt() * (s1[0] * s1[0] + s1[1] * s1[1]).sqrt();
        if step_norms <= 0.0 || (step_det / step_norms).abs() < MIN_PROBE_INDEPENDENCE {
            return Err(SequencerError(
                "the two probe steps are too nearly parallel to span the tool's xy plane".into(),
            ));
        }
        // m = shifts * steps^-1, columns being the probes.
        let m = [
            [
                (d0[0] * s1[1] - d1[0] * s0[1]) / step_det,
                (d1[0] * s0[0] - d0[0] * s1[0]) / step_det,
            ],
            [
                (d0[1] * s1[1] - d1[1] * s0[1]) / step_det,
                (d1[1] * s0[0] - d0[1] * s1[0]) / step_det,
            ],
        ];
        let det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
        let norms = (m[0][0] * m[0][0] + m[1][0] * m[1][0]).sqrt()
            * (m[0][1] * m[0][1] + m[1][1] * m[1][1]).sqrt();
        if norms <= 0.0 || (det / norms).abs() < MIN_PROBE_INDEPENDENCE {
            return Err(SequencerError(
                "the measured image shifts do not span the frame; is the camera along tool z?"
                    .into(),
            ));
        }
        Ok(Self { m })
    }

    /// The tool-frame xy step, in metres, that moves the tag by
    /// `shift_px` in the image.
    pub fn tool_step(&self, shift_px: [f64; 2]) -> [f64; 2] {
        let det = self.m[0][0] * self.m[1][1] - self.m[0][1] * self.m[1][0];
        [
            (self.m[1][1] * shift_px[0] - self.m[0][1] * shift_px[1]) / det,
            (self.m[0][0] * shift_px[1] - self.m[1][0] * shift_px[0]) / det,
        ]
    }

    /// Reported once per capture. A camera looking along the tool's z
    /// makes this map a scaled rotation, so printing it as a scale and a
    /// roll turns "the probe went wrong" into something visible at a
    /// glance instead of four numbers nobody checks.
    pub fn summary(&self) -> String {
        let scale = ((self.m[0][0].powi(2) + self.m[1][0].powi(2)).sqrt()
            + (self.m[0][1].powi(2) + self.m[1][1].powi(2)).sqrt())
            / 2.0;
        format!(
            "{:.2} px/mm, roll {:.1} deg",
            scale / 1000.0,
            self.m[1][0].atan2(self.m[0][0]).to_degrees()
        )
    }
}

/// Poses that walk the tag out to the frame's edges and corners, as
/// tool-frame xy offsets in metres.
///
/// `calibrateCamera` determines the lens model only where it has been
/// shown corners, and turning the tool in place keeps them inside about
/// r = 236 px of a 405 px half-diagonal. Within that band a change in k1
/// is absorbed by a change in fx and the principal point: two models
/// fitted to the same capture agreed on the observed corners to 0.08 px
/// while disagreeing 9 % on fx and 27 mm on the tag's range. Out at the
/// frame corner those same two models place a point 27 px apart, far
/// above the 0.07 px the detector repeats to — so one corner observed
/// there rejects one of them outright.
///
/// Pure translation, no tilt: the offsets come from inverting a Jacobian
/// measured for translation, and a rotation about the TCP also swings the
/// camera through the mount's lever arm, which would land the tag
/// somewhere other than where it was asked for. The orientation variety
/// `calibrateCamera` needs to separate fx from range comes from the
/// rotation set running alongside.
pub fn frame_sweep(
    jacobian: &ImageJacobian,
    image_size: [u32; 2],
    at_home: &Detection,
) -> Vec<(String, [f64; 2])> {
    let (mut min, mut max) = (at_home.corners_px[0], at_home.corners_px[0]);
    for c in &at_home.corners_px {
        min = [min[0].min(c[0]), min[1].min(c[1])];
        max = [max[0].max(c[0]), max[1].max(c[1])];
    }
    let (half_w, half_h) = ((max[0] - min[0]) / 2.0, (max[1] - min[1]) / 2.0);
    let (w, h) = (image_size[0] as f64, image_size[1] as f64);
    let x_lo = SWEEP_MARGIN_PX + half_w;
    let x_hi = w - SWEEP_MARGIN_PX - half_w;
    let y_lo = SWEEP_MARGIN_PX + half_h;
    let y_hi = h - SWEEP_MARGIN_PX - half_h;
    // A tag that already spans the frame cannot be walked anywhere in it.
    // Backing off is the standoff set's job, not this one's.
    if x_lo >= x_hi || y_lo >= y_hi {
        return Vec::new();
    }

    // The centre of a square's bounding box is the square's centre
    // whatever its rotation, so the tag's reported centre is what these
    // limits are on.
    let (cx, cy) = (at_home.center_px[0], at_home.center_px[1]);
    let anchors = [
        ("tl", x_lo, y_lo),
        ("t", cx, y_lo),
        ("tr", x_hi, y_lo),
        ("l", x_lo, cy),
        ("r", x_hi, cy),
        ("bl", x_lo, y_hi),
        ("b", cx, y_hi),
        ("br", x_hi, y_hi),
    ];
    let mut sweep = Vec::new();
    for (name, ax, ay) in anchors {
        for fraction in SWEEP_FRACTIONS {
            let shift = [(ax - cx) * fraction, (ay - cy) * fraction];
            sweep.push((
                format!("f{name}{:.0}", fraction * 100.0),
                jacobian.tool_step(shift),
            ));
        }
    }
    sweep
}

/// Rotations of `home_pose` about each tool axis, repeated at every
/// standoff, solved back to joints on the home IK branch so every
/// excursion is the same arm posture with the wrist turned, not a
/// different way through the cell.
///
/// `standoffs_mm` shifts the set along the tool's own z before turning
/// it. At a single standoff every sample views the tag from the same
/// distance, so solvePnP's depth error — the weak axis of a planar
/// target — is the same in all of them, which is indistinguishable from
/// a camera mounted that much further out: `calibrateHandEye` absorbs it
/// into the translation it reports and no residual complains. Repeating
/// the rotations at another distance is what separates the two. The
/// un-turned pose each extra standoff contributes is a pure translation,
/// which constrains the rotation rather than the translation.
///
/// `sweep` carries [`frame_sweep`]'s tool-xy offsets, visited at the
/// aiming standoff only. They exist for the lens model rather than for
/// AX = XB, and the aiming standoff is where the same image displacement
/// costs the least tool travel — a pixel is worth Z/fx metres, so the far
/// standoffs would ask for the largest moves in the tightest part of the
/// reach and lose the poses to the clearance check.
///
/// `clear` decides whether the arm can actually get from `home` to a
/// solved pose, and asking it is not redundant with IK converging: a
/// solution can be perfectly valid kinematically while the straight line
/// to it leaves the arm's reach or crosses the stage. Because the capture
/// moves along that line and nothing else, this is the one question that
/// decides usability — answering it here means a refused move during the
/// capture is a real fault, not an expected outcome the loop has to
/// absorb.
pub fn schedule(
    model: &Model,
    home: &JointMap,
    home_pose: &Isometry3<f64>,
    angle_deg: f64,
    standoffs_mm: &[f64],
    sweep: &[(String, [f64; 2])],
    mut clear: impl FnMut(&JointMap) -> Result<bool, SequencerError>,
) -> Result<Schedule, SequencerError> {
    let mut targets: Vec<(String, Isometry3<f64>)> = Vec::new();
    for &standoff_mm in standoffs_mm {
        let shifted = |rotation: UnitQuaternion<f64>| {
            home_pose
                * Isometry3::from_parts(
                    nalgebra::Translation3::new(0.0, 0.0, standoff_mm / 1000.0),
                    rotation,
                )
        };
        let prefix = if standoff_mm == 0.0 {
            String::new()
        } else {
            format!("z{standoff_mm:+.0} ")
        };
        // The capture samples the aiming pose itself, so only the other
        // standoffs owe an un-turned pose.
        if standoff_mm != 0.0 {
            targets.push((
                format!("z{standoff_mm:+.0}"),
                shifted(UnitQuaternion::identity()),
            ));
        }
        for (name, axis) in AXES {
            for fraction in FRACTIONS {
                for sign in [1.0, -1.0] {
                    let deg = angle_deg * fraction * sign;
                    let rotation = UnitQuaternion::from_axis_angle(
                        &Unit::new_normalize(Vector3::new(axis[0], axis[1], axis[2])),
                        deg.to_radians(),
                    );
                    targets.push((format!("{prefix}r{name}{deg:+.1}"), shifted(rotation)));
                }
            }
        }
    }
    for (label, offset) in sweep {
        targets.push((
            label.clone(),
            home_pose
                * Isometry3::from_parts(
                    nalgebra::Translation3::new(offset[0], offset[1], 0.0),
                    UnitQuaternion::identity(),
                ),
        ));
    }

    let mut poses = Vec::new();
    let mut dropped = Vec::new();
    for (label, target) in targets {
        match model.ik_from_seed(home, &target, &label)? {
            Some(joints) if clear(&joints)? => poses.push((label, joints)),
            Some(_) => dropped.push((label, "no clear path")),
            None => dropped.push((label, "no IK")),
        }
    }
    Ok(Schedule { poses, dropped })
}

/// The lens model the detector solved every pose under, read from the
/// camera's own PVs at startup.
///
/// Recorded with the samples because a corner is only meaningful against
/// the model that turned it into a pose, and because the model changes
/// under you: the intrinsics come from the RealSense stream profile, so a
/// different resolution is a different `K`. It is also the thing under
/// suspicion — a capture whose ranges disagree with the robot's own
/// travel is either a mis-sized tag or a wrong `fx`, and neither is
/// answerable from a file that did not record them.
pub struct Intrinsics {
    /// Camera matrix, row-major.
    pub k: [f64; 9],
    /// Distortion coefficients in OpenCV order, however many the camera
    /// reports.
    pub dist: Vec<f64>,
    /// Tag edge length the object points were built from (m).
    pub tag_size_m: f64,
    pub image_size: [u32; 2],
    /// The depth stream's own camera matrix, row-major, when the camera
    /// serves one. Not the same as `k`: the plane is fitted by
    /// deprojecting with this, and re-solving the file has to use the
    /// same one or the plane moves.
    pub depth_k: Option<[f64; 9]>,
}

/// The Python detector, kept alive across the capture so cv2/numpy import
/// once instead of per pose.
pub struct Detector {
    child: Child,
    stdin: ChildStdin,
    stdout: BufReader<ChildStdout>,
    intrinsics: Intrinsics,
}

impl Detector {
    pub fn spawn(config: &HandEyeConfig) -> Result<Self, SequencerError> {
        let mut child = Command::new(&config.python)
            .arg(&config.detector)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .map_err(|e| {
                SequencerError(format!(
                    "cannot start detector '{} {}': {e}",
                    config.python.display(),
                    config.detector.display()
                ))
            })?;
        let stdin = child.stdin.take().expect("piped stdin");
        let mut stdout = BufReader::new(child.stdout.take().expect("piped stdout"));
        // The detector announces itself once it has the intrinsics and the
        // image channel; that line IS the readiness signal, so there is
        // nothing to ask it. Sending a ping here as well left one extra
        // reply in the pipe forever, and from then on every request was
        // answered by the one before it — in the capture loop that pairs
        // each robot pose with the previous pose's tag, which no
        // downstream check can catch.
        let hello = read_reply(&mut stdout, "hello")?;
        log::info(&format!("  detector: {}", field_str(&hello, "message")));
        // Read here rather than kept as an `Option` filled in later: the
        // capture has no use for a detector whose model is unknown, so
        // the state where one exists without the other should not be
        // constructible.
        let intrinsics = parse_intrinsics(&hello)?;
        Ok(Self {
            child,
            stdin,
            stdout,
            intrinsics,
        })
    }

    /// The lens model this detector is solving under.
    pub fn intrinsics(&self) -> &Intrinsics {
        &self.intrinsics
    }

    fn request(&mut self, cmd: &str) -> Result<String, SequencerError> {
        writeln!(self.stdin, "{{\"cmd\":\"{cmd}\"}}")
            .and_then(|()| self.stdin.flush())
            .map_err(|e| SequencerError(format!("detector write: {e}")))?;
        read_reply(&mut self.stdout, cmd)
    }

    /// `Ok(None)` when the tag was not in the frame — an ordinary outcome
    /// for a pose that turned too far, not a failure of the capture.
    pub fn detect(&mut self) -> Result<Option<Detection>, SequencerError> {
        let reply = self.request("detect")?;
        if !field_f64(&reply, "ok").is_some_and(|v| v != 0.0) {
            return Ok(None);
        }
        let t = field_vec(&reply, "t")?;
        let r = field_vec(&reply, "R")?;
        if t.len() != 3 || r.len() != 9 {
            return Err(SequencerError(format!(
                "detector returned t[{}] R[{}], want t[3] R[9]",
                t.len(),
                r.len()
            )));
        }
        let rotation = nalgebra::Rotation3::from_matrix_unchecked(nalgebra::Matrix3::new(
            r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8],
        ));
        let pose = Isometry3::from_parts(
            nalgebra::Translation3::new(t[0], t[1], t[2]),
            UnitQuaternion::from_rotation_matrix(&rotation),
        );
        let center = field_vec(&reply, "center")?;
        // Demanded, not defaulted: a detector too old to send corners
        // would otherwise write samples that look complete and cannot be
        // re-solved, which is the whole point of recording them.
        let corners = field_vec(&reply, "corners")?;
        let corners: [[f64; 2]; 4] = match corners.chunks_exact(2).collect::<Vec<_>>()[..] {
            [a, b, c, d] => [[a[0], a[1]], [b[0], b[1]], [c[0], c[1]], [d[0], d[1]]],
            _ => {
                return Err(SequencerError(format!(
                    "detector returned {} corner values, want 8",
                    corners.len()
                )));
            }
        };
        Ok(Some(Detection {
            cam_t_tag: pose,
            reproj_px: field_f64(&reply, "reproj").unwrap_or(f64::NAN),
            side_px: field_f64(&reply, "side_px").unwrap_or(f64::NAN),
            center_px: [
                center.first().copied().unwrap_or(f64::NAN),
                center.get(1).copied().unwrap_or(f64::NAN),
            ],
            corners_px: corners,
            depth: parse_plane(&reply)?,
        }))
    }
}

/// The depth plane out of a detect reply, `None` when the detector sent
/// none. A malformed one is an error rather than a silent `None`: a
/// plane that fails to parse is a detector the daemon does not
/// understand, and a capture that quietly drops the only absolute range
/// it has would be indistinguishable from one that never had depth.
fn parse_plane(reply: &str) -> Result<Option<DepthPlane>, SequencerError> {
    if !reply.contains("\"plane\"") {
        return Ok(None);
    }
    let plane = field_vec(reply, "plane")?;
    let [nx, ny, nz, d] = plane[..] else {
        return Err(SequencerError(format!(
            "detector returned {} plane values, want 4",
            plane.len()
        )));
    };
    Ok(Some(DepthPlane {
        normal: [nx, ny, nz],
        offset_m: d,
        range_m: field_f64(reply, "plane_range").unwrap_or(f64::NAN),
        rms_m: field_f64(reply, "plane_rms").unwrap_or(f64::NAN),
        pixels: field_f64(reply, "plane_px").unwrap_or(0.0) as u32,
    }))
}

/// How long a detector gets to exit on its own before it is killed. It
/// can be mid-read on the image PV, which blocks for up to five seconds,
/// but only one request is ever outstanding.
const QUIT_GRACE: std::time::Duration = std::time::Duration::from_secs(6);

impl Drop for Detector {
    fn drop(&mut self) {
        // Ask first so cv2 tears down cleanly, then kill if it will not
        // go. A plain blocking `wait()` here would be the daemon's
        // problem, not a one-shot tool's: the capture returns to the
        // trigger loop, and a detector wedged in a CA read would hold it
        // there forever.
        let _ = writeln!(self.stdin, "{{\"cmd\":\"quit\"}}");
        let _ = self.stdin.flush();
        let deadline = std::time::Instant::now() + QUIT_GRACE;
        loop {
            match self.child.try_wait() {
                Ok(Some(_)) => return,
                Err(_) => break,
                Ok(None) if std::time::Instant::now() >= deadline => break,
                Ok(None) => std::thread::sleep(std::time::Duration::from_millis(50)),
            }
        }
        log::warn("detector did not exit on request, killing it");
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// Reads one reply and refuses it unless it answers `expect`.
///
/// The echo check is the whole defence against a shifted stream: a
/// desynchronised pipe otherwise delivers well-formed, plausible
/// detections that belong to a different pose.
///
/// A free function over the reader rather than a method so the readiness
/// line can be read — and the model it carries parsed — before the
/// `Detector` is built.
fn read_reply(stdout: &mut BufReader<ChildStdout>, expect: &str) -> Result<String, SequencerError> {
    let mut line = String::new();
    let read = stdout
        .read_line(&mut line)
        .map_err(|e| SequencerError(format!("detector read: {e}")))?;
    if read == 0 {
        return Err(SequencerError("detector exited".into()));
    }
    let answered = field_str(&line, "cmd");
    if answered != expect {
        return Err(SequencerError(format!(
            "detector reply is out of step: expected an answer to '{expect}', \
             got one to '{answered}': {}",
            line.trim()
        )));
    }
    Ok(line)
}

/// Pulls the lens model out of the readiness line.
fn parse_intrinsics(hello: &str) -> Result<Intrinsics, SequencerError> {
    let k = field_vec(hello, "K")?;
    let k: [f64; 9] = k
        .clone()
        .try_into()
        .map_err(|_| SequencerError(format!("detector sent K[{}], want K[9]", k.len())))?;
    let size = field_vec(hello, "image_size")?;
    let [width, height] = size[..] else {
        return Err(SequencerError(format!(
            "detector sent image_size[{}], want [width, height]",
            size.len()
        )));
    };
    Ok(Intrinsics {
        k,
        dist: field_vec(hello, "dist")?,
        tag_size_m: field_f64(hello, "tag_size_m").ok_or_else(|| {
            SequencerError("detector did not report the tag size it assumed".into())
        })?,
        image_size: [width as u32, height as u32],
        // A camera without depth sends null here, which `field_vec`
        // reads as no numbers; a camera with one owes all nine.
        depth_k: match field_vec(hello, "depth_K") {
            Ok(v) if v.is_empty() => None,
            Ok(v) => Some(v.clone().try_into().map_err(|_| {
                SequencerError(format!("detector sent depth_K[{}], want 9", v.len()))
            })?),
            Err(_) => None,
        },
    })
}

// The replies are flat JSON of numbers and one string, so a full parser
// would be a dependency for nothing. These read that shape and no more.

fn field_str(json: &str, key: &str) -> String {
    let Some(rest) = json.split(&format!("\"{key}\"")).nth(1) else {
        return String::new();
    };
    let Some(open) = rest.find('"') else {
        return String::new();
    };
    let tail = &rest[open + 1..];
    tail.find('"')
        .map(|end| tail[..end].to_string())
        .unwrap_or_default()
}

fn field_f64(json: &str, key: &str) -> Option<f64> {
    let rest = json.split(&format!("\"{key}\"")).nth(1)?;
    let rest = rest.trim_start().strip_prefix(':')?.trim_start();
    if rest.starts_with("true") {
        return Some(1.0);
    }
    if rest.starts_with("false") || rest.starts_with("null") {
        return Some(0.0);
    }
    let end = rest
        .find(|c: char| !(c.is_ascii_digit() || c == '.' || c == '-' || c == '+' || c == 'e'))
        .unwrap_or(rest.len());
    rest[..end].parse().ok()
}

fn field_vec(json: &str, key: &str) -> Result<Vec<f64>, SequencerError> {
    let rest = json
        .split(&format!("\"{key}\""))
        .nth(1)
        .ok_or_else(|| SequencerError(format!("detector reply has no '{key}': {json}")))?;
    // A null is no numbers, not "look further along the line". Without
    // this the scan runs past it to whichever array comes next and
    // returns another key's values under this key's name.
    if rest.trim_start_matches([':', ' ']).starts_with("null") {
        return Ok(Vec::new());
    }
    let open = rest
        .find('[')
        .ok_or_else(|| SequencerError(format!("'{key}' is not an array: {json}")))?;
    let close = rest[open..]
        .find(']')
        .ok_or_else(|| SequencerError(format!("'{key}' is unterminated: {json}")))?;
    rest[open + 1..open + close]
        .split(',')
        .map(|piece| {
            piece
                .trim()
                .parse::<f64>()
                .map_err(|e| SequencerError(format!("'{key}' element '{piece}': {e}")))
        })
        .collect()
}

/// Writes the (robot pose, tag pose) pairs `tools/handeye/solve_joint.py` reads.
pub fn write_samples(
    path: &Path,
    samples: &[Sample],
    angle_deg: f64,
    intrinsics: &Intrinsics,
) -> Result<(), SequencerError> {
    let mut out = String::from(
        "# Hand-eye capture samples. base_t_ee is the ik_frame pose in the\n\
         # base frame (metres, quaternion xyzw); cam_t_tag is the tag pose\n\
         # in the camera frame from solvePnP. Feed to solve_joint.py, which\n\
         # holds the FK fixed and fits one camera pose and one tag pose for\n\
         # the whole capture; solve.py treats each view independently and is\n\
         # kept only as a cross-check.\n\
         #\n\
         # corners_px is the raw measurement each cam_t_tag was solved from,\n\
         # in the detector's object-point order, and intrinsics is the model\n\
         # it was solved under. Together they make a recalibrated lens a\n\
         # re-solve of this file rather than another capture on the robot.\n\
         #\n\
         # depth is the tag's plane as the depth stream measured it, n.X = d\n\
         # in the depth camera's frame (metres), deprojected with\n\
         # intrinsics.depth_camera_matrix. It is the only absolute range\n\
         # here: in the image, fx and the tag's true edge length trade\n\
         # against each other exactly, and no corner separates them.\n",
    );
    out.push_str(&format!("schedule_angle_deg: {angle_deg}\n"));
    out.push_str("intrinsics:\n");
    out.push_str(&format!(
        "  image_size: [{}, {}]\n  tag_size_m: {:.6}\n",
        intrinsics.image_size[0], intrinsics.image_size[1], intrinsics.tag_size_m
    ));
    let k = intrinsics.k;
    out.push_str(&format!(
        "  camera_matrix: [{:.6}, {:.6}, {:.6}, {:.6}, {:.6}, {:.6}, {:.6}, {:.6}, {:.6}]\n",
        k[0], k[1], k[2], k[3], k[4], k[5], k[6], k[7], k[8]
    ));
    let dist: Vec<String> = intrinsics.dist.iter().map(|c| format!("{c:.9}")).collect();
    out.push_str(&format!("  dist_coeffs: [{}]\n", dist.join(", ")));
    if let Some(dk) = intrinsics.depth_k {
        out.push_str(&format!(
            "  depth_camera_matrix: [{:.6}, {:.6}, {:.6}, {:.6}, {:.6}, {:.6}, {:.6}, {:.6}, {:.6}]\n",
            dk[0], dk[1], dk[2], dk[3], dk[4], dk[5], dk[6], dk[7], dk[8]
        ));
    }
    out.push_str("samples:\n");
    for s in samples {
        let t = s.base_t_ee.translation.vector;
        let q = s.base_t_ee.rotation.quaternion();
        let ct = s.seen.cam_t_tag.translation.vector;
        let cq = s.seen.cam_t_tag.rotation.quaternion();
        out.push_str(&format!("  - label: \"{}\"\n", s.label));
        out.push_str(&format!(
            "    base_t_ee: [{:.9}, {:.9}, {:.9}, {:.9}, {:.9}, {:.9}, {:.9}]\n",
            t.x, t.y, t.z, q.i, q.j, q.k, q.w
        ));
        out.push_str(&format!(
            "    cam_t_tag: [{:.9}, {:.9}, {:.9}, {:.9}, {:.9}, {:.9}, {:.9}]\n",
            ct.x, ct.y, ct.z, cq.i, cq.j, cq.k, cq.w
        ));
        out.push_str(&format!(
            "    reproj_px: {:.4}\n    side_px: {:.2}\n    center_px: [{:.1}, {:.1}]\n",
            s.seen.reproj_px, s.seen.side_px, s.seen.center_px[0], s.seen.center_px[1]
        ));
        let c = s.seen.corners_px;
        out.push_str(&format!(
            "    corners_px: [{:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}]\n",
            c[0][0], c[0][1], c[1][0], c[1][1], c[2][0], c[2][1], c[3][0], c[3][1]
        ));
        if let Some(d) = &s.seen.depth {
            out.push_str(&format!(
                "    depth: {{plane: [{:.9}, {:.9}, {:.9}, {:.9}], range_m: {:.6}, \
                 rms_m: {:.6}, pixels: {}}}\n",
                d.normal[0], d.normal[1], d.normal[2], d.offset_m, d.range_m, d.rms_m, d.pixels
            ));
        }
        out.push_str("    joints:\n");
        for (name, value) in &s.joints {
            out.push_str(&format!("      {name}: {value:.9}\n"));
        }
    }
    std::fs::write(path, out)
        .map_err(|e| SequencerError(format!("cannot write {}: {e}", path.display())))
}

/// Where [`save_aim_pose`] keeps the pose, inside the capture's `out_dir`
/// so a calibration's inputs and outputs stay in one place.
const AIM_POSE_FILE: &str = "aim_pose.yaml";

/// Records the pose a capture worked from, so a later re-calibration —
/// after the camera is moved or remounted — starts where the tag was
/// last seen instead of being jogged back by hand.
///
/// Called only once a capture has produced a usable samples file. A pose
/// that saw nothing is worse than no pose at all: it would send the next
/// run's aiming hold to a place the camera is known not to see the
/// target from, and the operator would have to jog out of it first.
pub fn save_aim_pose(dir: &Path, joints: &JointMap) -> Result<(), SequencerError> {
    let path = dir.join(AIM_POSE_FILE);
    let mut out = String::from(
        "# The pose the last usable hand-eye capture started from.\n\
         # CalibMode=3 returns here before the aiming hold, so the camera\n\
         # starts out looking at the tag. Delete this file to aim from\n\
         # wherever the arm happens to be instead.\n",
    );
    for (name, value) in joints {
        out.push_str(&format!("{name}: {value:.9}\n"));
    }
    std::fs::write(&path, out)
        .map_err(|e| SequencerError(format!("cannot write {}: {e}", path.display())))
}

/// The saved aiming pose, or `None` when no capture has succeeded here
/// yet.
///
/// A file that exists but cannot be read is an error rather than a
/// `None`: silently aiming from somewhere else would look identical to
/// the first-ever run, and the operator would have no way to tell that
/// the pose they saved is being ignored.
pub fn load_aim_pose(dir: &Path) -> Result<Option<JointMap>, SequencerError> {
    let path = dir.join(AIM_POSE_FILE);
    let text = match std::fs::read_to_string(&path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => {
            return Err(SequencerError(format!(
                "cannot read {}: {e}",
                path.display()
            )));
        }
    };
    let mut joints = JointMap::new();
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let (name, value) = line.split_once(':').ok_or_else(|| {
            SequencerError(format!(
                "{}: '{line}' is not 'joint: value'",
                path.display()
            ))
        })?;
        let value = value
            .trim()
            .parse::<f64>()
            .map_err(|e| SequencerError(format!("{}: {name}: '{value}': {e}", path.display())))?;
        joints.insert(name.trim().to_string(), value);
    }
    if joints.is_empty() {
        return Err(SequencerError(format!(
            "{}: no joints in the file",
            path.display()
        )));
    }
    Ok(Some(joints))
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;
    use crate::config::Config;
    use crate::waypoints::WaypointData;

    /// A taught pose to schedule rotations about, so the IK branch and the
    /// arm posture are the production ones rather than an invented state.
    fn production_home() -> (Model, JointMap, Isometry3<f64>) {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer.yaml"
        ));
        let config = Config::load(path).expect("load config");
        let model = Model::load(&config).expect("load model");
        let waypoints = WaypointData::load(&config.sequence.waypoints_yaml).expect("waypoints");
        let home: JointMap = WaypointData::arm_joints(&waypoints.holder1_standby)
            .into_iter()
            .collect();
        let pose = model.fk(&home).expect("fk");
        (model, home, pose)
    }

    /// The schedule must consult the clearance predicate, not only IK.
    /// A pose IK solves but the arm cannot reach along a straight line has
    /// to be dropped here: surviving to the capture loop means the move is
    /// refused mid-capture, and that error exits the daemon several poses
    /// in — which is how a run ended at `rx+8.0` with one usable sample.
    #[test]
    fn a_pose_that_is_not_clear_is_dropped() {
        let (model, home, home_pose) = production_home();

        let cleared =
            schedule(&model, &home, &home_pose, 8.0, &[0.0], &[], |_| Ok(true)).expect("schedule");
        assert!(
            !cleared.poses.is_empty(),
            "IK must solve at least one rotation about the taught pose"
        );
        assert!(
            cleared.dropped.iter().all(|(_, why)| *why == "no IK"),
            "a predicate that clears every path can only drop for IK"
        );

        let blocked =
            schedule(&model, &home, &home_pose, 8.0, &[0.0], &[], |_| Ok(false)).expect("schedule");
        assert!(
            blocked.poses.is_empty(),
            "no pose may survive a predicate that clears nothing"
        );
        assert_eq!(
            blocked
                .dropped
                .iter()
                .filter(|(_, why)| *why == "no clear path")
                .count(),
            cleared.poses.len(),
            "every pose IK solved must be dropped for its path, not lost"
        );
    }

    /// A clearance check that fails structurally is not a drop. Swallowing
    /// it would silently shrink the schedule instead of reporting a broken
    /// scene, and a capture that quietly visits fewer poses still writes a
    /// samples file the solver will happily accept.
    #[test]
    fn a_failing_clearance_check_is_an_error() {
        let (model, home, home_pose) = production_home();
        let result = schedule(&model, &home, &home_pose, 8.0, &[0.0], &[], |_| {
            Err(SequencerError("scene is broken".into()))
        });
        assert!(result.is_err(), "the check's error must propagate");
    }

    /// Every standoff must contribute the whole rotation set, at the
    /// distance it names and nowhere else. A standoff that silently
    /// collapsed onto the aiming pose would leave the depth error just as
    /// correlated as one distance, while the sample count says otherwise.
    #[test]
    fn each_standoff_repeats_the_rotations_at_its_own_distance() {
        let (model, home, home_pose) = production_home();

        let one =
            schedule(&model, &home, &home_pose, 8.0, &[0.0], &[], |_| Ok(true)).expect("schedule");
        let three = schedule(
            &model,
            &home,
            &home_pose,
            8.0,
            &[0.0, -50.0, 30.0],
            &[],
            |_| Ok(true),
        )
        .expect("schedule");

        let rotations = one.poses.len() + one.dropped.len();
        assert_eq!(
            three.poses.len() + three.dropped.len(),
            // Two extra standoffs, each a rotation set plus its own
            // un-turned pose.
            3 * rotations + 2,
            "a standoff must add a full rotation set, not a subset"
        );

        let labels: Vec<&str> = three
            .poses
            .iter()
            .map(|(label, _)| label.as_str())
            .chain(three.dropped.iter().map(|(label, _)| label.as_str()))
            .collect();
        assert!(
            labels.contains(&"rx+8.0"),
            "the aiming standoff keeps the bare label"
        );
        assert!(
            labels.contains(&"z-50 rx+8.0"),
            "a shifted standoff names itself"
        );
        assert!(
            labels.contains(&"z-50"),
            "a shifted standoff owes an un-turned pose"
        );
        assert!(!labels.contains(&"z+0"), "the aiming standoff owes none");

        // The shift is along the tool's own z, so it must show up as the
        // named distance between the two poses however the tool is turned.
        let at = |label: &str| {
            let joints = three
                .poses
                .iter()
                .find(|(l, _)| l == label)
                .map(|(_, j)| j)
                .unwrap_or_else(|| panic!("{label} must be in the schedule"));
            model.fk(joints).expect("fk").translation.vector
        };
        let shift = (at("z-50 rx+8.0") - at("rx+8.0")).norm();
        assert!(
            (shift - 0.050).abs() < 1e-3,
            "z-50 must sit 50 mm from its unshifted twin, got {:.1} mm",
            shift * 1000.0
        );
    }

    /// A camera bolted on at an arbitrary roll, at the working distance:
    /// 393 px focal over 290 mm is 1355 px per metre, turned 34 deg.
    const TEST_SCALE: f64 = 1355.0;
    const TEST_ROLL: f64 = 0.6;

    /// What the detector would report for a 20 mm step along each tool
    /// axis under that mounting.
    fn synthetic_probes() -> [([f64; 2], [f64; 2]); 2] {
        let p = 0.020;
        [([p, 0.0], forward([p, 0.0])), ([0.0, p], forward([0.0, p]))]
    }

    /// The truth the fit is supposed to recover.
    fn forward(step: [f64; 2]) -> [f64; 2] {
        let (c, s) = (TEST_ROLL.cos(), TEST_ROLL.sin());
        [
            TEST_SCALE * (c * step[0] - s * step[1]),
            TEST_SCALE * (s * step[0] + c * step[1]),
        ]
    }

    fn tag_at(cx: f64, cy: f64, side: f64) -> Detection {
        let h = side / 2.0;
        Detection {
            cam_t_tag: Isometry3::identity(),
            reproj_px: 0.0,
            side_px: side,
            center_px: [cx, cy],
            corners_px: [
                [cx - h, cy - h],
                [cx + h, cy - h],
                [cx + h, cy + h],
                [cx - h, cy + h],
            ],
            depth: None,
        }
    }

    /// The whole sweep is the inverse of this map applied to a desired
    /// pixel displacement, so a sign or a transpose wrong here sends the
    /// arm the opposite way and the tag straight off the frame.
    #[test]
    fn a_measured_jacobian_inverts_what_it_measured() {
        let jacobian = ImageJacobian::from_probes(&synthetic_probes()).expect("two good probes");
        for want in [[240.0, 165.0], [-300.0, 0.0], [0.0, -120.0]] {
            let got = forward(jacobian.tool_step(want));
            assert!(
                (got[0] - want[0]).hypot(got[1] - want[1]) < 1e-6,
                "asking for {want:?} px must produce {want:?} px, got {got:?}"
            );
        }
    }

    /// The probe pair is the only evidence the sweep has. Fitting to a
    /// pair that carries no direction still yields four finite numbers,
    /// and the arm would then execute a 200 mm move computed from noise.
    #[test]
    fn probes_that_carry_no_direction_are_refused() {
        let good = synthetic_probes();
        assert!(
            ImageJacobian::from_probes(&good[..1]).is_err(),
            "one probe cannot span the plane"
        );
        assert!(
            ImageJacobian::from_probes(&[good[0], (good[0].0, [1.0, 0.5])]).is_err(),
            "a tag that barely moved means the arm or the frame did not"
        );
        assert!(
            ImageJacobian::from_probes(&[good[0], ([0.040, 0.0], forward([0.040, 0.0]))]).is_err(),
            "two steps along the same tool axis leave the other undetermined"
        );
    }

    /// The point of the sweep: corners out where the lens model is
    /// otherwise unconstrained. In-place rotation reached r = 236 px of a
    /// 400 px half-diagonal, and inside that band a change in k1 is
    /// absorbed by fx and the principal point.
    #[test]
    fn the_sweep_puts_corners_where_rotation_cannot() {
        let jacobian = ImageJacobian::from_probes(&synthetic_probes()).expect("probes");
        let (w, h) = (640.0, 480.0);
        let at_home = tag_at(w / 2.0, h / 2.0, 135.0);
        let sweep = frame_sweep(&jacobian, [w as u32, h as u32], &at_home);
        assert_eq!(sweep.len(), 16, "eight directions at two radii");

        let half = at_home.side_px / 2.0;
        let mut furthest: f64 = 0.0;
        for (label, offset) in &sweep {
            let shift = forward(*offset);
            let (cx, cy) = (
                at_home.center_px[0] + shift[0],
                at_home.center_px[1] + shift[1],
            );
            assert!(
                cx - half >= SWEEP_MARGIN_PX - 1e-6
                    && cx + half <= w - SWEEP_MARGIN_PX + 1e-6
                    && cy - half >= SWEEP_MARGIN_PX - 1e-6
                    && cy + half <= h - SWEEP_MARGIN_PX + 1e-6,
                "{label} puts the tag partly outside the frame at ({cx:.0}, {cy:.0})"
            );
            for corner in [
                [cx - half, cy - half],
                [cx + half, cy - half],
                [cx + half, cy + half],
                [cx - half, cy + half],
            ] {
                furthest = furthest.max((corner[0] - w / 2.0).hypot(corner[1] - h / 2.0));
            }
        }
        assert!(
            furthest > 350.0,
            "the sweep must reach past the rotation set's r = 236 px, got {furthest:.0} px"
        );
    }

    /// A tag that already spans the frame has nowhere to be walked to.
    /// Offsets computed from a negative span would drive the arm inward
    /// past the tag, and the request that made the tag that big is a
    /// standoff, not a sweep.
    #[test]
    fn a_tag_that_fills_the_frame_gets_no_sweep() {
        let jacobian = ImageJacobian::from_probes(&synthetic_probes()).expect("probes");
        let filling = tag_at(320.0, 240.0, 470.0);
        assert!(frame_sweep(&jacobian, [640, 480], &filling).is_empty());
    }

    /// The saved aiming pose has to survive the round trip exactly: it is
    /// fed straight back to the arm as a joint goal, so a joint lost to a
    /// parse quirk moves the camera somewhere else entirely.
    #[test]
    fn an_aim_pose_round_trips() {
        let dir = std::env::temp_dir().join("handeye_aim_pose_round_trip");
        std::fs::create_dir_all(&dir).expect("temp dir");
        let _ = std::fs::remove_file(dir.join(AIM_POSE_FILE));
        assert!(
            load_aim_pose(&dir)
                .expect("absent is not an error")
                .is_none(),
            "no file means no saved pose, not a failure"
        );

        let mut pose = JointMap::new();
        pose.insert("shoulder_pan_joint".into(), -1.162864033);
        pose.insert("wrist_3_joint".into(), 0.010902119);
        save_aim_pose(&dir, &pose).expect("save");

        let read = load_aim_pose(&dir)
            .expect("load")
            .expect("a pose was saved");
        assert_eq!(read, pose, "every joint must come back with its value");

        std::fs::write(
            dir.join(AIM_POSE_FILE),
            "shoulder_pan_joint: not-a-number\n",
        )
        .expect("write");
        assert!(
            load_aim_pose(&dir).is_err(),
            "an unreadable file must be reported, not silently aimed past"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn reads_the_detector_reply_shape() {
        let reply = r#"{"ok":true,"cmd":"detect","id":0,"t":[0.01,-0.02,0.29],"R":[1,0,0,0,1,0,0,0,1],"reproj":0.31,"side_px":137.5,"center":[321.7,246.3],"corners":[253.0,177.9,390.4,178.1,390.3,315.4,252.9,315.2]}"#;
        assert_eq!(field_f64(reply, "ok"), Some(1.0));
        assert_eq!(field_str(reply, "cmd"), "detect");
        assert_eq!(field_f64(reply, "reproj"), Some(0.31));
        assert_eq!(field_vec(reply, "t").unwrap(), vec![0.01, -0.02, 0.29]);
        assert_eq!(field_vec(reply, "R").unwrap().len(), 9);
        assert_eq!(field_vec(reply, "center").unwrap(), vec![321.7, 246.3]);
        assert_eq!(field_vec(reply, "corners").unwrap().len(), 8);
    }

    /// The readiness line carries the model every pose is solved under,
    /// and a capture that cannot state its model writes samples nobody
    /// can re-solve. Missing fields must therefore fail the spawn, not
    /// default to something plausible.
    #[test]
    fn reads_the_model_off_the_readiness_line() {
        let hello = r#"{"ok":true,"cmd":"hello","message":"RS405: 640x480","K":[393.284,0.0,321.745,0.0,392.673,246.323,0.0,0.0,1.0],"dist":[-0.0503777,0.0602241,0.00047613,0.00129567,-0.0205373],"tag_size_m":0.1,"image_size":[640,480]}"#;
        let got = parse_intrinsics(hello).expect("a complete readiness line parses");
        assert_eq!(got.k[0], 393.284, "fx");
        assert_eq!(got.k[2], 321.745, "cx");
        assert_eq!(got.dist.len(), 5);
        assert_eq!(got.tag_size_m, 0.1);
        assert_eq!(got.image_size, [640, 480]);

        let old = r#"{"ok":true,"cmd":"hello","message":"RS405: 640x480"}"#;
        assert!(
            parse_intrinsics(old).is_err(),
            "a detector that does not report its model must not spawn"
        );
    }

    /// `depth_K: null` sits immediately before `K` in the readiness line.
    /// A scan that treats the null as "keep looking for an array" reads
    /// the colour camera's matrix and files it as the depth camera's —
    /// well-formed, plausible, and off by 4 px of principal point.
    #[test]
    fn a_null_field_is_no_numbers_and_not_the_next_array() {
        let no_depth = r#"{"ok":true,"cmd":"hello","message":"m","depth_K":null,"K":[393.284,0.0,321.745,0.0,392.673,246.323,0.0,0.0,1.0],"dist":[-0.05],"tag_size_m":0.1,"image_size":[640,480]}"#;
        assert_eq!(field_vec(no_depth, "depth_K").unwrap(), Vec::<f64>::new());
        let got = parse_intrinsics(no_depth).expect("a camera without depth still parses");
        assert!(got.depth_k.is_none(), "null must not become K");

        let with_depth = no_depth.replace(
            "\"depth_K\":null",
            "\"depth_K\":[389.555,0.0,322.806,0.0,389.555,244.102,0.0,0.0,1.0]",
        );
        let got = parse_intrinsics(&with_depth).expect("parse");
        let dk = got.depth_k.expect("a depth camera matrix was sent");
        assert_eq!((dk[0], dk[2], dk[5]), (389.555, 322.806, 244.102));
        assert_ne!(
            dk[0], got.k[0],
            "the two streams do not share a focal length"
        );
    }

    /// A reply with no plane is a pose without depth; a reply with a
    /// malformed one is a detector the daemon does not understand. The
    /// second must not be read as the first — a capture that quietly
    /// dropped its only absolute range would look exactly like one taken
    /// on a camera that has none.
    #[test]
    fn a_depth_plane_is_read_or_refused_but_never_dropped() {
        let bare = r#"{"ok":true,"cmd":"detect","t":[0,0,0.26],"R":[1,0,0,0,1,0,0,0,1]}"#;
        assert!(
            parse_plane(bare)
                .expect("no plane is not an error")
                .is_none()
        );

        let good = r#"{"ok":true,"cmd":"detect","plane":[0.14,-0.02,0.99,0.2547],"plane_range":0.25471,"plane_rms":0.00218,"plane_px":17979}"#;
        let d = parse_plane(good).expect("parse").expect("a plane was sent");
        assert_eq!(d.normal, [0.14, -0.02, 0.99]);
        assert_eq!(d.offset_m, 0.2547);
        assert_eq!(d.pixels, 17979);
        assert!((d.range_m - 0.25471).abs() < 1e-9);

        let short = r#"{"ok":true,"cmd":"detect","plane":[0.14,-0.02,0.99]}"#;
        assert!(
            parse_plane(short).is_err(),
            "a plane that is not four numbers is a detector mismatch"
        );

        // "plane_range" must not be mistaken for "plane".
        let ranged = r#"{"ok":true,"cmd":"detect","plane_range":0.25471}"#;
        assert!(parse_plane(ranged).expect("no plane key").is_none());
    }

    /// A failed detect carries `ok:false` and a reason string, and must
    /// not be mistaken for a success with a missing field.
    #[test]
    fn reads_a_failed_reply() {
        let reply = r#"{"ok":false,"cmd":"detect","reason":"no tag in frame"}"#;
        assert_eq!(field_f64(reply, "ok"), Some(0.0));
        assert_eq!(field_str(reply, "cmd"), "detect");
        assert_eq!(field_str(reply, "reason"), "no tag in frame");
        assert!(field_vec(reply, "t").is_err());
    }

    /// The shifted-stream case this echo exists for: a well-formed
    /// detection that answers the *previous* request must be rejected,
    /// not consumed. Reading the readiness line as if it were a detect is
    /// exactly how the stream got shifted on the robot.
    #[test]
    fn a_reply_to_another_command_is_not_a_detection() {
        let hello = r#"{"ok":true,"cmd":"hello","message":"RS405: 640x480"}"#;
        let detect = r#"{"ok":true,"cmd":"detect","t":[0.0,0.0,0.29]}"#;
        assert_ne!(field_str(hello, "cmd"), "detect");
        assert_eq!(field_str(detect, "cmd"), "detect");
        // Without the echo both look like successes: `ok` is true and the
        // only difference is a field the old parser never inspected.
        assert_eq!(field_f64(hello, "ok"), field_f64(detect, "ok"));
    }

    /// Negative exponents appear in the pose fields at these magnitudes.
    #[test]
    fn reads_exponent_notation() {
        let reply = r#"{"t":[1.5e-05,-2.5e-3,0.29]}"#;
        assert_eq!(field_vec(reply, "t").unwrap()[0], 1.5e-05);
        assert_eq!(field_vec(reply, "t").unwrap()[1], -2.5e-3);
    }
}
