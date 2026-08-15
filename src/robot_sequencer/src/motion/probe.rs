//! Motion that stops on contact, and the bracket it exists to serve.
//!
//! Everything else in this module drives the arm to a pose that was taught
//! or planned. This drives it until something pushes back. The difference
//! matters because the numbers the sequence needs — where a bore actually
//! is, how deep its floor actually sits — are properties of the metal, and
//! a taught pose is a record of where an operator once thought it was.
//!
//! # Why stepping rather than a trajectory that gets cancelled
//!
//! The obvious build is to start a slow move at the wall and cancel it when
//! the force trips. It was not built that way, for two reasons that both
//! come out of the same measurement (§16.1):
//!
//! - **Noise.** Standing still, Fz scatters 0.073 N. Moving 5 mm in free
//!   air the lateral force swings 0.56 to 1.07 N — inertia, not contact. A
//!   threshold that has to clear the moving noise sits above 1 N, which is
//!   also where a real rub starts; one that only has to clear the still
//!   noise sits at 0.5 N and is still seven sigma clear of it. Reading
//!   between steps rather than during them buys that order of magnitude,
//!   and it is the whole reason a gentle probe is possible at all.
//! - **Overshoot.** A cancelled move still decelerates, and whatever force
//!   builds during that deceleration is applied to the sample. Stepping has
//!   no momentum to shed: the worst case is one step of travel past first
//!   contact, which `step_mm` bounds by construction rather than by tuning.
//!
//! The cost is wall-clock, and a probe is a commissioning operation rather
//! than something a cycle does, so wall-clock is the cheap axis here.
//!
//! # Why the answer does not depend on the threshold
//!
//! A threshold decides when to *stop*, and stopping early or late would
//! move the answer if the answer were "the position where it tripped". It
//! is not. Two axes, two different reasons:
//!
//! - **Lateral (x, y).** The bore has a wall on both sides, so the probe
//!   brackets: contact at `+d`, contact at `-d`, and the centre is the
//!   midpoint. A threshold that trips consistently late pushes both
//!   contacts outward by the same amount and leaves the midpoint where it
//!   was. What it changes is the *clearance* estimate, which is reported
//!   with its own caveat.
//! - **Vertical (z).** One floor, no bracket, so the midpoint trick is not
//!   available. Instead the force-versus-depth samples after contact are
//!   fitted and extrapolated back to zero: the intercept is the touch
//!   point whatever threshold collected the samples. That is what
//!   [`Contact::touch_at`] returns, and it needs the samples, which is the
//!   other reason every step is kept rather than only the last.
//!
//! # What bounds the force
//!
//! Above, the grip: the sequence closes the Hand-E at full scale and stalls
//! it on the sample, and slip would have to beat that, so the gripper is
//! nowhere near the binding constraint. Below, the sample: nothing in this
//! file knows what is inside the puck, so the ceiling is configuration and
//! the default is set where contact is unambiguous rather than where the
//! hardware would allow.

use cspace_core::geometry::Vector3;

use super::Motion;
use crate::config::ProbeAxisConfig;
use crate::error::SequencerError;
use crate::log;
use crate::model::JointMap;

/// Fraction of a commanded step that must actually appear in the arm's
/// position for the step to count as taken.
///
/// [`Motion::jog`] returns without moving when its interpolation produces
/// fewer than two states, and at these step sizes that is a real
/// possibility. A probe that reports a clearance it never traversed is
/// worse than one that stops, so a step that does not show up in FK is an
/// error — contact, the legitimate reason for the arm to stop advancing,
/// has already returned by the time this is checked.
const STEP_TAKEN_FRACTION: f64 = 0.2;

/// One probe's worth of stepping: where it started, every place it stood,
/// and what pushed back there.
#[derive(Debug, Clone)]
pub struct Contact {
    /// Signed travel from the start pose along the probe direction, mm,
    /// one per step taken. Measured from FK rather than counted from the
    /// commands: under contact the servo gives up ground, and it is the
    /// ground actually given up that the force belongs to.
    pub travel_mm: Vec<f64>,
    /// Lateral force magnitude relative to the start pose, N, per step.
    pub lateral_n: Vec<f64>,
    /// Force along the probe direction relative to the start pose, N.
    pub along_n: Vec<f64>,
    /// Travel at which the threshold tripped, or `None` if the probe ran
    /// out of allowance without touching anything.
    pub tripped_mm: Option<f64>,
    /// Joints at the last step, so a caller can put the arm back.
    pub end_joints: JointMap,
}

impl Contact {
    /// Where the force would have been zero, from the samples after
    /// contact — the touch point without a threshold in it.
    ///
    /// A straight line through the rising samples, extrapolated back. Fewer
    /// than two of them is not a slope, and a fit that comes out flat or
    /// backwards is not contact, so both give `None` rather than a number
    /// that would look like a measurement.
    pub fn touch_at(&self, threshold_n: f64) -> Option<f64> {
        let rising: Vec<(f64, f64)> = self
            .travel_mm
            .iter()
            .zip(&self.along_n)
            .filter(|(_, f)| f.abs() >= threshold_n * 0.5)
            .map(|(d, f)| (*d, f.abs()))
            .collect();
        if rising.len() < 2 {
            return None;
        }
        let n = rising.len() as f64;
        let mean_d = rising.iter().map(|(d, _)| d).sum::<f64>() / n;
        let mean_f = rising.iter().map(|(_, f)| f).sum::<f64>() / n;
        let sxy: f64 = rising
            .iter()
            .map(|(d, f)| (d - mean_d) * (f - mean_f))
            .sum();
        let sxx: f64 = rising.iter().map(|(d, _)| (d - mean_d).powi(2)).sum();
        if sxx <= f64::EPSILON {
            return None;
        }
        let slope = sxy / sxx;
        // Stiffness has a sign: pushing further in must read harder. A fit
        // that says otherwise is drift or a slipping grip, not a floor.
        if slope.abs() < f64::EPSILON || (slope > 0.0) != (mean_d > 0.0) {
            return None;
        }
        Some(mean_d - mean_f / slope)
    }
}

/// How a probe is allowed to move and when it must stop.
#[derive(Debug, Clone, Copy)]
pub struct ProbeLimits {
    /// One step, mm. Also the worst-case overshoot past first contact.
    pub step_mm: f64,
    /// Total travel allowed in one direction, mm.
    pub travel_mm: f64,
    /// Force change from the start pose that counts as contact, N.
    pub threshold_n: f64,
    /// Force change that aborts the probe outright, N. Above the threshold
    /// so one hard step cannot be mistaken for the gentle contact.
    pub abort_n: f64,
    /// Velocity scale for each step.
    pub velocity_scale: f64,
}

impl ProbeLimits {
    /// One configured direction at the shared probe speed.
    pub fn new(axis: &ProbeAxisConfig, velocity_scale: f64) -> Self {
        Self {
            step_mm: axis.step_mm,
            travel_mm: axis.travel_mm,
            threshold_n: axis.threshold_n,
            abort_n: axis.abort_n,
            velocity_scale,
        }
    }
}

impl Motion<'_> {
    /// Step along a tool-frame direction until something pushes back.
    ///
    /// `dir` is a unit-ish vector in the `ik_frame`, the same frame
    /// [`Motion::jog`] takes, so a caller asks for "+x at the tool" without
    /// knowing where the arm is standing. Force is read **between** steps
    /// with the arm stationary — see the module doc for why that is the
    /// whole design and not an implementation detail.
    ///
    /// Returns as soon as the threshold trips, having taken at most one
    /// step past first contact. Running out of `travel_mm` without a trip
    /// is not an error: "nothing within 0.6 mm" is an answer about the
    /// clearance, and the caller is the one that knows whether it expected
    /// to touch.
    ///
    /// Exceeding `abort_n` *is* an error. It means a step met something
    /// much harder than a bore wall — the floor while probing sideways, a
    /// puck that never entered the bore — and continuing to push at that
    /// is how a sample gets damaged.
    pub fn probe_until_contact(
        &mut self,
        dir: Vector3,
        limits: ProbeLimits,
        label: &str,
    ) -> Result<Contact, SequencerError> {
        let norm = dir.norm();
        if norm < f64::EPSILON {
            return Err(SequencerError(format!("{label}: probe direction is zero")));
        }
        let unit = dir / norm;
        if limits.step_mm <= 0.0 || limits.travel_mm <= 0.0 {
            return Err(SequencerError(format!(
                "{label}: probe step {} mm and travel {} mm must both be positive",
                limits.step_mm, limits.travel_mm
            )));
        }

        let start_joints = self.current_joints()?;
        let start_pose = self.model.fk(&start_joints)?;
        // The probe axis in base, taken once at the start pose. A pure
        // translation leaves the tool orientation alone, so this is the
        // axis for every step, and using one axis for both the travel and
        // the force projection is what keeps the slope fit meaningful.
        let base_dir = start_pose.rotation * unit;
        let start_wrench = self.rtde.session()?.fresh_wrench()?;
        let steps = (limits.travel_mm / limits.step_mm).floor() as usize;
        log::info(&format!(
            "{label}: probing up to {:.2} mm in {} steps of {:.3} mm, \
             contact at {:.2} N, abort at {:.2} N",
            limits.travel_mm, steps, limits.step_mm, limits.threshold_n, limits.abort_n
        ));

        let mut out = Contact {
            travel_mm: Vec::with_capacity(steps),
            lateral_n: Vec::with_capacity(steps),
            along_n: Vec::with_capacity(steps),
            tripped_mm: None,
            end_joints: start_joints.clone(),
        };

        let mut previous = 0.0;
        for _ in 1..=steps {
            let d = unit * limits.step_mm;
            self.jog(d.x, d.y, d.z, limits.velocity_scale)?;
            out.end_joints = self.current_joints()?;

            let now = self.rtde.session()?.fresh_wrench()?;
            let df = Vector3::new(
                now[0] - start_wrench[0],
                now[1] - start_wrench[1],
                now[2] - start_wrench[2],
            );
            // Split the change into "along the way I am pushing" and "across
            // it". The along component is what a floor gives back and what
            // the slope fit needs; the lateral magnitude is what a wall
            // gives when the probe is sideways. Both are taken against the
            // start pose, which is where the arm's own bias cancels.
            let along = df.dot(&base_dir);
            let lateral = (df - base_dir * along).norm();
            let here = self.model.fk(&out.end_joints)?;
            let travel =
                (here.translation.vector - start_pose.translation.vector).dot(&base_dir) * 1000.0;
            out.travel_mm.push(travel);
            out.along_n.push(along);
            out.lateral_n.push(lateral);

            let felt = along.abs().max(lateral);
            log::info(&format!(
                "  {label}: {travel:+.3} mm, along {along:+.3} N, lateral {lateral:.3} N"
            ));
            if felt >= limits.abort_n {
                return Err(SequencerError(format!(
                    "{label}: {felt:.2} N at {travel:+.3} mm exceeds the {:.2} N abort limit \
                     — the probe met something harder than a bore wall",
                    limits.abort_n
                )));
            }
            if felt >= limits.threshold_n {
                out.tripped_mm = Some(travel);
                log::info(&format!("  {label}: contact at {travel:+.3} mm"));
                return Ok(out);
            }
            if travel - previous < limits.step_mm * STEP_TAKEN_FRACTION {
                return Err(SequencerError(format!(
                    "{label}: commanded {:.3} mm and moved {:.3} mm with only {felt:.2} N \
                     pushing back — the step did not execute, so any clearance reported \
                     from here would be travel that never happened",
                    limits.step_mm,
                    travel - previous
                )));
            }
            previous = travel;
        }
        log::info(&format!(
            "  {label}: no contact within {:.2} mm",
            limits.travel_mm
        ));
        Ok(out)
    }

    /// Both walls along one tool axis, and the middle between them.
    ///
    /// Returns `(centre_mm, half_gap_mm)` measured from the pose the arm
    /// starts at, plus the two probes. `centre_mm` is what a caller adds to
    /// the current position to sit in the middle of the bore; `half_gap_mm`
    /// is the radial clearance, which unlike the centre *does* carry the
    /// threshold in it.
    ///
    /// Both directions start from the same pose and the arm is returned to
    /// it in between, so the two contacts are measured against one origin
    /// rather than against each other.
    ///
    /// The returns are [`Motion::move_direct`] and never
    /// [`Motion::move_cartesian`]: the latter falls back to the planner
    /// when the line comes up short, and RRT-Connect joining two poses a
    /// millimetre apart has been measured swinging the TCP 0.71 m. With a
    /// puck seated in a bore that is not a longer route to the same place,
    /// it is the sample being dragged out sideways. A line that cannot be
    /// followed back the way it came means the arm is not where this
    /// thinks it is, and stopping is the only safe answer to that.
    pub fn bracket_axis(
        &mut self,
        dir: Vector3,
        limits: ProbeLimits,
        label: &str,
    ) -> Result<(Option<f64>, Option<f64>, Contact, Contact), SequencerError> {
        let home = self.current_joints()?;

        let plus = self.probe_until_contact(dir, limits, &format!("{label}+"))?;
        self.move_direct(
            &home,
            limits.velocity_scale,
            limits.velocity_scale,
            "probe return",
        )?;

        let minus = self.probe_until_contact(-dir, limits, &format!("{label}-"))?;
        self.move_direct(
            &home,
            limits.velocity_scale,
            limits.velocity_scale,
            "probe return",
        )?;

        let (centre, half_gap) = match (plus.tripped_mm, minus.tripped_mm) {
            (Some(p), Some(m)) => (Some((p - m) / 2.0), Some((p + m) / 2.0)),
            _ => (None, None),
        };
        match (centre, half_gap) {
            (Some(c), Some(h)) => log::info(&format!(
                "{label}: walls at {:+.3} and {:+.3} mm -> centre {c:+.3} mm, clearance {h:.3} mm",
                plus.tripped_mm.unwrap_or(f64::NAN),
                -minus.tripped_mm.unwrap_or(f64::NAN),
            )),
            _ => log::warn(&format!(
                "{label}: only one side touched — no centre from a single wall"
            )),
        }
        Ok((centre, half_gap, plus, minus))
    }

    /// A base-frame direction expressed in the tool frame at the current
    /// pose, which is the frame [`Motion::jog`] and therefore
    /// [`Motion::probe_until_contact`] take.
    ///
    /// A caller asks in base because that is the frame the answer is
    /// wanted in: the rack is a base-axis-aligned grid, and "the bore
    /// centre is 0.23 mm along base y from here" is directly comparable
    /// to the per-holder pitch the waypoints already carry. Which tool
    /// axis happens to point that way is a fact about the gripper mount,
    /// and nothing that measures the rack should have to know it.
    pub fn base_dir_in_tool(&mut self, base: &Vector3) -> Result<Vector3, SequencerError> {
        let here = self.current_joints()?;
        let tf = self.model.fk(&here)?;
        Ok(tf.rotation.inverse() * *base)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Samples as the arm would record them pushing down: force stays at
    /// the noise until the floor, then grows against the motion, so the
    /// component along the probe direction is negative.
    fn contact(travel: &[f64], along: &[f64]) -> Contact {
        Contact {
            travel_mm: travel.to_vec(),
            lateral_n: vec![0.0; travel.len()],
            along_n: along.to_vec(),
            tripped_mm: travel.last().copied(),
            end_joints: JointMap::new(),
        }
    }

    /// The point of the fit: the intercept is where the force would have
    /// been zero, which is the floor, not the 1.0 N line the probe
    /// happened to stop at.
    #[test]
    fn the_fit_recovers_the_floor_the_threshold_overshot() {
        let c = contact(&[0.1, 0.2, 0.3, 0.4, 0.5], &[0.0, 0.0, -1.0, -3.0, -5.0]);
        let touch = c.touch_at(1.0).expect("three rising samples are a slope");
        assert!(
            (touch - 0.25).abs() < 1e-9,
            "floor at {touch:.6} mm, expected 0.25"
        );
        assert_eq!(c.tripped_mm, Some(0.5), "and the trip point is later");
    }

    #[test]
    fn one_rising_sample_is_not_a_slope() {
        let c = contact(&[0.1, 0.2, 0.3], &[0.0, 0.0, -1.0]);
        assert_eq!(c.touch_at(1.0), None);
    }

    /// Force that falls as the probe pushes deeper is drift or a grip
    /// letting go. Extrapolating it would put the "floor" above the arm.
    #[test]
    fn a_backwards_slope_is_not_a_floor() {
        let c = contact(&[0.3, 0.4, 0.5], &[-5.0, -3.0, -1.0]);
        assert_eq!(c.touch_at(1.0), None);
    }

    /// Every rising sample at the same depth: the arm stopped advancing
    /// while the force kept climbing, which is a stall, not a fit.
    #[test]
    fn no_spread_in_depth_is_not_a_fit() {
        let c = contact(&[0.3, 0.3], &[-1.0, -3.0]);
        assert_eq!(c.touch_at(1.0), None);
    }
}
