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
//! is not, on any axis: every wall and the floor alike come from
//! [`Contact::wall_mm`], which fits the force-versus-travel samples after
//! contact and extrapolates back to the level the force sat at before it.
//! Where the slope *began* is a property of the metal; where a level was
//! *crossed* is a property of the threshold and of how big a step the
//! probe happened to be taking. That is the other reason every step is
//! kept rather than only the last, and the reason the probe takes a few
//! steps past contact rather than returning on it.
//!
//! It was not always one rule. The floor was fitted from the beginning
//! while the two lateral walls were read off the trip point, on the
//! argument that a threshold tripping late pushes both walls outward
//! equally and cancels in the midpoint. It does not cancel when the two
//! sides do not trip alike, which is exactly what a real seat gives: on
//! 2026-08-18 `base x+` ramped over 0.7 mm against a steady drag while
//! `base x-` went from noise to tripped in one step. Two meanings of
//! "where it touched" in one file is what produced that, so there is now
//! one.
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

use super::{Motion, q_to_map};
use crate::config::ProbeAxisConfig;
use crate::error::SequencerError;
use crate::log;
use crate::model::JointMap;

/// Fraction of a commanded step that must actually appear in the arm's
/// position for the step to count as taken.
///
/// The guard is not theoretical. Before [`Motion::probe_step`] existed the
/// arm executed 0.05 and 0.10 mm jogs as exactly 0.000 mm — TOTG dropped
/// the only other waypoint and the move was reported complete — and this
/// is what stopped the probe from reporting 1.5 mm of clearance it had
/// never traversed. It stays because a silent no-op is the failure this
/// primitive cannot tolerate, and the mechanism that produced one may not
/// be the only one. Contact, the legitimate reason for the arm to stop
/// advancing, has already returned by the time this is checked.
const STEP_TAKEN_FRACTION: f64 = 0.2;

/// RTDE packages averaged for each between-steps reading. At 125 Hz this
/// is 0.2 s of standing still, which puts the pose scatter under a micron
/// against a 0.05 mm step — see [`Session::mean_q_and_wrench`].
const SAMPLES_PER_READING: usize = 25;

/// The share of the abort limit at which the overtravel stops asking for
/// another sample.
///
/// Those steps are ones the probe chooses to take into something it
/// already knows is there, so they must never be the reason the abort
/// fires. Measured 2026-08-18: three 0.05 mm steps past the `base y+`
/// wall took the load from 2.48 N to 5.24 N and aborted a run that had
/// already found what it came for. Stopping at half leaves the abort
/// meaning what it meant before — one step meeting something much harder
/// than the step before it.
const OVERTRAVEL_LOAD_FRACTION: f64 = 0.5;
/// Slack on the step count so a travel that is an exact multiple of the
/// step in decimal is not one step short in binary.
const STEP_COUNT_EPSILON: f64 = 1e-9;

/// How many whole steps fit in `travel_mm`.
///
/// Rounded before flooring: 0.3 mm of travel in 0.1 mm steps is
/// 2.9999999999999996 in binary, and a bare floor turns a move asked for
/// in tenths into one step short of where it was sent — which for
/// [`Motion::probe_reposition`] is a height the caller then reports
/// wrongly.
fn steps_in(travel_mm: f64, step_mm: f64) -> usize {
    (travel_mm / step_mm + STEP_COUNT_EPSILON).floor() as usize
}

/// What ends a run of steps — the question the run is asking.
///
/// A probe asks *where* something is, and stops the moment the force
/// says it is here. A move asks to *be* somewhere, and force along the
/// way is only the abort's business. They cannot share one rule: leaving
/// a preloaded pose changes the force by as much as touching a wall
/// does, and the first 0.1 mm of a lift off the taught seat pose
/// released 5.00 N — five times the depth threshold — with nothing in
/// the way at all (measured 2026-08-18, doc §16.10).
#[derive(Debug, Clone, Copy)]
enum StopAt {
    /// Force along the axis reaching this, N.
    Contact(f64),
    /// The commanded travel, however the force behaves getting there.
    Travel,
}

impl StopAt {
    fn reached(&self, reading: &Reading) -> bool {
        match self {
            Self::Contact(threshold_n) => reading.is_contact(*threshold_n),
            Self::Travel => false,
        }
    }

    /// The force this run calls contact, if it is looking for any.
    fn threshold_n(&self) -> Option<f64> {
        match self {
            Self::Contact(n) => Some(*n),
            Self::Travel => None,
        }
    }
}

/// Why a run of steps ended, when it ended for a reason the mode can go
/// on measuring past.
///
/// The abort force is not a fault. It is the thing this mode exists to
/// notice, the arm is walked back out of it, and the run stands where it
/// started — so one direction meeting something hard must not throw away
/// the directions already measured. Faults that leave the arm somewhere
/// unknown (an RTDE read, a retrace that could not be flown, a step that
/// did not execute) stay `Err` and stop everything.
#[derive(Debug)]
enum Stopped {
    /// Ran out of allowance, or found what it was looking for.
    Done,
    /// Force passed `abort_n`, with the reason already worded for the log.
    TooHard(String),
}

/// What one direction produced.
#[derive(Debug, Clone)]
pub enum Probed {
    /// Samples, and whatever they say about where the wall is.
    Wall(Contact),
    /// The direction was given up on at the abort force. There is no
    /// wall in it — a probe that had to shove is not measuring a bore.
    TooHard(String),
}

impl Probed {
    fn contact(&self) -> Option<&Contact> {
        match self {
            Self::Wall(c) => Some(c),
            Self::TooHard(_) => None,
        }
    }

    /// The travel at which contact tripped, if this direction found one.
    pub fn tripped_mm(&self) -> Option<f64> {
        self.contact()?.tripped_mm()
    }

    /// The fitted wall, if this direction found one.
    pub fn wall_mm(&self) -> Option<f64> {
        self.contact()?.wall_mm()
    }

    /// Why this direction was given up on, if it was.
    pub fn too_hard(&self) -> Option<&str> {
        match self {
            Self::Wall(_) => None,
            Self::TooHard(why) => Some(why),
        }
    }
}

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
    /// What ended this run. Carried with the samples because it is what
    /// makes them mean anything: a run that was never looking for a wall
    /// has not found one, and [`Contact::wall_mm`] needs the threshold to
    /// tell a ramp from a rise that is still nothing.
    stop: StopAt,
    /// Index into the sample vectors of the step that tripped the contact
    /// threshold, or `None` if the probe ran out of allowance without
    /// touching anything.
    ///
    /// The index rather than the travel, because both readers need it:
    /// [`Contact::tripped_mm`] is one lookup away, and [`Contact::wall_mm`]
    /// needs to know which samples are *before* contact to have a baseline
    /// to measure the wall against.
    pub tripped: Option<usize>,
    /// Every pose the arm actually stood in, starting at the pose the
    /// probe began from and in the order they were reached.
    ///
    /// This is the way back. Measured rather than commanded, so under
    /// contact — where the servo gives up ground and the arm is not where
    /// it was told to be — the return still describes poses the arm has
    /// occupied. See [`Motion::probe_until_contact`], which flies it in
    /// reverse before returning.
    pub visited: Vec<JointMap>,
}

impl Contact {
    /// An empty record for a probe that is about to run.
    fn new(stop: StopAt) -> Self {
        Self {
            travel_mm: Vec::new(),
            lateral_n: Vec::new(),
            along_n: Vec::new(),
            stop,
            tripped: None,
            visited: Vec::new(),
        }
    }

    /// Travel at which the contact threshold tripped.
    pub fn tripped_mm(&self) -> Option<f64> {
        self.tripped.map(|i| self.travel_mm[i])
    }

    /// Where the wall is: the travel at which the force left the level it
    /// had been sitting at before contact.
    ///
    /// This, not the trip point, is the answer the mode exists to give.
    /// The trip point carries two things that are properties of the probe
    /// rather than of the metal — the threshold it had to clear, and up to
    /// one step of overshoot past first contact — and both are removed by
    /// asking where the *slope* began instead of where the *level* was
    /// crossed. The floor has been read this way since the beginning; the
    /// two lateral walls were read from the trip point until 2026-08-18,
    /// which is two meanings of "where it touched" in one file.
    ///
    /// Against the baseline, not against zero. A probe that is dragging
    /// something reads a steady force before it meets anything — `base x+`
    /// sat at 0.22-0.27 N for 0.7 mm on 2026-08-18 — and extrapolating
    /// that ramp back to zero would put the wall most of a millimetre
    /// short. The baseline is the median of the samples before the trip,
    /// which is the level the drag settled at, and the spread around it is
    /// the median absolute deviation, which is what "still just noise"
    /// means for this run rather than for a nominal one.
    ///
    /// `None` rather than a number that would look like a measurement
    /// when: nothing tripped, there are too few pre-contact samples to
    /// establish a baseline, the ramp is a single sample (which is not a
    /// slope), or the fit comes out flat or backwards (drift, or a grip
    /// letting go — pushing further in must read harder).
    pub fn wall_mm(&self) -> Option<f64> {
        let threshold_n = self.stop.threshold_n()?;
        let trip = self.tripped?;
        let force: Vec<f64> = self.along_n.iter().map(|f| f.abs()).collect();
        let before = &force[..trip];
        if before.len() < MIN_BASELINE_SAMPLES {
            return None;
        }
        let baseline = median(before);
        let deviations: Vec<f64> = before.iter().map(|f| (f - baseline).abs()).collect();
        // Out of the noise by whichever is the larger claim: what this run
        // actually scattered, or half of what was decided in advance to be
        // real contact on this axis. The scatter alone is not enough — on
        // the depth axis it sits near 0.1 N, which swept a 0.4 N shoulder
        // that is not the floor into the fit and halved the slope
        // (2026-08-18). The threshold alone is not enough either: it is
        // what the lateral ramp clears in a single step.
        let quiet = baseline + (NOISE_MADS * median(&deviations)).max(threshold_n / 2.0);

        // The ramp is the run of samples at the end that are out of the
        // noise, found from the last sample backwards: everything after
        // the probe met the wall, and nothing from before it.
        let ramp: Vec<(f64, f64)> = force
            .iter()
            .enumerate()
            .rev()
            .take_while(|(_, f)| **f > quiet)
            .map(|(i, f)| (self.travel_mm[i], *f))
            .collect();
        if ramp.len() < 2 {
            return None;
        }

        let n = ramp.len() as f64;
        let mean_d = ramp.iter().map(|(d, _)| d).sum::<f64>() / n;
        let mean_f = ramp.iter().map(|(_, f)| f).sum::<f64>() / n;
        let sxy: f64 = ramp.iter().map(|(d, f)| (d - mean_d) * (f - mean_f)).sum();
        let sxx: f64 = ramp.iter().map(|(d, _)| (d - mean_d).powi(2)).sum();
        if sxx <= f64::EPSILON {
            return None;
        }
        let slope = sxy / sxx;
        // Stiffness has a sign: pushing further in must read harder. A fit
        // that says otherwise is drift or a slipping grip, not a wall.
        if slope.abs() < f64::EPSILON || (slope > 0.0) != (mean_d > 0.0) {
            return None;
        }
        Some(mean_d - (mean_f - baseline) / slope)
    }
}

/// Pre-contact samples needed before a baseline is a baseline rather than
/// one reading that happened to be first.
const MIN_BASELINE_SAMPLES: usize = 3;

/// How many median absolute deviations above the baseline a sample has to
/// sit to count as part of the ramp rather than as more of the same noise.
const NOISE_MADS: f64 = 3.0;

/// Median of a non-empty slice, by value. Not a mean: the samples this is
/// asked about include the ones where the probe met something, and one
/// contact reading is large enough to drag a mean out of the noise it was
/// supposed to describe.
fn median(values: &[f64]) -> f64 {
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mid = sorted.len() / 2;
    if sorted.len() % 2 == 0 {
        (sorted[mid - 1] + sorted[mid]) / 2.0
    } else {
        sorted[mid]
    }
}

/// One between-steps force reading, split into the two things a probe
/// has to decide from it.
///
/// They are separate because they were once the same number
/// (`along.abs().max(lateral)`), and that let a force across the probe
/// direction declare a wall along it: measured 2026-08-18, the `base x-`
/// probe tripped on 1.037 N of lateral with only 0.256 N along, and put a
/// wall in the record at a travel that had nothing to do with one.
#[derive(Debug, Clone, Copy)]
struct Reading {
    /// Force change along the probe direction. A wall reacts along its own
    /// normal, which for a probe driven into it is this axis — sideways as
    /// much as downwards — so this alone is contact.
    along: f64,
    /// Force change across the probe direction: friction, the tool
    /// bending, the next joint settling. Recorded and reported, never a
    /// wall.
    lateral: f64,
    /// The whole force change. How hard the arm is leaning on the sample
    /// does not depend on which way the probe happens to be driving, so
    /// this is what the abort limit reads.
    load: f64,
}

impl Reading {
    fn new(df: Vector3, base_dir: &Vector3) -> Self {
        let along = df.dot(base_dir);
        Self {
            along,
            lateral: (df - base_dir * along).norm(),
            load: df.norm(),
        }
    }

    /// Something is pushing back along the way the probe is going.
    fn is_contact(&self, threshold_n: f64) -> bool {
        self.along.abs() >= threshold_n
    }

    /// The arm is leaning on the sample harder than this mode may.
    ///
    /// Never below the `max(|along|, lateral)` it replaced, so nothing
    /// that used to abort stops aborting.
    fn is_overload(&self, abort_n: f64) -> bool {
        self.load >= abort_n
    }
}

/// Two opposed probes from one pose, and what they did or did not find.
///
/// The pair is kept rather than reduced to a centre because "neither side
/// touched" and "one side touched" are different facts about the seat, and
/// a caller that only received `None` could not tell them apart. The first
/// free-air run reported "only one side touched" when neither had.
#[derive(Debug, Clone)]
pub struct Bracket {
    label: String,
    /// Travel allowed in each direction, for the message when nothing was
    /// found within it.
    travel_mm: f64,
    pub plus: Probed,
    pub minus: Probed,
}

impl Bracket {
    /// Where both walls are, and which estimator found them.
    ///
    /// One estimator for both sides, never one each. A fit on one wall and
    /// a trip point on the other differ by that side's overshoot, and the
    /// midpoint of the two would carry the whole of it — the exact error
    /// the fit was introduced to remove, reappearing as a bias instead of
    /// as a symmetric offset. So if either side cannot be fitted, both
    /// fall back to the trip point.
    fn walls_mm(&self) -> Option<(f64, f64, &'static str)> {
        if let (Some(p), Some(m)) = (self.plus.wall_mm(), self.minus.wall_mm()) {
            return Some((p, m, "fitted"));
        }
        match (self.plus.tripped_mm(), self.minus.tripped_mm()) {
            (Some(p), Some(m)) => Some((p, m, "at the trip point, no slope to fit")),
            _ => None,
        }
    }

    /// What to add to the current position to sit between the two walls,
    /// or `None` unless both were found.
    pub fn centre_mm(&self) -> Option<f64> {
        self.walls_mm().map(|(p, m, _)| (p - m) / 2.0)
    }

    /// Half the distance between the two walls: the radial clearance.
    ///
    /// Unlike the centre, this does not cancel anything — an estimator
    /// that reads both walls late reports a gap that is too wide by twice
    /// that, which is why [`Contact::wall_mm`] is worth having.
    pub fn half_gap_mm(&self) -> Option<f64> {
        self.walls_mm().map(|(p, m, _)| (p + m) / 2.0)
    }

    /// The directions this axis gave up on at the abort force.
    fn too_hard(&self) -> Vec<&str> {
        [self.plus.too_hard(), self.minus.too_hard()]
            .into_iter()
            .flatten()
            .collect()
    }

    /// One line saying which of the things happened.
    pub fn summary(&self) -> String {
        let label = &self.label;
        // Said first, because a direction that had to shove is not a
        // direction that found nothing, and the arithmetic below cannot
        // tell them apart.
        let hard = self.too_hard();
        if !hard.is_empty() {
            return format!("{label}: {} — no centre from this axis", hard.join("; "));
        }
        match (
            self.walls_mm(),
            self.plus.tripped_mm(),
            self.minus.tripped_mm(),
        ) {
            (Some((p, m, how)), _, _) => format!(
                "{label}: walls at {p:+.3} and {:+.3} mm ({how}) -> centre {:+.3} mm, \
                 clearance {:.3} mm per side",
                -m,
                self.centre_mm().unwrap_or(f64::NAN),
                self.half_gap_mm().unwrap_or(f64::NAN),
            ),
            (None, None, None) => format!(
                "{label}: nothing within {:.2} mm either way — no wall to measure from",
                self.travel_mm
            ),
            (None, Some(p), None) => format!(
                "{label}: only the + side touched, at {p:+.3} mm — one wall is not a centre"
            ),
            (None, None, Some(m)) => format!(
                "{label}: only the - side touched, at {:+.3} mm — one wall is not a centre",
                -m
            ),
            // Unreachable: both sides tripping is the first arm.
            (None, Some(_), Some(_)) => unreachable!("both sides tripped but neither wall placed"),
        }
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
    /// Steps to keep taking after contact, so the ramp the wall is fitted
    /// from has more than one sample in it.
    pub overtravel_steps: usize,
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
            overtravel_steps: axis.overtravel_steps,
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
    /// Returns once the threshold has tripped and the wall has been
    /// stepped past — `overtravel_steps` further steps, or fewer if the
    /// load reaches half the abort limit first, so the worst case is
    /// `1 + overtravel_steps` steps of travel past first contact and the
    /// probe never walks itself into the abort. Still bounded by
    /// `step_mm` by construction rather than by tuning, and `abort_n` is
    /// checked on every one of them.
    /// Those steps are what [`Contact::wall_mm`] fits: with the lateral
    /// threshold at 0.5 N and a wall this stiff, contact goes from noise
    /// to tripped in one step, and one sample is not a slope.
    ///
    /// Running out of `travel_mm` without a trip is not an error:
    /// "nothing within 0.6 mm" is an answer about the clearance, and the
    /// caller is the one that knows whether it expected to touch.
    ///
    /// Exceeding `abort_n` *is* an error. It means a step met something
    /// much harder than a bore wall — the floor while probing sideways, a
    /// puck that never entered the bore — and continuing to push at that
    /// is how a sample gets damaged. That limit reads the whole force
    /// change rather than the component along the probe: a sample being
    /// leant on sideways is being leant on, whichever way the probe
    /// happened to be driving.
    ///
    /// **The arm always ends where it started.** Every exit — contact,
    /// travel exhausted, the abort limit, a step that did not execute —
    /// goes back out along [`Contact::visited`] first. A caller cannot
    /// forget to return the arm, and cannot return it wrongly, because it
    /// is not the caller's job: leaving the arm pushing into a bore is the
    /// state this primitive must never hand back, least of all on the
    /// error paths where something has already gone wrong.
    pub fn probe_until_contact(
        &mut self,
        dir: Vector3,
        limits: ProbeLimits,
        label: &str,
    ) -> Result<Probed, SequencerError> {
        let mut out = Contact::new(StopAt::Contact(limits.threshold_n));
        let stepping = self.step_along(
            dir,
            limits,
            StopAt::Contact(limits.threshold_n),
            label,
            &mut out,
        );
        // Reversed, so it leaves from where the arm is standing now and
        // ends at the pose the probe began from.
        let back: Vec<JointMap> = out.visited.iter().rev().cloned().collect();
        let returned = self.retrace(&back, limits.velocity_scale, label);
        match (stepping, returned) {
            (Ok(Stopped::Done), Ok(())) => Ok(Probed::Wall(out)),
            (Ok(Stopped::TooHard(why)), Ok(())) => {
                log::warn(&format!("  {why}"));
                Ok(Probed::TooHard(why))
            }
            (Ok(_), Err(b)) => Err(b),
            (Err(e), Ok(())) => Err(e),
            // The probe failure is the one the operator has to act on; the
            // retrace failure is why the arm is not where they will look
            // for it, so neither can be dropped.
            (Err(e), Err(b)) => Err(SequencerError(format!(
                "{e} — and the arm could not be walked back out: {b}"
            ))),
        }
    }

    /// The stepping half of [`Motion::probe_until_contact`] and of
    /// [`Motion::probe_reposition`], which own the return. Split out so
    /// that every way this can leave — including `?` on an RTDE read — is
    /// a return into code that walks the arm back.
    fn step_along(
        &mut self,
        dir: Vector3,
        limits: ProbeLimits,
        stop: StopAt,
        label: &str,
        out: &mut Contact,
    ) -> Result<Stopped, SequencerError> {
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

        let (start_q, start_wrench) = self
            .rtde
            .session()?
            .mean_q_and_wrench(SAMPLES_PER_READING)?;
        let start_joints = q_to_map(&start_q);
        let start_pose = self.model.fk(&start_joints)?;
        // The probe axis in base, taken once at the start pose. A pure
        // translation leaves the tool orientation alone, so this is the
        // axis for every step, and using one axis for both the travel and
        // the force projection is what keeps the slope fit meaningful.
        let base_dir = start_pose.rotation * unit;
        let steps = steps_in(limits.travel_mm, limits.step_mm);
        let stopping = match stop {
            StopAt::Contact(n) => format!("contact at {n:.2} N"),
            StopAt::Travel => "no contact stop — this is a move".to_string(),
        };
        log::info(&format!(
            "{label}: probing up to {:.2} mm in {} steps of {:.3} mm, \
             {stopping}, abort at {:.2} N",
            limits.travel_mm, steps, limits.step_mm, limits.abort_n
        ));

        out.travel_mm.reserve(steps);
        out.lateral_n.reserve(steps);
        out.along_n.reserve(steps);
        // The way back starts at the pose the probe began from, so it is
        // recorded before the first step rather than after it.
        out.visited.push(start_joints.clone());

        let mut previous = 0.0;
        // Steps taken before contact, against the travel allowance, and
        // steps taken after it, against `overtravel_steps`. Two counters
        // because they are bounded by two different things: the allowance
        // says how far the arm may go looking for a wall, the overtravel
        // says how far past one it may push to measure it.
        let mut taken = 0usize;
        let mut extra = 0usize;
        // What the last step read, so the overtravel can stop on the force
        // it has already produced rather than on the one it is about to.
        let mut last_load = 0.0;
        loop {
            let overtravelling = out.tripped.is_some();
            if !overtravelling && taken >= steps {
                break;
            }
            if overtravelling
                && (extra >= limits.overtravel_steps
                    || last_load >= limits.abort_n * OVERTRAVEL_LOAD_FRACTION)
            {
                break;
            }
            let d = unit * limits.step_mm;
            self.probe_step(d.x, d.y, d.z, limits.velocity_scale)?;

            let (q, now) = self
                .rtde
                .session()?
                .mean_q_and_wrench(SAMPLES_PER_READING)?;
            let here_joints = q_to_map(&q);
            // Recorded before anything can fail below: a step that put the
            // arm somewhere and then tripped the abort limit still has to
            // be walked back out of.
            out.visited.push(here_joints.clone());
            taken += 1;
            if overtravelling {
                extra += 1;
            }
            let df = Vector3::new(
                now[0] - start_wrench[0],
                now[1] - start_wrench[1],
                now[2] - start_wrench[2],
            );
            let reading = Reading::new(df, &base_dir);
            let here = self.model.fk(&here_joints)?;
            let travel =
                (here.translation.vector - start_pose.translation.vector).dot(&base_dir) * 1000.0;
            out.travel_mm.push(travel);
            out.along_n.push(reading.along);
            out.lateral_n.push(reading.lateral);

            let load = reading.load;
            last_load = load;
            log::info(&format!(
                "  {label}: {travel:+.3} mm, along {:+.3} N, lateral {:.3} N",
                reading.along, reading.lateral
            ));
            if reading.is_overload(limits.abort_n) {
                return Ok(Stopped::TooHard(format!(
                    "{label}: {load:.2} N at {travel:+.3} mm exceeds the {:.2} N abort limit \
                     — the probe met something harder than a bore wall",
                    limits.abort_n
                )));
            }
            if !overtravelling && stop.reached(&reading) {
                out.tripped = Some(out.travel_mm.len() - 1);
                log::info(&format!("  {label}: contact at {travel:+.3} mm"));
            }
            // Only while the arm is still looking for the wall. Past
            // contact a step that does not fully execute is the servo
            // giving up ground to something solid, which is the
            // measurement rather than a fault — the same reasoning the
            // guard's own doc gives for why contact returns before it.
            if out.tripped.is_none() && travel - previous < limits.step_mm * STEP_TAKEN_FRACTION {
                return Err(SequencerError(format!(
                    "{label}: commanded {:.3} mm and moved {:.3} mm with only {load:.2} N \
                     pushing back — the step did not execute, so any clearance reported \
                     from here would be travel that never happened",
                    limits.step_mm,
                    travel - previous
                )));
            }
            previous = travel;
        }
        if out.tripped.is_none() {
            log::info(&format!(
                "  {label}: no contact within {:.2} mm",
                limits.travel_mm
            ));
        }
        Ok(Stopped::Done)
    }

    /// Moves `mm` along `dir` in probe-sized steps and **stays there**,
    /// failing if the arm cannot get there.
    ///
    /// The one motion in this mode that is not a measurement. It exists
    /// because the operator jog cannot do it: [`Motion::jog`] gates on the
    /// scene, the stage mesh is a convex decomposition, and a convex hull
    /// cannot represent a bore — so from a seated pose every jog is
    /// refused as a collision before it starts (measured 2026-08-15, and
    /// again 2026-08-18 on a 3 mm jog straight up). The probe's own step
    /// is guarded by contact instead of by geometry, which is the guard
    /// that means anything inside a bore, so the height change is made of
    /// probe steps.
    ///
    /// Arriving is the goal, so this asks nothing about where a wall is
    /// and takes no contact threshold ([`StopAt::Travel`]). What is left
    /// is the pair of guards that answer "did the arm get there": every
    /// step must actually execute, and the abort force still bounds how
    /// hard the arm may lean on the way. A jam trips one or the other —
    /// the arm stops moving, or it shoves. The arm is walked back out
    /// before the error returns, as everywhere else in this file.
    pub fn probe_reposition(
        &mut self,
        dir: Vector3,
        mm: f64,
        limits: ProbeLimits,
        label: &str,
    ) -> Result<(), SequencerError> {
        if mm.abs() < f64::EPSILON {
            return Ok(());
        }
        // The step is a granularity here, not a quantum: a probe's step
        // is the resolution it reports a wall at and its travel is a
        // bound, but a move has to land exactly where it was sent. Whole
        // configured steps do not always divide the distance — 0.05 mm in
        // 0.10 mm steps is zero of them — so the distance is split into
        // the fewest steps no larger than the configured one.
        let steps = (mm.abs() / limits.step_mm - STEP_COUNT_EPSILON)
            .ceil()
            .max(1.0);
        let limits = ProbeLimits {
            travel_mm: mm.abs(),
            step_mm: mm.abs() / steps,
            // Nothing to fit: this is not measuring a wall.
            overtravel_steps: 0,
            ..limits
        };
        let dir = if mm < 0.0 { -dir } else { dir };
        let mut out = Contact::new(StopAt::Travel);
        // Here the abort force *is* a fault, unlike in a probe: a move
        // that had to shove did not arrive, and everything measured from
        // where it stopped would be at a height nothing recorded.
        let why = match self.step_along(dir, limits, StopAt::Travel, label, &mut out) {
            Ok(Stopped::Done) => return Ok(()),
            Ok(Stopped::TooHard(why)) => SequencerError(why),
            Err(e) => e,
        };
        let back: Vec<JointMap> = out.visited.iter().rev().cloned().collect();
        // `why` is already labelled: it comes from the stepping, which
        // names itself in everything it returns.
        Err(match self.retrace(&back, limits.velocity_scale, label) {
            Ok(()) => why,
            Err(b) => SequencerError(format!(
                "{why} — and the arm could not be walked back out: {b}"
            )),
        })
    }

    /// Both walls along one tool axis, and the middle between them.
    ///
    /// Everything is measured from the pose the arm starts at.
    ///
    /// Both directions start from the same pose, because
    /// [`Motion::probe_until_contact`] puts the arm back before it
    /// returns. The two contacts are therefore measured against one
    /// origin rather than against each other, and this function contains
    /// no motion of its own — there is no return here to get wrong.
    pub fn bracket_axis(
        &mut self,
        dir: Vector3,
        limits: ProbeLimits,
        label: &str,
    ) -> Result<Bracket, SequencerError> {
        let plus = self.probe_until_contact(dir, limits, &format!("{label}+"))?;
        let minus = self.probe_until_contact(-dir, limits, &format!("{label}-"))?;

        let bracket = Bracket {
            label: label.to_string(),
            plus,
            minus,
            travel_mm: limits.travel_mm,
        };
        match bracket.centre_mm() {
            Some(_) => log::info(&bracket.summary()),
            None => log::warn(&bracket.summary()),
        }
        Ok(bracket)
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
    use crate::config::Config;
    use crate::model::Model;
    use crate::waypoints::WaypointData;

    /// Why [`Motion::probe_step`] carries its own TOTG threshold, as an
    /// assertion rather than a comment: at the poses the probe actually
    /// runs at, one configured step moves every joint less than TOTG's
    /// ordinary de-duplication threshold, so the ordinary path would drop
    /// the move and report it done. Measured on the arm before this was
    /// found — 0.05 and 0.10 mm jogs travelled 0.000 mm.
    ///
    /// It also pins the other side: the fine threshold has to keep the
    /// step, and by a margin that is not a coincidence of one pose.
    #[test]
    fn a_probe_step_is_smaller_than_totg_keeps_by_default() {
        let path = std::path::Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer.yaml"
        ));
        let config = Config::load(path).expect("load config");
        let model = Model::load(&config).expect("load model");
        let w = WaypointData::load(&config.sequence.waypoints_yaml).expect("waypoints");
        let step_mm = config.probe.lateral.step_mm.min(config.probe.depth.step_mm);

        for (name, taught) in [
            ("holder1_on_position", &w.holder1_on_position),
            ("sample_holder_on_position", &w.sample_holder_on_position),
        ] {
            let joints: JointMap = WaypointData::arm_joints(taught).into_iter().collect();
            for axis in 0..3 {
                let mut offset = [0.0; 3];
                offset[axis] = step_mm / 1000.0;
                let shifted = model
                    .apply_cartesian_offset(&joints, offset, false, "probe step")
                    .expect("offset");
                let moved = joints
                    .iter()
                    .map(|(k, v)| (shifted[k] - v).abs())
                    .fold(0.0f64, f64::max);
                assert!(
                    moved < super::super::MIN_ANGLE_CHANGE,
                    "{name} axis {axis}: {step_mm} mm moves a joint {moved} rad, which the \
                     ordinary path would keep — probe_step may no longer need its own \
                     threshold, but check every pose before dropping it"
                );
                assert!(
                    moved > super::super::FINE_MIN_ANGLE_CHANGE * 100.0,
                    "{name} axis {axis}: {step_mm} mm moves a joint only {moved} rad, too \
                     close to the fine threshold to survive it"
                );
            }
        }
    }

    /// The defect [`Reading`] exists for: base x- on 2026-08-18 read
    /// +0.256 N along the probe and 1.037 N across it, and the old
    /// `max` declared a wall there. Contact reads the probe direction
    /// alone; the load, which the abort limit reads, still counts it.
    #[test]
    fn a_force_across_the_probe_is_not_a_wall_along_it() {
        let dir = Vector3::x();
        let r = Reading::new(Vector3::new(0.256, 1.037, 0.0), &dir);
        assert!(
            !r.is_contact(0.5),
            "along {:.3} N is under the threshold",
            r.along
        );
        assert!((r.lateral - 1.037).abs() < 1e-12);
        assert!(
            r.is_overload(1.0),
            "load {:.3} N is what leans on the sample",
            r.load
        );
    }

    /// Contact does not care which way along the probe axis the force
    /// points: pushing down, the floor reads negative.
    #[test]
    fn contact_is_the_probe_axis_either_way() {
        let down = -Vector3::z();
        assert!(Reading::new(Vector3::new(0.0, 0.0, 2.6), &down).is_contact(1.0));
        assert!(Reading::new(Vector3::new(0.0, 0.0, -2.6), &down).is_contact(1.0));
    }

    /// The abort is never made later by the split — it reads the whole
    /// force, which is at least the larger component it replaced.
    #[test]
    fn the_overload_is_never_below_the_larger_component() {
        let dir = Vector3::x();
        for df in [
            Vector3::new(3.0, 4.0, 0.0),
            Vector3::new(0.0, 5.0, 0.0),
            Vector3::new(5.0, 0.0, 0.0),
        ] {
            let r = Reading::new(df, &dir);
            assert!(r.load >= r.along.abs().max(r.lateral) - 1e-12, "{r:?}");
        }
    }

    /// Samples as the arm would record them: force sits at whatever level
    /// it was already at, then grows against the motion once the probe
    /// meets something.
    fn contact(travel: &[f64], along: &[f64], tripped: Option<usize>) -> Contact {
        with_threshold(travel, along, tripped, 1.0)
    }

    fn with_threshold(
        travel: &[f64],
        along: &[f64],
        tripped: Option<usize>,
        threshold_n: f64,
    ) -> Contact {
        Contact {
            travel_mm: travel.to_vec(),
            lateral_n: vec![0.0; travel.len()],
            along_n: along.to_vec(),
            stop: StopAt::Contact(threshold_n),
            tripped,
            visited: Vec::new(),
        }
    }

    /// One direction shoving is not the same as it finding nothing, and
    /// the arithmetic cannot tell them apart — both leave no wall.
    #[test]
    fn a_direction_given_up_on_is_not_a_direction_that_found_nothing() {
        let hard = Bracket {
            label: "base y".to_string(),
            travel_mm: 3.0,
            plus: Probed::TooHard("base y+: 8.65 N at +0.051 mm".to_string()),
            minus: Probed::Wall(contact(&[0.1, 0.2, 0.3], &[0.0, 0.0, 0.0], None)),
        };
        assert!(hard.centre_mm().is_none());
        assert!(hard.summary().contains("8.65 N"));

        let empty = bracket(None, None);
        assert!(empty.centre_mm().is_none());
        assert!(empty.summary().contains("nothing within"));
    }

    /// A move is not a measurement. The samples can look exactly like a
    /// wall — they will, on the way out of a preloaded seat — and there
    /// is still no wall in them, because nothing was looking for one.
    #[test]
    fn a_move_has_no_wall_however_its_force_behaved() {
        let mut c = with_threshold(
            &[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            &[0.0, 0.0, 0.0, 0.0, -1.0, -3.0, -5.0],
            Some(4),
            1.0,
        );
        assert!(c.wall_mm().is_some());
        c.stop = StopAt::Travel;
        assert!(c.wall_mm().is_none());
    }

    /// The force that ends a probe does not end a move: releasing a 5 N
    /// preload reads the same as meeting something.
    #[test]
    fn a_move_does_not_stop_for_force() {
        let up = Vector3::z();
        let releasing = Reading::new(Vector3::new(0.0, 0.0, -5.0), &up);
        assert!(StopAt::Contact(1.0).reached(&releasing));
        assert!(!StopAt::Travel.reached(&releasing));
    }

    /// A move shorter than one configured step is still a move.
    #[test]
    fn a_move_is_split_into_steps_that_land_on_it() {
        // What `probe_reposition` computes, kept honest here because the
        // arm is what runs the real one.
        let split = |mm: f64, step: f64| {
            let steps = (mm / step - STEP_COUNT_EPSILON).ceil().max(1.0);
            (steps as usize, mm / steps)
        };
        assert_eq!(split(0.05, 0.10).0, 1);
        assert_eq!(split(0.30, 0.10).0, 3);
        assert_eq!(split(3.00, 0.10).0, 30);
        // Every split lands on the distance, and no step is bigger than
        // the configured one.
        for (mm, step) in [(0.05, 0.1), (0.3, 0.1), (0.25, 0.1), (3.0, 0.1)] {
            let (n, taken) = split(mm, step);
            assert!(taken <= step + 1e-12, "{mm} in {step} steps");
            assert!((n as f64 * taken - mm).abs() < 1e-12);
            assert_eq!(steps_in(mm, taken), n);
        }
    }

    /// Tenths of a millimetre are not exact in binary, and a move that
    /// stops one step short puts the whole level at the wrong height.
    #[test]
    fn a_travel_in_tenths_is_not_a_step_short() {
        assert_eq!(steps_in(0.3, 0.1), 3);
        assert_eq!(steps_in(0.5, 0.1), 5);
        assert_eq!(steps_in(3.0, 0.1), 30);
        assert_eq!(steps_in(1.5, 0.05), 30);
        // Not rounding up: a travel allowance is a bound, and 0.35 mm
        // buys three whole steps of 0.1, not four.
        assert_eq!(steps_in(0.35, 0.1), 3);
    }

    /// The point of the fit: the wall is where the force left the level it
    /// was sitting at, not the 1.0 N line the probe happened to stop at.
    #[test]
    fn the_fit_recovers_the_floor_the_threshold_overshot() {
        let c = contact(
            &[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            &[0.0, 0.0, 0.0, 0.0, -1.0, -3.0, -5.0],
            Some(4),
        );
        let wall = c.wall_mm().expect("three rising samples are a slope");
        assert!(
            (wall - 0.45).abs() < 1e-9,
            "floor at {wall:.6} mm, expected 0.45"
        );
        assert_eq!(c.tripped_mm(), Some(0.5), "and the trip point is later");
    }

    /// A probe that is dragging something reads a steady force before it
    /// meets anything — `base x+` sat at 0.22-0.27 N for 0.7 mm on
    /// 2026-08-18. The wall is where the ramp leaves *that* level; against
    /// zero the same samples put it short.
    #[test]
    fn the_wall_is_measured_against_the_drag_not_against_zero() {
        let c = with_threshold(
            &[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
            &[0.24, 0.26, 0.25, 0.24, 0.26, 0.25, 0.55, 0.95],
            Some(6),
            0.5,
        );
        let wall = c.wall_mm().expect("two ramp samples are a slope");
        assert!(
            (wall - 0.625).abs() < 1e-9,
            "wall at {wall:.6} mm, expected 0.625"
        );
        // The same ramp extrapolated to zero force, which is what ignoring
        // the drag would give.
        assert!(wall > 0.5625, "and that is later than the zero intercept");
    }

    /// The depth run of 2026-08-18: force sat at the noise, rose to a
    /// 0.4-0.5 N shoulder that is not the floor, then jumped. The
    /// shoulder is far outside the run's own scatter, so the scatter
    /// alone would take it into the fit and halve the slope; half the
    /// contact threshold keeps it out.
    #[test]
    fn a_shoulder_under_the_threshold_is_not_part_of_the_ramp() {
        let c = with_threshold(
            &[0.10, 0.20, 0.32, 0.42, 0.52, 0.63, 0.74, 0.85, 0.96, 1.06],
            &[
                0.002, -0.030, -0.005, -0.057, -0.040, -0.032, -0.495, -0.382, -0.472, -2.133,
            ],
            Some(9),
            1.0,
        );
        // One sample clears baseline + threshold/2, and one is not a slope.
        assert_eq!(c.wall_mm(), None);
        // With the scatter alone as the cut, the shoulder joins the ramp
        // and the fit reports a floor 0.2 mm above where the jump is.
        assert_eq!(c.tripped_mm(), Some(1.06));
    }

    #[test]
    fn one_ramp_sample_is_not_a_slope() {
        let c = contact(&[0.1, 0.2, 0.3, 0.4], &[0.0, 0.0, 0.0, -1.0], Some(3));
        assert_eq!(c.wall_mm(), None);
    }

    /// Force that falls as the probe pushes deeper is drift or a grip
    /// letting go. Extrapolating it would put the "floor" above the arm.
    #[test]
    fn a_backwards_slope_is_not_a_wall() {
        let c = contact(
            &[0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            &[0.0, 0.0, 0.0, -5.0, -3.0, -1.0],
            Some(3),
        );
        assert_eq!(c.wall_mm(), None);
    }

    /// Every ramp sample at the same depth: the arm stopped advancing
    /// while the force kept climbing, which is a stall, not a fit.
    #[test]
    fn no_spread_in_depth_is_not_a_fit() {
        let c = contact(
            &[0.1, 0.2, 0.3, 0.3, 0.3],
            &[0.0, 0.0, 0.0, -1.0, -3.0],
            Some(3),
        );
        assert_eq!(c.wall_mm(), None);
    }

    /// A wall met on the second step has nothing to measure itself
    /// against, and a baseline from one reading is that reading.
    #[test]
    fn too_few_samples_before_contact_have_no_baseline() {
        let c = contact(&[0.1, 0.2, 0.3], &[0.0, -1.0, -3.0], Some(1));
        assert_eq!(c.wall_mm(), None);
    }

    /// Nothing tripped, so nothing is a wall — however the force behaved.
    #[test]
    fn no_contact_is_no_wall() {
        let c = contact(&[0.1, 0.2, 0.3, 0.4], &[0.0, 0.0, 0.1, 0.2], None);
        assert_eq!(c.wall_mm(), None);
        assert_eq!(c.tripped_mm(), None);
    }

    /// Trip points only, no ramp to fit either side: the bracket falls
    /// back and says so.
    fn bracket(plus: Option<f64>, minus: Option<f64>) -> Bracket {
        let side = |t: Option<f64>| match t {
            Some(travel) => contact(&[travel], &[9.0], Some(0)),
            None => contact(&[], &[], None),
        };
        Bracket {
            label: "base x".into(),
            travel_mm: 1.5,
            plus: Probed::Wall(side(plus)),
            minus: Probed::Wall(side(minus)),
        }
    }

    /// Both probes report their own travel as positive, each along its own
    /// direction, so the minus wall sits at `-minus` on the plus axis.
    /// A bore whose centre is 0.1 mm in the plus direction from the start
    /// pose, with 0.5 mm of clearance, gives 0.6 and 0.4.
    #[test]
    fn the_centre_is_the_midpoint_of_the_two_walls() {
        let b = bracket(Some(0.6), Some(0.4));
        assert!((b.centre_mm().unwrap() - 0.1).abs() < 1e-12);
        assert!((b.half_gap_mm().unwrap() - 0.5).abs() < 1e-12);
    }

    /// One side fitted and the other read off its trip point differ by
    /// that side's overshoot alone, and the midpoint would carry all of
    /// it. Both sides use the estimator that both sides can support.
    #[test]
    fn a_bracket_never_mixes_a_fitted_wall_with_a_trip_point() {
        let fitted = contact(
            &[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            &[0.0, 0.0, 0.0, 0.0, 1.0, 3.0, 5.0],
            Some(4),
        );
        assert!(fitted.wall_mm().is_some(), "the plus side can be fitted");
        let b = Bracket {
            label: "base x".into(),
            travel_mm: 1.5,
            plus: Probed::Wall(fitted),
            minus: Probed::Wall(contact(&[0.4], &[9.0], Some(0))),
        };
        assert_eq!(b.minus.wall_mm(), None, "the minus side cannot");
        // 0.5 and 0.4, the two trip points, giving 0.05 — not the fitted
        // 0.45 against the same 0.4, which would give 0.025.
        assert!((b.centre_mm().unwrap() - 0.05).abs() < 1e-12);
        assert!(b.summary().contains("trip point"), "{}", b.summary());
    }

    /// Both sides fitted: the report says so, and the numbers are the
    /// fitted ones.
    #[test]
    fn two_fitted_walls_are_reported_as_fitted() {
        let side = |sign: f64| {
            contact(
                &[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
                &[0.0, 0.0, 0.0, 0.0, sign, sign * 3.0, sign * 5.0],
                Some(4),
            )
        };
        let b = Bracket {
            label: "base x".into(),
            travel_mm: 1.5,
            plus: Probed::Wall(side(1.0)),
            minus: Probed::Wall(side(-1.0)),
        };
        assert!((b.centre_mm().unwrap()).abs() < 1e-12);
        assert!((b.half_gap_mm().unwrap() - 0.45).abs() < 1e-12);
        assert!(b.summary().contains("fitted"), "{}", b.summary());
    }

    /// The defect this type was introduced for: the first free-air run
    /// touched neither wall and the report called it one.
    #[test]
    fn neither_wall_is_not_one_wall() {
        let neither = bracket(None, None).summary();
        assert!(
            neither.contains("nothing within") && !neither.contains("only"),
            "{neither}"
        );
        assert!(bracket(Some(0.6), None).summary().contains("only the +"));
        assert!(bracket(None, Some(0.4)).summary().contains("only the -"));
        assert_eq!(bracket(None, None).centre_mm(), None);
        assert_eq!(bracket(Some(0.6), None).centre_mm(), None);
    }
}
