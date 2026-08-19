//! Sequencer configuration (`config/sequencer.yaml`).
//!
//! Field-for-field mirror of the replaced ROS node's parameters plus the
//! pieces that used to live in separate ROS nodes: the stage collision
//! scene (`stage_scene_utils` + `setup_stage_acm.py`) and the joint-limit
//! overrides (`ur3e_hande_moveit_config/config/joint_limits.yaml`).

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::error::SequencerError;
use crate::motion::{MIN_BASELINE_SAMPLES, MIN_EXECUTABLE_MM};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub robot: RobotConfig,
    pub joint_limits: JointLimitsConfig,
    pub epics: EpicsConfig,
    pub sequence: SequenceConfig,
    pub gripper: GripperConfig,
    pub scene: SceneConfig,
    #[serde(default)]
    pub vision: VisionConfig,
    #[serde(default)]
    pub handeye: HandEyeConfig,
    #[serde(default)]
    pub probe: ProbeConfig,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotConfig {
    pub ip: String,
    pub urdf: PathBuf,
    pub srdf: PathBuf,
    pub mesh_packages: HashMap<String, PathBuf>,
    pub group: String,
    pub ik_frame: String,
    pub script_file: PathBuf,
    pub output_recipe: PathBuf,
    pub input_recipe: PathBuf,
    /// RTDE output stream rate.
    ///
    /// Nothing here needs the controller's 500 Hz: trajectories go out
    /// over the trajectory interface, and RTDE is read only for a joint
    /// sample before planning and for the execution wait loop. What the
    /// rate does set is how fast unread packages fill the 131 KB socket
    /// buffer — at 500 Hz that is half a second, and URControl drops a
    /// client that lets it fill.
    ///
    /// Independent of the servoj control period, which
    /// `Motion::connect` takes from `max_frequency()` — the
    /// controller's own rate, not this request.
    #[serde(default = "default_rtde_frequency_hz")]
    pub rtde_frequency_hz: f64,
}

fn default_rtde_frequency_hz() -> f64 {
    50.0
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointLimitsConfig {
    pub max_velocity: f64,
    pub max_acceleration: f64,
    #[serde(default)]
    pub position_overrides: HashMap<String, [f64; 2]>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EpicsConfig {
    pub trigger_pv: String,
    pub start_step_pv: String,
    pub wait_pv: String,
    pub holder_pv: String,
    pub stop_pv: String,
    pub current_step_pv: String,
    pub gripper_pv: String,
    pub gripper_rbv_pv: String,
    pub pause_step_pv: String,
    pub calib_mode_pv: String,
    pub loaded_pv: String,
    pub jog_x_pv: String,
    pub jog_y_pv: String,
    pub jog_z_pv: String,
    pub jog_step_pv: String,
    /// Defaulted so configs that predate the record keep loading; the
    /// channel itself is optional in `Epics::connect` for the same
    /// reason.
    #[serde(default = "default_map_source_pv")]
    pub map_source_pv: String,
}

fn default_map_source_pv() -> String {
    "Robot:MapSource".into()
}

/// The level-tool path constraint (see [`SequenceConfig::level_tool`]).
///
/// The constraint is "the tool's approach axis stays in the horizontal
/// plane", not "the tool keeps one fixed orientation": the taught poses
/// approach the holder along -Y and the stage along +X, 92 degrees
/// apart, so a fixed orientation would make the stage unreachable.
/// Rotation within the horizontal plane is therefore left free.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LevelToolConfig {
    pub enabled: bool,
    /// How far the approach axis may leave horizontal, in degrees. The
    /// taught poses are themselves 1.64 and 0.84 degrees off level, so
    /// anything below ~2 makes them unreachable.
    pub tolerance_deg: f64,
}

impl Default for LevelToolConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            tolerance_deg: 3.0,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SequenceConfig {
    pub waypoints_yaml: PathBuf,
    pub holder_offset: f64,
    pub velocity_scale: f64,
    pub acceleration_scale: f64,
    pub cartesian_translation_step: f64,
    pub cartesian_rotation_step: f64,
    pub cartesian_min_fraction: f64,
    /// Keep the gripper level through joint-space plans.
    ///
    /// Joint-space planning only constrains the goal, so RRT-Connect is
    /// free to roll the tool over on the way there — measured on the
    /// real robot, the 63-degree move between the holder and the stage
    /// swung the gripper to vertical mid-path. Cartesian steps already
    /// interpolate the pose and are unaffected.
    #[serde(default)]
    pub level_tool: LevelToolConfig,
    pub jog_velocity_scale: f64,
    /// Extra scale on every Cartesian step. The Cartesian steps are the
    /// choreography around the rack and the stage — approach, insert,
    /// extract, retreat — while the free-air transits (steps 1, 7, 18)
    /// are planned moves; this multiplies
    /// `velocity_scale`/`acceleration_scale` on the former only, so the
    /// arm moves gently near hardware and keeps its pace between.
    #[serde(default = "default_cartesian_velocity_scale")]
    pub cartesian_velocity_scale: f64,
}

fn default_cartesian_velocity_scale() -> f64 {
    1.0
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GripperConfig {
    pub mode: GripperMode,
    pub open_position: f64,
    pub close_position: f64,
    pub reach_tolerance: f64,
    pub settle_timeout: f64,
    pub open_threshold: f64,
    /// Hand-E background communication rate. Every cycle is two Modbus
    /// transactions, and the tool-communication bridge drops roughly one
    /// transaction in a few thousand, so halving the rate halves how
    /// often that costs anything. 20 ms between position samples is well
    /// inside what the settle wait needs.
    #[serde(default = "default_gripper_poll_hz")]
    pub poll_hz: u32,
    /// How hard the fingers close on a sample, on [0, 1] of the Hand-E's
    /// 20-185 N.
    ///
    /// Full scale is what every close used to send, and it is 185 N onto
    /// a puck that then has to fit a bore with 0.50 mm of nominal radial
    /// clearance. The gripped puck was measured held within 0.05 mm in
    /// all six directions at the taught seat pose while a free one has
    /// ten times that (doc §16.12), so how hard it is squeezed is not a
    /// detail of the grip — it is part of whether the puck fits.
    #[serde(default = "default_grip_force")]
    pub grip_force: f64,
    /// How fast the fingers move, on [0, 1] of the Hand-E's 20-150 mm/s.
    ///
    /// The close is commanded to `close_position` and stalls on the
    /// sample, so this is the speed the pads arrive at it with.
    #[serde(default = "default_grip_speed")]
    pub grip_speed: f64,
    /// A sequence close that settles narrower than this gripped nothing
    /// — the pads met each other, not a puck — and the step fails
    /// instead of the run building on an empty hand. Metres; 0 disables.
    /// Ignored by the simulated gripper, whose fingers always reach the
    /// commanded position exactly.
    ///
    /// It is a *settle* width, so it only became measurable once the
    /// close stopped ending on a position band: measured 2026-08-19, a
    /// held puck settles at 3.9 mm and the pads meeting each other in
    /// free air at 0.7 mm. The 8 mm this carried before was read off the
    /// band-exit values, which a close reported at 11.0-11.4 mm whether
    /// or not anything was between the fingers — above a real grip, so
    /// on the true readings it failed every successful pick.
    #[serde(default)]
    pub min_grip_position: f64,
}

fn default_gripper_poll_hz() -> u32 {
    50
}

fn default_grip_force() -> f64 {
    0.05
}

fn default_grip_speed() -> f64 {
    0.0
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum GripperMode {
    Hande,
    Simulated,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SceneConfig {
    #[serde(default)]
    pub objects: Vec<SceneObject>,
    #[serde(default)]
    pub allow_collisions_with: Vec<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SceneObject {
    pub id: String,
    pub stl: PathBuf,
    pub scale: [f64; 3],
    pub position: [f64; 3],
    pub rpy: [f64; 3],
}

/// Vision look-then-move correction (doc/vision_camera_setup.md).
/// Disabled by default; when enabled every listed PV must connect at
/// startup and a failed or invalid measurement stops the sequence (the
/// resume semantics handle it like any other step failure).
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct VisionConfig {
    pub enabled: bool,
    /// Phase C observation mode: measure and log at every hook, but
    /// never move and never fail the sequence on the verdict.
    pub observe_only: bool,
    /// Seconds to wait for the vision node to answer a request.
    pub timeout: f64,
    /// Corrections below this magnitude (mm) are skipped as noise.
    pub min_correction: f64,
    /// Corrections above this magnitude (mm) stop the sequence — a
    /// mis-detection, a wrong slot, or a moved rack, never auto-applied.
    pub max_correction: f64,
    /// Per-hook enables (all only meaningful when `enabled`).
    pub pick_align: bool,
    pub grip_offset: bool,
    pub place_align: bool,
    pub seating_check: bool,
    pub req_pv: String,
    pub kind_pv: String,
    pub done_pv: String,
    pub valid_pv: String,
    pub dx_pv: String,
    pub dy_pv: String,
    pub dz_pv: String,
    pub quality_pv: String,
    pub seated_pv: String,
    pub tilt_pv: String,
}

impl Default for VisionConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            observe_only: false,
            timeout: 2.0,
            // Above the arm's own re-approach spread at the observation
            // pose, measured over 20 production cycles: sigma (0.022,
            // 0.007) mm, peak-to-peak (0.095, 0.026) mm (doc §14).
            min_correction: 0.10,
            max_correction: 3.0,
            pick_align: true,
            grip_offset: true,
            place_align: true,
            seating_check: true,
            req_pv: "Robot:Vision:Req".into(),
            kind_pv: "Robot:Vision:Kind".into(),
            done_pv: "Robot:Vision:Done".into(),
            valid_pv: "Robot:Vision:Valid".into(),
            dx_pv: "Robot:Vision:DX".into(),
            dy_pv: "Robot:Vision:DY".into(),
            dz_pv: "Robot:Vision:DZ".into(),
            quality_pv: "Robot:Vision:Quality".into(),
            seated_pv: "Robot:Vision:Seated".into(),
            tilt_pv: "Robot:Vision:Tilt".into(),
        }
    }
}

/// One probe direction's step size, allowance and force limits.
///
/// Two of these and not eight prefixed fields on [`ProbeConfig`] because
/// the numbers genuinely differ between the two directions: a bore wall is
/// half a millimetre away and answers a light touch, a seat floor is
/// several millimetres down and is what the puck's own weight already
/// rests on.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct ProbeAxisConfig {
    /// One step, mm. Also the worst-case overshoot past first contact,
    /// since the probe reads force between steps and stops on the first
    /// one that trips.
    pub step_mm: f64,
    /// Total travel allowed in one direction, mm. Running out without
    /// touching anything is an answer, not a failure.
    pub travel_mm: f64,
    /// Force change from the start pose that counts as contact, N.
    pub threshold_n: f64,
    /// Force change that aborts the probe outright, N.
    pub abort_n: f64,
    /// Steps to keep taking after the contact threshold trips, so the wall
    /// can be fitted from a slope instead of read off the trip point,
    /// which carries the threshold and up to one step of overshoot in it.
    ///
    /// Needed on both axes, for the same reason: what the probe reads
    /// below the contact threshold is not the ramp. Laterally it is
    /// nothing at all — 0.17 N to 0.59 N in one 0.05 mm step, 2026-08-18.
    /// Downwards it is a 0.4-0.5 N shoulder that appears half a
    /// millimetre before the jump and is not the floor; fitting it in
    /// halves the slope and moves the floor 0.2 mm.
    ///
    /// An upper bound, not a count: the overtravel also stops once the
    /// load reaches half of `abort_n`, so a wall stiffer than the one this
    /// was set for costs samples rather than an aborted run.
    pub overtravel_steps: usize,
}

/// How a climb between probe heights keeps the puck unloaded on the way.
///
/// Its own block because it is a controller, not a probe: it has no
/// contact threshold and no travel to report, and the numbers it needs
/// are what counts as nothing, how big a nudge is, and how far it may
/// wander before the answer is that something is dragging the puck.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct CentringConfig {
    /// Sideways force the climb calls settled, N.
    ///
    /// The floor is set by the arm, not by taste: 0.05 mm is the
    /// smallest step it executes, and the seat answered a lift with
    /// 30 N/mm, so one step is 1.5 N there and nothing below that is
    /// reachable by stepping. This sits under that on purpose — the
    /// correction stops on its own once two steps in a row fail to
    /// relieve the load, and reports what is left — so that a direction
    /// which *is* soft enough gets centred properly instead of stopping
    /// at a threshold chosen for the stiff one. Free air, gripped and
    /// clear of the holder, is 0.14 N (doc §16.13).
    pub settled_n: f64,
    /// One sideways correction, mm.
    pub step_mm: f64,
    /// Total sideways path one climb may spend, mm.
    ///
    /// Path, not net offset: an oscillation spends this without going
    /// anywhere, which is what stops it running for ever.
    pub travel_mm: f64,
}

impl Default for CentringConfig {
    fn default() -> Self {
        Self {
            settled_n: 1.0,
            // The lateral probe's step, and for the same reason: it is
            // the smallest move this arm reliably executes.
            step_mm: 0.05,
            // Twice the nominal 0.50 mm radial clearance. Past that the
            // puck is not being centred in its bore, it is being pushed
            // somewhere else.
            travel_mm: 1.0,
        }
    }
}

/// Turning the sample in its seat instead of pushing it around in it.
///
/// Off by default (`sweep_deg` 0): it is the answer to a question a
/// bracket raised — a sample pinched at every position may be held
/// crooked rather than held tight — and it turns a gripped sample inside
/// a seat, which is not something to do on the way past.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct TiltConfig {
    /// One tilt step, deg.
    pub step_deg: f64,
    /// How far each way from the pose the level was measured at, deg.
    /// Zero runs no tilt at all.
    pub sweep_deg: f64,
    /// Force change that ends a direction, N.
    pub abort_n: f64,
    /// Torque change that ends it, Nm. A tilt loads a seat in torque
    /// before it loads it in force, so the force limit alone would let
    /// the arm pry.
    pub abort_nm: f64,
}

impl Default for TiltConfig {
    fn default() -> Self {
        Self {
            step_deg: 0.05,
            // Off unless a config asks for it.
            sweep_deg: 0.0,
            abort_n: 5.0,
            abort_nm: 0.5,
        }
    }
}

/// The stage bore's lateral probe (`CalibMode` = Seat Probe).
///
/// Its own block, and not one `lateral:` shared with the holder wells,
/// because the two seats are not the same measurement at either end. The
/// bore holds the puck with 0.50 mm of radial clearance and needs the
/// fingers opened to get the pads out of the way. Reusing these numbers
/// at a well put the walls closer than one step, so contact landed
/// mid-step and the rest of that step was driven into a rigid wall past
/// the abort (h7 and h10, 2026-08-19).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct BoreConfig {
    /// How far the fingers open before the probe steps, mm.
    ///
    /// A gripped, seated puck closes the gripper-puck-bore loop: measured
    /// 2026-08-15, all three directions saturated on the first 0.05 mm step
    /// (1.208 N, 1.549 N, 6.293 N) with no free travel in front of them, so
    /// the arm was reading its own deformation of that loop rather than a
    /// wall. This is the play that reopens it. In millimetres, like every
    /// other number in this block, though the gripper speaks metres.
    ///
    /// Along the jaw axis the free run is half of this before a finger
    /// touches the puck again, and only then does the puck cross its own
    /// clearance to the bore wall. So the play has to be more than twice
    /// the radial clearance for "nothing in the free run" to mean the
    /// fingers were not the thing in the way: at 0.4 mm the free run was
    /// 0.2 mm against a nominal 0.50 mm clearance (§16.2) and could not
    /// decide it.
    pub loosen_mm: f64,
    /// Sideways, toward a bore wall.
    pub lateral: ProbeAxisConfig,
    /// Downward, toward the bore floor.
    pub depth: ProbeAxisConfig,
    /// Keeping the sideways force at nothing while changing height.
    ///
    /// The bore's, and only the bore's: `WellConfig` has no counterpart
    /// on purpose, the same way it has no `loosen_mm`. Yielding sideways
    /// needs somewhere to yield into, and 0.50 mm of radial clearance
    /// has it where 0.05 mm does not.
    ///
    /// A straight climb out of the taught seat pose built 14.30 N in
    /// base y by +0.5 mm while the TCP stayed within 0.018 mm of the
    /// line it was sent along, and the same climb in free air stayed
    /// under 0.15 N (doc §16.13). Brackets measured at the top of the
    /// straight climb measure the arm's deflection under that load; the
    /// point of the correction is that they measure the hole instead.
    pub centring: CentringConfig,
    /// Heights above the pose the probe was triggered at, mm, probed in
    /// the order given and returned from at the end.
    ///
    /// Empty probes once, where it was triggered, and moves the arm
    /// nowhere. A list asks the other question: a gripped puck at the
    /// taught pose has under 0.05 mm of lateral freedom in every
    /// direction and 8.11 N in 0.044 mm (2026-08-18), which is a closed
    /// loop rather than a clearance, and the height at which that loop
    /// opens is what says how deep the seat really engages.
    ///
    /// The moves are probe steps, not jogs: the operator jog gates on a
    /// scene whose stage is a convex decomposition, and a convex hull
    /// cannot represent a bore, so from a seated pose every jog is
    /// refused before it starts.
    pub heights_mm: Vec<f64>,
}

/// A holder well's lateral probe (`CalibMode` = Holder Map).
///
/// There is no `loosen_mm` here on purpose. A well holds its puck by
/// gravity alone, so pads leaving the neck pivot it up and out — at h2
/// the loosened bore probe lost the puck entirely (2026-08-19). "A well
/// is probed clamped" is therefore not a number an operator may set: it
/// is what [`WellConfig::seat_probe`] builds, and there is nowhere in
/// the file to say otherwise.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct WellConfig {
    /// Sideways, toward a well wall.
    pub lateral: ProbeAxisConfig,
    /// Downward, toward the well floor.
    ///
    /// Its own block for the same reason the lateral one is: the taught
    /// pose hovers `holder_on_position_lift` (0.15 mm) above the floor,
    /// and the bore's 0.10 mm step puts the whole descent inside one or
    /// two samples — h7 reported "too few rising samples to fit a floor"
    /// with it, which is the depth axis failing in exactly the way the
    /// lateral one did.
    pub depth: ProbeAxisConfig,
    /// Heights above the pose the probe was triggered at, mm, probed in
    /// the order given and returned from at the end.
    ///
    /// Empty probes once, where it was triggered, and moves the arm
    /// nowhere. A list asks the other question: a gripped puck at the
    /// taught pose has under 0.05 mm of lateral freedom in every
    /// direction and 8.11 N in 0.044 mm (2026-08-18), which is a closed
    /// loop rather than a clearance, and the height at which that loop
    /// opens is what says how deep the seat really engages.
    ///
    /// The moves are probe steps, not jogs: the operator jog gates on a
    /// scene whose stage is a convex decomposition, and a convex hull
    /// cannot represent a bore, so from a seated pose every jog is
    /// refused before it starts.
    pub heights_mm: Vec<f64>,
    /// Smallest measured off-centre worth writing to the trim file, mm.
    ///
    /// Its own number and not `lateral.step_mm` again, which is what the
    /// holder map used to read. Those two being one constant is why no
    /// well could ever be written: a well's whole play is about one bore
    /// step, so every centre it could honestly measure was inside the
    /// deadband by construction (h4, h7, h10). This says how small a
    /// correction is not worth moving the taught pose for; the step says
    /// how finely the walls are approached. They are different questions.
    pub persist_deadband_mm: f64,
}

/// What a probe needs to know about the seat in front of it: how much
/// play to open before stepping, and how finely to step toward a wall
/// and toward the floor.
///
/// Built by the seat's own config rather than assembled at the call
/// site, so that "clamped" stays a property of being a well.
#[derive(Debug, Clone)]
pub struct SeatProbe {
    pub loosen_mm: f64,
    pub lateral: ProbeAxisConfig,
    pub depth: ProbeAxisConfig,
    pub heights_mm: Vec<f64>,
    /// How a climb between heights holds its line, or `None` for
    /// straight up the taught corridor. See [`Motion::climb_centred`].
    pub centring: Option<CentringConfig>,
}

impl BoreConfig {
    /// The bore is probed with the pads out of the way.
    pub fn seat_probe(&self) -> SeatProbe {
        SeatProbe {
            loosen_mm: self.loosen_mm,
            lateral: self.lateral,
            depth: self.depth,
            heights_mm: self.heights_mm.clone(),
            centring: Some(self.centring),
        }
    }
}

impl WellConfig {
    /// A well is probed clamped, always — see [`WellConfig`].
    pub fn seat_probe(&self) -> SeatProbe {
        SeatProbe {
            loosen_mm: 0.0,
            lateral: self.lateral,
            depth: self.depth,
            heights_mm: self.heights_mm.clone(),
            centring: None,
        }
    }
}

/// Force-stopped seat probing (`CalibMode` = Seat Probe). Commissioning
/// only: it measures where a bore actually is, which a taught pose only
/// records an operator's belief about.
///
/// It exists because the criterion for the offsets is that the puck goes
/// in and comes out without rubbing — lateral contact is what shakes the
/// sample — and image agreement is a proxy for that which cannot see a
/// bore whose axis differs from where its rim looks.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct ProbeConfig {
    /// Velocity and acceleration scale for every probe step and every
    /// return. One value rather than one per direction: it says how
    /// gently the arm may move next to a sample, which does not change
    /// between pushing sideways and pushing down.
    pub velocity_scale: f64,
    /// Force allowed while moving between heights, N.
    ///
    /// Separate from the probes' own aborts because it bounds a different
    /// act. A probe's abort says how hard the arm may lean on a bore wall
    /// it is trying to find. This one says how hard it may pull a gripped
    /// puck out of the seat it is sitting in — which is not a fault
    /// condition at all: the sequence's own pick does it on every run, and
    /// borrowing the depth probe's 8.00 N stopped every lift at
    /// +0.31 mm with 7.6-8.2 N sideways (three repeats, doc §16.12) before
    /// any height above that could be measured.
    pub lift_abort_n: f64,
    /// One step of a move between heights, mm.
    ///
    /// Its own number for the same reason `lift_abort_n` is: a climb is
    /// transport, not a measurement. Borrowing the depth probe's step
    /// tied the two together, and once the well's depth step went to
    /// 0.02 mm to resolve a 0.15 mm hover, a 2 mm lift became a hundred
    /// steps of a size the arm can barely execute (`MIN_EXECUTABLE_MM`).
    pub lift_step_mm: f64,
    /// The stage bore, probed loosened (`CalibMode` = Seat Probe).
    pub bore: BoreConfig,
    /// A holder well, probed clamped (`CalibMode` = Holder Map).
    pub well: WellConfig,
    /// Turning the sample in place at every level, instead of pushing it
    /// around.
    pub tilt: TiltConfig,
}

impl Default for BoreConfig {
    fn default() -> Self {
        Self {
            // Twice what it took to drop base y+ from 6.008 N to
            // 0.652 N, because at that value the first step still met
            // something in every direction and the pads had not been
            // ruled out by a margin.
            loosen_mm: 2.5,
            lateral: ProbeAxisConfig {
                // Ten steps to a wall at the nominal 0.50 mm radial
                // clearance, and 0.05 mm of overshoot past it.
                step_mm: 0.05,
                // Past the clearance by enough that "no contact" means the
                // bore is not where the pose says, rather than that the
                // probe was too short. At 1.5 that was not yet true: two
                // of the four lateral directions ran out with nothing in
                // front of them (doc §16.7) and the bracket had no
                // midpoint. This clears the loosened fingers' own free
                // run, half of `loosen_mm`, plus the nominal 0.50 mm
                // radial clearance, with room left over.
                travel_mm: 3.0,
                // Seven times the 0.073 N the arm scatters standing still
                // (doc/vision_correction_plan.md §16.1). Reading between
                // steps is what makes a threshold this low usable at all.
                threshold_n: 0.5,
                // Well under the 8.5-22.9 N the arm was measured pushing
                // through a rubbing insert: that is the force this mode
                // exists to stop the sequence from applying, not a level
                // to probe up to.
                abort_n: 5.0,
                // A wall this stiff trips in one step, so the fit would
                // have a single sample without these. Three of them at
                // 0.05 mm is 0.15 mm further into a bore wall, and the
                // abort limit is checked on every one.
                overtravel_steps: 3,
            },
            depth: ProbeAxisConfig {
                step_mm: 0.10,
                travel_mm: 4.0,
                threshold_n: 1.0,
                abort_n: 8.0,
                // Two, not three: the floor is stiffer than a bore wall
                // (16 N/mm against 4), so the load bound stops this early
                // anyway, and 0.2 mm is what it takes to put a second
                // sample in the hard part of the contact.
                overtravel_steps: 2,
            },
            centring: CentringConfig::default(),
            heights_mm: Vec::new(),
        }
    }
}

impl Default for WellConfig {
    /// An order of magnitude finer than the bore, because that is what
    /// the wells measured: h4 walls at 0.050 mm per side, h7 at 0.052,
    /// h10 at 0.026-0.032 (2026-08-18/19). At the bore's 0.05 mm step
    /// every one of those walls lands inside the first step.
    fn default() -> Self {
        Self {
            lateral: ProbeAxisConfig {
                // The bore's step, because it is the only one this arm is
                // measured to execute. A well's seat is tighter than it,
                // which is what `heights_mm` is for: the bracket is run
                // where the cross-section has room for this step, not
                // shrunk below the floor the arm moves at.
                step_mm: 0.05,
                // The bore's travel, not ten times a seat's play. The
                // bracket is run at the top of `heights_mm`, where the
                // point is to measure with no tension on the seat — and
                // that is exactly where the walls are further out than
                // the seat's own clearance. At h7, 0.5 mm found nothing
                // either way from +2 mm (2026-08-19), which is a scan
                // too narrow rather than an absent wall.
                travel_mm: 3.0,
                // The bore's threshold: it is set by what the arm can
                // tell from its own standing scatter, which does not
                // change with the seat.
                threshold_n: 0.5,
                // Clamped probing has no finger play to absorb the seat's
                // own cross-axis tension, and that tension is what tripped
                // the bore's 5.00 N: h7 base x+ read 8.12 N total while
                // the along-axis force was 1.17 N. Below `lift_abort_n`,
                // and below the 23 N of a rubbing insert.
                abort_n: 12.0,
                // Five, not the bore's three: a well wall is rigid and
                // the run stops at half of `abort_n` anyway, so asking
                // for more steps buys rising samples for the fit on a
                // wall that gives, and costs nothing on one that does
                // not. Three left every clean well bracket reporting
                // "at the trip point, no slope to fit".
                overtravel_steps: 5,
            },
            depth: ProbeAxisConfig {
                // The smallest step this arm executes. Three of them
                // reach the 0.15 mm the taught pose hovers by, which is
                // as much resolution as the hover has room for: 0.02 mm
                // was tried to buy more and the arm simply did not take
                // it (commanded 0.020, moved -0.002 at h7, 2026-08-19).
                step_mm: 0.05,
                // The descent starts at the top of `heights_mm`, so it
                // has to cover the lift plus the hover with margin.
                // "No floor within" then says the seat is not where the
                // pose believes rather than that the probe stopped short.
                travel_mm: 3.0,
                // The floor answers at about 16 N/mm, so one step is
                // 0.32 N and contact is caught within two of them. The
                // bore's 1.0 N would be three steps deep by the time it
                // tripped, which is most of the hover.
                threshold_n: 0.5,
                // A push onto a floor, which is bounded by what the arm
                // may do to a sample and not by which seat it is in.
                abort_n: 8.0,
                // Four rising samples for the fit, against the bore's
                // two, because the descent that reaches this floor is
                // three steps long and has none of its own to spare.
                // The half-abort bound stops it sooner on a stiff floor.
                overtravel_steps: 4,
            },
            // Up off the seat, where the brackets can measure with no
            // tension on it, then back down so the floor probe starts
            // from the centre they found -- but not all the way down.
            // The taught seat hovers 0.075 mm over its floor (h7), which
            // is one step, and a fit with one pre-contact sample has no
            // baseline. From +0.3 the descent is 0.375 mm and still
            // inside the well mouth (0.8 mm up).
            heights_mm: vec![2.0, 0.3],
            // Not one lateral step, which is what the map used to read:
            // a bracket's walls are fitted from the force slope over
            // MEASURED travel, and the arm undershoots its commands, so
            // a centre is resolved to well inside the step that found
            // it. Tying the two together made a well's whole play about
            // one step and left no writable window at all.
            persist_deadband_mm: 0.01,
        }
    }
}

impl Default for ProbeConfig {
    fn default() -> Self {
        Self {
            velocity_scale: 0.02,
            bore: BoreConfig::default(),
            well: WellConfig::default(),
            // Above the drag a lift actually carries, and below the
            // 23 N a rubbing insert was measured at (doc §16.2) — the
            // level this whole mode exists to keep the sequence away
            // from. 15.0 was not above the drag: pulling h7's puck up
            // its well plateaus at 14.5 N (14.49, 14.50, 14.55 over
            // 0.2 mm — flat, so sliding friction and not a wedge), and
            // the bore's own straight climb reaches 14.30 N, so which
            // side of the limit a run lands on was noise. Two identical
            // h7 climbs split on it: one carried 14.65 N to +1.94 mm,
            // the next tripped at 15.02 N by +1.04 mm (2026-08-19).
            lift_abort_n: 20.0,
            // What the bore's depth probe used, which is proven to
            // execute on this rig: 2 mm is twenty steps.
            lift_step_mm: 0.10,
            tilt: TiltConfig::default(),
        }
    }
}

impl Default for ProbeAxisConfig {
    /// Only reachable through a partial `lateral:`/`depth:` block, where
    /// serde fills the unnamed fields from here. A complete `bore:` or
    /// `well:` block never reaches it: those two have their own defaults,
    /// so neither seat can inherit the other's numbers.
    fn default() -> Self {
        ProbeConfig::default().bore.lateral
    }
}

/// Eye-in-hand calibration capture (`CalibMode` = Hand-Eye). Commissioning
/// only: it produces the `T_ee_cam` that [`VisionConfig`]'s node needs
/// before it can convert pixels to a TCP-local correction at all.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct HandEyeConfig {
    /// Interpreter for `detector`; needs cv2, numpy and pyepics.
    pub python: PathBuf,
    /// Interpreter for the solver; needs cv2, numpy, scipy and yaml. A
    /// second field and not `python` again because the two roles do not
    /// fit in one environment here: the detector's needs pyepics and has
    /// no yaml, so the command the capture prints on completion used to
    /// name an interpreter that could not run it.
    pub solve_python: PathBuf,
    /// The AprilTag detector driven over stdin/stdout.
    pub detector: PathBuf,
    /// Directory each capture's `samples_<timestamp>.yaml` is written to,
    /// alongside the one `aim_pose.yaml` they share.
    pub out_dir: PathBuf,
    /// Largest tool rotation in the schedule (deg). The tag must stay in
    /// frame at the extremes, so this is bounded by how near the image
    /// centre the tag sits at the home pose, not by the arm.
    pub angle_deg: f64,
    /// Tool-z offsets (mm) the rotation set is repeated at, measured from
    /// the aiming pose; positive is toward whatever the camera is looking
    /// at. A single standoff leaves solvePnP's depth error identical in
    /// every sample, where it is indistinguishable from a camera mounted
    /// that much further out and lands in `T_ee_cam`'s translation
    /// unchallenged. Every entry costs a full rotation set of poses.
    pub standoff_mm: Vec<f64>,
    /// Velocity and acceleration scale for the capture moves — slower
    /// than the sequence, because every pose is a fresh excursion into a
    /// cramped cell rather than a taught path.
    pub velocity_scale: f64,
    /// Below this many detected poses the capture is reported as failed
    /// rather than written: `calibrateHandEye` returns an answer for
    /// under-determined input as readily as for good input.
    pub min_samples: usize,
}

impl Default for HandEyeConfig {
    fn default() -> Self {
        Self {
            python: PathBuf::from("python3"),
            solve_python: PathBuf::from("python3"),
            detector: PathBuf::from("tools/handeye/detector.py"),
            out_dir: PathBuf::from("handeye_samples"),
            angle_deg: 8.0,
            standoff_mm: vec![0.0],
            velocity_scale: 0.1,
            min_samples: 5,
        }
    }
}

impl Config {
    /// Loads the YAML and resolves every relative path against the config
    /// file's directory, so the daemon can be started from anywhere.
    pub fn load(path: &Path) -> Result<Self, SequencerError> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| SequencerError(format!("cannot read config {}: {e}", path.display())))?;
        let mut config: Config = serde_yaml::from_str(&text)
            .map_err(|e| SequencerError(format!("cannot parse config {}: {e}", path.display())))?;
        let base = path.parent().unwrap_or(Path::new("."));
        let anchor = |p: &mut PathBuf| {
            if p.is_relative() {
                *p = base.join(&*p);
            }
        };
        anchor(&mut config.robot.urdf);
        anchor(&mut config.robot.srdf);
        anchor(&mut config.robot.script_file);
        anchor(&mut config.robot.output_recipe);
        anchor(&mut config.robot.input_recipe);
        for dir in config.robot.mesh_packages.values_mut() {
            anchor(dir);
        }
        anchor(&mut config.sequence.waypoints_yaml);
        for object in &mut config.scene.objects {
            anchor(&mut object.stl);
        }
        anchor(&mut config.handeye.detector);
        anchor(&mut config.handeye.out_dir);
        // Checked always, not only when the mode is selected: there is no
        // enable flag to gate on, and an out-of-range angle should be a
        // startup error rather than a surprise after the arm has moved.
        let h = &config.handeye;
        if !(0.0..=30.0).contains(&h.angle_deg) {
            return Err(SequencerError(
                "handeye.angle_deg must be within 0..30".into(),
            ));
        }
        if !(0.0..=0.5).contains(&h.velocity_scale) {
            return Err(SequencerError(
                "handeye.velocity_scale must be within 0..0.5".into(),
            ));
        }
        if h.min_samples < 3 {
            return Err(SequencerError(
                "handeye.min_samples must be at least 3 (calibrateHandEye needs three \
                 relative motions)"
                    .into(),
            ));
        }
        // Same reasoning as the hand-eye block above, with more at stake:
        // these numbers are the only thing bounding how hard the arm is
        // allowed to push on a sample, and a probe that reads them for the
        // first time has already been triggered.
        for (name, axis) in [
            ("probe.bore.lateral", &config.probe.bore.lateral),
            ("probe.bore.depth", &config.probe.bore.depth),
            ("probe.well.lateral", &config.probe.well.lateral),
            ("probe.well.depth", &config.probe.well.depth),
        ] {
            // Positive is not enough: below the floor the arm executes,
            // a step is a command it does not carry out, and a probe
            // built from those reports clearance that was never
            // travelled. h7's well bracket commanded 0.010 mm, moved
            // -0.004 mm against 0.74 N, and only the step-taken guard
            // caught it (doc §16.4).
            if axis.step_mm < MIN_EXECUTABLE_MM {
                return Err(SequencerError(format!(
                    "{name}.step_mm must be at least {MIN_EXECUTABLE_MM} mm, the smallest \
                     move this arm executes"
                )));
            }
            if axis.travel_mm < axis.step_mm {
                return Err(SequencerError(format!(
                    "{name}.travel_mm must be at least one {name}.step_mm"
                )));
            }
            if axis.threshold_n <= 0.0 {
                return Err(SequencerError(format!(
                    "{name}.threshold_n must be positive"
                )));
            }
            // An abort at or below the contact threshold aborts every
            // probe the moment it succeeds, which reads as "the probe
            // cannot touch anything" rather than as a misconfiguration.
            if axis.abort_n <= axis.threshold_n {
                return Err(SequencerError(format!(
                    "{name}.abort_n must be above {name}.threshold_n"
                )));
            }
            // Each one is another step driven into something already known
            // to be there, so this is a push limit like the rest of the
            // block rather than a fit-quality knob.
            if axis.overtravel_steps > 10 {
                return Err(SequencerError(format!(
                    "{name}.overtravel_steps must be at most 10 (each one pushes further \
                     into a sample that is already in contact)"
                )));
            }
        }
        if !(0.0..=0.1).contains(&config.probe.velocity_scale) {
            return Err(SequencerError(
                "probe.velocity_scale must be within 0..0.1 (a probe steps into contact)".into(),
            ));
        }
        // Bounded above because the fingers are holding a sample over an
        // open rack while this runs: a decimal-point slip here is the
        // difference between play and a dropped puck.
        // Arm motion inside a rack with a sample in the fingers, so the
        // list is bounded like every other number in this block. Below the
        // trigger pose is allowed but barely: down is where the seat is.
        for (name, heights, depth) in [
            (
                "probe.bore.heights_mm",
                &config.probe.bore.heights_mm,
                &config.probe.bore.depth,
            ),
            (
                "probe.well.heights_mm",
                &config.probe.well.heights_mm,
                &config.probe.well.depth,
            ),
        ] {
            if heights.len() > 8 {
                return Err(SequencerError(format!(
                    "{name} must have at most 8 entries (each is a full bracket set)"
                )));
            }
            if heights.iter().any(|h| !(-2.0..=10.0).contains(h)) {
                return Err(SequencerError(format!(
                    "{name} entries must be within -2..10 mm of the trigger pose"
                )));
            }
            // The floor is probed at the last level, and a floor is fitted
            // from a force slope against a baseline taken before contact.
            // A level parked on the seat leaves no room for that baseline:
            // at h7 the taught pose hovers 0.075 mm, the descent tripped on
            // its second sample, and the whole map was thrown away at the
            // end for "the floor fit did not measure" (2026-08-19). A
            // ladder that names its floor level has to lift it clear.
            let baseline_mm = MIN_BASELINE_SAMPLES as f64 * depth.step_mm;
            if heights.last().is_some_and(|h| *h < baseline_mm) {
                return Err(SequencerError(format!(
                    "{name} must end at least {baseline_mm} mm above the trigger pose                      ({MIN_BASELINE_SAMPLES} depth steps), so the floor probe has                      somewhere to take its baseline before it meets the floor"
                )));
            }
        }
        // Below the floor the arm can execute, a climb is a list of
        // commands that do not move it; above one step of the shortest
        // useful lift it is not a climb, it is a jump.
        if !(MIN_EXECUTABLE_MM..=1.0).contains(&config.probe.lift_step_mm) {
            return Err(SequencerError(format!(
                "probe.lift_step_mm must be within {MIN_EXECUTABLE_MM}..1 mm (below that \
                 the arm does not execute the step at all)"
            )));
        }
        if !(1.0..=25.0).contains(&config.probe.lift_abort_n) {
            return Err(SequencerError(
                "probe.lift_abort_n must be within 1..25 N (a lift the arm cannot make \
                 is not a measurement, and 23 N is a rubbing insert)"
                    .into(),
            ));
        }
        // Both ends of each range are real commands on a Hand-E: 0.0 is
        // its 20 N / 20 mm/s minimum, not a gripper that does nothing.
        // Out of range is what has no meaning.
        if !(0.0..=1.0).contains(&config.gripper.grip_force) {
            return Err(SequencerError(
                "gripper.grip_force must be within 0..1 (0 is the Hand-E's 20 N minimum)".into(),
            ));
        }
        if !(0.0..=1.0).contains(&config.gripper.grip_speed) {
            return Err(SequencerError(
                "gripper.grip_speed must be within 0..1 (0 is the Hand-E's 20 mm/s minimum)".into(),
            ));
        }
        let tilt = &config.probe.tilt;
        // A quarter of a degree already swings the far edge of a 20 mm
        // puck by 0.04 mm, which is the whole of the freedom one was
        // measured to have in its seat. Past a degree this is not a tilt
        // scan, it is levering.
        if !(0.0..=1.0).contains(&tilt.sweep_deg) {
            return Err(SequencerError(
                "probe.tilt.sweep_deg must be within 0..1 deg (0 runs no tilt)".into(),
            ));
        }
        if !(0.005..=0.2).contains(&tilt.step_deg) {
            return Err(SequencerError(
                "probe.tilt.step_deg must be within 0.005..0.2 deg".into(),
            ));
        }
        if tilt.sweep_deg > 0.0 && tilt.sweep_deg < tilt.step_deg {
            return Err(SequencerError(
                "probe.tilt.sweep_deg must be at least one probe.tilt.step_deg".into(),
            ));
        }
        if !(1.0..=15.0).contains(&tilt.abort_n) {
            return Err(SequencerError(
                "probe.tilt.abort_n must be within 1..15 N".into(),
            ));
        }
        if !(0.05..=2.0).contains(&tilt.abort_nm) {
            return Err(SequencerError(
                "probe.tilt.abort_nm must be within 0.05..2 Nm".into(),
            ));
        }
        let centring = &config.probe.bore.centring;
        // Below the arm's own scatter standing still (0.073 N, §16.1)
        // there is no load to null; above a lateral probe's contact
        // threshold the climb would carry a load the brackets then call
        // a wall.
        if !(0.2..=5.0).contains(&centring.settled_n) {
            return Err(SequencerError(
                "probe.bore.centring.settled_n must be within 0.2..5 N (below that is \
                 the arm's own noise, above it is what a probe calls a wall)"
                    .into(),
            ));
        }
        // A correction below the smallest step the arm executes does not
        // happen at all, and one above the radial clearance moves the
        // puck across the bore in a single nudge.
        if !(MIN_EXECUTABLE_MM..=0.5).contains(&centring.step_mm) {
            return Err(SequencerError(format!(
                "probe.bore.centring.step_mm must be within {MIN_EXECUTABLE_MM}..0.5 mm"
            )));
        }
        // Zero would fail the first correction it needed rather than
        // disable correcting, so the range starts at one step's worth.
        if !(centring.step_mm..=3.0).contains(&centring.travel_mm) {
            return Err(SequencerError(
                "probe.bore.centring.travel_mm must be within one step and 3 mm (a climb \
                 that has moved further sideways than that is not centring)"
                    .into(),
            ));
        }
        if !(0.0..=5.0).contains(&config.probe.bore.loosen_mm) {
            return Err(SequencerError(
                "probe.bore.loosen_mm must be within 0..5 (the fingers must find the sample again)"
                    .into(),
            ));
        }
        // Zero would write the measurement's own grain into a taught pose
        // on every map; anything past the persist cap can never fire,
        // because a centre that large is refused as not-a-trim first.
        if !(0.0..=1.0).contains(&config.probe.well.persist_deadband_mm)
            || config.probe.well.persist_deadband_mm == 0.0
        {
            return Err(SequencerError(
                "probe.well.persist_deadband_mm must be within 0..1 mm, exclusive of zero \
                 (a centre past 1 mm is refused as not a trim error at all)"
                    .into(),
            ));
        }
        if config.vision.enabled {
            let v = &config.vision;
            // Below ~0.002 mm the IK offset helper's epsilon treats the
            // correction as zero and the strict fallback check misfires.
            if v.min_correction < 0.002 {
                return Err(SequencerError(
                    "vision.min_correction must be >= 0.002 mm".into(),
                ));
            }
            if v.max_correction < v.min_correction {
                return Err(SequencerError(
                    "vision.max_correction must be >= vision.min_correction".into(),
                ));
            }
            if v.timeout <= 0.0 {
                return Err(SequencerError("vision.timeout must be positive".into()));
            }
        }
        Ok(config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loads_the_production_config() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer.yaml"
        ));
        let config = Config::load(path).expect("load");
        assert_eq!(config.robot.group, "ur_manipulator");
        assert_eq!(config.robot.ik_frame, "robotiq_hande_end");
        assert_eq!(config.gripper.mode, GripperMode::Hande);
        assert_eq!(config.joint_limits.position_overrides.len(), 3);
        // The stage ships as an approximate convex decomposition (see
        // the scene comment in sequencer.yaml), so this is a part count,
        // not a single mesh — but an empty scene would mean the daemon
        // plans with no stage at all, which must not pass silently.
        assert!(!config.scene.objects.is_empty());
        // Every anchored path must exist in the checkout.
        for path in [
            &config.robot.urdf,
            &config.robot.srdf,
            &config.robot.script_file,
            &config.robot.output_recipe,
            &config.robot.input_recipe,
            &config.sequence.waypoints_yaml,
        ] {
            assert!(path.exists(), "missing: {}", path.display());
        }
        for object in &config.scene.objects {
            assert!(
                object.stl.exists(),
                "missing scene mesh: {}",
                object.stl.display()
            );
        }
        for dir in config.robot.mesh_packages.values() {
            assert!(dir.is_dir(), "missing mesh package dir: {}", dir.display());
        }
        // Production carries an explicit vision section, disabled.
        assert!(!config.vision.enabled);
        assert!(!config.vision.observe_only);
        // The hand-eye detector is spawned by path, so a typo there
        // surfaces only when an operator selects the mode on the robot.
        assert!(
            config.handeye.detector.exists(),
            "missing hand-eye detector: {}",
            config.handeye.detector.display()
        );
        assert!(
            config.handeye.python.is_absolute(),
            "handeye.python must name an interpreter with cv2/pyepics, not inherit PATH"
        );
        assert!(
            config.handeye.solve_python.is_absolute(),
            "handeye.solve_python must name an interpreter with cv2/scipy/yaml, not inherit PATH"
        );
    }

    /// The URSim config has no vision section: the serde default path
    /// must yield a disabled feature with the documented thresholds.
    #[test]
    fn ursim_config_defaults_vision_off() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer_ursim.yaml"
        ));
        let config = Config::load(path).expect("load");
        let v = &config.vision;
        assert!(!v.enabled);
        assert_eq!(v.min_correction, 0.10);
        assert_eq!(v.max_correction, 3.0);
        assert_eq!(v.req_pv, "Robot:Vision:Req");
        // Same for hand-eye: absent from the URSim config, and its
        // defaults must be the ones the daemon banner advertises.
        assert_eq!(config.handeye.angle_deg, 8.0);
        assert_eq!(config.handeye.min_samples, 5);
    }

    /// Same reasoning as the hand-eye ranges, on the numbers that bound
    /// how hard the arm may push on a sample. `abort_n` at or below
    /// `threshold_n` is the one that would not look like a
    /// misconfiguration at runtime: every probe would abort at the
    /// instant it succeeded, and read as a probe that cannot touch
    /// anything.
    #[test]
    fn rejects_probe_limits_that_would_abort_on_contact() {
        let base = std::fs::read_to_string(Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer_ursim.yaml"
        )))
        .expect("read");
        let dir = std::env::temp_dir().join("probe_config_bounds");
        std::fs::create_dir_all(&dir).expect("tmpdir");
        for (name, section, want) in [
            (
                "abort_at_threshold",
                "probe:\n  bore:\n    lateral:\n      threshold_n: 0.5\n      abort_n: 0.5\n",
                "probe.bore.lateral.abort_n",
            ),
            (
                "abort_under_threshold",
                "probe:\n  bore:\n    depth:\n      threshold_n: 2.0\n      abort_n: 1.0\n",
                "probe.bore.depth.abort_n",
            ),
            (
                "step_zero",
                "probe:\n  well:\n    lateral:\n      step_mm: 0.0\n",
                "probe.well.lateral.step_mm",
            ),
            // Positive but below the floor the arm executes: h7's well
            // bracket commanded exactly this and travelled -0.004 mm.
            (
                "step_under_the_executable_floor",
                "probe:\n  well:\n    lateral:\n      step_mm: 0.01\n",
                "probe.well.lateral.step_mm",
            ),
            // The ladder's last level is where the floor is probed, and
            // this one is parked on the seat: exactly the run14 h7 shape,
            // where the descent tripped on its second sample and the fit
            // had no baseline to work from.
            (
                "floor_level_on_the_seat",
                "probe:\n  well:\n    heights_mm: [2.0, 0.0]\n",
                "probe.well.heights_mm",
            ),
            (
                "travel_under_one_step",
                "probe:\n  well:\n    depth:\n      step_mm: 0.5\n      travel_mm: 0.2\n",
                "probe.well.depth.travel_mm",
            ),
            (
                "velocity_hi",
                "probe:\n  velocity_scale: 0.5\n",
                "probe.velocity_scale",
            ),
            // Each of these is a step driven into something already known
            // to be in contact, so the count is a push limit like the rest.
            (
                "overtravel_deep",
                "probe:\n  bore:\n    lateral:\n      overtravel_steps: 11\n",
                "probe.bore.lateral.overtravel_steps",
            ),
            // The move between heights carries a gripped sample out of a
            // seat, so its ceiling is bounded like every other force here.
            (
                "lift_abort_high",
                "probe:\n  lift_abort_n: 30.0\n",
                "probe.lift_abort_n",
            ),
            (
                "tilt_sweep_too_wide",
                "probe:\n  tilt:\n    sweep_deg: 2.0\n",
                "probe.tilt.sweep_deg",
            ),
            (
                "tilt_sweep_under_one_step",
                "probe:\n  tilt:\n    step_deg: 0.1\n    sweep_deg: 0.05\n",
                "probe.tilt.sweep_deg",
            ),
            (
                "tilt_torque_limit_high",
                "probe:\n  tilt:\n    abort_nm: 3.0\n",
                "probe.tilt.abort_nm",
            ),
            // The climb's correction has the same shape of risk as the
            // probes' own numbers: it moves a gripped sample sideways
            // inside a bore on a force reading.
            (
                "centring_settled_under_noise",
                "probe:\n  bore:\n    centring:\n      settled_n: 0.05\n",
                "probe.bore.centring.settled_n",
            ),
            (
                "centring_step_too_small",
                "probe:\n  bore:\n    centring:\n      step_mm: 0.005\n",
                "probe.bore.centring.step_mm",
            ),
            (
                "centring_travel_under_one_step",
                "probe:\n  bore:\n    centring:\n      step_mm: 0.1\n      travel_mm: 0.05\n",
                "probe.bore.centring.travel_mm",
            ),
            // A slipped decimal point here opens the fingers 12 mm over an
            // open rack with a sample in them.
            (
                "loosen_wide",
                "probe:\n  bore:\n    loosen_mm: 12.0\n",
                "probe.bore.loosen_mm",
            ),
            // The deadband is what decides whether a measured centre is
            // written at all, so zero is a misconfiguration and not "no
            // deadband".
            (
                "deadband_zero",
                "probe:\n  well:\n    persist_deadband_mm: 0.0\n",
                "probe.well.persist_deadband_mm",
            ),
        ] {
            let path = dir.join(format!("{name}.yaml"));
            std::fs::write(&path, format!("{base}\n{section}")).expect("write");
            let err = Config::load(&path).expect_err(name).to_string();
            assert!(err.contains(want), "{name}: unexpected error {err}");
        }
    }

    /// The range checks run at load, not at mode selection: an operator
    /// finding out mid-capture that the schedule was nonsense has already
    /// moved the arm.
    #[test]
    fn rejects_out_of_range_handeye_limits() {
        let base = std::fs::read_to_string(Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer_ursim.yaml"
        )))
        .expect("read");
        let dir = std::env::temp_dir().join("handeye_config_bounds");
        std::fs::create_dir_all(&dir).expect("tmpdir");
        for (name, section, want) in [
            ("angle_hi", "handeye:\n  angle_deg: 45.0\n", "angle_deg"),
            ("angle_neg", "handeye:\n  angle_deg: -1.0\n", "angle_deg"),
            (
                "velocity_hi",
                "handeye:\n  velocity_scale: 0.9\n",
                "velocity_scale",
            ),
            ("samples_lo", "handeye:\n  min_samples: 2\n", "min_samples"),
        ] {
            let path = dir.join(format!("{name}.yaml"));
            std::fs::write(&path, format!("{base}\n{section}")).expect("write");
            let err = Config::load(&path).expect_err(name).to_string();
            assert!(err.contains(want), "{name}: unexpected error {err}");
        }
    }

    /// The boundary values themselves are legal — 30 deg and 0.5 are the
    /// documented maxima, not the first rejected value.
    #[test]
    fn accepts_the_handeye_limit_boundaries() {
        let base = std::fs::read_to_string(Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../config/sequencer_ursim.yaml"
        )))
        .expect("read");
        let dir = std::env::temp_dir().join("handeye_config_bounds");
        std::fs::create_dir_all(&dir).expect("tmpdir");
        let path = dir.join("edges.yaml");
        std::fs::write(
            &path,
            format!(
                "{base}\nhandeye:\n  angle_deg: 30.0\n  velocity_scale: 0.5\n  min_samples: 3\n"
            ),
        )
        .expect("write");
        let config = Config::load(&path).expect("boundary values are legal");
        assert_eq!(config.handeye.angle_deg, 30.0);
        assert_eq!(config.handeye.min_samples, 3);
    }
}
