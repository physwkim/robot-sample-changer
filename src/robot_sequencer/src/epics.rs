//! Channel Access facade over epics-ca-rs.
//!
//! The sequence thread is synchronous (like the rest of the daemon);
//! this facade owns a private tokio runtime and exposes blocking calls
//! that mirror the C++ node's libca usage: polled `ca_get`s with 1 s
//! completion timeouts, no subscriptions. Error semantics are preserved
//! per PV — each read degrades to the same default the C++ helpers
//! returned on a failed `ca_get`/`ca_pend_io`, so IOC hiccups do not
//! change sequence behavior between the two implementations.

use std::time::Duration;

use epics_ca_rs::EpicsValue;
use epics_ca_rs::client::{CaChannel, CaClient};
use tokio::runtime::Runtime;

use crate::config::{EpicsConfig, SeatCheckConfig, VisionConfig};
use crate::error::SequencerError;
use crate::log;
use crate::seatcheck::{DepthCamera, DepthFrame, Lens};

const GET_TIMEOUT: Duration = Duration::from_secs(1);
const JOG_TIMEOUT: Duration = Duration::from_millis(500);
const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);
const VISION_POLL: Duration = Duration::from_millis(50);
const FRAME_POLL: Duration = Duration::from_millis(10);

/// `Robot:Wait` states (mbbo: 0=Wait, 1=Continue, 2=Abort).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaitStatus {
    Waiting,
    Continue,
    Skip,
}

/// `Robot:CalibMode` states (mbbo: 0=Normal, 1=Holder, 2=Sample Holder,
/// 3=Hand-Eye, 4=Recover, 5=Seat Probe).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibMode {
    Normal,
    Holder,
    SampleHolder,
    /// Eye-in-hand camera calibration capture. Unlike the other two it
    /// touches no holder and no gripper — it rotates the tool in place to
    /// give `cv2.calibrateHandEye` its rotational diversity.
    HandEye,
    /// Put the arm back at the holder standby after a run stopped part
    /// way. Not a calibration; it shares the PV because "pick a mode,
    /// then trigger" is the entry every non-Normal behavior already
    /// uses, and a recovery the operator has to remember a second
    /// mechanism for is one they will not reach for.
    Recover,
    /// Feel for the walls and floor of the seat the arm is standing in.
    /// Like Hand-Eye it measures and writes nothing back into the
    /// sequence; unlike every other mode it moves until something pushes
    /// back rather than to a pose.
    SeatProbe,
    /// One trigger drives this holder's taught seat pose to where the
    /// fingers close on its puck without loading it: pick, read the
    /// wrench the close left behind, put the puck back, write the trim
    /// the wrench asks for, repeat until the load is at the noise floor.
    ///
    /// It took the slot the seat-probe-based holder map had, because the
    /// map measured the wrong thing. The probe pushes the *arm* against
    /// well walls the puck is already touching, so its brackets land
    /// inside its own persist deadband; the close pushes the *pads*
    /// against the puck, where the contact is asymmetric by exactly the
    /// pose error.
    GripNull,
    /// Carry one puck from `MapSource` to `Holder` and leave it there.
    ///
    /// The pick and place the sequence already does, with the stage leg
    /// cut out and nothing measured: the arm retreats from the source
    /// and goes straight to the target seat.
    HolderTransfer,
}

impl CalibMode {
    /// Whether this mode drives the fingers into a seat.
    ///
    /// The two that do not are the two an operator reaches for when a
    /// run has already gone wrong, and gating them on the gripper would
    /// take away the way out: [`CalibMode::Recover`] returns to standby
    /// without entering anything and deliberately leaves the fingers as
    /// it found them, and [`CalibMode::HandEye`] rotates the camera in
    /// place. [`CalibMode::SeatProbe`] probes the seat the arm is
    /// already standing in rather than driving into one.
    pub fn enters_a_seat(self) -> bool {
        match self {
            Self::Normal
            | Self::Holder
            | Self::SampleHolder
            | Self::GripNull
            | Self::HolderTransfer => true,
            Self::Recover | Self::HandEye | Self::SeatProbe => false,
        }
    }
}

/// `Robot:Vision:Kind` request codes (mbbo).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VisionKind {
    /// Empty gripper above a puck: correction to center the grip.
    PickAlign = 1,
    /// Puck in gripper at the above pose: extra correction the grip
    /// deviation adds to the next place.
    GripOffset = 2,
    /// Puck in gripper above a slot: correction to center the slot.
    PlaceAlign = 3,
    /// Puck released, gripper back at above: seated / tilt verdict.
    Seating = 4,
}

/// One answered vision request. `dx/dy/dz` are the TCP-local move to
/// apply, in mm (the vision node owns the pixel→mm conversion and the
/// sign convention).
#[derive(Debug, Clone, Copy)]
pub struct VisionResult {
    pub valid: bool,
    pub dx: f64,
    pub dy: f64,
    pub dz: f64,
    pub quality: f64,
    pub seated: bool,
    pub tilt: f64,
}

struct VisionChannels {
    req: CaChannel,
    kind: CaChannel,
    done: CaChannel,
    valid: CaChannel,
    dx: CaChannel,
    dy: CaChannel,
    dz: CaChannel,
    quality: CaChannel,
    seated: CaChannel,
    tilt: CaChannel,
}

pub struct Epics {
    rt: Runtime,
    _client: CaClient,
    trigger: CaChannel,
    start_step: CaChannel,
    wait: CaChannel,
    holder: CaChannel,
    stop: CaChannel,
    current_step: CaChannel,
    gripper_cmd: CaChannel,
    gripper_rbv: CaChannel,
    pause_step: CaChannel,
    calib_mode: CaChannel,
    loaded: CaChannel,
    map_source: Option<CaChannel>,
    jog_x: Option<CaChannel>,
    jog_y: Option<CaChannel>,
    jog_z: Option<CaChannel>,
    jog_step: Option<CaChannel>,
    vision: Option<VisionChannels>,
    null: Option<NullChannels>,
    jog_total: Option<JogChannels>,
    depth: Option<DepthChannels>,
}

/// The D405 depth stream: the frame, the counter that says whether it
/// is this exposure or the last one, and the geometry it was taken
/// under.
///
/// The geometry is read once at connect and kept, not re-read per
/// frame: it changes with the stream profile, and a profile change is a
/// restart of the camera IOC, not something that happens between two
/// steps of a sequence.
struct DepthChannels {
    data: CaChannel,
    counter: CaChannel,
    camera: DepthCamera,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every mode is classified, and the three that are not gated are
    /// the three an operator reaches for when a run has already gone
    /// wrong or when nothing is being carried. A mode added without
    /// deciding fails to compile in `enters_a_seat`; a mode reclassified
    /// by accident fails here.
    #[test]
    fn the_modes_that_drive_into_a_seat_are_the_ones_the_gripper_gate_covers() {
        for mode in [
            CalibMode::Normal,
            CalibMode::Holder,
            CalibMode::SampleHolder,
            CalibMode::GripNull,
            CalibMode::HolderTransfer,
        ] {
            assert!(mode.enters_a_seat(), "{mode:?} must be gated");
        }
        for mode in [CalibMode::Recover, CalibMode::HandEye, CalibMode::SeatProbe] {
            assert!(!mode.enters_a_seat(), "{mode:?} must stay ungated");
        }
    }
}

/// The grip null's progress records. All seven or none: a half-present
/// family would show the operator a state without the numbers that
/// explain it, which is worse than showing nothing.
struct NullChannels {
    state: CaChannel,
    iteration: CaChannel,
    dx: CaChannel,
    dy: CaChannel,
    dz: CaChannel,
    force: CaChannel,
    message: CaChannel,
}

/// What `Robot:Null:State` means. The labels live in the GUI, next to
/// `step_name`, so the record stays a plain longin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NullState {
    Idle,
    Running,
    Settled,
    Failed,
}

impl NullState {
    fn code(self) -> i32 {
        match self {
            Self::Idle => 0,
            Self::Running => 1,
            Self::Settled => 2,
            Self::Failed => 3,
        }
    }
}

/// One snapshot of the grip null, published as a whole.
///
/// Publishing a struct rather than seven setters is deliberate: the
/// state and the numbers that justify it are written together or not at
/// all, so no path can advance the state and leave a stale correction
/// on the screen.
#[derive(Debug, Clone)]
pub struct NullReport {
    pub state: NullState,
    pub iteration: i32,
    /// Cumulative move so far, mm, in tool x, tool y (depth) and
    /// tool z. The depth entry stays zero: nothing steers it (see
    /// `NULL_STEERED`), and it is kept in the triple because the trim
    /// slots it lines up with are still three.
    pub total_mm: [f64; 3],
    /// Magnitude of the last close wrench's force, N.
    pub force_n: f64,
    /// One line for the operator. `stringin` holds 39 characters, and
    /// [`Epics::publish_null`] truncates on a character boundary.
    pub message: String,
}

/// The jog accumulator's records. All five or none, like the null
/// family: the three totals are what an operator reads before pressing
/// Apply, and `Target` is what tells them the press will land
/// somewhere.
struct JogChannels {
    d: [CaChannel; 3],
    target: CaChannel,
    apply: CaChannel,
}

/// Connect the whole `Robot:Jog:` family, or none of it.
fn jog_channels(optional: &dyn Fn(&str) -> Option<CaChannel>, prefix: &str) -> Option<JogChannels> {
    Some(JogChannels {
        d: [
            optional(&format!("{prefix}DX"))?,
            optional(&format!("{prefix}DY"))?,
            optional(&format!("{prefix}DZ"))?,
        ],
        target: optional(&format!("{prefix}Target"))?,
        apply: optional(&format!("{prefix}Apply"))?,
    })
}

/// Connect the whole `Robot:Null:` family, or none of it.
fn null_channels(
    optional: &dyn Fn(&str) -> Option<CaChannel>,
    prefix: &str,
) -> Option<NullChannels> {
    Some(NullChannels {
        state: optional(&format!("{prefix}State"))?,
        iteration: optional(&format!("{prefix}Iter"))?,
        dx: optional(&format!("{prefix}DX"))?,
        dy: optional(&format!("{prefix}DY"))?,
        dz: optional(&format!("{prefix}DZ"))?,
        force: optional(&format!("{prefix}Force"))?,
        message: optional(&format!("{prefix}Msg"))?,
    })
}

fn value_to_i32(value: &EpicsValue) -> Option<i32> {
    match value {
        EpicsValue::Long(v) => Some(*v),
        EpicsValue::Enum(v) => Some(i32::from(*v)),
        EpicsValue::Short(v) => Some(i32::from(*v)),
        EpicsValue::Char(v) => Some(i32::from(*v)),
        EpicsValue::Double(v) => Some(*v as i32),
        EpicsValue::Float(v) => Some(*v as i32),
        _ => None,
    }
}

fn value_to_f64(value: &EpicsValue) -> Option<f64> {
    match value {
        EpicsValue::Double(v) => Some(*v),
        EpicsValue::Float(v) => Some(f64::from(*v)),
        EpicsValue::Long(v) => Some(f64::from(*v)),
        EpicsValue::Enum(v) => Some(f64::from(*v)),
        EpicsValue::Short(v) => Some(f64::from(*v)),
        _ => None,
    }
}

impl Epics {
    /// Connects every PV. The jog and map-source PVs are optional (warn
    /// and disable, as
    /// the C++ node did); all others are required. When `vision` is
    /// given (vision enabled), every vision PV is required too — failing
    /// at startup beats failing mid-sequence over a slot.
    pub fn connect(
        config: &EpicsConfig,
        vision: Option<&VisionConfig>,
        seat_check: Option<&SeatCheckConfig>,
    ) -> Result<Self, SequencerError> {
        let rt = Runtime::new()
            .map_err(|e| SequencerError(format!("cannot create tokio runtime: {e}")))?;
        let client = rt
            .block_on(CaClient::new())
            .map_err(|e| SequencerError(format!("cannot create CA client: {e}")))?;

        let required = |name: &str| -> Result<CaChannel, SequencerError> {
            let channel = client.create_channel(name);
            rt.block_on(channel.wait_connected(CONNECT_TIMEOUT))
                .map_err(|_| SequencerError(format!("PV '{name}' is not connected")))?;
            Ok(channel)
        };
        let optional = |name: &str| -> Option<CaChannel> {
            let channel = client.create_channel(name);
            match rt.block_on(channel.wait_connected(CONNECT_TIMEOUT)) {
                Ok(()) => Some(channel),
                Err(_) => {
                    log::warn(&format!(
                        "PV '{name}' not connected — optional, continuing without it"
                    ));
                    None
                }
            }
        };

        let vision = match vision {
            Some(v) => Some(VisionChannels {
                req: required(&v.req_pv)?,
                kind: required(&v.kind_pv)?,
                done: required(&v.done_pv)?,
                valid: required(&v.valid_pv)?,
                dx: required(&v.dx_pv)?,
                dy: required(&v.dy_pv)?,
                dz: required(&v.dz_pv)?,
                quality: required(&v.quality_pv)?,
                seated: required(&v.seated_pv)?,
                tilt: required(&v.tilt_pv)?,
            }),
            None => None,
        };

        // Required when the check is on, for the reason vision's PVs are:
        // a gate that quietly turns itself off because a camera PV did
        // not answer at startup is worse than no gate, because the
        // operator believes they have one.
        let depth = match seat_check {
            Some(s) => {
                let data = required(&format!("{}ArrayData", s.depth_prefix))?;
                let counter = required(&format!("{}ArrayCounter_RBV", s.depth_prefix))?;
                let number = |name: &str| -> Result<f64, SequencerError> {
                    let channel = required(name)?;
                    rt.block_on(channel.get_with_timeout(GET_TIMEOUT))
                        .ok()
                        .and_then(|(_, v)| value_to_f64(&v))
                        .ok_or_else(|| SequencerError(format!("PV '{name}' did not read")))
                };
                let c = &s.camera_prefix;
                let (fx, fy) = (
                    number(&format!("{c}RSDepthFx_RBV"))?,
                    number(&format!("{c}RSDepthFy_RBV"))?,
                );
                let (ppx, ppy) = (
                    number(&format!("{c}RSDepthPPx_RBV"))?,
                    number(&format!("{c}RSDepthPPy_RBV"))?,
                );
                let unit_m = number(&format!("{c}RSDepthUnits_RBV"))?;
                let width = number(&format!("{}ArraySize0_RBV", s.depth_prefix))? as usize;
                let height = number(&format!("{}ArraySize1_RBV", s.depth_prefix))? as usize;
                if width == 0 || height == 0 || unit_m <= 0.0 {
                    return Err(SequencerError(format!(
                        "seat check: {}ArraySize is {width}x{height} and {c}RSDepthUnits_RBV \
                         is {unit_m} — the depth stream is not running",
                        s.depth_prefix
                    )));
                }
                log::info(&format!(
                    "Seat check: depth {width}x{height}, fx {fx:.3}, ppx ({ppx:.1}, {ppy:.1}), \
                     {:.4} mm per count",
                    unit_m * 1000.0
                ));
                Some(DepthChannels {
                    data,
                    counter,
                    // The depth stream is rectified before it leaves the
                    // camera, so the model that describes it is the
                    // pinhole alone.
                    camera: DepthCamera {
                        lens: Lens {
                            k: [fx, 0.0, ppx, 0.0, fy, ppy, 0.0, 0.0, 1.0],
                            dist: [0.0; 5],
                        },
                        unit_m,
                        width,
                        height,
                    },
                })
            }
            None => None,
        };

        let epics = Self {
            trigger: required(&config.trigger_pv)?,
            start_step: required(&config.start_step_pv)?,
            wait: required(&config.wait_pv)?,
            holder: required(&config.holder_pv)?,
            stop: required(&config.stop_pv)?,
            current_step: required(&config.current_step_pv)?,
            gripper_cmd: required(&config.gripper_pv)?,
            gripper_rbv: required(&config.gripper_rbv_pv)?,
            pause_step: required(&config.pause_step_pv)?,
            calib_mode: required(&config.calib_mode_pv)?,
            loaded: required(&config.loaded_pv)?,
            map_source: optional(&config.map_source_pv),
            jog_x: optional(&config.jog_x_pv),
            jog_y: optional(&config.jog_y_pv),
            jog_z: optional(&config.jog_z_pv),
            jog_step: optional(&config.jog_step_pv),
            vision,
            null: null_channels(&optional, &config.null_prefix_pv),
            jog_total: jog_channels(&optional, &config.jog_prefix_pv),
            depth,
            _client: client,
            rt,
        };
        log::info("Connected to EPICS PVs");
        Ok(epics)
    }

    fn get_i32(&self, channel: &CaChannel, timeout: Duration) -> Option<i32> {
        self.rt
            .block_on(channel.get_with_timeout(timeout))
            .ok()
            .and_then(|(_, value)| value_to_i32(&value))
    }

    fn put_i32(&self, channel: &CaChannel, value: i32, timeout: Duration) -> bool {
        self.rt
            .block_on(channel.put_with_timeout(&EpicsValue::Long(value), timeout))
            .is_ok()
    }

    fn put_f64(&self, channel: &CaChannel, value: f64, timeout: Duration) -> bool {
        self.rt
            .block_on(channel.put_with_timeout(&EpicsValue::Double(value), timeout))
            .is_ok()
    }

    fn put_str(&self, channel: &CaChannel, value: &str, timeout: Duration) -> bool {
        self.rt
            .block_on(channel.put_with_timeout(&EpicsValue::String(value.into()), timeout))
            .is_ok()
    }

    /// Publish a whole grip-null snapshot. A no-op against an IOC whose
    /// database predates the records, and a failed put is logged at most
    /// once per call: this reports progress, it does not gate it, so a
    /// sequence must not die because a status write did not land.
    pub fn publish_null(&self, report: &NullReport) {
        let Some(n) = &self.null else {
            return;
        };
        let mut ok = self.put_i32(&n.state, report.state.code(), GET_TIMEOUT);
        ok &= self.put_i32(&n.iteration, report.iteration, GET_TIMEOUT);
        for (channel, value) in [
            (&n.dx, report.total_mm[0]),
            (&n.dy, report.total_mm[1]),
            (&n.dz, report.total_mm[2]),
            (&n.force, report.force_n),
        ] {
            ok &= self.put_f64(channel, value, GET_TIMEOUT);
        }
        // `stringin` is 40 bytes including the terminator, and the cut
        // has to land on a character boundary or the put is not UTF-8.
        let mut end = report.message.len().min(39);
        while !report.message.is_char_boundary(end) {
            end -= 1;
        }
        ok &= self.put_str(&n.message, &report.message[..end], GET_TIMEOUT);
        if !ok {
            log::warn("grip null status: at least one Robot:Null: put failed");
        }
    }

    /// Trigger value, `-1` on read error (the C++ node's sentinel).
    pub fn read_trigger(&self) -> i32 {
        self.get_i32(&self.trigger, GET_TIMEOUT).unwrap_or(-1)
    }

    pub fn write_trigger(&self, value: i32) -> bool {
        self.put_i32(&self.trigger, value, GET_TIMEOUT)
    }

    pub fn read_start_step(&self) -> i32 {
        self.get_i32(&self.start_step, GET_TIMEOUT).unwrap_or(0)
    }

    pub fn write_start_step(&self, value: i32) -> bool {
        self.put_i32(&self.start_step, value, GET_TIMEOUT)
    }

    pub fn read_wait(&self) -> WaitStatus {
        match self.get_i32(&self.wait, GET_TIMEOUT) {
            Some(0) => WaitStatus::Waiting,
            Some(2) => WaitStatus::Skip,
            // 1, any other value, or a read error: continue (C++ default).
            _ => WaitStatus::Continue,
        }
    }

    pub fn write_wait(&self, value: i32) -> bool {
        self.put_i32(&self.wait, value, GET_TIMEOUT)
    }

    /// The seat number as the PV has it, 1 if it cannot be read.
    ///
    /// Not clamped to the rack. It used to be — 1-10 or else 1, which
    /// is where `Robot:Holder = 0` went when the stage was added: the
    /// stage seat was coerced to holder 1 and the run fetched a puck
    /// into it, with only a warning line to say so. What a seat number
    /// means belongs to the mode that acts on it, so the range lives
    /// where it is used: `Sequencer::run_grip_null` reads 0 as the
    /// stage, and `Sequencer::compute_run_waypoints` refuses anything
    /// that is not a rack seat rather than extrapolating the pitch.
    pub fn read_holder(&self) -> i32 {
        match self.get_i32(&self.holder, GET_TIMEOUT) {
            Some(holder) => holder,
            None => {
                log::warn("Cannot read the holder PV, using 1");
                1
            }
        }
    }

    /// Map-mode puck source holder; 0 — also a missing PV or a bad read
    /// — means the target holder itself, probed in place.
    pub fn read_map_source(&self) -> i32 {
        let v = self
            .map_source
            .as_ref()
            .and_then(|ch| self.get_i32(ch, GET_TIMEOUT))
            .unwrap_or(0);
        if (0..=10).contains(&v) {
            v
        } else {
            log::warn(&format!(
                "Invalid map source {v} from PV, using 0 (in place)"
            ));
            0
        }
    }

    pub fn read_stop(&self) -> i32 {
        self.get_i32(&self.stop, GET_TIMEOUT).unwrap_or(0)
    }

    pub fn write_current_step(&self, value: i32) -> bool {
        self.put_i32(&self.current_step, value, GET_TIMEOUT)
    }

    pub fn read_pause_step(&self) -> i32 {
        self.get_i32(&self.pause_step, GET_TIMEOUT).unwrap_or(0)
    }

    pub fn read_calib_mode(&self) -> CalibMode {
        match self.get_i32(&self.calib_mode, GET_TIMEOUT) {
            Some(1) => CalibMode::Holder,
            Some(2) => CalibMode::SampleHolder,
            Some(3) => CalibMode::HandEye,
            Some(4) => CalibMode::Recover,
            Some(5) => CalibMode::SeatProbe,
            Some(6) => CalibMode::GripNull,
            Some(7) => CalibMode::HolderTransfer,
            _ => CalibMode::Normal,
        }
    }

    /// Gripper command (0=close, 1=open), `-1` on read error.
    pub fn read_gripper_cmd(&self) -> i32 {
        self.get_i32(&self.gripper_cmd, GET_TIMEOUT).unwrap_or(-1)
    }

    pub fn write_gripper_rbv(&self, value: i32) -> bool {
        self.put_i32(&self.gripper_rbv, value, GET_TIMEOUT)
    }

    pub fn write_loaded(&self, value: i32) -> bool {
        let ok = self.put_i32(&self.loaded, value, GET_TIMEOUT);
        if ok {
            log::info(&format!("Set Loaded PV to {value}"));
        }
        ok
    }

    /// Reads one jog PV (-1/0/+1) and resets it to 0 when non-zero, the
    /// C++ read-and-reset idiom. Returns 0 when jog is disabled or the
    /// read fails.
    fn read_and_reset_jog(&self, channel: &Option<CaChannel>, label: &str) -> i32 {
        let Some(channel) = channel else { return 0 };
        let Some(value) = self.get_i32(channel, JOG_TIMEOUT) else {
            return 0;
        };
        if value != 0 {
            let _ = self.put_i32(channel, 0, JOG_TIMEOUT);
            log::info(&format!("Jog PV '{label}' = {value} (reset to 0)"));
        }
        value
    }

    /// (x, y, z) jog request, each -1/0/+1.
    pub fn read_jog_request(&self) -> (i32, i32, i32) {
        (
            self.read_and_reset_jog(&self.jog_x, "JogX"),
            self.read_and_reset_jog(&self.jog_y, "JogY"),
            self.read_and_reset_jog(&self.jog_z, "JogZ"),
        )
    }

    /// Jog step size in mm, default 1.0 (C++ behavior).
    pub fn read_jog_step_mm(&self) -> f64 {
        let Some(channel) = &self.jog_step else {
            return 1.0;
        };
        self.rt
            .block_on(channel.get_with_timeout(JOG_TIMEOUT))
            .ok()
            .and_then(|(_, value)| value_to_f64(&value))
            .unwrap_or(1.0)
    }

    /// Publish the jog accumulator: the three tool-frame totals in mm
    /// and the seat an apply would write them to (empty = none, which is
    /// how the GUI knows to grey the button out). A no-op against an IOC
    /// whose database predates the records; a failed put is logged, not
    /// returned, because this reports state and does not gate it.
    pub fn publish_jog_total(&self, mm: [f64; 3], target: &str) {
        let Some(j) = &self.jog_total else {
            return;
        };
        let mut ok = true;
        for (channel, value) in j.d.iter().zip(mm) {
            ok &= self.put_f64(channel, value, GET_TIMEOUT);
        }
        ok &= self.put_str(&j.target, target, GET_TIMEOUT);
        if !ok {
            log::warn("jog total: at least one Robot:Jog: put failed");
        }
    }

    /// Takes a pending apply request, resetting the record the way
    /// `read_and_reset_jog` does: the request is an edge, and leaving it
    /// latched would re-apply the next accumulation without anyone
    /// asking.
    pub fn take_jog_apply(&self) -> bool {
        let Some(j) = &self.jog_total else {
            return false;
        };
        let Some(value) = self.get_i32(&j.apply, JOG_TIMEOUT) else {
            return false;
        };
        if value == 0 {
            return false;
        }
        let _ = self.put_i32(&j.apply, 0, JOG_TIMEOUT);
        true
    }

    /// The depth stream's geometry, or `None` when the seat check is off.
    pub fn depth_camera(&self) -> Option<&DepthCamera> {
        self.depth.as_ref().map(|d| &d.camera)
    }

    /// A depth frame the camera exposed after this call started.
    ///
    /// Waiting for the counter to advance, rather than reading the
    /// pixels straight away, is the same discipline
    /// `tools/handeye/detector.py` arrived at: a plain read lands on the
    /// frame that was in flight while the arm was still moving, and a
    /// seat check answered from the previous pose's view is a check that
    /// passes for the wrong reason. Two counts, not one, because the
    /// frame being clocked out when the call starts was exposed before
    /// it.
    pub fn depth_frame(&self, timeout: Duration) -> Option<DepthFrame> {
        let depth = self.depth.as_ref()?;
        let pixels = depth.camera.width * depth.camera.height;
        let start = self.get_i32(&depth.counter, GET_TIMEOUT)?;
        let deadline = std::time::Instant::now() + timeout;
        loop {
            match self.get_i32(&depth.counter, GET_TIMEOUT) {
                Some(now) if now >= start + 2 => break,
                _ if std::time::Instant::now() >= deadline => {
                    log::warn(&format!(
                        "seat check: no depth frame in {:.1} s (counter stuck at {start}) — \
                         check the plugin's EnableCallbacks",
                        timeout.as_secs_f64()
                    ));
                    return None;
                }
                _ => std::thread::sleep(FRAME_POLL),
            }
        }
        let (_, value) = self
            .rt
            .block_on(
                depth
                    .data
                    .get_with_timeout_count(timeout, u32::try_from(pixels).ok()?),
            )
            .ok()?;
        // Z16 counts travel as a SHORT waveform, so everything past
        // 3.2768 m arrives negative; `as u16` puts the bits back the way
        // the camera wrote them.
        let counts: Vec<u16> = match &value {
            EpicsValue::ShortArray(a) => a.iter().map(|v| *v as u16).collect(),
            EpicsValue::UShortArray(a) => a.clone(),
            EpicsValue::LongArray(a) => a.iter().map(|v| *v as u16).collect(),
            other => {
                log::warn(&format!("seat check: depth frame came back as {other:?}"));
                return None;
            }
        };
        if counts.len() < pixels {
            log::warn(&format!(
                "seat check: depth frame is {} pixels, expected {pixels}",
                counts.len()
            ));
            return None;
        }
        Some(DepthFrame { counts })
    }

    fn get_f64(&self, channel: &CaChannel, timeout: Duration) -> Option<f64> {
        self.rt
            .block_on(channel.get_with_timeout(timeout))
            .ok()
            .and_then(|(_, value)| value_to_f64(&value))
    }

    /// Last request id the vision node acknowledged (`Done`'s current
    /// value), 0 when vision is off or the read fails. Request ids must
    /// start ABOVE this: `Done` persists in the IOC across daemon
    /// restarts, and a fresh id sequence starting at 1 would alias the
    /// previous run's stale echo as an instant (wrong) answer.
    pub fn vision_last_done(&self) -> i32 {
        let Some(v) = &self.vision else { return 0 };
        self.get_i32(&v.done, GET_TIMEOUT).unwrap_or(0)
    }

    /// One vision handshake: write `Kind`, write `Req = req_id`, poll
    /// `Done` until it echoes `req_id`, then read the result registers.
    /// The id echo makes a stale answer from an earlier request
    /// unreadable as fresh. Every failure is an error — unlike the
    /// sequence-control reads there is no safe default for a
    /// measurement, and the caller stops the sequence.
    pub fn vision_request(
        &self,
        kind: VisionKind,
        req_id: i32,
        timeout: Duration,
    ) -> Result<VisionResult, SequencerError> {
        let v = self
            .vision
            .as_ref()
            .ok_or_else(|| SequencerError("vision PVs are not connected".into()))?;

        if !self.put_i32(&v.kind, kind as i32, GET_TIMEOUT) {
            return Err(SequencerError("vision: cannot write Kind PV".into()));
        }
        if !self.put_i32(&v.req, req_id, GET_TIMEOUT) {
            return Err(SequencerError("vision: cannot write Req PV".into()));
        }

        let deadline = std::time::Instant::now() + timeout;
        loop {
            if self.get_i32(&v.done, GET_TIMEOUT) == Some(req_id) {
                break;
            }
            if std::time::Instant::now() >= deadline {
                return Err(SequencerError(format!(
                    "vision: no answer to request {req_id} (kind {:?}) within {:.1}s",
                    kind,
                    timeout.as_secs_f64()
                )));
            }
            std::thread::sleep(VISION_POLL);
        }

        let read_f64 = |channel: &CaChannel, name: &str| -> Result<f64, SequencerError> {
            self.get_f64(channel, GET_TIMEOUT)
                .ok_or_else(|| SequencerError(format!("vision: cannot read {name} PV")))
        };
        let read_bit = |channel: &CaChannel, name: &str| -> Result<bool, SequencerError> {
            self.get_i32(channel, GET_TIMEOUT)
                .map(|value| value != 0)
                .ok_or_else(|| SequencerError(format!("vision: cannot read {name} PV")))
        };
        Ok(VisionResult {
            valid: read_bit(&v.valid, "Valid")?,
            dx: read_f64(&v.dx, "DX")?,
            dy: read_f64(&v.dy, "DY")?,
            dz: read_f64(&v.dz, "DZ")?,
            quality: read_f64(&v.quality, "Quality")?,
            seated: read_bit(&v.seated, "Seated")?,
            tilt: read_f64(&v.tilt, "Tilt")?,
        })
    }
}
