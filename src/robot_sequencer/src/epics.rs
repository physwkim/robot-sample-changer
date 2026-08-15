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

use crate::config::{EpicsConfig, VisionConfig};
use crate::error::SequencerError;
use crate::log;

const GET_TIMEOUT: Duration = Duration::from_secs(1);
const JOG_TIMEOUT: Duration = Duration::from_millis(500);
const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);
const VISION_POLL: Duration = Duration::from_millis(50);

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
    jog_x: Option<CaChannel>,
    jog_y: Option<CaChannel>,
    jog_z: Option<CaChannel>,
    jog_step: Option<CaChannel>,
    vision: Option<VisionChannels>,
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
    /// Connects every PV. The jog PVs are optional (warn and disable, as
    /// the C++ node did); all others are required. When `vision` is
    /// given (vision enabled), every vision PV is required too — failing
    /// at startup beats failing mid-sequence over a slot.
    pub fn connect(
        config: &EpicsConfig,
        vision: Option<&VisionConfig>,
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
                    log::warn(&format!("PV '{name}' not connected (jog disabled)"));
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
            jog_x: optional(&config.jog_x_pv),
            jog_y: optional(&config.jog_y_pv),
            jog_z: optional(&config.jog_z_pv),
            jog_step: optional(&config.jog_step_pv),
            vision,
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

    /// Holder number clamped to 1-10, defaulting to 1 (C++ behavior).
    pub fn read_holder(&self) -> i32 {
        let holder = self.get_i32(&self.holder, GET_TIMEOUT).unwrap_or(1);
        if !(1..=10).contains(&holder) {
            log::warn(&format!("Invalid holder number {holder} from PV, using 1"));
            return 1;
        }
        holder
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
