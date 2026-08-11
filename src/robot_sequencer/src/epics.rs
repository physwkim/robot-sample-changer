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

use crate::config::EpicsConfig;
use crate::error::SequencerError;
use crate::log;

const GET_TIMEOUT: Duration = Duration::from_secs(1);
const JOG_TIMEOUT: Duration = Duration::from_millis(500);
const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);

/// `Robot:Wait` states (mbbo: 0=Wait, 1=Continue, 2=Abort).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaitStatus {
    Waiting,
    Continue,
    Skip,
}

/// `Robot:CalibMode` states (mbbo: 0=Normal, 1=Holder, 2=Sample Holder).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibMode {
    Normal,
    Holder,
    SampleHolder,
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
    /// the C++ node did); all others are required.
    pub fn connect(config: &EpicsConfig) -> Result<Self, SequencerError> {
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
}
