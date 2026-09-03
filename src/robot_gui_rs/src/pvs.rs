//! PV addresses and the fixed enums of `db/robot.db`.

/// `ca://` prefix for every robot PV. The daemon's IOC serves these; the
/// GUI finds them by broadcast search like every other client on this
/// host (no `EPICS_CA_ADDR_LIST` pinning — see CLAUDE.md).
pub const ROBOT: &str = "ca://Robot:";

/// `ca://` prefix for the D405 camera IOC's control records. Only the
/// images travel over pvAccess; Acquire/ImageMode/state stay CA.
pub const CAM: &str = "ca://RS405:";

pub fn robot(suffix: &str) -> String {
    format!("{ROBOT}{suffix}")
}

pub fn cam(suffix: &str) -> String {
    format!("{CAM}{suffix}")
}

/// `Robot:CalibMode` labels, index == PV value.
pub const MODE_NAMES: [&str; 8] = [
    "Normal",
    "Holder Calib",
    "Sample Holder Calib",
    "Hand-Eye Calib",
    "Recover",
    "Seat Probe",
    "Grip Null",
    "Holder Transfer",
];

/// `Robot:State` labels, index == PV value. The record is a plain
/// longin, like `Robot:Null:State`, so its labels live here next to
/// `step_name` rather than in the database.
pub fn state_name(state: i64) -> &'static str {
    match state {
        0 => "idle",
        1 => "running",
        2 => "measurement wait",
        3 => "paused",
        4 => "hold",
        _ => "?",
    }
}

/// `Robot:Seat:*` labels, index == PV value. Plain longins like
/// `Robot:State`, so the labels live here rather than in the database.
///
/// Zero is the state the records come up in and never one the daemon
/// writes: it says nothing has looked at that seat, which only the
/// database is in a position to claim.
pub fn seat_state_name(value: i64) -> &'static str {
    match value {
        1 => "empty",
        2 => "occupied",
        _ => "nothing has looked there since the IOC started",
    }
}

/// What the sequencer does at each `Robot:CurrentStep` value.
pub fn step_name(step: i64) -> &'static str {
    match step {
        0 => "open gripper",
        1 => "holder standby",
        2 => "above holder",
        3 => "at holder seat",
        4 => "grip puck",
        5 => "lift",
        6 => "retreat",
        7 => "stage standby",
        8 => "above stage",
        9 => "on stage seat",
        10 => "release",
        11 => "lift",
        12 => "stage standby — waiting",
        13 => "above stage",
        14 => "on stage seat",
        15 => "grip puck",
        16 => "lift",
        17 => "stage standby",
        18 => "holder standby",
        19 => "above holder",
        20 => "at holder seat",
        21 => "release",
        22 => "lift",
        23 => "holder standby",
        _ => "",
    }
}
