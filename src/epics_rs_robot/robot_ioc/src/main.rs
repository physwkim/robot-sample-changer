//! Rust soft-record IOC for the UR3e + Robotiq Hand-E robot.
//!
//! Replaces `softIoc -d db/robot.db` (EPICS base is not installed on this host).
//! Serves the `Robot:*` PVs over Channel Access for `robot_gui`, the CA↔ROS2
//! bridge, and any other CA client. The existing `ws/db/robot.db` is reused
//! unchanged — all records are plain soft records (bo/mbbo/longout/longin/bi/ao).
//!
//! Usage:
//!   robot_ioc [path/to/st.cmd]
//! With no argument it runs the bundled `ioc/st.cmd`.

use std::sync::{Arc, Mutex};

use epics_base_rs::error::CaResult;
use epics_base_rs::server::autosave::startup::AutosaveStartupConfig;
use epics_ca_rs::server::ioc_app::IocApplication;
use epics_ca_rs::server::run_ca_ioc;

#[epics_base_rs::epics_main]
async fn main() -> CaResult<()> {
    // Default the macros used by st.cmd. Override by exporting them before launch.
    epics_base_rs::runtime::env::set_default("ROBOT_DB", "/home/bl9b/ws/db");
    epics_base_rs::runtime::env::set_default("ROBOT_IOC", env!("CARGO_MANIFEST_DIR"));

    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        format!("{}/ioc/st.cmd", env!("CARGO_MANIFEST_DIR"))
    };

    // Autosave so the robot run-state PVs (CurrentStep/Holder/CalibMode/Loaded/
    // StartStep) survive an IOC or power restart — this is what makes resume-after-
    // crash possible. The .req/.sav paths are configured in st.cmd.
    let autosave_config = Arc::new(Mutex::new(AutosaveStartupConfig::new()));

    IocApplication::new()
        .autosave_startup(autosave_config)
        .startup_script(&script)
        .run(run_ca_ioc)
        .await
}
