//! UR3e sample changer control GUI, RsDM edition.
//!
//! The robot PVs travel over `ca://` (broadcast search — do not pin
//! `EPICS_CA_ADDR_LIST`, see CLAUDE.md); the camera images travel over
//! `pva://` by default, reached through a direct TCP connection to the
//! D405 IOC's PVA server because this host's UDP 5076 is shared by
//! several IOCs the same way 5064 is (`ROBOT_GUI_PVA_SERVER` overrides,
//! default `127.0.0.1:5085` = `st.d405.cmd`'s `EPICS_PVAS_SERVER_PORT`).

mod calib;
mod camera;
mod ops;
mod pvs;
mod yamledit;

use std::path::PathBuf;
use std::sync::Arc;

use eframe::egui;
use rsdm::{Engine, PvaPlugin};

/// `--camera` opens on the Camera tab (the desktop camera-viewer
/// launcher); a bare argument is the waypoints file.
fn cli() -> (bool, Option<PathBuf>) {
    let mut camera = false;
    let mut path = None;
    for arg in std::env::args().skip(1) {
        if arg == "--camera" {
            camera = true;
        } else {
            path = Some(PathBuf::from(arg));
        }
    }
    (camera, path)
}

/// `taught_waypoints.yaml`: an explicit argument wins, then the
/// checkout this binary was built in (exe under `target/release/`),
/// then the compile-time path for `cargo run`.
fn waypoints_path(explicit: Option<PathBuf>) -> PathBuf {
    if let Some(arg) = explicit {
        return arg;
    }
    if let Ok(exe) = std::env::current_exe()
        && let Some(dir) = exe.parent()
    {
        let repo = dir.join("../../../../config/taught_waypoints.yaml");
        if repo.exists() {
            return repo;
        }
    }
    PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../config/taught_waypoints.yaml"
    ))
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Tab {
    Operate,
    Camera,
    Calibration,
}

struct RobotGui {
    // The engine owns the tokio runtime and every connection; it must
    // outlive the widgets holding Channel handles.
    _engine: Engine,
    tab: Tab,
    ops: ops::OpsPanel,
    camera: camera::CameraPanel,
    calib: calib::CalibPanel,
}

impl RobotGui {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let rs = cc
            .wgpu_render_state
            .as_ref()
            .expect("eframe must use the wgpu renderer (NativeOptions.renderer = Wgpu)");
        rsplot::install(rs);

        let engine = Engine::new();
        engine.attach_repaint(cc.egui_ctx.clone());
        let pva_server =
            std::env::var("ROBOT_GUI_PVA_SERVER").unwrap_or_else(|_| "127.0.0.1:5085".into());
        match pva_server.parse() {
            Ok(addr) => engine.register_plugin(Arc::new(PvaPlugin::with_server(addr))),
            Err(e) => log::warn!(
                "ROBOT_GUI_PVA_SERVER '{pva_server}' is not host:port ({e}); \
                 keeping UDP search"
            ),
        }

        let (start_on_camera, yaml_arg) = cli();
        let ops = ops::OpsPanel::new(&engine).expect("connect robot PVs");
        let camera = camera::CameraPanel::new(&engine, rs).expect("connect camera channels");
        let calib =
            calib::CalibPanel::new(&engine, waypoints_path(yaml_arg)).expect("connect jog PVs");
        Self {
            _engine: engine,
            tab: if start_on_camera {
                Tab::Camera
            } else {
                Tab::Operate
            },
            ops,
            camera,
            calib,
        }
    }
}

impl eframe::App for RobotGui {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        ui.horizontal(|ui| {
            ui.heading("UR3e Sample Changer");
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.selectable_value(&mut self.tab, Tab::Calibration, "Calibration");
                ui.selectable_value(&mut self.tab, Tab::Camera, "Camera");
                ui.selectable_value(&mut self.tab, Tab::Operate, "Operate");
            });
        });
        ui.separator();
        match self.tab {
            Tab::Operate => self.ops.show(ui),
            Tab::Camera => self.camera.show(ui),
            Tab::Calibration => self.calib.show(ui),
        }
    }
}

fn main() -> eframe::Result {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    eframe::run_native(
        "UR3e Sample Changer",
        eframe::NativeOptions {
            renderer: eframe::Renderer::Wgpu,
            viewport: egui::ViewportBuilder::default().with_inner_size([1080.0, 720.0]),
            ..Default::default()
        },
        Box::new(|cc| Ok(Box::new(RobotGui::new(cc)) as Box<dyn eframe::App>)),
    )
}
