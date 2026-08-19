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

/// `--camera` makes this process the camera viewer and nothing else
/// (the desktop camera-viewer launcher); a bare argument is the
/// waypoints file.
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

/// The camera window's id. Fixed, so reopening it reuses the window
/// rather than stacking a second one.
const CAMERA_VIEWPORT: &str = "d405-camera";

/// Space between cards, and the page's outer margin.
const GAP: f32 = 8.0;

/// Row heights, each set by the tallest card in that row so the row is
/// a clean rectangle. A `ui.group` on its own shrinks to its content,
/// which is what left the panel looking like scattered boxes.
const H_STATE: f32 = 248.0;
const H_RUN: f32 = 216.0;
const H_MANUAL: f32 = 140.0;
const H_TEACH: f32 = 430.0;

/// One box on the page's three-column grid.
///
/// The width comes from the column, never from the content: two cards
/// in the same column across two rows share an edge only if both were
/// told the same width.
fn card(ui: &mut egui::Ui, size: egui::Vec2, add: impl FnOnce(&mut egui::Ui)) {
    ui.allocate_ui(size, |ui| {
        ui.group(|ui| {
            ui.set_min_size(size - egui::Vec2::splat(GAP));
            ui.set_max_width(size.x - GAP);
            ui.vertical(add);
        });
    });
}

/// The two pages. Teaching is a different errand from running: it
/// edits the taught file and it is the only page whose numbers are not
/// live robot state, so it does not share a scroll position with the
/// controls that move the arm.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Tab {
    Operate,
    Teach,
}

struct RobotGui {
    // The engine owns the tokio runtime and every connection; it must
    // outlive the widgets holding Channel handles.
    _engine: Engine,
    /// `--camera`: this process IS the camera viewer, so the main
    /// window carries the camera and no control surface at all. The
    /// desktop launcher runs a second copy of this binary that way, and
    /// two windows of controls onto one robot is one too many.
    camera_only: bool,
    /// Whether the separate camera window is open.
    camera_open: bool,
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

        let (camera_only, yaml_arg) = cli();
        let ops = ops::OpsPanel::new(&engine).expect("connect robot PVs");
        let camera = camera::CameraPanel::new(&engine).expect("connect camera channels");
        let calib =
            calib::CalibPanel::new(&engine, waypoints_path(yaml_arg)).expect("connect jog PVs");
        Self {
            _engine: engine,
            camera_only,
            camera_open: false,
            tab: Tab::Operate,
            ops,
            camera,
            calib,
        }
    }

    /// The width of one and two columns of the page's three-column
    /// grid. A minimum rather than a fit: below it the cards would clip
    /// their own contents, and a horizontal scrollbar is the better
    /// failure.
    fn columns(ui: &egui::Ui) -> (f32, f32) {
        let one = (((ui.available_width() - 2.0 * GAP) / 3.0) - GAP).max(248.0);
        (one, one * 2.0 + GAP)
    }

    /// Everything that reads or moves the robot: state at the top, the
    /// things that start a move below it. Every card is one or two
    /// columns wide and every row is one height, so the edges line up
    /// down the page.
    fn operate_page(&mut self, ui: &mut egui::Ui) {
        let (one, two) = Self::columns(ui);

        ui.heading("State");
        ui.horizontal_top(|ui| {
            card(ui, egui::vec2(one, H_STATE), |ui| self.ops.status_group(ui));
            card(ui, egui::vec2(two, H_STATE), |ui| {
                self.ops.null_status_group(ui)
            });
        });

        ui.add_space(GAP);
        ui.heading("Run");
        ui.horizontal_top(|ui| {
            card(ui, egui::vec2(one, H_RUN), |ui| self.ops.sample_group(ui));
            card(ui, egui::vec2(one, H_RUN), |ui| {
                self.ops.grip_null_group(ui)
            });
            card(ui, egui::vec2(one, H_RUN), |ui| {
                self.ops.move_puck_group(ui)
            });
        });
        ui.horizontal_top(|ui| {
            card(ui, egui::vec2(one, H_MANUAL), |ui| {
                self.ops.gripper_group(ui)
            });
            card(ui, egui::vec2(two, H_MANUAL), |ui| {
                self.ops.advanced_group(ui)
            });
        });
        self.ops.note_line(ui);
    }

    /// The taught numbers and the jog that measures them.
    fn teach_page(&mut self, ui: &mut egui::Ui) {
        let (one, two) = Self::columns(ui);
        ui.horizontal_top(|ui| {
            card(ui, egui::vec2(one, H_TEACH), |ui| self.calib.jog_group(ui));
            card(ui, egui::vec2(two, H_TEACH), |ui| {
                self.calib.table_group(ui)
            });
        });
        self.calib.note_line(ui);
    }

    /// The camera in its own native window, for as long as it is open.
    fn camera_window(&mut self, ctx: &egui::Context) {
        if !self.camera_open {
            return;
        }
        let mut close = false;
        ctx.show_viewport_immediate(
            egui::ViewportId::from_hash_of(CAMERA_VIEWPORT),
            egui::ViewportBuilder::default()
                .with_title("D405 Camera")
                .with_inner_size([1100.0, 620.0]),
            |ui, _class| {
                self.camera.show(ui);
                if ui.input(|i| i.viewport().close_requested()) {
                    close = true;
                }
            },
        );
        if close {
            self.camera_open = false;
        }
    }
}

impl eframe::App for RobotGui {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        if self.camera_only {
            self.camera.show(ui);
            return;
        }
        self.ops.begin(ui.ctx());
        ui.horizontal(|ui| {
            ui.heading("UR3e Sample Changer");
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let label = if self.camera_open {
                    "Close camera"
                } else {
                    "Camera window"
                };
                if ui.button(label).clicked() {
                    self.camera_open = !self.camera_open;
                }
            });
        });
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.tab, Tab::Operate, "Operate");
            ui.selectable_value(&mut self.tab, Tab::Teach, "Teach");
        });
        ui.separator();
        // One scroll area per tab: sharing it would carry the operate
        // page's offset into a shorter Teach page and open it scrolled
        // past its own top.
        match self.tab {
            Tab::Operate => egui::ScrollArea::both()
                .id_salt("page-operate")
                .show(ui, |ui| self.operate_page(ui)),
            Tab::Teach => egui::ScrollArea::both()
                .id_salt("page-teach")
                .show(ui, |ui| self.teach_page(ui)),
        };
        self.camera_window(ui.ctx());
    }
}

fn main() -> eframe::Result {
    // This host is multihomed and every robot PV is served on both
    // interfaces, so each CA beacon arrives twice and the client's
    // anomaly detector reads the halved period as "IOC may have
    // restarted", several times a minute, forever. It is wrong here by
    // construction, and a CA failure that matters already shows on the
    // page as a red DISCONNECTED rather than in a log the operator does
    // not have open. `RUST_LOG` still overrides this.
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("warn,epics_ca_rs::client=error"),
    )
    .init();
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
