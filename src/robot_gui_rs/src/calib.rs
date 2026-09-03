//! Calibration: TCP jog during calibration holds, and the offsets/tilt
//! table over `taught_waypoints.yaml` with edited-cells-only saving.

use std::path::PathBuf;

use eframe::egui;
use rsdm::{Channel, Engine, EngineError, PvValue};

use crate::daemon::{DaemonWatch, HOLD};
use crate::pvs::robot;
use crate::yamledit::{self, Slot};

/// The one red in this panel: a daemon that is not listening, and a
/// table that would not load.
const RED: egui::Color32 = egui::Color32::from_rgb(0xf4, 0x43, 0x36);

const COLS: usize = 5; // X mm, Y mm, Z mm, Tilt X deg, Tilt Z deg
const ROWS: usize = 12; // stage + the rack base + holders 1-10

/// Display-unit cell values (mm for 0-2, deg for 3-4); `None` = the
/// parameter does not exist for that row (stage tilt).
type Cells = [[Option<f64>; COLS]; ROWS];

fn slot_for(row: usize, col: usize) -> Option<Slot> {
    // Row 0 = stage (sample holder), row 1 = the rack's own base and
    // lean, rows 2-11 = holder N at list index N-1. Holder 1 used to BE
    // row 1, sharing its cells with the rack base, so trimming it moved
    // every other seat by the same amount.
    match (row, col) {
        (0, 0) => Some(Slot::Scalar("sample_holder_on_position_x_offset")),
        (0, 1) => Some(Slot::Scalar("sample_holder_on_position_y_offset")),
        (0, 2) => Some(Slot::Scalar("sample_holder_on_position_z_offset")),
        (0, _) => None,
        (1, 0) => Some(Slot::Scalar("holder_rack_x_offset")),
        (1, 1) => Some(Slot::Scalar("holder_rack_y_offset")),
        (1, 2) => Some(Slot::Scalar("holder_rack_z_offset")),
        (1, 3) => Some(Slot::Scalar("holder_on_position_tilt_x_deg")),
        (1, 4) => Some(Slot::Scalar("holder_on_position_tilt_z_deg")),
        (r, 0) => Some(Slot::List("holder_multi_x_offsets", r - 2)),
        (r, 1) => Some(Slot::List("holder_multi_y_offsets", r - 2)),
        (r, 2) => Some(Slot::List("holder_multi_z_offsets", r - 2)),
        (r, 3) => Some(Slot::List("holder_multi_tilt_x_deg", r - 2)),
        (r, 4) => Some(Slot::List("holder_multi_tilt_z_deg", r - 2)),
        _ => None,
    }
}

fn fval(ch: &Channel) -> Option<f64> {
    ch.state().value.as_ref().and_then(PvValue::as_f64)
}

fn sval(ch: &Channel) -> Option<String> {
    match ch.state().value.as_ref() {
        Some(PvValue::Str(s)) => Some(s.to_string()),
        _ => None,
    }
}

fn row_name(row: usize) -> String {
    match row {
        0 => "Stage".to_string(),
        1 => "Rack".to_string(),
        n => format!("Holder {}", n - 1),
    }
}

/// mm cells carry metres in the file; tilt cells carry degrees as-is.
fn to_file_units(col: usize, display: f64) -> f64 {
    if col < 3 { display / 1000.0 } else { display }
}

fn load_cells(path: &std::path::Path) -> Result<Cells, String> {
    let p = yamledit::load(path)?;
    let mut cells: Cells = Default::default();
    let scalar = |key: &str| yamledit::f64_at(&p, key);
    cells[0][0] = Some(scalar("sample_holder_on_position_x_offset") * 1000.0);
    cells[0][1] = Some(scalar("sample_holder_on_position_y_offset") * 1000.0);
    cells[0][2] = Some(scalar("sample_holder_on_position_z_offset") * 1000.0);
    cells[1][0] = Some(scalar("holder_rack_x_offset") * 1000.0);
    cells[1][1] = Some(scalar("holder_rack_y_offset") * 1000.0);
    cells[1][2] = Some(scalar("holder_rack_z_offset") * 1000.0);
    cells[1][3] = Some(scalar("holder_on_position_tilt_x_deg"));
    cells[1][4] = Some(scalar("holder_on_position_tilt_z_deg"));
    let lists = [
        yamledit::vec_at(&p, "holder_multi_x_offsets", ROWS - 2)?,
        yamledit::vec_at(&p, "holder_multi_y_offsets", ROWS - 2)?,
        yamledit::vec_at(&p, "holder_multi_z_offsets", ROWS - 2)?,
        yamledit::vec_at(&p, "holder_multi_tilt_x_deg", ROWS - 2)?,
        yamledit::vec_at(&p, "holder_multi_tilt_z_deg", ROWS - 2)?,
    ];
    for row in 2..ROWS {
        for (col, list) in lists.iter().enumerate() {
            let v = list[row - 2];
            cells[row][col] = Some(if col < 3 { v * 1000.0 } else { v });
        }
    }
    Ok(cells)
}

pub struct CalibPanel {
    jog: [Channel; 3],
    jog_step: Channel,
    travel: [Channel; 3],
    apply_target: Channel,
    apply: Channel,
    /// The gate on every button in this panel: the jog moves the arm
    /// and Apply rewrites the taught file, and both are read only in
    /// the daemon's service pass. See [`crate::daemon`].
    daemon: DaemonWatch,
    step_mm: f64,
    yaml_path: PathBuf,
    cells: Cells,
    loaded: Cells,
    have_table: bool,
    note: String,
}

/// One arrowhead at `tip`, pointing away from `tail`.
fn head(p: &egui::Painter, tip: egui::Pos2, tail: egui::Pos2, stroke: egui::Stroke) {
    let d = (tip - tail).normalized();
    let n = egui::vec2(-d.y, d.x);
    p.line_segment([tip, tip - d * 5.0 + n * 3.0], stroke);
    p.line_segment([tip, tip - d * 5.0 - n * 3.0], stroke);
}

/// A double-headed arrow: one jog axis, both of its buttons.
fn axis_arrow(p: &egui::Painter, a: egui::Pos2, b: egui::Pos2, stroke: egui::Stroke) {
    p.line_segment([a, b], stroke);
    head(p, a, b, stroke);
    head(p, b, a, stroke);
}

fn box_at(p: &egui::Painter, x: (f32, f32), y: (f32, f32), fill: egui::Color32, s: egui::Stroke) {
    let r = egui::Rect::from_min_max(egui::pos2(x.0, y.0), egui::pos2(x.1, y.1));
    p.rect_filled(r, 1.0, fill);
    p.rect_stroke(r, 1.0, s, egui::StrokeKind::Inside);
}

/// Which way the three jog buttons actually move the gripper, drawn
/// rather than described.
///
/// The frame is `ik_frame` (`robotiq_hande_end`) — what `Motion::jog`
/// rotates its millimetres out of, and what the trim columns next door
/// are written in. Not `tool0`: the URDF turns the coupler 180° about z
/// away from it, so tool0's x and y point the other way.
///
/// The URDF fixes two of the three: the fingers slide along ±x
/// (`robotiq_hande_*_finger_joint`, `axis 1 0 0`, and every joint from
/// there to `robotiq_hande_end` is rpy 0), and the gripper reaches out
/// along +z. FK of the four taught seats puts +y within 0.1° of straight
/// down at all of them, which is what lets this label +y "into the seat"
/// — and is why `above_y_offset` is negative to stand 5 mm clear.
fn axis_figure(ui: &mut egui::Ui) {
    let w = ui.available_width().clamp(220.0, 320.0);
    let (rect, _) = ui.allocate_exact_size(egui::vec2(w, 104.0), egui::Sense::hover());
    let p = ui.painter_at(rect);
    let v = ui.visuals();
    let ink = egui::Stroke::new(1.0, v.text_color());
    let faint = egui::Stroke::new(1.0, v.weak_text_color());
    let metal = v.widgets.inactive.bg_fill;
    let ghost = v.widgets.noninteractive.bg_fill;
    let puck = v.selection.bg_fill;
    let cap = egui::FontId::proportional(9.5);
    let tag = egui::FontId::proportional(10.5);
    let label = |at: egui::Pos2, anchor: egui::Align2, s: &str| {
        p.text(at, anchor, s, tag.clone(), v.text_color());
    };

    let half = (rect.width() - 10.0) / 2.0;
    let front = egui::Rect::from_min_size(rect.min, egui::vec2(half, rect.height()));
    let plan = egui::Rect::from_min_size(
        rect.min + egui::vec2(half + 10.0, 0.0),
        egui::vec2(half, rect.height()),
    );

    // Seen from in front of the jaws. The body is directly behind the
    // pads at a seat — z is horizontal there — so it is a silhouette,
    // not a box stacked on top of them.
    p.text(
        front.center_top(),
        egui::Align2::CENTER_TOP,
        "from the front",
        cap.clone(),
        v.weak_text_color(),
    );
    let (cx, t) = (front.center().x, front.top() + 16.0);
    box_at(&p, (cx - 20.0, cx + 20.0), (t, t + 26.0), ghost, faint);
    for (x0, x1) in [(cx - 18.0, cx - 9.0), (cx + 9.0, cx + 18.0)] {
        box_at(&p, (x0, x1), (t + 10.0, t + 40.0), metal, ink);
    }
    p.circle_filled(egui::pos2(cx, t + 34.0), 7.0, puck);
    p.line_segment(
        [
            egui::pos2(cx - 28.0, t + 45.0),
            egui::pos2(cx + 28.0, t + 45.0),
        ],
        faint,
    );
    for i in 0..5 {
        let x = cx - 24.0 + 12.0 * i as f32;
        p.line_segment(
            [egui::pos2(x, t + 45.0), egui::pos2(x - 4.0, t + 49.0)],
            faint,
        );
    }
    let ax = front.left() + 10.0;
    axis_arrow(&p, egui::pos2(ax, t + 4.0), egui::pos2(ax, t + 42.0), ink);
    label(egui::pos2(ax, t - 1.0), egui::Align2::CENTER_BOTTOM, "−Y");
    label(egui::pos2(ax, t + 47.0), egui::Align2::CENTER_TOP, "+Y");
    axis_arrow(
        &p,
        egui::pos2(cx - 26.0, t + 60.0),
        egui::pos2(cx + 26.0, t + 60.0),
        ink,
    );
    label(
        egui::pos2(cx - 30.0, t + 60.0),
        egui::Align2::RIGHT_CENTER,
        "−X",
    );
    label(
        egui::pos2(cx + 30.0, t + 60.0),
        egui::Align2::LEFT_CENTER,
        "+X",
    );

    // Looking straight down the axis the arm descends on.
    p.text(
        plan.center_top(),
        egui::Align2::CENTER_TOP,
        "from above",
        cap,
        v.weak_text_color(),
    );
    let (px, pt) = (plan.center().x, plan.top() + 16.0);
    box_at(
        &p,
        (px - 19.0, px + 19.0),
        (pt + 34.0, pt + 46.0),
        ghost,
        faint,
    );
    for (x0, x1) in [(px - 16.0, px - 7.0), (px + 7.0, px + 16.0)] {
        box_at(&p, (x0, x1), (pt + 8.0, pt + 34.0), metal, ink);
    }
    p.circle_filled(egui::pos2(px, pt + 21.0), 7.0, puck);
    let az = plan.right() - 10.0;
    axis_arrow(&p, egui::pos2(az, pt + 6.0), egui::pos2(az, pt + 46.0), ink);
    label(egui::pos2(az, pt + 1.0), egui::Align2::CENTER_BOTTOM, "+Z");
    label(egui::pos2(az, pt + 51.0), egui::Align2::CENTER_TOP, "−Z");
    axis_arrow(
        &p,
        egui::pos2(px - 26.0, pt + 60.0),
        egui::pos2(px + 26.0, pt + 60.0),
        ink,
    );
    label(
        egui::pos2(px - 30.0, pt + 60.0),
        egui::Align2::RIGHT_CENTER,
        "−X",
    );
    label(
        egui::pos2(px + 30.0, pt + 60.0),
        egui::Align2::LEFT_CENTER,
        "+X",
    );
}

impl CalibPanel {
    pub fn new(engine: &Engine, yaml_path: PathBuf) -> Result<Self, EngineError> {
        let mut panel = Self {
            jog: [
                engine.connect(&robot("JogX"))?,
                engine.connect(&robot("JogY"))?,
                engine.connect(&robot("JogZ"))?,
            ],
            jog_step: engine.connect(&robot("JogStep"))?,
            travel: [
                engine.connect(&robot("Jog:DX"))?,
                engine.connect(&robot("Jog:DY"))?,
                engine.connect(&robot("Jog:DZ"))?,
            ],
            apply_target: engine.connect(&robot("Jog:Target"))?,
            apply: engine.connect(&robot("Jog:Apply"))?,
            daemon: DaemonWatch::new(engine)?,
            step_mm: 1.0,
            yaml_path,
            cells: Default::default(),
            loaded: Default::default(),
            have_table: false,
            note: String::new(),
        };
        panel.reload();
        Ok(panel)
    }

    fn reload(&mut self) {
        match load_cells(&self.yaml_path) {
            Ok(cells) => {
                self.cells = cells;
                self.loaded = cells;
                self.have_table = true;
                self.note = format!("Loaded {}", self.yaml_path.display());
            }
            Err(e) => {
                self.have_table = false;
                self.note = format!("Load failed: {e}");
            }
        }
    }

    /// Writes only the cells that differ from their loaded value.
    /// `apply_edits` itself starts from a fresh read of the file, so a
    /// trim the daemon's grip null wrote since our load survives.
    fn save(&mut self) {
        let mut edits = Vec::new();
        for row in 0..ROWS {
            for col in 0..COLS {
                if let (Some(now), Some(was)) = (self.cells[row][col], self.loaded[row][col])
                    && (now - was).abs() > 1e-9
                    && let Some(slot) = slot_for(row, col)
                {
                    edits.push((slot, to_file_units(col, now)));
                }
            }
        }
        if edits.is_empty() {
            self.note = "Nothing edited — file untouched".to_string();
            return;
        }
        let count = edits.len();
        match yamledit::apply_edits(&self.yaml_path, &edits) {
            Ok(()) => {
                self.reload();
                self.note = format!(
                    "Saved {count} value(s) — the next trigger reloads them ({})",
                    self.yaml_path.display()
                );
            }
            Err(e) => self.note = format!("Save failed: {e}"),
        }
    }

    pub fn jog_group(&mut self, ui: &mut egui::Ui) {
        ui.strong("TCP jog (tool frame, serviced whenever the arm waits)");
        ui.horizontal(|ui| {
            ui.label("Step (mm):");
            ui.add(
                egui::DragValue::new(&mut self.step_mm)
                    .range(0.01..=5.0)
                    .speed(0.05)
                    .fixed_decimals(2),
            );
        });
        // The daemon reads `Robot:JogX/Y/Z` in its service pass, which
        // runs only while it is standing in a wait. A press it never
        // reads is not a jog that failed, it is a button that lied.
        let live = self.daemon.servicing();
        if !live {
            ui.colored_label(RED, format!("({})", self.daemon.note()));
        }
        ui.add_enabled_ui(live, |ui| {
            egui::Grid::new("jog").show(ui, |ui| {
                for (i, axis) in ["X", "Y", "Z"].iter().enumerate() {
                    ui.label(*axis);
                    for dir in [-1i64, 1] {
                        let label = if dir < 0 { "−" } else { "+" };
                        if ui.button(format!("{axis}{label}")).clicked() {
                            self.jog_step.put(PvValue::Float(self.step_mm));
                            self.jog[i].put(PvValue::Int(dir));
                        }
                    }
                    ui.end_row();
                }
            });
        });
        axis_figure(ui);
        ui.small(
            "X closes across the pads, Z runs along the jaw, +Y goes down \
             into the seat. The trim columns are these same axes.",
        );
        ui.separator();
        ui.label("Jogged this run (tool x, y, z):");
        let travel: Vec<Option<f64>> = self.travel.iter().map(fval).collect();
        match (travel[0], travel[1], travel[2]) {
            (Some(x), Some(y), Some(z)) => ui.monospace(format!("{x:+7.3} {y:+7.3} {z:+7.3} mm")),
            _ => ui.label("-"),
        };
        // Apply lands only where the daemon is standing at a seat,
        // which is exactly its hold state. `Jog:Target` names that seat
        // but does not date it -- it keeps the last hold's name after
        // the daemon is gone, and enabling from it offered Apply over a
        // dead run, where the press writes nothing and says nothing.
        let holding = self.daemon.is(HOLD);
        let target = sval(&self.apply_target).unwrap_or_default();
        ui.add_enabled_ui(holding, |ui| {
            let label = match (holding, target.is_empty()) {
                (true, false) => format!("Apply to {target}"),
                (true, true) => "Apply to this seat".to_string(),
                (false, _) => format!("Apply to seat trims ({})", self.daemon.note()),
            };
            if ui.button(label).clicked() {
                self.apply.put(PvValue::Int(1));
            }
        });
        ui.label(
            "Apply adds the travel to that seat's X/Y/Z trims and zeroes \
             it; the next trigger reloads the file.",
        );
    }

    pub fn table_group(&mut self, ui: &mut egui::Ui) {
        ui.strong("Seat offsets and tilts (taught_waypoints.yaml)");
        if !self.have_table {
            ui.colored_label(RED, &self.note);
            if ui.button("Retry load").clicked() {
                self.reload();
            }
            return;
        }
        egui::Grid::new("trims").striped(true).show(ui, |ui| {
            for header in [
                "",
                "X (mm)",
                "Y (mm)",
                "Z (mm)",
                "Tilt X (deg)",
                "Tilt Z (deg)",
            ] {
                ui.strong(header);
            }
            ui.end_row();
            for row in 0..ROWS {
                ui.label(row_name(row));
                for col in 0..COLS {
                    match &mut self.cells[row][col] {
                        Some(value) => {
                            let (range, step) = if col < 3 {
                                (-50.0..=50.0, 0.01)
                            } else {
                                (-5.0..=5.0, 0.01)
                            };
                            let edited = self.loaded[row][col]
                                .is_some_and(|was| (*value - was).abs() > 1e-9);
                            let widget = egui::DragValue::new(value)
                                .range(range)
                                .speed(step)
                                .fixed_decimals(3);
                            let resp = ui.add(widget);
                            if edited {
                                // Mark unsaved edits the way the * did in
                                // the silx panel.
                                ui.painter().rect_stroke(
                                    resp.rect,
                                    2.0,
                                    egui::Stroke::new(1.0, egui::Color32::YELLOW),
                                    egui::StrokeKind::Outside,
                                );
                            }
                        }
                        None => {
                            ui.label("—");
                        }
                    }
                }
                ui.end_row();
            }
        });
        ui.horizontal(|ui| {
            if ui.button("Reload").clicked() {
                self.reload();
            }
            if ui.button("Save edited cells").clicked() {
                self.save();
            }
        });
        ui.label(
            "Row 1's tilts are the shared base for every holder; rows 2-10 \
             add their own trim. Grip null writes X/Y/Z here by itself.",
        );
    }

    /// What the table last did — loaded, saved, or refused.
    pub fn note_line(&self, ui: &mut egui::Ui) {
        if !self.note.is_empty() {
            ui.label(&self.note);
        }
    }
}
