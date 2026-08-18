//! Calibration: TCP jog during calibration holds, and the offsets/tilt
//! table over `taught_waypoints.yaml` with edited-cells-only saving.

use std::path::PathBuf;

use eframe::egui;
use rsdm::{Channel, Engine, EngineError, PvValue};

use crate::pvs::robot;
use crate::yamledit::{self, Slot};

const COLS: usize = 5; // X mm, Y mm, Z mm, Tilt X deg, Tilt Z deg
const ROWS: usize = 11; // stage + holders 1-10

/// Display-unit cell values (mm for 0-2, deg for 3-4); `None` = the
/// parameter does not exist for that row (stage tilt).
type Cells = [[Option<f64>; COLS]; ROWS];

fn slot_for(row: usize, col: usize) -> Option<Slot> {
    // Row 0 = stage (sample holder), row 1 = holder 1 + the shared tilt
    // bases, rows 2-10 = holder N at list index N-2.
    match (row, col) {
        (0, 0) => Some(Slot::Scalar("sample_holder_on_position_x_offset")),
        (0, 1) => Some(Slot::Scalar("sample_holder_on_position_y_offset")),
        (0, 2) => Some(Slot::Scalar("sample_holder_on_position_z_offset")),
        (0, _) => None,
        (1, 0) => Some(Slot::Scalar("holder1_on_position_x_offset")),
        (1, 1) => Some(Slot::Scalar("holder1_on_position_y_offset")),
        (1, 2) => Some(Slot::Scalar("holder1_on_position_z_offset")),
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

fn row_name(row: usize) -> String {
    match row {
        0 => "Stage".to_string(),
        n => format!("Holder {n}"),
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
    cells[1][0] = Some(scalar("holder1_on_position_x_offset") * 1000.0);
    cells[1][1] = Some(scalar("holder1_on_position_y_offset") * 1000.0);
    cells[1][2] = Some(scalar("holder1_on_position_z_offset") * 1000.0);
    cells[1][3] = Some(scalar("holder_on_position_tilt_x_deg"));
    cells[1][4] = Some(scalar("holder_on_position_tilt_z_deg"));
    let lists = [
        yamledit::vec_at(&p, "holder_multi_x_offsets", 9),
        yamledit::vec_at(&p, "holder_multi_y_offsets", 9),
        yamledit::vec_at(&p, "holder_multi_z_offsets", 9),
        yamledit::vec_at(&p, "holder_multi_tilt_x_deg", 9),
        yamledit::vec_at(&p, "holder_multi_tilt_z_deg", 9),
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
    step_mm: f64,
    yaml_path: PathBuf,
    cells: Cells,
    loaded: Cells,
    have_table: bool,
    note: String,
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
    /// trim the daemon's holder map wrote since our load survives.
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

    fn jog_ui(&mut self, ui: &mut egui::Ui) {
        ui.strong("TCP jog (only serviced during a calibration hold)");
        ui.horizontal(|ui| {
            ui.label("Step (mm):");
            ui.add(
                egui::DragValue::new(&mut self.step_mm)
                    .range(0.01..=5.0)
                    .speed(0.05)
                    .fixed_decimals(2),
            );
        });
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
    }

    fn table_ui(&mut self, ui: &mut egui::Ui) {
        ui.strong("Seat offsets and tilts (taught_waypoints.yaml)");
        if !self.have_table {
            ui.colored_label(egui::Color32::from_rgb(0xf4, 0x43, 0x36), &self.note);
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
             add their own trim. Holder map writes X/Z here by itself.",
        );
    }

    pub fn show(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_top(|ui| {
            ui.group(|ui| self.jog_ui(ui));
            ui.group(|ui| {
                ui.vertical(|ui| self.table_ui(ui));
            });
        });
        if !self.note.is_empty() {
            ui.label(&self.note);
        }
    }
}
