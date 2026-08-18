//! The D405 dual view: colour + depth side by side, images over
//! pvAccess by default.
//!
//! Depth (`pva://RS405:depthPva1:Image`, Z16 → `ushortValue`) renders
//! through [`RsdmImageView`]. Colour (`pva://RS405:Pva1:Image`) is RGB8
//! interleaved, which arrives as `PvValue::Bytes` — a form
//! `RsdmImageView` does not draw (and it has no RGB mode) — so a small
//! widget here turns each frame into an egui texture directly. That
//! also means no CC1 RGB→Mono rerouting, which the CA screen needed.
//! Camera control (Acquire/ImageMode/state) stays on `ca://`.

use eframe::egui;
use rsdm::widgets::{RsdmEnumComboBox, RsdmImageView, RsdmLabel, RsdmPushButton};
use rsdm::{Channel, Engine, EngineError, PvValue};
use rsplot::ColormapName;
use rsplot::egui_wgpu::RenderState;

use crate::pvs::cam;

/// D405 stream modes this camera actually offers, for sizing an
/// interleaved RGB buffer whose NTNDArray dimensions rsdm does not
/// surface. `pixels = len / 3`.
fn infer_dims(pixels: usize) -> Option<(usize, usize)> {
    for (w, h) in [(640, 480), (1280, 720), (848, 480), (424, 240), (320, 240)] {
        if w * h == pixels {
            return Some((w, h));
        }
    }
    None
}

/// Renders an interleaved-RGB8 NTNDArray channel as an egui texture.
struct RgbView {
    chan: Channel,
    tex: Option<egui::TextureHandle>,
    seen: u64,
    size: (usize, usize),
}

impl RgbView {
    fn new(engine: &Engine, address: &str) -> Result<Self, EngineError> {
        Ok(Self {
            chan: engine.connect(address)?,
            tex: None,
            seen: 0,
            size: (0, 0),
        })
    }

    fn show(&mut self, ui: &mut egui::Ui) {
        let state = self.chan.state();
        if state.stamp != self.seen {
            self.seen = state.stamp;
            if let Some(PvValue::Bytes(bytes)) = &state.value
                && bytes.len() % 3 == 0
                && let Some((w, h)) = infer_dims(bytes.len() / 3)
            {
                let image = egui::ColorImage::from_rgb([w, h], bytes);
                match &mut self.tex {
                    Some(tex) => tex.set(image, egui::TextureOptions::LINEAR),
                    None => {
                        self.tex = Some(ui.ctx().load_texture(
                            "d405-color",
                            image,
                            egui::TextureOptions::LINEAR,
                        ));
                    }
                }
                self.size = (w, h);
            }
        }
        match &self.tex {
            Some(tex) => {
                let (w, h) = self.size;
                let avail = ui.available_size();
                let scale = (avail.x / w as f32)
                    .min(avail.y / h as f32)
                    .clamp(0.05, 4.0);
                ui.add(
                    egui::Image::new(tex)
                        .fit_to_exact_size(egui::vec2(w as f32 * scale, h as f32 * scale)),
                );
            }
            None => {
                let text = if state.connected {
                    format!("{} — no frame yet", self.chan.address().raw())
                } else {
                    format!("{} — disconnected", self.chan.address().raw())
                };
                ui.colored_label(egui::Color32::GRAY, text);
            }
        }
    }
}

pub struct CameraPanel {
    color: RgbView,
    depth: RsdmImageView,
    acquire_start: RsdmPushButton,
    acquire_stop: RsdmPushButton,
    image_mode: RsdmEnumComboBox,
    det_state: RsdmLabel,
}

impl CameraPanel {
    pub fn new(engine: &Engine, rs: &RenderState) -> Result<Self, EngineError> {
        // The NTNDArray carries its dimensions, but rsdm's subfield
        // addressing cannot reach them; the D405 streams 640 wide in
        // every production mode, so the width is fixed rather than tied
        // to a CA plugin record the viewer otherwise no longer needs.
        let depth = RsdmImageView::new(engine, rs, 100, "pva://RS405:depthPva1:Image", None)?
            .with_width(640)
            .with_reading_order(rsdm::widgets::ReadingOrder::CLike)
            .with_colormap(ColormapName::Inferno)
            .with_normalize(true);
        Ok(Self {
            color: RgbView::new(engine, "pva://RS405:Pva1:Image")?,
            depth,
            acquire_start: RsdmPushButton::new(engine, &cam("cam1:Acquire"), "Start", "1")?,
            acquire_stop: RsdmPushButton::new(engine, &cam("cam1:Acquire"), "Stop", "0")?,
            image_mode: RsdmEnumComboBox::new(engine, &cam("cam1:ImageMode"))?,
            det_state: RsdmLabel::new(engine, &cam("cam1:DetectorState_RBV"))?,
        })
    }

    pub fn show(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            self.acquire_start.show(ui);
            self.acquire_stop.show(ui);
            ui.label("Mode:");
            self.image_mode.show(ui);
            ui.label("State:");
            self.det_state.show(ui);
        });
        ui.separator();
        let avail = ui.available_size();
        ui.horizontal_top(|ui| {
            ui.allocate_ui(egui::vec2(avail.x * 0.5 - 8.0, avail.y), |ui| {
                ui.vertical(|ui| {
                    ui.strong("Color (RGB, pva)");
                    self.color.show(ui);
                });
            });
            ui.allocate_ui(egui::vec2(avail.x * 0.5 - 8.0, avail.y), |ui| {
                ui.vertical(|ui| {
                    ui.strong("Depth (Z16, pva)");
                    self.depth.show(ui);
                });
            });
        });
    }
}
