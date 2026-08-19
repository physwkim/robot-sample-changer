//! The D405 dual view: colour and depth side by side, both over
//! pvAccess.
//!
//! Both streams go through one [`ImagePane`], so zoom, pan and the
//! cursor readout behave identically on either image. Depth
//! (`pva://RS405:depthPva1:Image`, Z16) is colour-mapped here against a
//! range the operator sets; colour (`pva://RS405:Pva1:Image`) is RGB8
//! interleaved and becomes a texture as-is. Camera control
//! (Acquire/ImageMode/state) stays on `ca://`.
//!
//! This replaced a silx-style `RsdmImageView` on the depth side. Its
//! side histograms and profile tool are not what this camera is used
//! for — the questions asked of it are "how far is that pixel" and
//! "what is in this corner", which want a readout and a zoom.

use eframe::egui;
use rsdm::widgets::{RsdmEnumComboBox, RsdmLabel, RsdmPushButton, value_to_image};
use rsdm::{Channel, Engine, EngineError, PvValue};
use rsplot::{Colormap, ColormapName};

use crate::pvs::cam;

/// D405 stream modes this camera actually offers, for sizing a frame
/// whose NTNDArray dimensions rsdm's subfield addressing cannot reach.
fn infer_dims(pixels: usize) -> Option<(usize, usize)> {
    for (w, h) in [(640, 480), (1280, 720), (848, 480), (424, 240), (320, 240)] {
        if w * h == pixels {
            return Some((w, h));
        }
    }
    None
}

/// What a pane is looking at, which decides how a frame becomes a
/// texture and what the cursor readout says about a pixel.
enum Kind {
    /// Interleaved RGB8: three bytes a pixel, drawn as they arrive.
    Colour,
    /// Single-channel depth counts, colour-mapped against `range`.
    /// `mm_per_count` comes from the detector rather than a constant —
    /// `RSDepthUnits_RBV` is metres per count and the D405 ships 0.0001.
    Depth {
        range: (f32, f32),
        auto: bool,
        mm_per_count: f64,
    },
}

/// One image: zoom, pan, and a readout of the pixel under the cursor.
///
/// `origin`/`scale` are `None` until the view is touched, which is what
/// "fit to the panel" means — the fit is recomputed every frame while
/// nobody has zoomed, so a resized window keeps filling.
struct ImagePane {
    chan: Channel,
    id: &'static str,
    kind: Kind,
    tex: Option<egui::TextureHandle>,
    seen: u64,
    dims: (usize, usize),
    /// The frame behind the texture, kept for the cursor readout.
    counts: Vec<f32>,
    rgb: Vec<u8>,
    origin: Option<egui::Pos2>,
    scale: Option<f32>,
    /// Set when the colour range changes under a frame that has already
    /// arrived, so the texture is rebuilt without waiting for the next.
    restage: bool,
}

impl ImagePane {
    fn new(
        engine: &Engine,
        address: &str,
        id: &'static str,
        kind: Kind,
    ) -> Result<Self, EngineError> {
        Ok(Self {
            chan: engine.connect(address)?,
            id,
            kind,
            tex: None,
            seen: 0,
            dims: (0, 0),
            counts: Vec::new(),
            rgb: Vec::new(),
            origin: None,
            scale: None,
            restage: false,
        })
    }

    /// Back to filling the panel.
    fn fit(&mut self) {
        self.origin = None;
        self.scale = None;
    }

    fn take_frame(&mut self, ui: &egui::Ui) {
        let state = self.chan.state();
        let fresh = state.stamp != self.seen;
        if !fresh && !self.restage {
            return;
        }
        self.restage = false;
        if fresh {
            self.seen = state.stamp;
            match (&self.kind, state.value.as_ref()) {
                (Kind::Colour, Some(PvValue::Bytes(bytes))) if bytes.len() % 3 == 0 => {
                    let Some(dims) = infer_dims(bytes.len() / 3) else {
                        return;
                    };
                    self.dims = dims;
                    self.rgb = bytes.to_vec();
                }
                (Kind::Depth { .. }, Some(value)) => {
                    let Some(counts) = value_to_image(value) else {
                        return;
                    };
                    let Some(dims) = infer_dims(counts.len()) else {
                        return;
                    };
                    self.dims = dims;
                    self.counts = counts;
                }
                _ => return,
            }
        }
        let (w, h) = self.dims;
        if w == 0 {
            return;
        }
        let image = match &self.kind {
            Kind::Colour => egui::ColorImage::from_rgb([w, h], &self.rgb),
            Kind::Depth { range, auto, .. } => {
                let (lo, hi) = if *auto {
                    auto_range(&self.counts)
                } else {
                    (f64::from(range.0), f64::from(range.1))
                };
                let cm = Colormap::new(ColormapName::Inferno, lo, hi.max(lo + 1.0));
                let mut rgb = Vec::with_capacity(w * h * 3);
                for v in &self.counts {
                    // Zero is "no return" on a stereo depth camera, not
                    // a near surface; painting it black keeps the holes
                    // from reading as the closest thing in frame.
                    let c = if *v == 0.0 {
                        [0, 0, 0, 255]
                    } else {
                        cm.color_at(f64::from(*v))
                    };
                    rgb.extend_from_slice(&c[..3]);
                }
                egui::ColorImage::from_rgb([w, h], &rgb)
            }
        };
        match &mut self.tex {
            Some(tex) => tex.set(image, egui::TextureOptions::NEAREST),
            None => {
                self.tex = Some(ui.ctx().load_texture(
                    self.id,
                    image,
                    egui::TextureOptions::NEAREST,
                ));
            }
        }
    }

    /// What the pixel under `pos` is, as one line.
    fn readout(&self, pixel: (usize, usize)) -> String {
        let (x, y) = pixel;
        let i = y * self.dims.0 + x;
        match &self.kind {
            Kind::Colour => match self.rgb.get(i * 3..i * 3 + 3) {
                Some(c) => format!("({x}, {y})  R{:3} G{:3} B{:3}", c[0], c[1], c[2]),
                None => format!("({x}, {y})"),
            },
            Kind::Depth { mm_per_count, .. } => match self.counts.get(i) {
                Some(v) if *v == 0.0 => format!("({x}, {y})  no return"),
                Some(v) => format!(
                    "({x}, {y})  {:.1} mm  ({v:.0} counts)",
                    f64::from(*v) * mm_per_count
                ),
                None => format!("({x}, {y})"),
            },
        }
    }

    fn show(&mut self, ui: &mut egui::Ui) {
        self.take_frame(ui);
        let (w, h) = (self.dims.0 as f32, self.dims.1 as f32);
        let rect = ui.available_rect_before_wrap();
        let (rect, resp) = ui.allocate_exact_size(rect.size(), egui::Sense::click_and_drag());
        let Some(tex) = &self.tex else {
            let state = self.chan.state();
            let text = if state.connected {
                format!("{} — no frame yet", self.chan.address().raw())
            } else {
                format!("{} — disconnected", self.chan.address().raw())
            };
            ui.painter().text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                text,
                egui::FontId::proportional(12.0),
                egui::Color32::GRAY,
            );
            return;
        };

        let fit = (rect.width() / w).min(rect.height() / h);
        let mut scale = self.scale.unwrap_or(fit);
        let mut origin = self
            .origin
            .unwrap_or_else(|| rect.center() - egui::vec2(w * scale, h * scale) / 2.0);

        if resp.dragged() {
            origin += resp.drag_delta();
        }
        // Wheel zoom about the cursor: the pixel under the pointer is
        // the one the operator is asking about, so it stays put.
        if let Some(pointer) = resp.hover_pos() {
            let wheel = ui.input(|i| i.smooth_scroll_delta.y);
            if wheel.abs() > 0.1 {
                let factor = (wheel * 0.004).exp().clamp(0.5, 2.0);
                let next = (scale * factor).clamp(fit * 0.5, fit * 32.0);
                let at = (pointer - origin) / scale;
                origin = pointer - at * next;
                scale = next;
            }
        }
        if resp.dragged() || resp.hovered() {
            self.scale = Some(scale);
            self.origin = Some(origin);
        }

        let painter = ui.painter_at(rect);
        let drawn = egui::Rect::from_min_size(origin, egui::vec2(w * scale, h * scale));
        painter.image(
            tex.id(),
            drawn,
            egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
            egui::Color32::WHITE,
        );
        painter.rect_stroke(
            rect,
            0.0,
            egui::Stroke::new(1.0, ui.visuals().weak_text_color()),
            egui::StrokeKind::Inside,
        );

        if let Some(pointer) = resp.hover_pos() {
            let at = (pointer - origin) / scale;
            if at.x >= 0.0 && at.y >= 0.0 && at.x < w && at.y < h {
                let text = self.readout((at.x as usize, at.y as usize));
                resp.on_hover_ui(|ui| {
                    ui.monospace(text);
                });
            }
        }
    }

    /// The controls that belong to this pane, on one line.
    fn controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            if ui.button("Fit").clicked() {
                self.fit();
            }
            if let Kind::Depth { range, auto, .. } = &mut self.kind {
                let before = (*auto, *range);
                ui.checkbox(auto, "Auto scale");
                ui.add_enabled_ui(!*auto, |ui| {
                    ui.label("counts:");
                    ui.add(
                        egui::DragValue::new(&mut range.0)
                            .range(0.0..=65535.0)
                            .speed(10.0),
                    );
                    ui.label("to");
                    ui.add(
                        egui::DragValue::new(&mut range.1)
                            .range(0.0..=65535.0)
                            .speed(10.0),
                    );
                });
                if before != (*auto, *range) {
                    self.restage = true;
                }
            }
        });
    }
}

/// The 2nd-to-98th percentile of the returns, so a few stray far pixels
/// do not flatten the whole scene into one colour.
fn auto_range(counts: &[f32]) -> (f64, f64) {
    let mut live: Vec<f32> = counts.iter().copied().filter(|v| *v > 0.0).collect();
    if live.is_empty() {
        return (0.0, 1.0);
    }
    live.sort_by(f32::total_cmp);
    let lo = live[live.len() * 2 / 100];
    let hi = live[(live.len() * 98 / 100).min(live.len() - 1)];
    (f64::from(lo), f64::from(hi.max(lo + 1.0)))
}

pub struct CameraPanel {
    colour: ImagePane,
    depth: ImagePane,
    depth_units: Channel,
    acquire_start: RsdmPushButton,
    acquire_stop: RsdmPushButton,
    image_mode: RsdmEnumComboBox,
    det_state: RsdmLabel,
}

impl CameraPanel {
    pub fn new(engine: &Engine) -> Result<Self, EngineError> {
        Ok(Self {
            colour: ImagePane::new(
                engine,
                "pva://RS405:Pva1:Image",
                "d405-colour",
                Kind::Colour,
            )?,
            depth: ImagePane::new(
                engine,
                "pva://RS405:depthPva1:Image",
                "d405-depth",
                Kind::Depth {
                    range: (0.0, 4000.0),
                    auto: true,
                    mm_per_count: 0.1,
                },
            )?,
            depth_units: engine.connect(&cam("cam1:RSDepthUnits_RBV"))?,
            acquire_start: RsdmPushButton::new(engine, &cam("cam1:Acquire"), "Start", "1")?,
            acquire_stop: RsdmPushButton::new(engine, &cam("cam1:Acquire"), "Stop", "0")?,
            image_mode: RsdmEnumComboBox::new(engine, &cam("cam1:ImageMode"))?,
            det_state: RsdmLabel::new(engine, &cam("cam1:DetectorState_RBV"))?,
        })
    }

    pub fn show(&mut self, ui: &mut egui::Ui) {
        // The detector reports metres per count; the readout is in mm.
        if let Some(m) = self
            .depth_units
            .state()
            .value
            .as_ref()
            .and_then(PvValue::as_f64)
            && m > 0.0
            && let Kind::Depth { mm_per_count, .. } = &mut self.depth.kind
        {
            *mm_per_count = m * 1000.0;
        }
        ui.horizontal(|ui| {
            self.acquire_start.show(ui);
            self.acquire_stop.show(ui);
            ui.separator();
            ui.label("Mode:");
            self.image_mode.show(ui);
            ui.separator();
            ui.label("State:");
            self.det_state.show(ui);
            ui.separator();
            ui.label("Scroll to zoom, drag to pan, hover for the value.");
        });
        ui.separator();
        let full = ui.available_rect_before_wrap();
        let half = (full.width() - 12.0) * 0.5;
        ui.horizontal_top(|ui| {
            ui.allocate_ui(egui::vec2(half, full.height()), |ui| {
                ui.vertical(|ui| {
                    ui.strong("Colour (RGB8)");
                    self.colour.controls(ui);
                    self.colour.show(ui);
                });
            });
            ui.separator();
            ui.allocate_ui(egui::vec2(half, full.height()), |ui| {
                ui.vertical(|ui| {
                    ui.strong("Depth (Z16)");
                    self.depth.controls(ui);
                    self.depth.show(ui);
                });
            });
        });
    }
}
