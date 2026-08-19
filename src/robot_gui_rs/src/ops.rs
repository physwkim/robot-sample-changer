//! The operations panel: status readbacks, sample mount/return, the
//! measurement wait, pause, recover, grip null, and the advanced
//! (mode + start-step) trigger — the same operator surface as the
//! silx/PyQt panel, driven through rsdm channels.

use eframe::egui;
use rsdm::widgets::RsdmLabel;
use rsdm::{Channel, Engine, EngineError, PvValue};

use crate::pvs::{MODE_NAMES, robot, step_name};

/// An action that writes PVs. Everything that starts the sequencer goes
/// through here so the busy confirmation cannot be bypassed.
#[derive(Clone, Debug)]
enum Action {
    Mount(i64),
    Return(i64),
    GripNull { target: i64, source: i64 },
    Transfer { target: i64, source: i64 },
    Advanced { mode: i64, start: i64 },
    Recover,
}

impl Action {
    fn describe(&self) -> String {
        match self {
            Action::Mount(h) => format!("Mount holder {h} on the stage"),
            Action::Return(h) => format!("Return the sample to holder {h}"),
            Action::GripNull { target, source } => {
                let from = if *source == 0 || source == target {
                    String::new()
                } else {
                    format!(" (fetching holder {source}'s puck first)")
                };
                format!("Null the grip wrench at holder {target} and write its trims{from}")
            }
            Action::Transfer { target, source } => {
                format!("Move the puck from holder {source} to holder {target}")
            }
            Action::Advanced { mode, start } => {
                let name = MODE_NAMES.get(*mode as usize).copied().unwrap_or("?");
                format!("Trigger mode {mode} ({name}) from step {start}")
            }
            Action::Recover => "Recover to holder standby".to_string(),
        }
    }
}

pub struct OpsPanel {
    trigger: Channel,
    wait: Channel,
    calib_mode: Channel,
    start_step: Channel,
    holder: Channel,
    map_source: Channel,
    stop: Channel,
    pause_step: Channel,
    current_step: Channel,
    loaded: Channel,
    gripper: Channel,
    gripper_rbv: RsdmLabel,
    wrench: Channel,

    holder_sel: i64,
    null_holder: i64,
    null_source: i64,
    xfer_target: i64,
    xfer_src: i64,
    adv_mode: usize,
    adv_start: i64,
    pause_step_input: i64,
    pending: Option<Action>,
    note: String,
}

fn ival(ch: &Channel) -> Option<i64> {
    ch.state().value.as_ref().and_then(PvValue::as_i64)
}

/// The three components and their magnitude, monospaced so the columns
/// hold still while the numbers move.
fn triple(v: &[f64]) -> String {
    let mag = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    format!("{:+7.2} {:+7.2} {:+7.2}   |{:.2}|", v[0], v[1], v[2], mag)
}

impl OpsPanel {
    pub fn new(engine: &Engine) -> Result<Self, EngineError> {
        let ch = |suffix: &str| engine.connect(&robot(suffix));
        Ok(Self {
            trigger: ch("Trigger")?,
            wait: ch("Wait")?,
            calib_mode: ch("CalibMode")?,
            start_step: ch("StartStep")?,
            holder: ch("Holder")?,
            map_source: ch("MapSource")?,
            stop: ch("Stop")?,
            pause_step: ch("PauseStep")?,
            current_step: ch("CurrentStep")?,
            loaded: ch("Loaded")?,
            gripper: ch("Gripper")?,
            gripper_rbv: RsdmLabel::new(engine, &robot("Gripper_RBV"))?,
            // Served by ur-monitor-ioc off its own RTDE receive stream,
            // so reading it here costs the sequencer nothing.
            wrench: ch("UR:Receive:ActualTCPForce")?,
            holder_sel: 1,
            null_holder: 1,
            null_source: 0,
            xfer_target: 6,
            xfer_src: 7,
            adv_mode: 0,
            adv_start: 0,
            pause_step_input: 0,
            pending: None,
            note: String::new(),
        })
    }

    /// Runs the writes behind an action. The daemon resets `Wait` to 0
    /// at every run start, so a pre-set Continue would be lost — every
    /// trigger writes `Wait = 0` explicitly to keep the panel honest.
    fn execute(&mut self, action: &Action) {
        let put = |ch: &Channel, v: i64| ch.put(PvValue::Int(v));
        match *action {
            Action::Mount(h) => {
                put(&self.holder, h);
                put(&self.calib_mode, 0);
                put(&self.start_step, 0);
                put(&self.pause_step, 0);
                self.pause_step_input = 0;
                put(&self.wait, 0);
                put(&self.trigger, 1);
                self.note = format!("Mounting holder {h} on the stage...");
            }
            Action::Return(h) => {
                put(&self.holder, h);
                put(&self.calib_mode, 0);
                // From wherever the arm is, step 7 plans a collision-
                // checked move to stage standby; the daemon then waits at
                // step 12 — Continue there retrieves the sample.
                put(&self.start_step, 7);
                put(&self.pause_step, 0);
                self.pause_step_input = 0;
                put(&self.wait, 0);
                put(&self.trigger, 1);
                self.note = format!(
                    "Going to the stage — press Continue at the wait to retrieve to holder {h}"
                );
            }
            Action::GripNull { target, source } => {
                put(&self.holder, target);
                // 0 means the puck already seated in the target; anything
                // else is fetched first, on the same trigger.
                put(&self.map_source, if source == target { 0 } else { source });
                put(&self.calib_mode, 6);
                // Grip null refuses mid-sequence resumes.
                put(&self.start_step, 0);
                put(&self.pause_step, 0);
                self.pause_step_input = 0;
                put(&self.wait, 0);
                put(&self.trigger, 1);
                self.note = if source == 0 || source == target {
                    format!(
                        "Nulling holder {target} — it picks and reseats that holder's own \
                         puck once per iteration and writes the trims to \
                         taught_waypoints.yaml"
                    )
                } else {
                    format!(
                        "Fetching holder {source}'s puck into holder {target}, then nulling \
                         there — the puck is left in holder {target}"
                    )
                };
            }
            Action::Transfer { target, source } => {
                put(&self.holder, target);
                put(&self.map_source, source);
                put(&self.calib_mode, 7);
                // Holder transfer refuses mid-sequence resumes, like map.
                put(&self.start_step, 0);
                put(&self.pause_step, 0);
                self.pause_step_input = 0;
                put(&self.wait, 0);
                put(&self.trigger, 1);
                self.note = format!(
                    "Moving the puck from holder {source} to holder {target} — \
                     straight across, no stage leg and no probe"
                );
            }
            Action::Advanced { mode, start } => {
                put(&self.calib_mode, mode);
                put(&self.start_step, start);
                put(&self.wait, 0);
                put(&self.trigger, 1);
                self.note = format!("Triggered mode {mode} from step {start}");
            }
            Action::Recover => {
                put(&self.calib_mode, 4);
                put(&self.wait, 0);
                put(&self.trigger, 1);
                self.note = "Recovering to holder standby...".to_string();
            }
        }
    }

    /// A trigger while `CurrentStep > 0` either interleaves with a live
    /// run or restarts an interrupted one — never silent. Recover always
    /// confirms: it moves the arm.
    fn request(&mut self, action: Action) {
        let busy = ival(&self.current_step).unwrap_or(0) > 0;
        if busy || matches!(action, Action::Recover) {
            self.pending = Some(action);
        } else {
            self.execute(&action);
        }
    }

    fn confirm_modal(&mut self, ctx: &egui::Context) {
        let Some(action) = self.pending.clone() else {
            return;
        };
        let step = ival(&self.current_step).unwrap_or(0);
        let modal = egui::Modal::new(egui::Id::new("ops-confirm")).show(ctx, |ui| {
            ui.set_max_width(360.0);
            ui.heading("Confirm");
            if step > 0 {
                ui.label(format!(
                    "CurrentStep is {step} ({}): a sequence is running or was \
                     interrupted there.",
                    step_name(step)
                ));
            }
            if matches!(action, Action::Recover) {
                ui.label(
                    "Unlock a protective stop if any, resend the robot program, and \
                     walk the arm back to holder standby. The gripper is not touched.",
                );
            }
            ui.label(format!("{}?", action.describe()));
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                if ui.button("Go").clicked() {
                    self.execute(&action);
                    self.pending = None;
                }
                if ui.button("Cancel").clicked() {
                    self.pending = None;
                }
            });
        });
        if modal.should_close() {
            self.pending = None;
        }
    }

    fn status_grid(&mut self, ui: &mut egui::Ui) {
        let connected = self.current_step.state().connected;
        let step = ival(&self.current_step);
        let mode = ival(&self.calib_mode);
        let holder = ival(&self.holder);
        let loaded = ival(&self.loaded).unwrap_or(0) != 0;
        let paused = ival(&self.stop).unwrap_or(0) != 0;
        egui::Grid::new("status").striped(true).show(ui, |ui| {
            ui.label("EPICS:");
            if connected {
                ui.colored_label(egui::Color32::from_rgb(0x4c, 0xaf, 0x50), "connected");
            } else {
                ui.colored_label(egui::Color32::from_rgb(0xf4, 0x43, 0x36), "DISCONNECTED");
            }
            ui.end_row();
            ui.label("Step:");
            match step {
                Some(s) => ui.label(format!("{s}  {}", step_name(s))),
                None => ui.label("-"),
            };
            ui.end_row();
            ui.label("Mode:");
            ui.label(
                mode.and_then(|m| MODE_NAMES.get(m as usize).copied())
                    .unwrap_or("-"),
            );
            ui.end_row();
            ui.label("Holder:");
            ui.label(holder.map_or("-".to_string(), |h| h.to_string()));
            ui.end_row();
            ui.label("Sample:");
            ui.label(if loaded { "Loaded" } else { "Not loaded" });
            ui.end_row();
            ui.label("Motion:");
            ui.label(if paused { "PAUSED" } else { "run" });
            ui.end_row();
            ui.label("Gripper:");
            self.gripper_rbv.show(ui);
            ui.end_row();
            // Base frame, the frame the grip-null trims are written in:
            // x -> holder x trim, y -> z trim, z -> depth (y) trim.
            let w = self.wrench.state();
            let w = w.value.as_ref().and_then(PvValue::as_f64_slice);
            let w = w.filter(|v| v.len() >= 6);
            ui.label("Force (N):");
            match w {
                Some(v) => ui.monospace(triple(&v[0..3])),
                None => ui.label("-"),
            };
            ui.end_row();
            ui.label("Torque (Nm):");
            match w {
                Some(v) => ui.monospace(triple(&v[3..6])),
                None => ui.label("-"),
            };
            ui.end_row();
        });
    }

    pub fn show(&mut self, ui: &mut egui::Ui) {
        self.confirm_modal(ui.ctx());
        let step = ival(&self.current_step).unwrap_or(0);
        let waiting = step == 12;
        let paused = ival(&self.stop).unwrap_or(0) != 0;

        ui.horizontal_top(|ui| {
            ui.group(|ui| {
                ui.vertical(|ui| {
                    ui.strong("Status");
                    self.status_grid(ui);
                });
            });
            ui.group(|ui| {
                ui.vertical(|ui| {
                    ui.strong("Sample");
                    ui.horizontal(|ui| {
                        ui.label("Holder:");
                        ui.add(egui::DragValue::new(&mut self.holder_sel).range(1..=10));
                    });
                    if ui.button("Mount on stage").clicked() {
                        self.request(Action::Mount(self.holder_sel));
                    }
                    if ui.button("Return from stage").clicked() {
                        self.request(Action::Return(self.holder_sel));
                    }
                    ui.add_space(4.0);
                    ui.scope(|ui| {
                        if waiting {
                            ui.strong("Measurement wait:");
                        } else {
                            ui.label("Measurement wait: (at step 12)");
                        }
                        ui.add_enabled_ui(waiting, |ui| {
                            ui.horizontal(|ui| {
                                if ui.button("Continue").clicked() {
                                    self.wait.put(PvValue::Int(1));
                                    self.note = "Continuing — retrieving the sample...".into();
                                }
                                if ui.button("Abort").clicked() {
                                    self.wait.put(PvValue::Int(2));
                                    self.trigger.put(PvValue::Int(0));
                                    self.note =
                                        "Stopped at the wait — sample left on the stage".into();
                                }
                            });
                        });
                    });
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        if paused {
                            if ui.button("Resume").clicked() {
                                self.stop.put(PvValue::Int(0));
                                self.note = "Resumed".into();
                            }
                        } else if ui.button("Pause").clicked() {
                            self.stop.put(PvValue::Int(1));
                            self.note = "Pause requested — stops after the current step".into();
                        }
                        if ui.button("Recover").clicked() {
                            self.request(Action::Recover);
                        }
                    });
                });
            });
        });

        ui.add_space(6.0);
        ui.horizontal_top(|ui| {
            ui.group(|ui| {
                ui.vertical(|ui| {
                    ui.strong("Grip null");
                    ui.label("Close on the puck, write the trims the wrench asks for.");
                    ui.horizontal(|ui| {
                        ui.label("Holder:");
                        ui.add(egui::DragValue::new(&mut self.null_holder).range(1..=10));
                        ui.label("Puck from:");
                        ui.add(egui::DragValue::new(&mut self.null_source).range(0..=10));
                    });
                    let own = self.null_source == 0 || self.null_source == self.null_holder;
                    ui.label(if own {
                        "(0 = the puck already in that holder)"
                    } else {
                        "(fetched first; the target seat must be empty)"
                    });
                    if ui.button("Null grip").clicked() {
                        self.request(Action::GripNull {
                            target: self.null_holder,
                            source: self.null_source,
                        });
                    }
                });
            });
            ui.group(|ui| {
                ui.vertical(|ui| {
                    ui.strong("Move puck");
                    ui.label("Holder to holder, no stage leg.");
                    ui.horizontal(|ui| {
                        ui.label("From:");
                        ui.add(egui::DragValue::new(&mut self.xfer_src).range(1..=10));
                        ui.label("To:");
                        ui.add(egui::DragValue::new(&mut self.xfer_target).range(1..=10));
                    });
                    let same = self.xfer_src == self.xfer_target;
                    ui.label(if same {
                        "(pick two different holders)"
                    } else {
                        "(the seat is not probed)"
                    });
                    if ui
                        .add_enabled(!same, egui::Button::new("Move puck"))
                        .clicked()
                    {
                        self.request(Action::Transfer {
                            target: self.xfer_target,
                            source: self.xfer_src,
                        });
                    }
                });
            });
            ui.group(|ui| {
                ui.vertical(|ui| {
                    ui.strong("Gripper");
                    ui.horizontal(|ui| {
                        if ui.button("Open").clicked() {
                            self.gripper.put(PvValue::Int(1));
                        }
                        if ui.button("Close").clicked() {
                            self.gripper.put(PvValue::Int(0));
                        }
                    });
                });
            });
            ui.group(|ui| {
                ui.vertical(|ui| {
                    ui.strong("Advanced");
                    ui.horizontal(|ui| {
                        ui.label("Mode:");
                        egui::ComboBox::from_id_salt("adv-mode")
                            .selected_text(MODE_NAMES[self.adv_mode])
                            .show_ui(ui, |ui| {
                                for (i, name) in MODE_NAMES.iter().enumerate() {
                                    ui.selectable_value(&mut self.adv_mode, i, *name);
                                }
                            });
                    });
                    ui.horizontal(|ui| {
                        ui.label("Start step:");
                        ui.add(egui::DragValue::new(&mut self.adv_start).range(0..=23));
                        if ui.button("Trigger").clicked() {
                            self.request(Action::Advanced {
                                mode: self.adv_mode as i64,
                                start: self.adv_start,
                            });
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Pause at step:");
                        ui.add(egui::DragValue::new(&mut self.pause_step_input).range(0..=23));
                        if ui.button("Set").clicked() {
                            self.pause_step.put(PvValue::Int(self.pause_step_input));
                        }
                        ui.label(format!("(now {})", ival(&self.pause_step).unwrap_or(0)));
                    });
                });
            });
        });

        if !self.note.is_empty() {
            ui.add_space(4.0);
            ui.label(&self.note);
        }
    }
}
