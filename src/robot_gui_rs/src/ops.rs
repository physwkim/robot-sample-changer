//! The operations panel: status readbacks, sample mount/return, the
//! measurement wait, pause, recover, grip null, and the advanced
//! (mode + start-step) trigger — the same operator surface as the
//! silx/PyQt panel, driven through rsdm channels.

use eframe::egui;
use rsdm::widgets::RsdmLabel;
use rsdm::{Channel, Engine, EngineError, PvValue};

use crate::daemon::{DaemonWatch, HOLD, MEASUREMENT_WAIT, Tone};
use crate::pvs::{MODE_NAMES, robot, seat_state_name, step_name};

/// An action that writes PVs. Everything that starts the sequencer goes
/// through here so the busy confirmation cannot be bypassed.
#[derive(Clone, Debug)]
enum Action {
    Mount(i64),
    Return(i64),
    GripNull {
        target: i64,
        source: i64,
    },
    Transfer {
        target: i64,
        source: i64,
    },
    /// A calibration hold: pick `holder`'s puck and stand where the jog
    /// can measure a seat -- above that holder (mode 1) or above the
    /// stage bore (mode 2, carrying the puck there).
    Hold {
        holder: i64,
        at_stage: bool,
    },
    Advanced {
        mode: i64,
        start: i64,
    },
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
                format!(
                    "Null the grip wrench at {} and write its trims{from}",
                    seat_name(*target)
                )
            }
            Action::Transfer { target, source } => {
                format!("Move the puck from holder {source} to holder {target}")
            }
            Action::Hold { holder, at_stage } => {
                if *at_stage {
                    format!("Carry holder {holder}'s puck to the stage and hold there for jogging")
                } else {
                    format!("Pick holder {holder}'s puck and hold above that seat for jogging")
                }
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
    /// The camera seat check's on/off record. Not gated on the daemon
    /// answering: it is a record the daemon reads at each gate, not a
    /// command handshake, so it can be flipped while the arm moves and
    /// takes effect at the next seat.
    seat_check: Channel,
    current_step: Channel,
    loaded: Channel,
    gripper: Channel,
    gripper_rbv: RsdmLabel,
    /// The seat the daemon says a hold is standing at, empty when none
    /// is. `CalibPanel` reads it too, for the Apply button; a second
    /// subscription to one PV costs a channel and keeps both panels
    /// able to answer the question without asking the other.
    jog_target: Channel,
    /// The daemon's one-line account of itself: the step it is on, the
    /// wait it is holding in, or why the last run stopped. Prose, not a
    /// gate -- nothing here is enabled or disabled from it.
    status: Channel,
    /// Where the pucks are, `seats[0]` the stage and `seats[h]` holder
    /// `h`, as the daemon last saw them (0=unknown, 1=empty,
    /// 2=occupied).
    seats: [Channel; 11],
    /// The seat check's own last line: the reading behind a verdict, or
    /// why there is no verdict -- switched off, out of frame, too few
    /// pixels. Until now that only existed in the daemon's terminal.
    seat_msg: Channel,
    wrench: Channel,
    null_state: Channel,
    null_iter: Channel,
    null_d: [Channel; 3],
    null_force: Channel,
    null_msg: Channel,
    /// What the daemon says it is doing, and whether it is still
    /// saying it. Every control that needs the daemon to be listening
    /// is enabled from here and never from a run value -- see
    /// [`crate::daemon`].
    daemon: DaemonWatch,

    holder_sel: i64,
    null_holder: i64,
    null_source: i64,
    xfer_target: i64,
    xfer_src: i64,
    hold_holder: i64,
    adv_mode: usize,
    adv_start: i64,
    pause_step_input: i64,
    pending: Option<Action>,
    note: String,
}

fn ival(ch: &Channel) -> Option<i64> {
    ch.state().value.as_ref().and_then(PvValue::as_i64)
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

const GREEN: egui::Color32 = egui::Color32::from_rgb(0x4c, 0xaf, 0x50);
const RED: egui::Color32 = egui::Color32::from_rgb(0xf4, 0x43, 0x36);
const AMBER: egui::Color32 = egui::Color32::from_rgb(0xff, 0xb3, 0x00);
/// The seat chips' two quiet backgrounds and the text on the dark one:
/// a seat that has been looked at and is empty, and one that has not.
const GRAY: egui::Color32 = egui::Color32::from_gray(0x55);
const DARK: egui::Color32 = egui::Color32::from_gray(0x28);
const DIM: egui::Color32 = egui::Color32::from_gray(0x88);

/// Width of every card's label column. Set here rather than left to the
/// content so the fields line up from one card to the next, not just
/// within one.
const LABEL_W: f32 = 92.0;

/// A card's label/field rows.
/// What a seat number is called: `Robot:Holder = 0` is the stage bore,
/// 1-10 the rack wells.
fn seat_name(n: i64) -> String {
    if n == 0 {
        "stage".into()
    } else {
        format!("holder {n}")
    }
}

fn fields(ui: &mut egui::Ui, id: &str, add: impl FnOnce(&mut egui::Ui)) {
    egui::Grid::new(id)
        .num_columns(2)
        .min_col_width(LABEL_W)
        .show(ui, add);
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
            seat_check: ch("SeatCheck")?,
            current_step: ch("CurrentStep")?,
            loaded: ch("Loaded")?,
            gripper: ch("Gripper")?,
            gripper_rbv: RsdmLabel::new(engine, &robot("Gripper_RBV"))?,
            jog_target: ch("Jog:Target")?,
            status: ch("Status")?,
            seats: [
                ch("Seat:Stage")?,
                ch("Seat:H1")?,
                ch("Seat:H2")?,
                ch("Seat:H3")?,
                ch("Seat:H4")?,
                ch("Seat:H5")?,
                ch("Seat:H6")?,
                ch("Seat:H7")?,
                ch("Seat:H8")?,
                ch("Seat:H9")?,
                ch("Seat:H10")?,
            ],
            seat_msg: ch("Seat:Msg")?,
            // Served by ur-monitor-ioc off its own RTDE receive stream,
            // so reading it here costs the sequencer nothing.
            wrench: ch("UR:Receive:ActualTCPForce")?,
            null_state: ch("Null:State")?,
            null_iter: ch("Null:Iter")?,
            null_d: [ch("Null:DX")?, ch("Null:DY")?, ch("Null:DZ")?],
            null_force: ch("Null:Force")?,
            null_msg: ch("Null:Msg")?,
            daemon: DaemonWatch::new(engine)?,
            holder_sel: 1,
            null_holder: 1,
            null_source: 0,
            xfer_target: 6,
            xfer_src: 7,
            hold_holder: 1,
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
                // Step 13 is the first step of the retrieval leg, and
                // the daemon plans its way to stage standby before
                // running it. Entering at 7 instead would re-run the
                // place leg — descend on the seat, open on nothing —
                // and then wait for a measurement that is already over.
                put(&self.start_step, 13);
                put(&self.pause_step, 0);
                self.pause_step_input = 0;
                put(&self.wait, 0);
                put(&self.trigger, 1);
                self.note = format!("Retrieving the sample from the stage back to holder {h}...");
            }
            Action::GripNull { target, source } => {
                put(&self.holder, target);
                // 0 means the puck already seated in the target; anything
                // else is a rack holder the daemon fetches from first, on
                // the same trigger, the stage included.
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
                        "Nulling {} — it picks and reseats that seat's own puck once per \
                         iteration and writes the trims to taught_waypoints.yaml",
                        seat_name(target)
                    )
                } else {
                    format!(
                        "Fetching holder {source}'s puck into {seat}, then nulling \
                         there — the puck is left in {seat}",
                        seat = seat_name(target)
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
            Action::Hold { holder, at_stage } => {
                put(&self.holder, holder);
                put(&self.calib_mode, if at_stage { 2 } else { 1 });
                // Both modes honour StartStep for their skip logic, so a
                // value left over from a resume would drop the pick and
                // hold with an empty gripper.
                put(&self.start_step, 0);
                put(&self.pause_step, 0);
                self.pause_step_input = 0;
                put(&self.trigger, 1);
                self.note = if at_stage {
                    format!(
                        "Carrying holder {holder}'s puck to the stage — jog there, Apply,                          then end the hold to put it back"
                    )
                } else {
                    format!(
                        "Picking holder {holder}'s puck — jog above that seat, Apply,                          then end the hold to put it back"
                    )
                };
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
        egui::Grid::new("status")
            .striped(true)
            .min_col_width(LABEL_W)
            .show(ui, |ui| {
                ui.label("EPICS:");
                if connected {
                    ui.colored_label(GREEN, "connected");
                } else {
                    ui.colored_label(RED, "DISCONNECTED");
                }
                ui.end_row();
                ui.label("Daemon:");
                let (say, tone) = self.daemon.status();
                ui.colored_label(
                    match tone {
                        Tone::Good => GREEN,
                        Tone::Busy => AMBER,
                        Tone::Bad => RED,
                    },
                    say,
                );
                ui.end_row();
                // The daemon's own words. The row above says whether it
                // is listening; this one says what it is doing, and
                // after a run that stopped it is the only place on the
                // screen that says why.
                ui.label("Doing:");
                match sval(&self.status).filter(|s| !s.is_empty()) {
                    Some(line) => ui.label(egui::RichText::new(line).italics()),
                    None => ui.label("-"),
                };
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
                ui.label("Seat:");
                ui.label(holder.map_or("-".to_string(), seat_name));
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
                // Base frame, as the RTDE stream reports it. The grip
                // null writes tool-frame trims and rotates this itself;
                // at a rack seat that comes out x -> x trim, y -> z
                // trim, z -> depth (y) trim, and at the stage it does
                // not, which is why the daemon does the turning.
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

    /// The pending-action confirmation. Drawn once a frame, before any
    /// group, because it is a modal over the whole page.
    pub fn begin(&mut self, ctx: &egui::Context) {
        self.confirm_modal(ctx);
    }

    /// What the robot is doing and what the tool feels.
    pub fn status_group(&mut self, ui: &mut egui::Ui) {
        ui.vertical(|ui| {
            ui.strong("Status");
            self.status_grid(ui);
        });
    }

    /// Where the pucks are, as far as anything has looked.
    ///
    /// Two things fill this in and both are observations: the camera
    /// seat check reading the seat the arm is about to enter, and the
    /// sequence itself, which knows a well is empty once it has lifted
    /// a puck out of it and full once it has released one into it. A
    /// seat nothing has visited since the IOC came up reads as not
    /// looked at, and says so rather than guessing from a taught map.
    pub fn seats_group(&mut self, ui: &mut egui::Ui) {
        ui.vertical(|ui| {
            ui.strong("Seats");
            ui.horizontal_wrapped(|ui| {
                for (index, channel) in self.seats.iter().enumerate() {
                    let name = if index == 0 {
                        "stage".to_string()
                    } else {
                        format!("{index}")
                    };
                    let live = channel.state().connected;
                    let value = ival(channel).unwrap_or(0);
                    let (fg, bg) = match value {
                        2 if live => (egui::Color32::BLACK, GREEN),
                        1 if live => (egui::Color32::WHITE, GRAY),
                        _ => (DIM, DARK),
                    };
                    let tip = if live {
                        seat_state_name(value)
                    } else {
                        "no Robot:Seat: record — restart the IOC"
                    };
                    ui.label(
                        egui::RichText::new(format!(" {name} "))
                            .monospace()
                            .color(fg)
                            .background_color(bg),
                    )
                    .on_hover_text(tip);
                }
            });
            ui.label(
                egui::RichText::new(
                    "green = puck, grey = empty, dark = nothing has looked there yet",
                )
                .small(),
            );
            // The reading behind the chips, including the answers that
            // never become a chip: the check switched off, the seat out
            // of frame, too few pixels to say either way.
            match sval(&self.seat_msg).filter(|s| !s.is_empty()) {
                Some(line) => ui.label(egui::RichText::new(format!("last check — {line}")).small()),
                None => ui.label(egui::RichText::new("last check — none yet").small()),
            };
        });
    }

    /// The grip null's progress and its result — the numbers the daemon
    /// writes to `Robot:Null:`, which is the only place a finished run
    /// says whether it worked.
    pub fn null_status_group(&mut self, ui: &mut egui::Ui) {
        ui.vertical(|ui| {
            ui.strong("Grip null result");
            let (label, color) = match ival(&self.null_state).unwrap_or(0) {
                1 => ("running", AMBER),
                2 => ("SETTLED", GREEN),
                3 => ("FAILED", RED),
                _ => ("idle", ui.visuals().weak_text_color()),
            };
            egui::Grid::new("nullstatus")
                .striped(true)
                .min_col_width(LABEL_W)
                .show(ui, |ui| {
                    ui.label("Result:");
                    ui.colored_label(color, label);
                    ui.end_row();
                    ui.label("Iteration:");
                    ui.label(ival(&self.null_iter).map_or("-".into(), |i| i.to_string()));
                    ui.end_row();
                    ui.label("Correction:");
                    let d: Vec<Option<f64>> = self.null_d.iter().map(fval).collect();
                    match (d[0], d[1], d[2]) {
                        (Some(x), Some(y), Some(z)) => {
                            ui.monospace(format!("{x:+7.3} {y:+7.3} {z:+7.3} mm"))
                        }
                        _ => ui.label("-"),
                    };
                    ui.end_row();
                    ui.label("Close wrench:");
                    match fval(&self.null_force) {
                        Some(f) => ui.label(format!("{f:.2} N")),
                        None => ui.label("-"),
                    };
                    ui.end_row();
                });
            ui.label(
                "(correction is tool x, tool y (depth, never steered), tool z — \
                 the trim columns X, Y, Z)",
            );
            let msg = sval(&self.null_msg).unwrap_or_default();
            if !msg.is_empty() {
                ui.label(egui::RichText::new(msg).italics());
            }
        });
    }

    /// The stage errand, and the controls that interrupt whatever is
    /// running.
    pub fn sample_group(&mut self, ui: &mut egui::Ui) {
        // The daemon saying it is in `wait_for_measurement` and
        // reading `Robot:Wait` this instant. The old test was
        // `CurrentStep == 12`, which is also true of a daemon that died
        // at step 12, of an IOC that restored 12 from autosave, and of
        // a carry that merely passed through it -- in all of which
        // Continue wrote a PV nobody was reading.
        let waiting = self.daemon.is(MEASUREMENT_WAIT);
        let paused = ival(&self.stop).unwrap_or(0) != 0;
        ui.vertical(|ui| {
            ui.strong("Sample");
            fields(ui, "sample-fields", |ui| {
                ui.label("Holder:");
                ui.add(egui::DragValue::new(&mut self.holder_sel).range(1..=10));
                ui.end_row();
            });
            if ui.button("Mount on stage").clicked() {
                self.request(Action::Mount(self.holder_sel));
            }
            if ui.button("Return from stage").clicked() {
                self.request(Action::Return(self.holder_sel));
            }
            ui.add_space(4.0);
            if waiting {
                ui.strong("Measurement wait:");
            } else {
                ui.label(format!("Measurement wait: ({})", self.daemon.note()));
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
                        self.note = "Stopped at the wait — sample left on the stage".into();
                    }
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
    }

    pub fn grip_null_group(&mut self, ui: &mut egui::Ui) {
        ui.vertical(|ui| {
            ui.strong("Grip null");
            ui.label("Close on the puck, write the trims the wrench asks for.");
            // Changing the seat can leave the source pointing at the
            // new target, which means "the puck already there" — say it
            // in those words here rather than at the trigger.
            if self.null_source == self.null_holder {
                self.null_source = 0;
            }
            fields(ui, "gripnull-fields", |ui| {
                ui.label("Seat:");
                // A list, not a spinner: the stage is not "holder zero"
                // to anyone standing at the rig, and dragging past it to
                // reach holder 1 reads as an off-by-one.
                egui::ComboBox::from_id_salt("gripnull-seat")
                    .selected_text(seat_name(self.null_holder))
                    .show_ui(ui, |ui| {
                        for n in 0..=10i64 {
                            ui.selectable_value(&mut self.null_holder, n, seat_name(n));
                        }
                    });
                ui.end_row();
                ui.label("Puck from:");
                // A rack holder or "already there": `MapSource` has no
                // number for the stage, so the stage is a destination
                // only. The target is left out of the list because
                // picking it means the same as picking "already there".
                let target = self.null_holder;
                let mut source = self.null_source;
                let named = |n: i64| {
                    if n == 0 {
                        format!("already in {}", seat_name(target))
                    } else {
                        seat_name(n)
                    }
                };
                egui::ComboBox::from_id_salt("gripnull-source")
                    .selected_text(named(source))
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut source, 0, named(0));
                        for n in 1..=10i64 {
                            if n != target {
                                ui.selectable_value(&mut source, n, seat_name(n));
                            }
                        }
                    });
                self.null_source = source;
                ui.end_row();
            });
            ui.label(if self.null_source == 0 {
                "(nulls the puck already seated there)"
            } else {
                "(fetched first; the target seat must be empty, and keeps the puck)"
            });
            if ui.button("Null grip").clicked() {
                self.request(Action::GripNull {
                    target: self.null_holder,
                    source: self.null_source,
                });
            }
        });
    }

    pub fn move_puck_group(&mut self, ui: &mut egui::Ui) {
        ui.vertical(|ui| {
            ui.strong("Move puck");
            ui.label("Holder to holder, no stage leg.");
            fields(ui, "movepuck-fields", |ui| {
                ui.label("From holder:");
                ui.add(egui::DragValue::new(&mut self.xfer_src).range(1..=10));
                ui.end_row();
                ui.label("To holder:");
                ui.add(egui::DragValue::new(&mut self.xfer_target).range(1..=10));
                ui.end_row();
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
    }

    pub fn gripper_group(&mut self, ui: &mut egui::Ui) {
        // The daemon reads `Robot:Gripper` in its service pass, which
        // does not run while a step is moving the arm. Greying the
        // buttons there says so; they used to accept the press and
        // leave the operator watching a gripper that never moved.
        let live = self.daemon.servicing();
        ui.vertical(|ui| {
            ui.strong("Gripper");
            ui.add_enabled_ui(live, |ui| {
                ui.horizontal(|ui| {
                    if ui.button("Open").clicked() {
                        self.gripper.put(PvValue::Int(1));
                    }
                    if ui.button("Close").clicked() {
                        self.gripper.put(PvValue::Int(0));
                    }
                });
            });
            if !live {
                ui.label(format!("({})", self.daemon.note()));
            }
        });
    }

    /// The two calibration holds, drawn on the Teach page because that
    /// is the errand they serve: each stands the arm where the daemon
    /// publishes a seat in `Jog:Target`, and those holds are the only
    /// places Apply has somewhere to land (CLAUDE.md, "Jog와 Jog
    /// Apply"). Mode 1 holds above the holder it picked from and mode 2
    /// carries that puck to the stage, so between them they reach the
    /// Holder rows and the Stage row of the table beside this card.
    ///
    /// Ending a hold is a second `Trigger`, not a `Wait`: the daemon
    /// parks in `calibration_hold`, which is `wait_for_trigger`. That
    /// write goes straight out instead of through [`Self::request`] for
    /// the reason the measurement wait's Continue does -- the run is
    /// deliberately standing here and this is its sanctioned
    /// continuation, so a "a sequence is running or was interrupted"
    /// confirmation would be describing the very thing being ended.
    /// Starting a hold still goes through `request`, because that does
    /// start a sequence.
    ///
    /// What enables the button is the daemon saying it is standing in
    /// that hold, not `Jog:Target` -- the seat name is left over from
    /// the last hold and outlives the daemon that stood there, so it
    /// would offer the button over a dead run, where the press starts a
    /// fresh one of whatever mode was left in `CalibMode` instead of
    /// ending anything. `Jog:Target` still names the seat in the label.
    pub fn calib_hold_group(&mut self, ui: &mut egui::Ui) {
        let target = sval(&self.jog_target).unwrap_or_default();
        let holding = self.daemon.is(HOLD);
        ui.vertical(|ui| {
            ui.strong("Calibration hold");
            ui.horizontal_top(|ui| {
                ui.vertical(|ui| {
                    fields(ui, "hold-fields", |ui| {
                        ui.label("Holder:");
                        ui.add(egui::DragValue::new(&mut self.hold_holder).range(1..=10));
                        ui.end_row();
                    });
                    // Both buttons name the pick, because both do the same
                    // one: mode 1 and mode 2 differ only in where they stand
                    // afterwards. "Hold at holder N" read as though the field
                    // chose a source the stage button was free to ignore.
                    ui.horizontal(|ui| {
                        if ui
                            .button(format!("Pick {}, hold there", self.hold_holder))
                            .clicked()
                        {
                            self.request(Action::Hold {
                                holder: self.hold_holder,
                                at_stage: false,
                            });
                        }
                        if ui
                            .button(format!("Pick {}, hold at stage", self.hold_holder))
                            .clicked()
                        {
                            self.request(Action::Hold {
                                holder: self.hold_holder,
                                at_stage: true,
                            });
                        }
                    });
                });
                ui.separator();
                ui.vertical(|ui| {
                    match (holding, target.is_empty()) {
                        (true, false) => ui
                            .colored_label(GREEN, format!("Holding at {target} — jog, then Apply")),
                        (true, true) => ui.colored_label(GREEN, "Holding — jog, then Apply"),
                        (false, _) => {
                            ui.label(format!("No hold standing ({}).", self.daemon.note()))
                        }
                    };
                    ui.add_enabled_ui(holding, |ui| {
                        if ui.button("Return the puck and end the hold").clicked() {
                            self.trigger.put(PvValue::Int(1));
                            self.note =
                                "Ending the hold — returning the puck to its holder".to_string();
                        }
                    });
                });
            });
            ui.label(
                "Either way the puck comes from this holder — a hold cannot \
                 fetch one seat's puck to another's. Apply lands on the seat \
                 named above, not on Holder.",
            );
        });
    }

    pub fn advanced_group(&mut self, ui: &mut egui::Ui) {
        ui.vertical(|ui| {
            ui.strong("Advanced");
            fields(ui, "advanced-fields", |ui| {
                ui.label("Mode:");
                egui::ComboBox::from_id_salt("adv-mode")
                    .selected_text(MODE_NAMES[self.adv_mode])
                    .show_ui(ui, |ui| {
                        for (i, name) in MODE_NAMES.iter().enumerate() {
                            ui.selectable_value(&mut self.adv_mode, i, *name);
                        }
                    });
                ui.end_row();
                ui.label("Start step:");
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut self.adv_start).range(0..=23));
                    if ui.button("Trigger").clicked() {
                        self.request(Action::Advanced {
                            mode: self.adv_mode as i64,
                            start: self.adv_start,
                        });
                    }
                });
                ui.end_row();
                ui.label("Pause at step:");
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut self.pause_step_input).range(0..=23));
                    if ui.button("Set").clicked() {
                        self.pause_step.put(PvValue::Int(self.pause_step_input));
                    }
                    ui.label(format!("(now {})", ival(&self.pause_step).unwrap_or(0)));
                });
                ui.end_row();
                ui.label("Seat check:");
                ui.horizontal(|ui| {
                    // The record is the truth and the box only shows it,
                    // so an Off set from caput or from another client
                    // reads back here.
                    let live = self.seat_check.state().connected;
                    let mut on = ival(&self.seat_check).unwrap_or(1) != 0;
                    ui.add_enabled_ui(live, |ui| {
                        if ui.checkbox(&mut on, "camera confirms the seat").changed() {
                            self.seat_check.put(PvValue::Int(i64::from(on)));
                        }
                    });
                    if !live {
                        // The record is newer than a running IOC that has
                        // not reloaded robot.db, and a box that writes
                        // nowhere would read as a check that is on.
                        ui.colored_label(RED, "no Robot:SeatCheck record — restart the IOC");
                    } else if !on {
                        ui.colored_label(RED, "off — seats are not checked");
                    }
                });
                ui.end_row();
            });
        });
    }

    /// The last thing this panel did, if anything.
    pub fn note_line(&self, ui: &mut egui::Ui) {
        if !self.note.is_empty() {
            ui.label(&self.note);
        }
    }
}
