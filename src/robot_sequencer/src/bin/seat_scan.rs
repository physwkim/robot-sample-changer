//! Where does the seat check's window actually land? One depth frame,
//! read from the arm's present pose, evaluated over a grid of candidate
//! seat positions. Writes nothing — no PV, no motion, no file.
//!
//! The check judges `median − predicted`, and both numbers come from the
//! trimmed seat pose, so a trim that moves the seat moves the window with
//! it. That is right when the trim describes where the seat *is*, and
//! wrong when it carries anything else (a grip offset, a mis-Applied
//! jog): the window then samples beside the puck and answers about a
//! patch nobody measured. The verdict alone cannot tell those apart —
//! "empty" is what both look like. This scan can: it prints the same
//! statistic at the trimmed pose and at neighbours around it, and the
//! cell reading like an occupied seat (−3 to −5 mm) is where the puck
//! really stands.
//!
//! ```text
//! seat_scan <config/sequencer.yaml> [stage|rack] [holder] [dx,dy,dz mm]
//! ```
//!
//! The last argument asks one question instead of a grid: what would the
//! check read if the seat trim moved by that much? A candidate trim can
//! be tried against the live frame before it is written anywhere.
//!
//! Run it with the arm parked where the check observes from — the stage
//! leg observes from `sample_holder_standby`, which is also where the
//! measurement wait stands, so the wait is the free moment to ask.
//! It reproduces the daemon's own geometry (same waypoint file, same
//! `apply_cartesian_offset`, same `HandEye::read_seat`); it does not ask
//! the arm where it is, so a scan taken from any other pose is fiction.

// The daemon's module tree is compiled into this bin as-is rather than
// re-implemented, so most of it is unused here. Silencing that is not
// hiding a defect: the same source is the daemon's live code.
#![allow(dead_code)]

#[path = "../config.rs"]
mod config;
#[path = "../epics.rs"]
mod epics;
#[path = "../error.rs"]
mod error;
#[path = "../gripper.rs"]
mod gripper;
#[path = "../handeye.rs"]
mod handeye;
#[path = "../log.rs"]
mod log;
#[path = "../model.rs"]
mod model;
#[path = "../motion/mod.rs"]
mod motion;
#[path = "../seatcheck.rs"]
mod seatcheck;
// Not used here, but the tree is compiled as one piece: `motion`
// re-exports for `sequence`'s benefit, and leaving it out turns those
// re-exports into unused imports in shared source.
#[path = "../sequence.rs"]
mod sequence;
#[path = "../stream.rs"]
mod stream;
#[path = "../waypoints.rs"]
mod waypoints;

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Duration;

use nalgebra::{Point3, Translation3};

use crate::config::Config;
use crate::epics::Epics;
use crate::error::SequencerError;
use crate::model::{JointMap, Model};
use crate::seatcheck::HandEye;
use crate::waypoints::WaypointData;

/// Half-width of the scan, mm, in the seat's own tool x and z. Wider
/// than any trim the rig has carried, so a window that has walked off
/// the puck is still inside the picture.
const REACH_MM: i32 = 16;
const STEP_MM: i32 = 2;

fn joints(values: &[f64]) -> JointMap {
    WaypointData::arm_joints(values).into_iter().collect()
}

/// `dx,dy,dz` in mm, the seat's own tool frame.
fn parse_at(text: &str) -> Result<[f64; 3], SequencerError> {
    let mut out = [0.0; 3];
    let mut parts = text.split(',');
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = parts
            .next()
            .and_then(|p| p.trim().parse::<f64>().ok())
            .ok_or_else(|| {
                SequencerError(format!(
                    "offset '{text}' is not three comma-separated mm (field {})",
                    i + 1
                ))
            })?
            / 1000.0;
    }
    Ok(out)
}

/// The frame around the seat, 3x, as a PPM: near is bright, far is dark,
/// no range at all is blue. The sampling window is drawn in red and the
/// grasp point in green, so where the window sits on the scene is a
/// thing you look at rather than infer.
#[allow(non_snake_case)]
fn write_ppm(
    path: &str,
    frame: &seatcheck::DepthFrame,
    cam: &seatcheck::DepthCamera,
    window: Option<seatcheck::Window>,
    centre: Option<(f64, f64, f64)>,
) -> Result<(), SequencerError> {
    let mut spec = path.split(':');
    let path = spec.next().unwrap_or_default();
    let num = |s: Option<&str>, d: f64| s.and_then(|v| v.parse::<f64>().ok()).unwrap_or(d);
    let half_w = num(spec.next(), 110.0) as i64;
    let half_h = num(spec.next(), 90.0) as i64;
    let near_m = num(spec.next(), 160.0) / 1000.0;
    let far_m = num(spec.next(), 340.0) / 1000.0;
    let zoom = num(spec.next(), 3.0) as i64;
    let (HALF_W, HALF_H, ZOOM, NEAR_M, FAR_M) = (half_w, half_h, zoom, near_m, far_m);

    let (cu, cv) = match centre {
        Some((u, v, _)) => (u as i64, v as i64),
        None => (cam.width as i64 / 2, cam.height as i64 / 2),
    };
    let (u0, v0) = (cu - HALF_W, cv - HALF_H);
    let (w, h) = (HALF_W * 2 * ZOOM, HALF_H * 2 * ZOOM);
    let mut out = format!("P6\n{w} {h}\n255\n").into_bytes();
    for py in 0..h {
        for px in 0..w {
            let u = u0 + px / ZOOM;
            let v = v0 + py / ZOOM;
            let inside = u >= 0 && v >= 0 && u < cam.width as i64 && v < cam.height as i64;
            let count = inside
                .then(|| {
                    frame
                        .counts
                        .get(v as usize * cam.width + u as usize)
                        .copied()
                })
                .flatten()
                .unwrap_or(0);
            let mut rgb = if count == 0 {
                [40u8, 40, 90]
            } else {
                let m = f64::from(count) * cam.unit_m;
                let t = ((FAR_M - m) / (FAR_M - NEAR_M)).clamp(0.0, 1.0);
                let g = (t * 255.0) as u8;
                [g, g, g]
            };
            if let Some(win) = window {
                let on_edge = (u == win.c0 as i64 || u == win.c1 as i64)
                    && (win.r0 as i64..=win.r1 as i64).contains(&v)
                    || (v == win.r0 as i64 || v == win.r1 as i64)
                        && (win.c0 as i64..=win.c1 as i64).contains(&u);
                if on_edge {
                    rgb = [255, 40, 40];
                }
            }
            if (u - cu).abs() <= 3 && v == cv || (v - cv).abs() <= 3 && u == cu {
                rgb = [40, 255, 40];
            }
            out.extend_from_slice(&rgb);
        }
    }
    std::fs::write(path, out).map_err(|e| SequencerError(format!("cannot write {path}: {e}")))?;
    Ok(())
}

fn run() -> Result<(), SequencerError> {
    let mut args = std::env::args().skip(1);
    let config_path: PathBuf = args
        .next()
        .ok_or_else(|| {
            SequencerError("usage: seat_scan <config.yaml> [stage|rack] [holder]".into())
        })?
        .into();
    let which = args.next().unwrap_or_else(|| "stage".into());
    let holder: i32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(1);
    let fourth = args.next();
    let dump = fourth.as_deref().and_then(|s| s.strip_prefix("dump:"));
    // A window bias moves only where the check samples; an `at` offset
    // moves the seat itself, prediction included. Tuning the first and
    // testing the second are different questions, so they are different
    // arguments rather than one number that means both.
    let try_bias = fourth
        .as_deref()
        .and_then(|s| s.strip_prefix("bias:"))
        .map(parse_at)
        .transpose()?;
    let at: Option<[f64; 3]> = match (&fourth, dump, try_bias) {
        (Some(s), None, None) => Some(parse_at(s)?),
        _ => None,
    };

    let config = Config::load(&config_path)?;
    let model = Model::load(&config)?;
    // The `enabled` flag decides whether the daemon runs the check, not
    // whether the solve exists; this reads the file either way, which is
    // the point when the check has been switched off to keep running.
    let eye = HandEye::load(&config.seat_check.hand_eye_yaml)?;
    let w = WaypointData::load(&config.sequence.waypoints_yaml)?;

    let (observe, seat_joints, label) = match which.as_str() {
        "stage" => (
            joints(&w.sample_holder_standby),
            model.apply_cartesian_offset(
                &joints(&w.sample_holder_on_position),
                [
                    w.sample_holder_on_x_offset,
                    w.sample_holder_on_y_offset,
                    w.sample_holder_on_z_offset,
                ],
                false,
                "sample_holder_on_position",
            )?,
            "stage".to_string(),
        ),
        "rack" => {
            let idx = (holder - 1).max(0) as usize;
            let y = f64::from(holder - 1) * config.sequence.holder_offset
                + w.holder_multi_y_offsets.get(idx).copied().unwrap_or(0.0);
            let x = w.holder_multi_x_offsets.get(idx).copied().unwrap_or(0.0);
            let z = w.holder_multi_z_offsets.get(idx).copied().unwrap_or(0.0);
            let rack = [w.rack_x_offset, w.rack_y_offset, w.rack_z_offset];
            let standby = model.apply_cartesian_offset(
                &joints(&w.holder1_standby),
                rack,
                false,
                "standby",
            )?;
            let on = model.apply_cartesian_offset(
                &joints(&w.holder1_on_position),
                rack,
                false,
                "on_position",
            )?;
            (
                model.apply_cartesian_offset(&standby, [x, y, z], false, "standby_holder")?,
                model.apply_cartesian_offset(&on, [x, y, z], false, "on_holder")?,
                format!("holder {holder}"),
            )
        }
        other => {
            return Err(SequencerError(format!(
                "unknown seat '{other}': stage or rack"
            )));
        }
    };

    let epics = Epics::connect(&config.epics, None, Some(&config.seat_check))?;
    let camera = epics
        .depth_camera()
        .ok_or_else(|| SequencerError("no depth camera: seat_check PVs did not connect".into()))?;
    let frame = epics
        .depth_frame(Duration::from_secs_f64(config.seat_check.timeout))
        .ok_or_else(|| {
            SequencerError(
                "no depth frame — RS405:image2:EnableCallbacks is the usual reason".into(),
            )
        })?;

    let base_t_ee = model.fk(&observe)?;
    let seat = model.fk(&seat_joints)?;
    let configured = match which.as_str() {
        "stage" => config.seat_check.stage_window_bias_mm,
        _ => config.seat_check.rack_window_bias_mm,
    }
    .map(|v| v / 1000.0);
    let bias = try_bias.unwrap_or(configured);

    println!("seat scan @{label}, observed from the taught standby");
    println!(
        "  the trimmed seat sits at [{:.4} {:.4} {:.4}] m in the model frame",
        seat.translation.x, seat.translation.y, seat.translation.z
    );
    match eye.read_seat(&frame, camera, &base_t_ee, &seat, bias) {
        Some(r) => println!(
            "  at the trim itself: {} — {:.1} mm against the seat's {:.1} mm ({:+.1} mm), \
             {:.0}% of pixels {}-{} x {}-{} valid",
            r.verdict.label(),
            r.median_m * 1000.0,
            r.predicted_m * 1000.0,
            r.delta_m * 1000.0,
            r.valid_fraction * 100.0,
            r.window.c0,
            r.window.c1,
            r.window.r0,
            r.window.r1
        ),
        None => println!("  at the trim itself: does not project into the frame"),
    }
    println!(
        "  window bias in use: [{:+.2} {:+.2} {:+.2}] mm",
        bias[0] * 1000.0,
        bias[1] * 1000.0,
        bias[2] * 1000.0
    );
    if try_bias.is_some() {
        return Ok(());
    }
    if let Some(path) = dump {
        // A picture instead of a statistic. Every verdict here is one
        // median over one small window, and a median cannot say whether
        // the window sits on the puck, beside it, or across its edge --
        // which is the whole question when a seat has moved. The frame
        // itself can.
        let window = eye.window(&base_t_ee, &seat, camera, bias);
        let centre = eye.look(
            &base_t_ee,
            &Point3::from(seat.translation.vector),
            &camera.lens,
        );
        write_ppm(path, &frame, camera, window, centre)?;
        println!("  wrote {path}");
        if let Some((u, v, _)) = centre {
            println!("  the trimmed seat projects to ({u:.1}, {v:.1})");
        }
        return Ok(());
    }
    if let Some(d) = at {
        let probe = seat * Translation3::new(d[0], d[1], d[2]);
        match eye.read_seat(&frame, camera, &base_t_ee, &probe, bias) {
            Some(r) => println!(
                "  at {:+.2},{:+.2},{:+.2} mm: {} — {:.1} mm against that point's {:.1} mm \
                 ({:+.1} mm), {:.0}% of pixels {}-{} x {}-{} valid",
                d[0] * 1000.0,
                d[1] * 1000.0,
                d[2] * 1000.0,
                r.verdict.label(),
                r.median_m * 1000.0,
                r.predicted_m * 1000.0,
                r.delta_m * 1000.0,
                r.valid_fraction * 100.0,
                r.window.c0,
                r.window.c1,
                r.window.r0,
                r.window.r1
            ),
            None => println!("  at that offset: does not project into the frame"),
        }
        // The same displacement in the model frame. A tool-frame trim is
        // what the waypoint file speaks, but the collision scene speaks
        // model coordinates, so a seat that moved has to be said twice;
        // this is the translation between the two.
        let world = seat.rotation * nalgebra::Vector3::new(d[0], d[1], d[2]);
        println!(
            "  that offset is [{:+.5} {:+.5} {:+.5}] m in the model frame",
            world.x, world.y, world.z
        );
        return Ok(());
    }
    println!();
    println!("median − predicted, mm, over seat positions shifted in the seat's own tool frame.");
    println!("An occupied seat reads −3 to −5; a rack well seen through reads +17 or more,");
    println!("the open stage bore +512. '.' is too few valid pixels to say, 'x' is off-frame.");

    let ticks: Vec<i32> = (-REACH_MM..=REACH_MM).step_by(STEP_MM as usize).collect();
    // Two planes rather than one: a seat that is off sideways and a seat
    // that is off in depth both read "empty", and the lateral grid alone
    // cannot tell them apart. Depth is the approach axis (tool y), the
    // one the window spans −6…+2 mm of, so an error there walks the
    // window off the puck top just as a lateral one does.
    for (rows, name, depth) in [
        ("dz", "lateral: rows are tool z, columns tool x", false),
        (
            "dy",
            "depth: rows are tool y (approach), columns tool x",
            true,
        ),
    ] {
        for (what, valid) in [("median − predicted, mm", false), ("valid pixels, %", true)] {
            println!();
            println!("{what} — {name}");
            print!("  {rows}\\dx ");
            for dx in &ticks {
                print!("{dx:>7}");
            }
            println!();
            for down in &ticks {
                print!("{down:>7}");
                for dx in &ticks {
                    let (dy, dz) = if depth {
                        (f64::from(*down) / 1000.0, 0.0)
                    } else {
                        (0.0, f64::from(*down) / 1000.0)
                    };
                    let probe = seat * Translation3::new(f64::from(*dx) / 1000.0, dy, dz);
                    match eye.read_seat(&frame, camera, &base_t_ee, &probe, bias) {
                        None => print!("{:>7}", "x"),
                        Some(r) if valid => print!("{:>7.0}", r.valid_fraction * 100.0),
                        Some(r) if r.verdict == seatcheck::Verdict::Unreadable => {
                            print!("{:>7}", ".")
                        }
                        Some(r) => print!("{:>7.1}", r.delta_m * 1000.0),
                    }
                }
                println!();
            }
        }
    }
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("seat_scan: {e}");
            ExitCode::FAILURE
        }
    }
}
