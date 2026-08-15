//! Log `actual_TCP_force` at the RTDE rate while the daemon runs, and write nothing.
//!
//! Vision decides before the arm moves. Force is the only signal that can say
//! what happened *while* it moved — whether a lift actually picked something
//! up, and what seating contact looks like — and neither question is
//! answerable without the trace first. `Robot:UR:Receive:ActualTCPForce`
//! already carries the same field but the monitor IOC publishes it at 0.5 Hz,
//! which cannot resolve a lift; this takes it from the source at 125 Hz.
//!
//! Running this beside the daemon is legal under CLAUDE.md's "읽기는 다중,
//! 쓰기는 하나": RTDE output is multiplexed per client recipe, and this
//! registers **no input recipe**, so it cannot reach the register/slider path
//! where a second writer silently wins. It never commands the arm.
//!
//! ```text
//! force_log <host> [out.csv]
//! ```
//!
//! Columns are the RTDE timestamp, the six force components, the TCP pose and
//! the joint angles, so a lift is found by pose afterwards rather than by eye.
//! Ctrl-C ends it; the file is flushed per line.

use std::io::Write;

use ur_driver::rtde::{RtdeClient, RtdeValue};

const RECIPE: [&str; 4] = [
    "timestamp",
    "actual_TCP_force",
    "actual_TCP_pose",
    "actual_q",
];
const PORT: u16 = 30004;
const FREQUENCY_HZ: f64 = 125.0;

fn six(package: &ur_driver::rtde::DataPackage, field: &str) -> [f64; 6] {
    match package.get(field) {
        Some(RtdeValue::V6D(v)) => *v,
        _ => [f64::NAN; 6],
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let host = args.next().unwrap_or_else(|| "192.168.192.10".into());
    let path = args.next().unwrap_or_else(|| "force.csv".into());

    let recipe: Vec<String> = RECIPE.iter().map(|s| s.to_string()).collect();
    let mut client = RtdeClient::connect(&host, PORT, recipe, Vec::new(), FREQUENCY_HZ)?;
    client.init()?;
    client.start()?;

    let mut out = std::io::BufWriter::new(std::fs::File::create(&path)?);
    writeln!(out, "t,fx,fy,fz,tx,ty,tz,x,y,z,rx,ry,rz,q0,q1,q2,q3,q4,q5")?;
    eprintln!("logging {host} at {FREQUENCY_HZ} Hz to {path} — Ctrl-C to stop");

    let mut n: u64 = 0;
    loop {
        let package = client.get_data_package()?;
        let t = match package.get("timestamp") {
            Some(RtdeValue::F64(v)) => *v,
            _ => f64::NAN,
        };
        let f = six(&package, "actual_TCP_force");
        let p = six(&package, "actual_TCP_pose");
        let q = six(&package, "actual_q");
        let row: Vec<String> = std::iter::once(t)
            .chain(f)
            .chain(p)
            .chain(q)
            .map(|v| format!("{v:.6}"))
            .collect();
        writeln!(out, "{}", row.join(","))?;
        n += 1;
        // Flushed on a cadence rather than per line: the point of taking this
        // from the source is the rate, and a per-line flush at 125 Hz is the
        // one thing in the loop that could make the reader fall behind — a
        // client that stops reading is dropped by URControl (see stream.rs).
        if n % 125 == 0 {
            out.flush()?;
            eprint!("\r{n} samples");
        }
    }
}
