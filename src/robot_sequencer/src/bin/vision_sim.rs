//! Vision-node simulator for camera-less rehearsal (URSim): answers the
//! sequencer's `Robot:Vision` handshake with fixed values from the
//! command line, standing in for the real wrist-camera node.
//!
//! Usage:
//!   vision_sim [--dx MM] [--dy MM] [--dz MM]
//!              [--grip-dx MM] [--grip-dy MM] [--grip-dz MM]
//!              [--quality Q] [--invalid] [--not-seated] [--tilt DEG]
//!
//! Pick/Place align requests answer with `--dx/--dy/--dz`, GripOffset
//! with `--grip-*`, Seating with `--not-seated`/`--tilt`. Every request
//! answers Valid unless `--invalid`. PV names are the sequencer's
//! defaults (`Robot:Vision:*`).

use std::process::ExitCode;
use std::time::Duration;

use epics_ca_rs::EpicsValue;
use epics_ca_rs::client::{CaChannel, CaClient};
use tokio::runtime::Runtime;

const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);
const IO_TIMEOUT: Duration = Duration::from_secs(1);
const POLL: Duration = Duration::from_millis(20);

struct SimConfig {
    align: [f64; 3],
    grip: [f64; 3],
    quality: f64,
    invalid: bool,
    seated: bool,
    tilt: f64,
}

fn parse_args() -> Result<SimConfig, String> {
    let mut sim = SimConfig {
        align: [0.0; 3],
        grip: [0.0; 3],
        quality: 1.0,
        invalid: false,
        seated: true,
        tilt: 0.0,
    };
    let mut args = std::env::args().skip(1);
    while let Some(flag) = args.next() {
        let mut value = |flag: &str| -> Result<f64, String> {
            args.next()
                .ok_or_else(|| format!("{flag} needs a value"))?
                .parse::<f64>()
                .map_err(|e| format!("{flag}: {e}"))
        };
        match flag.as_str() {
            "--dx" => sim.align[0] = value("--dx")?,
            "--dy" => sim.align[1] = value("--dy")?,
            "--dz" => sim.align[2] = value("--dz")?,
            "--grip-dx" => sim.grip[0] = value("--grip-dx")?,
            "--grip-dy" => sim.grip[1] = value("--grip-dy")?,
            "--grip-dz" => sim.grip[2] = value("--grip-dz")?,
            "--quality" => sim.quality = value("--quality")?,
            "--tilt" => sim.tilt = value("--tilt")?,
            "--invalid" => sim.invalid = true,
            "--not-seated" => sim.seated = false,
            other => return Err(format!("unknown flag '{other}'")),
        }
    }
    Ok(sim)
}

struct Pvs {
    rt: Runtime,
    _client: CaClient,
    req: CaChannel,
    kind: CaChannel,
    done: CaChannel,
    valid: CaChannel,
    dx: CaChannel,
    dy: CaChannel,
    dz: CaChannel,
    quality: CaChannel,
    seated: CaChannel,
    tilt: CaChannel,
}

impl Pvs {
    fn connect() -> Result<Self, String> {
        let rt = Runtime::new().map_err(|e| format!("tokio runtime: {e}"))?;
        let client = rt
            .block_on(CaClient::new())
            .map_err(|e| format!("CA client: {e}"))?;
        let ch = |name: &str| -> Result<CaChannel, String> {
            let channel = client.create_channel(name);
            rt.block_on(channel.wait_connected(CONNECT_TIMEOUT))
                .map_err(|_| format!("PV '{name}' is not connected"))?;
            Ok(channel)
        };
        Ok(Self {
            req: ch("Robot:Vision:Req")?,
            kind: ch("Robot:Vision:Kind")?,
            done: ch("Robot:Vision:Done")?,
            valid: ch("Robot:Vision:Valid")?,
            dx: ch("Robot:Vision:DX")?,
            dy: ch("Robot:Vision:DY")?,
            dz: ch("Robot:Vision:DZ")?,
            quality: ch("Robot:Vision:Quality")?,
            seated: ch("Robot:Vision:Seated")?,
            tilt: ch("Robot:Vision:Tilt")?,
            _client: client,
            rt,
        })
    }

    fn get_i32(&self, channel: &CaChannel) -> Result<i32, String> {
        let (_, value) = self
            .rt
            .block_on(channel.get_with_timeout(IO_TIMEOUT))
            .map_err(|e| format!("get: {e}"))?;
        match value {
            EpicsValue::Long(v) => Ok(v),
            EpicsValue::Enum(v) => Ok(i32::from(v)),
            EpicsValue::Short(v) => Ok(i32::from(v)),
            other => Err(format!("unexpected value {other:?}")),
        }
    }

    fn put_i32(&self, channel: &CaChannel, value: i32) -> Result<(), String> {
        self.rt
            .block_on(channel.put_with_timeout(&EpicsValue::Long(value), IO_TIMEOUT))
            .map_err(|e| format!("put: {e}"))
    }

    fn put_f64(&self, channel: &CaChannel, value: f64) -> Result<(), String> {
        self.rt
            .block_on(channel.put_with_timeout(&EpicsValue::Double(value), IO_TIMEOUT))
            .map_err(|e| format!("put: {e}"))
    }
}

fn run() -> Result<(), String> {
    let sim = parse_args()?;
    let pvs = Pvs::connect()?;
    println!(
        "vision_sim ready: align=({:.3},{:.3},{:.3})mm grip=({:.3},{:.3},{:.3})mm \
         quality={:.2} valid={} seated={} tilt={:.2}deg",
        sim.align[0],
        sim.align[1],
        sim.align[2],
        sim.grip[0],
        sim.grip[1],
        sim.grip[2],
        sim.quality,
        !sim.invalid,
        sim.seated,
        sim.tilt
    );

    // Start from the last ANSWERED id so a request issued before the
    // simulator came up is still served on the first poll.
    let mut last = pvs.get_i32(&pvs.done)?;
    loop {
        let req = pvs.get_i32(&pvs.req)?;
        if req != last {
            let kind = pvs.get_i32(&pvs.kind)?;
            let (d, label) = match kind {
                1 => (sim.align, "PickAlign"),
                2 => (sim.grip, "GripOffset"),
                3 => (sim.align, "PlaceAlign"),
                4 => ([0.0; 3], "Seating"),
                other => {
                    println!("request {req}: unknown kind {other}, answering zeros");
                    ([0.0; 3], "?")
                }
            };
            pvs.put_f64(&pvs.dx, d[0])?;
            pvs.put_f64(&pvs.dy, d[1])?;
            pvs.put_f64(&pvs.dz, d[2])?;
            pvs.put_f64(&pvs.quality, sim.quality)?;
            pvs.put_f64(&pvs.tilt, sim.tilt)?;
            pvs.put_i32(&pvs.seated, i32::from(sim.seated))?;
            pvs.put_i32(&pvs.valid, i32::from(!sim.invalid))?;
            // Done last: the sequencer treats its echo as "results ready".
            pvs.put_i32(&pvs.done, req)?;
            println!(
                "request {req} ({label}): answered d=({:.3},{:.3},{:.3})mm valid={}",
                d[0], d[1], d[2], !sim.invalid
            );
            last = req;
        }
        std::thread::sleep(POLL);
    }
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("vision_sim: {e}");
            ExitCode::FAILURE
        }
    }
}
