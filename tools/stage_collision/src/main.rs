//! Builds the stage's collision geometry from its CAD mesh.
//!
//! `resources/stage.stl` is a full-fidelity export — 231,812 triangles —
//! and using it directly costs ~150 s of parsing and BVH build at every
//! daemon start plus ~930 ms per joint-space plan, almost all of it in
//! collision queries against that mesh.
//!
//! Two reductions, in order:
//!
//! 1. Crop to what the arm can reach. A UR3e reaches 500 mm from the
//!    base; everything past `--reach` (default 0.8 m, so 500 mm plus the
//!    tool and margin) cannot be touched, and dropping it cannot change
//!    a collision verdict. Here that is 46% of the mesh — the support
//!    structure below the table and the far end of the stage.
//!
//! 2. Approximate convex decomposition of the remainder. Collision
//!    queries against convex pieces are far cheaper than against a
//!    triangle soup. `compute_exact_convex_hulls` takes the hull of each
//!    partition of the *original* geometry, and a hull contains its set,
//!    so the union of the pieces contains the mesh: the result can only
//!    ever report more collisions than the true shape, never fewer.
//!    That direction is the safe one — it refuses paths, it does not
//!    miss obstacles.
//!
//! The cost of (2) is exactly that lost freedom. Thin concavities do not
//! survive it: the holder slot fills in, so `holder1_on_position` reads
//! as a collision. That pose is only ever reached by Cartesian steps,
//! which do not collision-check, and every joint-space goal in the
//! sequence is a standby pose — but a joint-space plan into a slot would
//! now fail, and that is the tradeoff being made.
//!
//! Usage:
//!   stage-collision <stage.stl> <out-dir> [--reach 0.8] [--hulls 64]
//!                   [--resolution 256] [--concavity 0.001]
//!                   [--scale 0.01] [--yaw 3.14159]
//!                   [--at -0.15,0.39,-0.002]
use std::fs;
use std::io::Write;

use parry3d_f64::math::Vector;
use parry3d_f64::transformation::vhacd::{VHACD, VHACDParameters};

fn read_stl(path: &str) -> (Vec<Vector>, Vec<[u32; 3]>) {
    let raw = fs::read(path).unwrap_or_else(|e| panic!("cannot read {path}: {e}"));
    let n = u32::from_le_bytes(raw[80..84].try_into().unwrap()) as usize;
    let mut pts = Vec::with_capacity(n * 3);
    let mut idx = Vec::with_capacity(n);
    for i in 0..n {
        let o = 84 + i * 50 + 12;
        let mut tri = [0u32; 3];
        for (k, slot) in tri.iter_mut().enumerate() {
            let b = o + k * 12;
            let f = |j: usize| {
                f32::from_le_bytes(raw[b + j * 4..b + j * 4 + 4].try_into().unwrap()) as f64
            };
            pts.push(Vector::new(f(0), f(1), f(2)));
            *slot = (pts.len() - 1) as u32;
        }
        idx.push(tri);
    }
    (pts, idx)
}

fn write_stl(path: &str, tris: &[[Vector; 3]], header: &str) {
    let mut out = Vec::new();
    let mut h = header.as_bytes().to_vec();
    h.resize(80, 0);
    out.extend_from_slice(&h);
    out.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for t in tris {
        let n = (t[1] - t[0]).cross(t[2] - t[0]).normalize();
        for c in [n.x, n.y, n.z] {
            out.extend_from_slice(&(c as f32).to_le_bytes());
        }
        for p in t {
            for c in [p.x, p.y, p.z] {
                out.extend_from_slice(&(c as f32).to_le_bytes());
            }
        }
        out.extend_from_slice(&0u16.to_le_bytes());
    }
    fs::File::create(path).unwrap().write_all(&out).unwrap();
}

fn arg<T: std::str::FromStr>(args: &[String], name: &str, default: T) -> T {
    args.iter()
        .position(|a| a == name)
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let src = args.get(1).expect("usage: stage-collision <stl> <out-dir> [opts]");
    let out_dir = args.get(2).expect("usage: stage-collision <stl> <out-dir> [opts]");
    let reach: f64 = arg(&args, "--reach", 0.8);
    let hulls: u32 = arg(&args, "--hulls", 64);
    let resolution: u32 = arg(&args, "--resolution", 256);
    let concavity: f64 = arg(&args, "--concavity", 0.001);
    let scale: f64 = arg(&args, "--scale", 0.01);
    let yaw: f64 = arg(&args, "--yaw", std::f64::consts::PI);
    let at: Vec<f64> = arg(&args, "--at", "-0.15,0.39,-0.002".to_string())
        .split(',')
        .map(|v| v.parse().unwrap())
        .collect();

    let (pts, idx) = read_stl(src);
    println!("{src}: {} triangles", idx.len());

    // Crop in the world frame the scene places the mesh in, keeping the
    // vertices in mesh coordinates so the same scale/pose still applies.
    let (sy, cy) = yaw.sin_cos();
    let world = |p: &Vector| {
        let (x, y, z) = (p.x * scale, p.y * scale, p.z * scale);
        Vector::new(x * cy - y * sy + at[0], x * sy + y * cy + at[1], z + at[2])
    };
    let kept: Vec<[u32; 3]> = idx
        .iter()
        .copied()
        .filter(|t| t.iter().any(|&i| world(&pts[i as usize]).length() <= reach))
        .collect();
    println!(
        "within {reach} m of the base: {} triangles ({:.1}%)",
        kept.len(),
        kept.len() as f64 / idx.len() as f64 * 100.0
    );

    let params = VHACDParameters {
        max_convex_hulls: hulls,
        resolution,
        concavity,
        ..VHACDParameters::default()
    };
    let t = std::time::Instant::now();
    let decomp = VHACD::decompose(&params, &pts, &kept, true);
    let parts = decomp.compute_exact_convex_hulls(&pts, &kept);
    println!("VHACD {:.1} s -> {} convex parts", t.elapsed().as_secs_f64(), parts.len());

    fs::create_dir_all(out_dir).unwrap();
    for entry in fs::read_dir(out_dir).unwrap().flatten() {
        if entry.path().extension().is_some_and(|e| e == "stl") {
            fs::remove_file(entry.path()).unwrap();
        }
    }
    let mut total = 0;
    for (i, (hv, hi)) in parts.iter().enumerate() {
        let tris: Vec<[Vector; 3]> = hi
            .iter()
            .map(|f| [hv[f[0] as usize], hv[f[1] as usize], hv[f[2] as usize]])
            .collect();
        total += tris.len();
        write_stl(
            &format!("{out_dir}/stage_part{i:02}.stl"),
            &tris,
            &format!("stage collision part {i}, see tools/stage_collision"),
        );
    }
    println!(
        "wrote {} parts, {total} triangles ({:.2}% of the source)",
        parts.len(),
        total as f64 / idx.len() as f64 * 100.0
    );
    println!("\nscene.objects entries:");
    for i in 0..parts.len() {
        println!("    - {{ id: \"stage{i}\", stl: \"../resources/stage_parts/stage_part{i:02}.stl\", scale: [{scale}, {scale}, {scale}], position: [{}, {}, {}], rpy: [0.0, 0.0, {yaw}] }}", at[0], at[1], at[2]);
    }
}
