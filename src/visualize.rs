use std::sync::OnceLock;

use nalgebra::Vector3;
use rerun::{AsComponents, RecordingStream};

pub use rerun::{Color, Position3D};

pub static RERUN: Option<OnceLock<RecordingStream>> = Some(OnceLock::new());

static GRID: OnceLock<Vec<[f32; 3]>> = OnceLock::new();

pub fn construct() -> RecordingStream {
    let rec = rerun::RecordingStreamBuilder::new("horyzn_core")
        .connect()
        .unwrap();
    rec.set_time_seconds("stable_time", 0f64);
    rec
}

pub fn init() {
    if let Some(rec) = &RERUN {
        rec.get_or_init(construct);
    }
}

pub fn set_time(secs: f64) {
    if let Some(rec) = &RERUN {
        let rec = rec.get_or_init(construct);
        rec.set_time_seconds("stable_time", secs);
    }
}

pub fn log(topic: &str, comp: &impl AsComponents) {
    if let Some(rec) = &RERUN {
        let rec = rec.get_or_init(construct);
        rec.log(topic, comp).unwrap();
    }
}

pub fn log_pos(topic: &str, pos: Vector3<f64>, color: Color) {
    log(
        topic,
        &rerun::Points3D::new([Position3D::new(pos.x as f32, pos.y as f32, -pos.z as f32)])
            .with_colors([color]),
    );
}

pub fn log_xyz(topic: &str, v: Vector3<f64>) {
    log(&format!("{topic}/x"), &rerun::Scalar::new(v.x));
    log(&format!("{topic}/y"), &rerun::Scalar::new(v.y));
    log(&format!("{topic}/z"), &rerun::Scalar::new(v.z));
}

pub fn send_grid() {
    if let Some(rec) = &RERUN {
        let rec = rec.get_or_init(construct);

        if GRID.get().is_some() {
            return;
        }

        let grid = GRID.get_or_init(|| {
            let iterator_h = (-1000..1000)
                .step_by(10)
                .map(|i| [[-1000.0, i as f32, 0.0], [1000.0, i as f32, 0.0]])
                .flatten();

            let iterator_v = (-1000..1000)
                .step_by(10)
                .map(|i| [[i as f32, -1000.0, 0.0], [i as f32, 1000.0, 0.0]])
                .flatten();

            let mesh: Vec<[f32; 3]> = iterator_h.chain(iterator_v).collect();
            mesh
        });

        rec.log(
            "base_grid",
            &rerun::LineStrips3D::new(grid.chunks(2))
                .with_colors([Color::from_rgb(150, 150, 150)])
                .with_radii([0.005]),
        )
        .unwrap();
    }
}
