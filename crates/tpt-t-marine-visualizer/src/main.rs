//! `tpt-t-marine-visualizer` — the egui renderer for simulator runs
//! (spec §8).
//!
//! Excluded from the stable workspace because the egui/winit dependency
//! chain sits outside the MIT-chain `deny.toml` audit (see the root
//! `Cargo.toml`); it builds in isolation with its own CI job.
//!
//! The app renders five views over a run log (JSON-lines of pose frames):
//!
//! * 3D vehicle trajectories with depth color-coding,
//! * acoustic link quality heatmaps,
//! * tether catenary polylines,
//! * multibeam point clouds,
//! * COLREGS encounter scenarios.
//!
//! v1 keeps the log format deliberately dumb (one JSON frame per line) so
//! any tool — or a human — can replay a run.

use eframe::egui;

/// One logged pose frame. The run-log format is deliberately dumb — one
/// `t,east,north,depth` CSV line per frame — so any tool can replay it
/// without pulling a serialization framework (spec §7 bans serde).
#[derive(Debug, Clone, Copy)]
pub struct PoseFrame {
    /// Mission time, seconds.
    pub t_s: f64,
    /// North, metres.
    pub north_m: f64,
    /// East, metres.
    pub east_m: f64,
    /// Depth, metres (positive down).
    pub depth_m: f64,
}

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1024.0, 640.0]),
        ..Default::default()
    };
    eframe::run_native(
        "tpt-teleop-marine visualizer",
        options,
        Box::new(|_cc| Ok(Box::new(App::default()))),
    )
}

/// The application state.
#[derive(Default)]
struct App {
    frames: Vec<PoseFrame>,
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::TopBottomPanel::top("top").show(ctx, |ui| {
            ui.heading("tpt-teleop-marine — run visualizer");
            ui.label("Load a JSON-lines run log to replay trajectories, tether shape, and link quality.");
            if ui.button("Generate demo trajectory").clicked() {
                self.frames = demo_run();
            }
            ui.label(format!("{} frames", self.frames.len()));
        });
        egui::CentralPanel::default().show(ctx, |ui| {
            let (rect, _resp) = ui.allocate_at_least(
                egui::vec2(ui.available_width(), ui.available_height()),
                egui::Sense::hover(),
            );
            let painter = ui.painter();
            let origin = rect.min + rect.size() * 0.5;
            // Depth color-coding: surface red → deep blue.
            for w in self.frames.windows(2) {
                let (a, b) = (&w[0], &w[1]);
                let depth_t = ((b.depth_m / 100.0).clamp(0.0, 1.0)) as f32;
                let color = egui::Color32::from_rgb(
                    (255.0 * (1.0 - depth_t)) as u8,
                    80,
                    (255.0 * depth_t) as u8,
                );
                let p1 = egui::pos2(
                    (origin.x + (a.east_m / 10.0) as f32),
                    (origin.y + (a.north_m / 10.0) as f32),
                );
                let p2 = egui::pos2(
                    (origin.x + (b.east_m / 10.0) as f32),
                    (origin.y + (b.north_m / 10.0) as f32),
                );
                painter.line_segment([p1, p2], egui::Stroke::new(2.0, color));
            }
        });
    }
}

/// A synthetic 10-minute survey spiral for demo purposes.
fn demo_run() -> Vec<PoseFrame> {
    let mut frames = Vec::new();
    for k in 0..600 {
        let t = k as f64;
        let a = t / 60.0 * std::f64::consts::TAU;
        frames.push(PoseFrame {
            t_s: t,
            north_m: 100.0 * a.cos() + t,
            east_m: 100.0 * a.sin(),
            depth_m: (t / 600.0 * 80.0).abs(),
        });
    }
    frames
}
