use gtk4::prelude::*;
use gtk4::DrawingArea;
use std::sync::Mutex;
use crate::homerobot::LidarPoint;

pub struct GuiState {
    pub lidar_points: Vec<LidarPoint>,
}

lazy_static::lazy_static! {
    pub static ref GUI_STATE: Mutex<GuiState> = Mutex::new(GuiState {
        lidar_points: Vec::new(),
    });
}

pub fn setup_lidar_drawing(lidar_canvas: &DrawingArea) {
    lidar_canvas.set_draw_func(move |_area, cr, width, height| {
        let state = GUI_STATE.lock().unwrap();
        let center_x = width as f64 / 2.0;
        let center_y = height as f64 / 2.0;
        let scale = 0.05;

        cr.set_source_rgb(0.05, 0.05, 0.1);
        cr.paint().unwrap();

        cr.set_source_rgb(0.2, 0.2, 0.3);
        cr.set_line_width(1.0);
        for i in 1..20 {
            let r = i as f64 * 500.0 * scale;
            cr.arc(center_x, center_y, r, 0.0, 2.0 * std::f64::consts::PI);
            cr.stroke().unwrap();
        }

        cr.set_source_rgb(1.0, 0.3, 0.3);
        cr.arc(center_x, center_y, 5.0, 0.0, 2.0 * std::f64::consts::PI);
        cr.fill().unwrap();

        cr.set_source_rgb(0.0, 1.0, 0.5);
        for p in &state.lidar_points {
            if p.distance_mm < 100.0 { continue; }
            let angle_rad = (p.angle_deg as f64).to_radians();
            let x = center_x + (p.distance_mm as f64 * scale * angle_rad.sin());
            let y = center_y - (p.distance_mm as f64 * scale * angle_rad.cos());
            cr.arc(x, y, 2.0, 0.0, 2.0 * std::f64::consts::PI);
            cr.fill().unwrap();
        }
    });
}
