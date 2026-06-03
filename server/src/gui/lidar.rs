use crate::homerobot::LidarPoint;
use gtk4::prelude::*;
use gtk4::DrawingArea;
use std::sync::Mutex;

pub struct GuiState {
    pub display_scan: Vec<LidarPoint>,
    pub robot_x: f32,
    pub robot_y: f32,
    pub robot_theta: f32,
    pub slam_x: f32,
    pub slam_y: f32,
    pub slam_theta: f32,
    pub map_width: usize,
    pub map_height: usize,
    pub map_data: Vec<i16>,
    pub frontiers: Vec<crate::mapping::Frontier>,
    pub current_path: Vec<(f32, f32)>,
    pub trajectory: Vec<(f32, f32)>,
    pub accel_history: std::collections::VecDeque<(f32, f32, f32)>,
    pub gyro_history: std::collections::VecDeque<(f32, f32, f32)>,
    pub mag_history: std::collections::VecDeque<(f32, f32, f32)>,
}

lazy_static::lazy_static! {
    pub static ref GUI_STATE: Mutex<GuiState> = Mutex::new(GuiState {
        display_scan: Vec::new(),
        robot_x: 0.0,
        robot_y: 0.0,
        robot_theta: 0.0,
        slam_x: 0.0,
        slam_y: 0.0,
        slam_theta: 0.0,
        map_width: 0,
        map_height: 0,
        map_data: Vec::new(),
        frontiers: Vec::new(),
        current_path: Vec::new(),
        trajectory: Vec::new(),
        accel_history: std::collections::VecDeque::with_capacity(100),
        gyro_history: std::collections::VecDeque::with_capacity(100),
        mag_history: std::collections::VecDeque::with_capacity(100),
    });
}

/// Updates the persistent scan data by replacing a "slice" of the scan.
/// The slice is defined as the angular arc from the first point in new_points 
/// to the last point in new_points, following the rotation direction.
pub fn update_scan(display_scan: &mut Vec<LidarPoint>, new_points: Vec<LidarPoint>) {
    if new_points.is_empty() {
        return;
    }

    let first_angle = new_points.first().unwrap().angle_deg;
    let last_angle = new_points.last().unwrap().angle_deg;

    // Remove old points that fall within the new sector (with margin)
    if first_angle <= last_angle {
        display_scan.retain(|p| p.angle_deg < first_angle || p.angle_deg > last_angle);
    } else {
        // Wrap-around case
        display_scan.retain(|p| p.angle_deg < first_angle && p.angle_deg > last_angle);
    }

    // Add all new points
    display_scan.extend(new_points);

    // Keep the vector sorted by angle for rendering
    display_scan.sort_by(|a, b| a.angle_deg.partial_cmp(&b.angle_deg).unwrap_or(std::cmp::Ordering::Equal));
}

pub fn setup_lidar_drawing(lidar_canvas: &DrawingArea) {
    lidar_canvas.set_draw_func(move |_area, cr, width, height| {
        let state = GUI_STATE.lock().unwrap();
        let world_center_x = width as f64 / 2.0;
        let world_center_y = height as f64 / 2.0;
        let scale = 0.05; // 1mm = 0.05px (1m = 50px)

        cr.set_source_rgb(0.05, 0.05, 0.1);
        cr.paint().unwrap();

        // Draw Map (Occupancy Grid)
        if !state.map_data.is_empty() {
            let res = 0.05; // 5cm
            let m_scale = res as f64 * 1000.0 * scale; // pixels per grid cell
            
            for y in 0..state.map_height {
                for x in 0..state.map_width {
                    let val = state.map_data[y * state.map_width + x];
                    if val == 0 { continue; } // Unknown

                    let color = if val > 0 { 
                        // Probability of occupied (Gray to Black)
                        let intensity = 0.8 - (val as f64 / 100.0 * 0.7);
                        (intensity, intensity, intensity)
                    } else { 
                        // Probability of free (Dark Blue)
                        (0.05, 0.05, 0.15) 
                    };
                    cr.set_source_rgb(color.0, color.1, color.2);

                    // Map origin is at the center of the grid
                    let world_x = (x as f64 - state.map_width as f64 / 2.0) * res as f64;
                    let world_y = (y as f64 - state.map_height as f64 / 2.0) * res as f64;

                    let dx = world_center_x - (world_y * 1000.0 * scale);
                    let dy = world_center_y - (world_x * 1000.0 * scale);

                    cr.rectangle(dx - m_scale/2.0, dy - m_scale/2.0, m_scale, m_scale);
                    cr.fill().unwrap();
                }
            }
        }

        // Draw World Grid (Fixed)
        cr.set_source_rgb(0.15, 0.15, 0.2);
        cr.set_line_width(0.5);
        for i in -10..=10 {
            let pos = i as f64 * 1000.0 * scale; // 1m grid
            cr.move_to(world_center_x + pos, 0.0);
            cr.line_to(world_center_x + pos, height as f64);
            cr.move_to(0.0, world_center_y + pos);
            cr.line_to(width as f64, world_center_y + pos);
            cr.stroke().unwrap();
        }

        // Draw Frontiers (Yellow points)
        cr.set_source_rgb(1.0, 1.0, 0.0);
        for f in &state.frontiers {
            let dx = world_center_x - (f.centroid_y as f64 * 1000.0 * scale);
            let dy = world_center_y - (f.centroid_x as f64 * 1000.0 * scale);
            cr.arc(dx, dy, 3.0, 0.0, 2.0 * std::f64::consts::PI);
            cr.fill().unwrap();
        }

        // Draw Planned Path (Blue line)
        if !state.current_path.is_empty() {
            cr.set_source_rgb(0.0, 0.5, 1.0);
            cr.set_line_width(2.0);
            let mut first = true;
            for p in &state.current_path {
                let dx = world_center_x - (p.1 as f64 * 1000.0 * scale);
                let dy = world_center_y - (p.0 as f64 * 1000.0 * scale);
                if first {
                    cr.move_to(dx, dy);
                    first = false;
                } else {
                    cr.line_to(dx, dy);
                }
            }
            cr.stroke().unwrap();
        }

        // Draw Trajectory (Corrected)
        cr.set_source_rgb(1.0, 1.0, 0.0);
        cr.set_line_width(1.0);
        cr.set_dash(&[5.0, 5.0], 0.0);
        if let Some((first_x, first_y)) = state.trajectory.first() {
            // wx = first_x, wy = first_y
            cr.move_to(
                world_center_x - (*first_y as f64 * 1000.0 * scale),
                world_center_y - (*first_x as f64 * 1000.0 * scale)
            );
            for (tx, ty) in state.trajectory.iter().skip(1) {
                cr.line_to(
                    world_center_x - (*ty as f64 * 1000.0 * scale),
                    world_center_y - (*tx as f64 * 1000.0 * scale)
                );
            }
            cr.stroke().unwrap();
        }
        cr.set_dash(&[], 0.0);

        // Calculate Robot Position on Canvas (Corrected for Screen Coordinates)
        // Gazebo X (Forward) -> Screen -Y (Up)
        // Gazebo Y (Left) -> Screen -X (Left)
        let robot_draw_x = world_center_x - (state.robot_y as f64 * 1000.0 * scale);
        let robot_draw_y = world_center_y - (state.robot_x as f64 * 1000.0 * scale);
        
        // Robot Theta 0 is +X (Forward/Up). 
        // Screen orientation: Up is -PI/2.
        // Because screen Y is inverted, increasing Theta (CCW) should move the 
        // screen vector towards the Left (-PI).
        let robot_theta = state.robot_theta as f64;
        let screen_theta = -std::f64::consts::FRAC_PI_2 - robot_theta;

        // Draw Robot
        cr.set_source_rgb(1.0, 0.3, 0.3);
        cr.arc(robot_draw_x, robot_draw_y, 10.0, 0.0, 2.0 * std::f64::consts::PI);
        cr.fill().unwrap();

        // Draw Robot Heading (White vector)
        cr.set_source_rgb(1.0, 1.0, 1.0);
        cr.set_line_width(2.0);
        cr.move_to(robot_draw_x, robot_draw_y);
        cr.line_to(
            robot_draw_x + (15.0 * screen_theta.cos()),
            robot_draw_y + (15.0 * screen_theta.sin())
        );
        cr.stroke().unwrap();

        // Draw current Lidar Scan (Corrected for Pose)
        cr.set_source_rgba(0.0, 1.0, 0.5, 0.8);
        for p in &state.display_scan {
            if p.distance_mm < 10.0 {
                continue;
            }
            
            // Lidar point in robot frame (Lidar 0 is Front)
            let angle_robot_rad = (p.angle_deg as f64).to_radians();
            
            // Total angle in world frame
            let total_angle_world = robot_theta + angle_robot_rad;
            
            // Transform to World Coordinates (Gazebo Frame)
            let wx = (state.robot_x as f64 * 1000.0) + (p.distance_mm as f64 * total_angle_world.cos());
            let wy = (state.robot_y as f64 * 1000.0) + (p.distance_mm as f64 * total_angle_world.sin());
            
            // Project to Screen Coordinates
            let dx = world_center_x - (wy * scale);
            let dy = world_center_y - (wx * scale);
            
            cr.arc(dx, dy, 2.0, 0.0, 2.0 * std::f64::consts::PI);
            cr.fill().unwrap();
        }
    });
}

pub fn draw_imu_plot(cr: &cairo::Context, width: f64, height: f64, history: &std::collections::VecDeque<(f32, f32, f32)>, auto_scale: bool) {
    cr.set_source_rgb(0.0, 0.0, 0.0);
    cr.paint().unwrap();

    if history.is_empty() { return; }

    let dx = width / 100.0;
    let mid_y = height / 2.0;

    // Find Max for scaling
    let mut max_val = 1.0f32;
    if auto_scale {
        for (x, y, z) in history {
            max_val = max_val.max(x.abs()).max(y.abs()).max(z.abs());
        }
    } else {
        max_val = 10.0; // Default for Accel (approx 1g)
    }
    let scale_y = (height / 2.0) / max_val as f64 * 0.8;

    // Draw Zero line
    cr.set_source_rgb(0.2, 0.2, 0.2);
    cr.set_line_width(1.0);
    cr.move_to(0.0, mid_y);
    cr.line_to(width, mid_y);
    cr.stroke().unwrap();

    let draw_axis = |cr: &cairo::Context, axis_idx: usize, color: (f64, f64, f64)| {
        cr.set_source_rgb(color.0, color.1, color.2);
        cr.set_line_width(1.5);
        for (i, sample) in history.iter().enumerate() {
            let val = match axis_idx {
                0 => sample.0,
                1 => sample.1,
                _ => sample.2,
            };
            let px = i as f64 * dx;
            let py = mid_y - (val as f64 * scale_y);
            if i == 0 { cr.move_to(px, py); } else { cr.line_to(px, py); }
        }
        cr.stroke().unwrap();
    };

    draw_axis(cr, 0, (1.0, 0.3, 0.3)); // X - Red
    draw_axis(cr, 1, (0.3, 1.0, 0.3)); // Y - Green
    draw_axis(cr, 2, (0.3, 0.3, 1.0)); // Z - Blue
}

pub fn setup_accel_plot(canvas: &DrawingArea) {
    canvas.set_draw_func(move |_area, cr, width, height| {
        let state = GUI_STATE.lock().unwrap();
        draw_imu_plot(cr, width as f64, height as f64, &state.accel_history, false);
    });
}

pub fn setup_gyro_plot(canvas: &DrawingArea) {
    canvas.set_draw_func(move |_area, cr, width, height| {
        let state = GUI_STATE.lock().unwrap();
        draw_imu_plot(cr, width as f64, height as f64, &state.gyro_history, true);
    });
}

pub fn setup_mag_plot(canvas: &DrawingArea) {
    canvas.set_draw_func(move |_area, cr, width, height| {
        let state = GUI_STATE.lock().unwrap();
        draw_imu_plot(cr, width as f64, height as f64, &state.mag_history, true);
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mock_point(angle: f32) -> LidarPoint {
        LidarPoint {
            angle_deg: angle,
            distance_mm: 1000.0,
            quality: 15,
            scan_completed: false,
        }
    }

    #[test]
    fn test_sector_replace_wrap_around() {
        // "If i have as old 1 2 3 and I get 4 1.5 the new vector should be 1.5 2 3 4"
        // Here 4 is the start of the sector, 1.5 is the end. 
        // Sector is [4, 360] and [0, 1.5]. 
        // 1.0 is inside [0, 1.5] -> removed.
        // 2.0 and 3.0 are outside -> preserved.
        let mut old = vec![mock_point(1.0), mock_point(2.0), mock_point(3.0)];
        let new = vec![mock_point(4.0), mock_point(1.5)];
        
        update_scan(&mut old, new);
        
        let angles: Vec<f32> = old.iter().map(|p| p.angle_deg).collect();
        assert_eq!(angles, vec![1.5, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_sector_replace_no_overlap() {
        // "IF i have as old 1 2 3 and i get 4 5 the new vector shoudld be 1 2 3 4 5"
        let mut old = vec![mock_point(1.0), mock_point(2.0), mock_point(3.0)];
        let new = vec![mock_point(4.0), mock_point(5.0)];
        
        update_scan(&mut old, new);
        
        let angles: Vec<f32> = old.iter().map(|p| p.angle_deg).collect();
        assert_eq!(angles, vec![1.0, 2.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn test_precision_boundary() {
        // "if the old scan have 1.9999 and the new scan have the starting range to 2 
        // the 1.9999 of the old scan should be preserverd."
        let mut old = vec![mock_point(1.9999)];
        let new = vec![mock_point(2.0), mock_point(3.0)];
        
        update_scan(&mut old, new);
        
        let angles: Vec<f32> = old.iter().map(|p| p.angle_deg).collect();
        assert!(angles.contains(&1.9999));
    }

    #[test]
    fn test_precision_removal() {
        // "But if the new scan start from 1 to 2 the 1.9999 point should be removed"
        let mut old = vec![mock_point(1.9999)];
        let new = vec![mock_point(1.0), mock_point(2.0)];
        
        update_scan(&mut old, new);
        
        let angles: Vec<f32> = old.iter().map(|p| p.angle_deg).collect();
        assert!(!angles.contains(&1.9999));
        assert_eq!(angles, vec![1.0, 2.0]);
    }
}
