use crate::homerobot::LidarPoint;
use gtk4::prelude::*;
use gtk4::DrawingArea;
use std::sync::Mutex;

pub struct GuiState {
    pub display_scan: Vec<LidarPoint>,
}

lazy_static::lazy_static! {
    pub static ref GUI_STATE: Mutex<GuiState> = Mutex::new(GuiState {
        display_scan: Vec::new(),
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

    // Remove old points that fall within the new sector
    if first_angle <= last_angle {
        // Normal case: sector is [first_angle, last_angle]
        // We retain points that are OUTSIDE this range
        display_scan.retain(|p| p.angle_deg < first_angle || p.angle_deg > last_angle);
    } else {
        // Wrap-around case: sector is [first_angle, 360] and [0, last_angle]
        // We retain points that are BETWEEN last and first (the "gap")
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
        let center_x = width as f64 / 2.0;
        let center_y = height as f64 / 2.0;
        let scale = 0.05;

        cr.set_source_rgb(0.05, 0.05, 0.1);
        cr.paint().unwrap();

        cr.set_source_rgb(0.2, 0.2, 0.3);
        cr.set_line_width(1.0);
        for i in 1..16 {
            let r = i as f64 * 500.0 * scale;
            cr.arc(center_x, center_y, r, 0.0, 2.0 * std::f64::consts::PI);
            cr.stroke().unwrap();
        }

        cr.set_source_rgb(1.0, 0.3, 0.3);
        cr.arc(center_x, center_y, 5.0, 0.0, 2.0 * std::f64::consts::PI);
        cr.fill().unwrap();

        cr.set_source_rgba(0.0, 1.0, 0.5, 0.8);
        for p in &state.display_scan {
            if p.distance_mm < 10.0 {
                continue;
            }
            let angle_rad = (p.angle_deg as f64).to_radians();
            let x = center_x + (p.distance_mm as f64 * scale * angle_rad.sin());
            let y = center_y - (p.distance_mm as f64 * scale * angle_rad.cos());
            
            cr.arc(x, y, 2.0, 0.0, 2.0 * std::f64::consts::PI);
            cr.fill().unwrap();
        }
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
