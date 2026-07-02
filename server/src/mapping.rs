#[derive(Clone, Debug, PartialEq)]
pub struct Frontier {
    pub centroid_x: f32,
    pub centroid_y: f32,
    pub size: usize,
}

#[derive(Clone)]
pub struct OccupancyGrid {
    pub width: usize,
    pub height: usize,
    pub resolution: f32, // meters per pixel
    pub origin_x: f32,   // world coordinates of grid center
    pub origin_y: f32,
    pub data: Vec<i16>,  // Log-odds probability: 0 is unknown, >0 is likely occupied, <0 is likely free
    /// Bounding box (min_x, min_y, max_x, max_y) of cells whose
    /// occupied-state flipped since the last [`take_dirty_occupancy`] call.
    /// Drives incremental likelihood-field rebuilds; only updates through
    /// [`update_cell`]/[`raytrace_free`] are tracked (direct `data` writes,
    /// as tests do, are not).
    dirty_occupancy: Option<(usize, usize, usize, usize)>,
}

const LOG_ODDS_OCCUPIED: i16 = 20; // Increase probability on hit
const LOG_ODDS_FREE: i16 = -5;     // Decrease probability on free ray
const LOG_ODDS_MAX: i16 = 100;     // Saturation cap
const LOG_ODDS_MIN: i16 = -100;

// Shared occupancy classification. Frontier detection, path planning and map
// export must agree on what counts as free, otherwise frontier detection can
// produce goals the planner considers unreachable by construction.
/// A cell is traversable free space at or below this log-odds value.
pub const CELL_FREE: i16 = -20;
/// A cell is an obstacle at or above this log-odds value.
pub const CELL_OCCUPIED: i16 = 20;

pub fn cell_is_free(v: i16) -> bool {
    v <= CELL_FREE
}

pub fn cell_is_occupied(v: i16) -> bool {
    v >= CELL_OCCUPIED
}

/// Never observed: no lidar hit and no free ray has touched it.
pub fn cell_is_unknown(v: i16) -> bool {
    v == 0
}

impl OccupancyGrid {
    pub fn new(width: usize, height: usize, resolution: f32) -> Self {
        Self {
            width,
            height,
            resolution,
            origin_x: 0.0,
            origin_y: 0.0,
            data: vec![0; width * height], // Start with 0 (50% probability in log-odds)
            dirty_occupancy: None,
        }
    }

    fn mark_dirty_cell(&mut self, x: usize, y: usize) {
        self.dirty_occupancy = Some(match self.dirty_occupancy {
            None => (x, y, x, y),
            Some((x0, y0, x1, y1)) => (x0.min(x), y0.min(y), x1.max(x), y1.max(y)),
        });
    }

    /// Consumes the dirty bounding box of occupied-state changes.
    #[allow(dead_code)] // consumed by the likelihood field; wired into SLAM by plan T10
    pub fn take_dirty_occupancy(&mut self) -> Option<(usize, usize, usize, usize)> {
        self.dirty_occupancy.take()
    }

    /// Converts world coordinates (meters) to grid indices
    pub fn world_to_grid(&self, x: f32, y: f32) -> Option<(usize, usize)> {
        let gx = ((x - self.origin_x) / self.resolution) + (self.width as f32 / 2.0);
        let gy = ((y - self.origin_y) / self.resolution) + (self.height as f32 / 2.0);

        if gx >= 0.0 && gx < self.width as f32 && gy >= 0.0 && gy < self.height as f32 {
            Some((gx as usize, gy as usize))
        } else {
            None
        }
    }

    /// Updates a cell probability
    pub fn update_cell(&mut self, x: f32, y: f32, delta: i16) {
        if let Some((gx, gy)) = self.world_to_grid(x, y) {
            let idx = gy * self.width + gx;
            let current = self.data[idx];
            let updated = (current + delta).clamp(LOG_ODDS_MIN, LOG_ODDS_MAX);
            self.data[idx] = updated;
            if cell_is_occupied(current) != cell_is_occupied(updated) {
                self.mark_dirty_cell(gx, gy);
            }
        }
    }

    /// Finds clusters of unknown cells adjacent to free space
    pub fn find_frontiers(&self) -> Vec<Frontier> {
        let mut visited = vec![false; self.width * self.height];
        let mut frontiers = Vec::new();

        for y in 1..self.height-1 {
            for x in 1..self.width-1 {
                let idx = y * self.width + x;
                
                // A frontier cell is a FREE cell with at least one UNKNOWN neighbor
                if cell_is_free(self.data[idx]) && !visited[idx] && self.is_frontier_cell(x, y) {
                    let cluster = self.expand_frontier_cluster(x, y, &mut visited);
                    if cluster.len() > 15 { // Ignore tiny frontiers / noise (minimum 75cm width)
                        frontiers.push(self.cluster_to_frontier(cluster));
                    }
                }
            }
        }
        frontiers
    }

    fn is_frontier_cell(&self, x: usize, y: usize) -> bool {
        // Must be free space
        if !cell_is_free(self.data[y * self.width + x]) { return false; }

        // Check 4-neighbors for unknown
        for &(dx, dy) in &[(0, 1), (0, -1), (1, 0), (-1, 0)] {
            let nx = (x as i32 + dx) as usize;
            let ny = (y as i32 + dy) as usize;
            if cell_is_unknown(self.data[ny * self.width + nx]) {
                return true;
            }
        }
        false
    }

    fn expand_frontier_cluster(&self, x: usize, y: usize, visited: &mut [bool]) -> Vec<(usize, usize)> {
        let mut cluster = Vec::new();
        let mut q = std::collections::VecDeque::new();
        q.push_back((x, y));
        visited[y * self.width + x] = true;

        while let Some((cx, cy)) = q.pop_front() {
            cluster.push((cx, cy));

            for &(dx, dy) in &[(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)] {
                let nx = (cx as i32 + dx) as usize;
                let ny = (cy as i32 + dy) as usize;
                
                if nx > 0 && nx < self.width-1 && ny > 0 && ny < self.height-1 {
                    let n_idx = ny * self.width + nx;
                    if !visited[n_idx] && self.is_frontier_cell(nx, ny) {
                        visited[n_idx] = true;
                        q.push_back((nx, ny));
                    }
                }
            }
        }
        cluster
    }

    fn cluster_to_frontier(&self, cluster: Vec<(usize, usize)>) -> Frontier {
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        for &(gx, gy) in &cluster {
            let wx = (gx as f32 - self.width as f32 / 2.0) * self.resolution + self.origin_x;
            let wy = (gy as f32 - self.height as f32 / 2.0) * self.resolution + self.origin_y;
            sum_x += wx;
            sum_y += wy;
        }
        Frontier {
            centroid_x: sum_x / cluster.len() as f32,
            centroid_y: sum_y / cluster.len() as f32,
            size: cluster.len(),
        }
    }

    /// Apply a deskewed LiDAR scan to the map
    pub fn update_from_deskewed_scan(&mut self, robot_pos: (f32, f32), deskewed_points: &[(f32, f32)]) {
        let (rx, ry) = match self.world_to_grid(robot_pos.0, robot_pos.1) {
            Some(coords) => coords,
            None => return,
        };

        for &(ox_world, oy_world) in deskewed_points {
            if let Some((ox, oy)) = self.world_to_grid(ox_world, oy_world) {
                self.update_cell(ox_world, oy_world, LOG_ODDS_OCCUPIED);
                self.raytrace_free(rx, ry, ox, oy);
            }
        }
    }

    fn raytrace_free(&mut self, x0: usize, y0: usize, x1: usize, y1: usize) {
        let mut x0 = x0 as i32;
        let mut y0 = y0 as i32;
        let x1 = x1 as i32;
        let y1 = y1 as i32;

        let dx = (x1 - x0).abs();
        let dy = -(y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;

        loop {
            if x0 == x1 && y0 == y1 { break; }

            let idx = (y0 as usize) * self.width + (x0 as usize);
            let current = self.data[idx];
            let updated = (current + LOG_ODDS_FREE).clamp(LOG_ODDS_MIN, LOG_ODDS_MAX);
            self.data[idx] = updated;
            if cell_is_occupied(current) != cell_is_occupied(updated) {
                self.mark_dirty_cell(x0 as usize, y0 as usize);
            }

            let e2 = 2 * err;
            if e2 >= dy {
                if x0 == x1 { break; }
                err += dy;
                x0 += sx;
            }
            if e2 <= dx {
                if y0 == y1 { break; }
                err += dx;
                y0 += sy;
            }
        }
    }

    /// Save the map as a PGM image
    pub fn save_pgm(&self, filename: &str) -> std::io::Result<()> {
        use std::fs::File;
        use std::io::Write;

        let mut file = File::create(filename)?;
        writeln!(file, "P5\n{} {}\n255", self.width, self.height)?;

        let mut pixels = Vec::with_capacity(self.width * self.height);
        for &val in &self.data {
            let pixel = if cell_is_occupied(val) {
                0 // Definitely occupied (Black)
            } else if cell_is_free(val) {
                255 // Definitely free (White)
            } else {
                127 // Unknown / Low confidence (Gray)
            };
            pixels.push(pixel);
        }

        file.write_all(&pixels)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FREE: i16 = CELL_FREE - 10;
    const WALL: i16 = CELL_OCCUPIED + 20;

    fn fill(g: &mut OccupancyGrid, x0: usize, y0: usize, x1: usize, y1: usize, v: i16) {
        for y in y0..=y1 {
            for x in x0..=x1 {
                g.data[y * g.width + x] = v;
            }
        }
    }

    /// 100x100 grid @5cm: a free room (30..70)² walled in, with an unknown
    /// opening in the right wall at y=40..60.
    fn room_with_opening() -> OccupancyGrid {
        let mut g = OccupancyGrid::new(100, 100, 0.05);
        fill(&mut g, 30, 30, 70, 70, FREE);
        fill(&mut g, 29, 29, 71, 29, WALL); // bottom wall
        fill(&mut g, 29, 71, 71, 71, WALL); // top wall
        fill(&mut g, 29, 29, 29, 71, WALL); // left wall
        fill(&mut g, 71, 29, 71, 71, WALL); // right wall
        fill(&mut g, 71, 40, 71, 60, 0); // opening: unknown
        g
    }

    #[test]
    fn world_to_grid_round_trips_and_rejects_out_of_bounds() {
        let g = OccupancyGrid::new(100, 100, 0.05);
        assert_eq!(g.world_to_grid(0.0, 0.0), Some((50, 50)));
        assert_eq!(g.world_to_grid(1.0, -1.0), Some((70, 30)));
        assert_eq!(g.world_to_grid(-2.5, 0.0), Some((0, 50)));
        assert_eq!(g.world_to_grid(10.0, 0.0), None);
        assert_eq!(g.world_to_grid(0.0, -2.51), None);
    }

    #[test]
    fn update_cell_clamps_log_odds() {
        let mut g = OccupancyGrid::new(10, 10, 0.05);
        for _ in 0..50 {
            g.update_cell(0.0, 0.0, LOG_ODDS_OCCUPIED);
        }
        assert_eq!(g.data[5 * 10 + 5], LOG_ODDS_MAX);
        for _ in 0..200 {
            g.update_cell(0.0, 0.0, LOG_ODDS_FREE);
        }
        assert_eq!(g.data[5 * 10 + 5], LOG_ODDS_MIN);
    }

    #[test]
    fn deskewed_scan_marks_hit_and_clears_ray_but_not_endpoint() {
        let mut g = OccupancyGrid::new(100, 100, 0.05);
        // Robot at origin, one obstacle hit 1m ahead.
        g.update_from_deskewed_scan((0.0, 0.0), &[(1.0, 0.0)]);

        let hit = g.data[50 * 100 + 70];
        let mid = g.data[50 * 100 + 60];
        let start = g.data[50 * 100 + 50];
        assert_eq!(hit, LOG_ODDS_OCCUPIED, "endpoint must stay marked as a hit");
        assert_eq!(mid, LOG_ODDS_FREE, "ray cells must be cleared");
        assert_eq!(start, LOG_ODDS_FREE, "robot cell is on the ray");
    }

    #[test]
    fn frontier_detected_at_room_opening() {
        let g = room_with_opening();
        let frontiers = g.find_frontiers();
        assert_eq!(frontiers.len(), 1, "expected exactly one frontier at the opening");
        let f = &frontiers[0];
        // The frontier hugs the inside of the opening: x=70 (1.0m), centered on y=50 (0.0m).
        assert!((f.centroid_x - 1.0).abs() < 0.05, "centroid_x = {}", f.centroid_x);
        assert!(f.centroid_y.abs() < 0.05, "centroid_y = {}", f.centroid_y);
        assert!(f.size > 15);
    }

    #[test]
    fn fully_enclosed_room_has_no_frontiers() {
        let mut g = room_with_opening();
        fill(&mut g, 71, 40, 71, 60, WALL); // close the opening
        assert!(g.find_frontiers().is_empty());
    }

    #[test]
    fn detected_frontiers_are_reachable_by_the_planner() {
        let g = room_with_opening();
        let frontiers = g.find_frontiers();
        assert!(!frontiers.is_empty());
        for f in frontiers {
            let path = crate::pathfinding::plan_path(&g, 0.0, 0.0, f.centroid_x, f.centroid_y);
            assert!(
                path.is_some(),
                "frontier at ({}, {}) must be plannable",
                f.centroid_x, f.centroid_y
            );
        }
    }
}
