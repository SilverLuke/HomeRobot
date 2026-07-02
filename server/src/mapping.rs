#[derive(Clone, Debug, PartialEq)]
pub struct Frontier {
    pub centroid_x: f32,
    pub centroid_y: f32,
    pub size: usize,
}

pub struct OccupancyGrid {
    pub width: usize,
    pub height: usize,
    pub resolution: f32, // meters per pixel
    pub origin_x: f32,   // world coordinates of grid center
    pub origin_y: f32,
    pub data: Vec<i16>,  // Log-odds probability: 0 is unknown, >0 is likely occupied, <0 is likely free
}

const LOG_ODDS_OCCUPIED: i16 = 20; // Increase probability on hit
const LOG_ODDS_FREE: i16 = -5;     // Decrease probability on free ray
const LOG_ODDS_MAX: i16 = 100;     // Saturation cap
const LOG_ODDS_MIN: i16 = -100;

impl OccupancyGrid {
    pub fn new(width: usize, height: usize, resolution: f32) -> Self {
        Self {
            width,
            height,
            resolution,
            origin_x: 0.0,
            origin_y: 0.0,
            data: vec![0; width * height], // Start with 0 (50% probability in log-odds)
        }
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
            self.data[idx] = (current + delta).clamp(LOG_ODDS_MIN, LOG_ODDS_MAX);
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
                if self.data[idx] < -10 && !visited[idx] {
                    if self.is_frontier_cell(x, y) {
                        let cluster = self.expand_frontier_cluster(x, y, &mut visited);
                        if cluster.len() > 15 { // Ignore tiny frontiers / noise (minimum 75cm width)
                            frontiers.push(self.cluster_to_frontier(cluster));
                        }
                    }
                }
            }
        }
        frontiers
    }

    fn is_frontier_cell(&self, x: usize, y: usize) -> bool {
        // Must be free space
        if self.data[y * self.width + x] >= -10 { return false; }

        // Check 4-neighbors for unknown (log-odds == 0)
        for &(dx, dy) in &[(0, 1), (0, -1), (1, 0), (-1, 0)] {
            let nx = (x as i32 + dx) as usize;
            let ny = (y as i32 + dy) as usize;
            if self.data[ny * self.width + nx] == 0 {
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
            self.data[idx] = (current + LOG_ODDS_FREE).clamp(LOG_ODDS_MIN, LOG_ODDS_MAX);

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
            let pixel = if val > 10 {
                0 // Definitely occupied (Black)
            } else if val < -5 {
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
