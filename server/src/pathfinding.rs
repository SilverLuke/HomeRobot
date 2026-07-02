use std::collections::BinaryHeap;
use std::cmp::Ordering;
use crate::mapping::OccupancyGrid;

/// Robot clearance in grid cells: 15cm robot radius + 10cm safety margin at
/// 0.05m resolution.
const INFLATION_RADIUS: usize = 5;
/// Grid step costs scaled by 10 so diagonals can cost ~sqrt(2) as integers.
const STRAIGHT_COST: i32 = 10;
const DIAGONAL_COST: i32 = 14;

#[derive(Copy, Clone, Eq, PartialEq)]
struct Node {
    x: usize,
    y: usize,
    f_score: i32,
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f_score.cmp(&self.f_score) // Reverse for min-heap
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Grid of cells safe for the robot center: free space with no obstacle or
/// unknown cell within [`INFLATION_RADIUS`]. Built once per plan instead of
/// re-scanning an O(r²) disc per neighbor expansion.
struct Costmap {
    width: usize,
    height: usize,
    safe: Vec<bool>,
}

impl Costmap {
    fn build(grid: &OccupancyGrid) -> Self {
        let (w, h) = (grid.width, grid.height);
        let r = INFLATION_RADIUS as i32;

        let free: Vec<bool> = grid.data.iter().map(|&v| crate::mapping::cell_is_free(v)).collect();
        let mut safe = free.clone();

        // Precompute the disc offsets once.
        let mut disc = Vec::new();
        for dy in -r..=r {
            for dx in -r..=r {
                if dx * dx + dy * dy <= r * r && (dx != 0 || dy != 0) {
                    disc.push((dx, dy));
                }
            }
        }

        // Erode free space around every non-free cell. Only obstacle cells
        // that BORDER free space need to erode: any free cell within the disc
        // of an interior obstacle cell is also within the disc of a boundary
        // one — this keeps a fresh, mostly-unknown map cheap to process.
        let has_free_neighbor = |x: usize, y: usize| -> bool {
            for dy in -1i32..=1 {
                for dx in -1i32..=1 {
                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;
                    if nx >= 0 && nx < w as i32 && ny >= 0 && ny < h as i32
                        && free[ny as usize * w + nx as usize]
                    {
                        return true;
                    }
                }
            }
            false
        };

        for y in 0..h {
            for x in 0..w {
                if free[y * w + x] || !has_free_neighbor(x, y) {
                    continue;
                }
                for &(dx, dy) in &disc {
                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;
                    if nx >= 0 && nx < w as i32 && ny >= 0 && ny < h as i32 {
                        safe[ny as usize * w + nx as usize] = false;
                    }
                }
            }
        }

        // Out-of-bounds counts as obstacle: the border band cannot fit the disc.
        let ru = INFLATION_RADIUS;
        for y in 0..h {
            for x in 0..w {
                if x < ru || y < ru || x >= w - ru || y >= h - ru {
                    safe[y * w + x] = false;
                }
            }
        }

        Self { width: w, height: h, safe }
    }

    fn is_safe(&self, x: usize, y: usize) -> bool {
        x < self.width && y < self.height && self.safe[y * self.width + x]
    }
}

/// Finds the nearest safe cell to `(cx, cy)`, searching outward in concentric
/// squares up to 40 cells (2.0m). Falls back to `(cx, cy)` if none is found.
fn find_nearest_safe_cell(costmap: &Costmap, cx: usize, cy: usize) -> (usize, usize) {
    if costmap.is_safe(cx, cy) {
        return (cx, cy);
    }
    for r in 1..40 {
        let r_i32 = r as i32;
        for dx in -r_i32..=r_i32 {
            for dy in -r_i32..=r_i32 {
                if dx.abs() == r_i32 || dy.abs() == r_i32 {
                    let nx = cx as i32 + dx;
                    let ny = cy as i32 + dy;
                    if nx >= 0 && ny >= 0 && costmap.is_safe(nx as usize, ny as usize) {
                        return (nx as usize, ny as usize);
                    }
                }
            }
        }
    }
    (cx, cy)
}

/// Octile distance: admissible for 8-connected grids with 10/14 step costs.
fn heuristic(x1: usize, y1: usize, x2: usize, y2: usize) -> i32 {
    let dx = (x1 as i32 - x2 as i32).abs();
    let dy = (y1 as i32 - y2 as i32).abs();
    STRAIGHT_COST * (dx + dy) + (DIAGONAL_COST - 2 * STRAIGHT_COST) * dx.min(dy)
}

/// Plans a path from `(start_x, start_y)` to `(goal_x, goal_y)` using A* over
/// the inflated costmap. Start and goal snap to the nearest safe cell.
pub fn plan_path(grid: &OccupancyGrid, start_x: f32, start_y: f32, goal_x: f32, goal_y: f32) -> Option<Vec<(f32, f32)>> {
    let (sx, sy) = grid.world_to_grid(start_x, start_y)?;
    let (gx, gy) = grid.world_to_grid(goal_x, goal_y)?;

    let costmap = Costmap::build(grid);
    let (sx, sy) = find_nearest_safe_cell(&costmap, sx, sy);
    let (gx, gy) = find_nearest_safe_cell(&costmap, gx, gy);
    if !costmap.is_safe(sx, sy) || !costmap.is_safe(gx, gy) {
        return None;
    }

    let w = grid.width;
    let mut open_set = BinaryHeap::new();
    let mut came_from: Vec<u32> = vec![u32::MAX; w * grid.height];
    let mut g_score: Vec<i32> = vec![i32::MAX; w * grid.height];
    let mut closed: Vec<bool> = vec![false; w * grid.height];

    g_score[sy * w + sx] = 0;
    open_set.push(Node { x: sx, y: sy, f_score: heuristic(sx, sy, gx, gy) });

    const STEPS: [(i32, i32, i32); 8] = [
        (0, 1, STRAIGHT_COST), (0, -1, STRAIGHT_COST), (1, 0, STRAIGHT_COST), (-1, 0, STRAIGHT_COST),
        (1, 1, DIAGONAL_COST), (1, -1, DIAGONAL_COST), (-1, 1, DIAGONAL_COST), (-1, -1, DIAGONAL_COST),
    ];

    while let Some(current) = open_set.pop() {
        let c_idx = current.y * w + current.x;
        if closed[c_idx] {
            continue; // stale heap entry
        }
        closed[c_idx] = true;

        if current.x == gx && current.y == gy {
            return Some(reconstruct_path(grid, &came_from, (gx, gy)));
        }

        for &(dx, dy, step_cost) in &STEPS {
            let nx = current.x as i32 + dx;
            let ny = current.y as i32 + dy;
            if nx < 0 || ny < 0 {
                continue;
            }
            let (nx, ny) = (nx as usize, ny as usize);
            if !costmap.is_safe(nx, ny) {
                continue;
            }
            let n_idx = ny * w + nx;
            if closed[n_idx] {
                continue;
            }

            let tentative_g = g_score[c_idx] + step_cost;
            if tentative_g < g_score[n_idx] {
                came_from[n_idx] = c_idx as u32;
                g_score[n_idx] = tentative_g;
                open_set.push(Node {
                    x: nx,
                    y: ny,
                    f_score: tentative_g + heuristic(nx, ny, gx, gy),
                });
            }
        }
    }

    None
}

fn reconstruct_path(grid: &OccupancyGrid, came_from: &[u32], goal: (usize, usize)) -> Vec<(f32, f32)> {
    let w = grid.width;
    let mut path = Vec::new();
    let mut current = goal.1 * w + goal.0;
    while came_from[current] != u32::MAX {
        let (cx, cy) = (current % w, current / w);
        let wx = (cx as f32 - grid.width as f32 / 2.0) * grid.resolution + grid.origin_x;
        let wy = (cy as f32 - grid.height as f32 / 2.0) * grid.resolution + grid.origin_y;
        path.push((wx, wy));
        current = came_from[current] as usize;
    }
    path.reverse();
    path
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mapping::{CELL_FREE, CELL_OCCUPIED};
    use std::time::Instant;

    const FREE: i16 = CELL_FREE - 10;
    const WALL: i16 = CELL_OCCUPIED + 20;

    /// Grid entirely free except the outer border.
    fn open_grid(size: usize) -> OccupancyGrid {
        let mut g = OccupancyGrid::new(size, size, 0.05);
        g.data.fill(FREE);
        g
    }

    fn set(g: &mut OccupancyGrid, x: usize, y: usize, v: i16) {
        let w = g.width;
        g.data[y * w + x] = v;
    }

    fn path_length(path: &[(f32, f32)]) -> f32 {
        path.windows(2)
            .map(|p| ((p[1].0 - p[0].0).powi(2) + (p[1].1 - p[0].1).powi(2)).sqrt())
            .sum()
    }

    #[test]
    fn straight_line_in_open_space_is_near_optimal() {
        let g = open_grid(100);
        let path = plan_path(&g, -1.0, 0.0, 1.0, 0.0).expect("path in open space");
        // 2.0m straight; allow one cell of slack for start/goal snapping.
        assert!(path_length(&path) <= 2.0 + 0.1, "length = {}", path_length(&path));
        // No zigzag: every waypoint stays on the straight corridor.
        for (_, y) in &path {
            assert!(y.abs() <= 0.051, "waypoint strayed off the straight line: y = {}", y);
        }
    }

    #[test]
    fn diagonal_path_costs_sqrt2_not_manhattan() {
        let g = open_grid(100);
        let path = plan_path(&g, -1.0, -1.0, 1.0, 1.0).expect("diagonal path");
        // Euclidean distance is ~2.83m; the old Manhattan-cost planner
        // accepted anything up to 4.0m.
        assert!(path_length(&path) <= 2.83 + 0.15, "length = {}", path_length(&path));
    }

    #[test]
    fn path_around_obstacle_respects_inflation() {
        let mut g = open_grid(100);
        // Vertical wall through the middle with a gap at the top.
        for y in 10..80 {
            set(&mut g, 50, y, WALL);
        }
        let path = plan_path(&g, -1.0, 0.0, 1.0, 0.0).expect("path around wall");
        // Every waypoint keeps the inflation distance (5 cells = 0.25m) from the wall.
        for &(x, y) in &path {
            if (x - 0.0).abs() < 0.05 {
                // Crossing the wall line: must be above the wall segment
                // (y >= 80 in grid = 1.5m) plus clearance.
                assert!(y > 1.5 + 0.2, "crossed the wall too close: ({}, {})", x, y);
            }
        }
    }

    #[test]
    fn unreachable_goal_returns_none() {
        let mut g = open_grid(100);
        // Seal a box around the goal.
        for i in 60..=80 {
            set(&mut g, i, 60, WALL);
            set(&mut g, i, 80, WALL);
            set(&mut g, 60, i, WALL);
            set(&mut g, 80, i, WALL);
        }
        // Goal at grid (70, 70) => world (1.0, 1.0)
        assert!(plan_path(&g, -1.0, -1.0, 1.0, 1.0).is_none());
    }

    #[test]
    fn full_size_map_plans_quickly() {
        let mut g = open_grid(600);
        // Sprinkle some obstacle structure.
        for y in 100..500 {
            set(&mut g, 300, y, WALL);
        }
        let start = Instant::now();
        let path = plan_path(&g, -10.0, -10.0, 10.0, 10.0);
        let elapsed = start.elapsed();
        assert!(path.is_some());
        assert!(elapsed.as_millis() < 500, "planning took {:?} (debug build)", elapsed);
    }
}
