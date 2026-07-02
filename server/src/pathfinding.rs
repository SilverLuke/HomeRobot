use std::collections::{BinaryHeap, HashMap};
use std::cmp::Ordering;
use crate::mapping::OccupancyGrid;

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

/// Checks if a circular region of a given radius in the occupancy grid is free of obstacles.
///
/// Returns `true` if all cells within the circular radius around the center `(cx, cy)`
/// are free space per the shared [`crate::mapping::cell_is_free`] classification.
/// Returns `false` if any cell is occupied/unknown or if the region goes out of bounds.
fn is_obstacle_free(grid: &OccupancyGrid, cx: usize, cy: usize, radius: usize) -> bool {
    let r_i32 = radius as i32;
    for dy in -r_i32..=r_i32 {
        for dx in -r_i32..=r_i32 {
            // Enforce a circular neighborhood check
            if dx * dx + dy * dy <= r_i32 * r_i32 {
                let nx = cx as i32 + dx;
                let ny = cy as i32 + dy;
                if nx >= 0 && nx < grid.width as i32 && ny >= 0 && ny < grid.height as i32 {
                    let idx = (ny as usize) * grid.width + (nx as usize);
                    if !crate::mapping::cell_is_free(grid.data[idx]) {
                        return false;
                    }
                } else {
                    return false; // Treat out of bounds as obstacle
                }
            }
        }
    }
    true
}

/// Finds the nearest cell to `(cx, cy)` that is obstacle-free including the inflation safety radius.
///
/// If `(cx, cy)` is already safe, returns it immediately. Otherwise, searches outward
/// in concentric squares up to 40 cells away (2.0 meters at 0.05m resolution) to find
/// the closest cell that satisfies the circular inflation safety requirement.
/// Falls back to `(cx, cy)` if no safe cell is found.
fn find_nearest_free_cell(grid: &OccupancyGrid, cx: usize, cy: usize, inflation_radius: usize) -> (usize, usize) {
    if is_obstacle_free(grid, cx, cy, inflation_radius) {
        return (cx, cy);
    }
    for r in 1..40 { // Search up to 40 cells (2.0m)
        let r_i32 = r as i32;
        for dx in -r_i32..=r_i32 {
            for dy in -r_i32..=r_i32 {
                if dx.abs() == r_i32 || dy.abs() == r_i32 {
                    let nx = cx as i32 + dx;
                    let ny = cy as i32 + dy;
                    if nx >= 0 && nx < grid.width as i32 && ny >= 0 && ny < grid.height as i32 {
                        let nx = nx as usize;
                        let ny = ny as usize;
                        if is_obstacle_free(grid, nx, ny, inflation_radius) {
                            return (nx, ny);
                        }
                    }
                }
            }
        }
    }
    (cx, cy)
}

/// Plans a path from `(start_x, start_y)` to `(goal_x, goal_y)` using A* search.
///
/// Converts the start and goal positions to grid indices and snaps them to the nearest
/// free cells satisfying the safety inflation radius.
/// Enforces obstacle inflation (5 cells = 25cm radius, representing the 15cm physical
/// robot radius + 10cm safety margin at 0.05m grid resolution) to prevent the robot
/// from getting stuck in narrow passages or scraping walls.
pub fn plan_path(grid: &OccupancyGrid, start_x: f32, start_y: f32, goal_x: f32, goal_y: f32) -> Option<Vec<(f32, f32)>> {
    let (sx, sy) = grid.world_to_grid(start_x, start_y)?;
    let (gx, gy) = grid.world_to_grid(goal_x, goal_y)?;

    let inflation_radius = 5; // 25cm (5 cells * 0.05m/cell) = 15cm robot radius + 10cm safety margin

    let (sx, sy) = find_nearest_free_cell(grid, sx, sy, inflation_radius);
    let (gx, gy) = find_nearest_free_cell(grid, gx, gy, inflation_radius);

    let mut open_set = BinaryHeap::new();
    let mut came_from = HashMap::new();
    let mut g_score = HashMap::new();
    
    g_score.insert((sx, sy), 0);
    open_set.push(Node { x: sx, y: sy, f_score: heuristic(sx, sy, gx, gy) });

    while let Some(current) = open_set.pop() {
        if current.x == gx && current.y == gy {
            return Some(reconstruct_path(grid, came_from, (gx, gy)));
        }

        for &(dx, dy) in &[(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)] {
            let nx = current.x as i32 + dx;
            let ny = current.y as i32 + dy;

            if nx >= 0 && nx < grid.width as i32 && ny >= 0 && ny < grid.height as i32 {
                let nx = nx as usize;
                let ny = ny as usize;

                // Collision Check: Avoid cells that violate the inflation safety radius
                if !is_obstacle_free(grid, nx, ny, inflation_radius) {
                    continue;
                }

                let tentative_g = g_score[&(current.x, current.y)] + 1;
                if tentative_g < *g_score.get(&(nx, ny)).unwrap_or(&i32::MAX) {
                    came_from.insert((nx, ny), (current.x, current.y));
                    g_score.insert((nx, ny), tentative_g);
                    open_set.push(Node {
                        x: nx,
                        y: ny,
                        f_score: tentative_g + heuristic(nx, ny, gx, gy),
                    });
                }
            }
        }
    }

    None
}

fn heuristic(x1: usize, y1: usize, x2: usize, y2: usize) -> i32 {
    let dx = (x1 as i32 - x2 as i32).abs();
    let dy = (y1 as i32 - y2 as i32).abs();
    dx + dy // Manhattan distance
}

fn reconstruct_path(grid: &OccupancyGrid, came_from: HashMap<(usize, usize), (usize, usize)>, mut current: (usize, usize)) -> Vec<(f32, f32)> {
    let mut path = Vec::new();
    while let Some(&prev) = came_from.get(&current) {
        // Convert grid to world
        let wx = (current.0 as f32 - grid.width as f32 / 2.0) * grid.resolution + grid.origin_x;
        let wy = (current.1 as f32 - grid.height as f32 / 2.0) * grid.resolution + grid.origin_y;
        path.push((wx, wy));
        current = prev;
    }
    path.reverse();
    path
}
