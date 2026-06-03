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

pub fn plan_path(grid: &OccupancyGrid, start_x: f32, start_y: f32, goal_x: f32, goal_y: f32) -> Option<Vec<(f32, f32)>> {
    let (sx, sy) = grid.world_to_grid(start_x, start_y)?;
    let (gx, gy) = grid.world_to_grid(goal_x, goal_y)?;

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

                // Collision Check: Avoid cells that are likely occupied (log-odds > -10)
                // (We use a safe margin, so even "Unknown" or "Low confidence free" is blocked)
                if grid.data[ny * grid.width + nx] > -20 {
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
