//! Likelihood field for scan-to-map matching (plan T7, design doc §4.1).
//!
//! A `u8` raster derived from the occupancy grid: 255 on confident-occupied
//! cells, Gaussian falloff (σ ≈ 1.5 cells) around them, 0 away from
//! structure. Only cells at or above `CELL_OCCUPIED` seed the field, so
//! unknown or weakly-hit cells never attract the matcher.
//!
//! Two resolutions: the fine level shares the grid's 5cm cells; the coarse
//! level is 4× decimated with each coarse cell holding the MAX of its fine
//! block, so a coarse score upper-bounds the fine score and coarse-to-fine
//! search cannot discard the true optimum.

#![allow(dead_code)] // wired into the matcher by plan T8

use crate::mapping::{cell_is_occupied, OccupancyGrid};

pub const COARSE_FACTOR: usize = 4;
/// Gaussian falloff width, in fine cells.
const SIGMA_CELLS: f32 = 1.5;
/// Kernel half-width: at 5 cells the Gaussian is < 1/255, i.e. exactly 0 in u8.
const KERNEL_RADIUS: usize = 5;
const KERNEL_SIZE: usize = 2 * KERNEL_RADIUS + 1;

fn gaussian_kernel() -> [[u8; KERNEL_SIZE]; KERNEL_SIZE] {
    let mut k = [[0u8; KERNEL_SIZE]; KERNEL_SIZE];
    let denom = 2.0 * SIGMA_CELLS * SIGMA_CELLS;
    for (dy, row) in k.iter_mut().enumerate() {
        for (dx, v) in row.iter_mut().enumerate() {
            let fx = dx as f32 - KERNEL_RADIUS as f32;
            let fy = dy as f32 - KERNEL_RADIUS as f32;
            *v = (255.0 * (-(fx * fx + fy * fy) / denom).exp()).round() as u8;
        }
    }
    k
}

pub struct LikelihoodField {
    pub width: usize,
    pub height: usize,
    fine: Vec<u8>,
    coarse_w: usize,
    coarse_h: usize,
    coarse: Vec<u8>,
    kernel: [[u8; KERNEL_SIZE]; KERNEL_SIZE],
}

impl LikelihoodField {
    pub fn new(width: usize, height: usize) -> Self {
        let coarse_w = width.div_ceil(COARSE_FACTOR);
        let coarse_h = height.div_ceil(COARSE_FACTOR);
        Self {
            width,
            height,
            fine: vec![0; width * height],
            coarse_w,
            coarse_h,
            coarse: vec![0; coarse_w * coarse_h],
            kernel: gaussian_kernel(),
        }
    }

    pub fn fine_at(&self, x: usize, y: usize) -> u8 {
        if x < self.width && y < self.height {
            self.fine[y * self.width + x]
        } else {
            0
        }
    }

    /// Coarse lookup takes COARSE indices (fine / COARSE_FACTOR).
    pub fn coarse_at(&self, cx: usize, cy: usize) -> u8 {
        if cx < self.coarse_w && cy < self.coarse_h {
            self.coarse[cy * self.coarse_w + cx]
        } else {
            0
        }
    }

    pub fn rebuild_full(&mut self, grid: &OccupancyGrid) {
        self.rebuild_region(grid, (0, 0, self.width - 1, self.height - 1));
    }

    /// Rebuilds the field around a dirty bounding box of occupied-state
    /// changes (from [`OccupancyGrid::take_dirty_occupancy`]). The rewrite
    /// area is the box expanded by the kernel radius — a flipped cell
    /// influences values that far out; contributors are scanned one radius
    /// further so cells just outside the rewrite area still stamp into it.
    pub fn rebuild_region(&mut self, grid: &OccupancyGrid, dirty: (usize, usize, usize, usize)) {
        debug_assert_eq!(grid.width, self.width);
        debug_assert_eq!(grid.height, self.height);
        let (dx0, dy0, dx1, dy1) = dirty;
        let r = KERNEL_RADIUS;
        // Rewrite window (clamped).
        let wx0 = dx0.saturating_sub(r);
        let wy0 = dy0.saturating_sub(r);
        let wx1 = (dx1 + r).min(self.width - 1);
        let wy1 = (dy1 + r).min(self.height - 1);

        for y in wy0..=wy1 {
            let row = y * self.width;
            self.fine[row + wx0..=row + wx1].fill(0);
        }

        // Contributor window: occupied cells whose kernel reaches the rewrite window.
        let sx0 = wx0.saturating_sub(r);
        let sy0 = wy0.saturating_sub(r);
        let sx1 = (wx1 + r).min(self.width - 1);
        let sy1 = (wy1 + r).min(self.height - 1);
        for y in sy0..=sy1 {
            for x in sx0..=sx1 {
                if !cell_is_occupied(grid.data[y * self.width + x]) {
                    continue;
                }
                self.stamp_clamped(x, y, (wx0, wy0, wx1, wy1));
            }
        }

        // Refresh every coarse cell overlapping the rewrite window.
        for cy in wy0 / COARSE_FACTOR..=wy1 / COARSE_FACTOR {
            for cx in wx0 / COARSE_FACTOR..=wx1 / COARSE_FACTOR {
                let mut best = 0u8;
                for fy in cy * COARSE_FACTOR..((cy + 1) * COARSE_FACTOR).min(self.height) {
                    for fx in cx * COARSE_FACTOR..((cx + 1) * COARSE_FACTOR).min(self.width) {
                        best = best.max(self.fine[fy * self.width + fx]);
                    }
                }
                self.coarse[cy * self.coarse_w + cx] = best;
            }
        }
    }

    /// Max-stamps the kernel centered on (x, y), writing only inside `win`.
    fn stamp_clamped(&mut self, x: usize, y: usize, win: (usize, usize, usize, usize)) {
        let (wx0, wy0, wx1, wy1) = win;
        let r = KERNEL_RADIUS as i64;
        for ky in 0..KERNEL_SIZE {
            let ty = y as i64 + ky as i64 - r;
            if ty < wy0 as i64 || ty > wy1 as i64 {
                continue;
            }
            let row = ty as usize * self.width;
            for kx in 0..KERNEL_SIZE {
                let tx = x as i64 + kx as i64 - r;
                if tx < wx0 as i64 || tx > wx1 as i64 {
                    continue;
                }
                let v = self.kernel[ky][kx];
                if v == 0 {
                    continue; // kernel corners round to zero; a max can't use them
                }
                let cell = &mut self.fine[row + tx as usize];
                if v > *cell {
                    *cell = v;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mapping::{CELL_OCCUPIED, OccupancyGrid};
    use std::time::Instant;

    fn grid_with(cells: &[(usize, usize, i16)]) -> OccupancyGrid {
        let mut g = OccupancyGrid::new(100, 100, 0.05);
        for &(x, y, v) in cells {
            g.data[y * g.width + x] = v;
        }
        g
    }

    fn expected(dist2: f32) -> u8 {
        (255.0 * (-dist2 / (2.0 * 1.5 * 1.5)).exp()).round() as u8
    }

    #[test]
    fn gaussian_falloff_around_a_single_occupied_cell() {
        let g = grid_with(&[(50, 50, CELL_OCCUPIED)]);
        let mut f = LikelihoodField::new(100, 100);
        f.rebuild_full(&g);

        assert_eq!(f.fine_at(50, 50), 255);
        assert_eq!(f.fine_at(51, 50), expected(1.0)); // ~204
        assert_eq!(f.fine_at(52, 50), expected(4.0)); // ~105
        assert_eq!(f.fine_at(51, 51), expected(2.0));
        assert_eq!(f.fine_at(55, 50), expected(25.0)); // ~1 at the kernel edge
        assert_eq!(f.fine_at(56, 50), 0, "zero beyond the kernel radius");
    }

    #[test]
    fn only_confident_occupied_cells_seed_the_field() {
        let g = grid_with(&[
            (20, 20, CELL_OCCUPIED - 1), // weak hit: not a seed
            (60, 60, -80),               // free: not a seed
        ]);
        let mut f = LikelihoodField::new(100, 100);
        f.rebuild_full(&g);
        assert!(f.fine.iter().all(|&v| v == 0));
    }

    #[test]
    fn coarse_upper_bounds_fine_everywhere() {
        let g = grid_with(&[(10, 10, 50), (11, 10, 90), (73, 31, CELL_OCCUPIED), (40, 88, 100)]);
        let mut f = LikelihoodField::new(100, 100);
        f.rebuild_full(&g);
        for y in 0..100 {
            for x in 0..100 {
                assert!(
                    f.coarse_at(x / COARSE_FACTOR, y / COARSE_FACTOR) >= f.fine_at(x, y),
                    "coarse < fine at ({}, {})",
                    x,
                    y
                );
            }
        }
    }

    #[test]
    fn dirty_region_rebuild_equals_full_rebuild() {
        let mut g = grid_with(&[(30, 30, 100), (31, 30, 100), (70, 70, 100)]);
        g.take_dirty_occupancy(); // direct data writes above are untracked; start clean

        let mut incremental = LikelihoodField::new(100, 100);
        incremental.rebuild_full(&g);

        // Mutate through the tracked API: new wall cell appears, one dies.
        // Grid center is (50,50); world coords for cell (52,50) = (0.1, 0.0).
        for _ in 0..2 {
            g.update_cell(0.1, 0.0, 20); // (52,50) crosses CELL_OCCUPIED
        }
        for _ in 0..20 {
            g.update_cell(-1.0, -1.0, -20); // (30,30) drops below and far past it
        }
        let dirty = g.take_dirty_occupancy().expect("occupancy flips must mark dirty");
        incremental.rebuild_region(&g, dirty);

        let mut full = LikelihoodField::new(100, 100);
        full.rebuild_full(&g);
        assert_eq!(incremental.fine, full.fine, "fine level must match a full rebuild");
        assert_eq!(incremental.coarse, full.coarse, "coarse level must match a full rebuild");
    }

    #[test]
    fn no_flip_no_dirty() {
        let mut g = OccupancyGrid::new(100, 100, 0.05);
        g.update_cell(0.0, 0.0, 5); // 0 -> 5: still not occupied
        assert!(g.take_dirty_occupancy().is_none());
        g.update_cell(0.0, 0.0, 20); // 5 -> 25: crosses the threshold
        assert!(g.take_dirty_occupancy().is_some());
    }

    #[test]
    fn full_rebuild_of_a_realistic_map_stays_fast() {
        // 600x600 with arena-scale wall structure (~2400 occupied cells).
        let mut g = OccupancyGrid::new(600, 600, 0.05);
        for i in 100..500 {
            for &(x, y) in &[(i, 100), (i, 499), (100, i), (499, i)] {
                g.data[y * g.width + x] = 100;
            }
            g.data[i * g.width + 300] = 100; // interior divider
        }
        let mut f = LikelihoodField::new(600, 600);
        let start = Instant::now();
        f.rebuild_full(&g);
        let elapsed = start.elapsed();
        assert!(f.fine_at(300, 100) == 255);
        // Budget rationale: the field rebuilds at most once per sweep and a
        // sweep period is 200ms at 5Hz; 25ms in a DEBUG build leaves an
        // order of magnitude of headroom (release is ~10-20x faster), and
        // per-sweep rebuilds are dirty-region, far smaller than this.
        assert!(
            elapsed.as_millis() < 25,
            "full rebuild took {:?} (debug build budget: 25ms)",
            elapsed
        );
    }
}
