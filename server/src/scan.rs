use std::collections::VecDeque;
use std::time::{Duration, Instant};

use crate::homerobot::LidarPoint;

/// A sweep needs at least this many points to be considered a real revolution.
const MIN_SWEEP_POINTS: usize = 30;
/// Minimum time between flushed sweeps (debounces duplicate sync markers).
const MIN_FLUSH_INTERVAL: Duration = Duration::from_millis(80);
/// Revolutions kept for the scan-rate average.
const RATE_WINDOW: usize = 5;
const SUMMARY_INTERVAL: Duration = Duration::from_secs(5);

/// One complete 360° lidar revolution, plus telemetry derived from it.
pub struct CompletedSweep {
    pub points: Vec<LidarPoint>,
    /// Rolling scan rate, available once two revolutions completed.
    pub rate_hz: Option<f32>,
    /// Human-readable 5-second statistics summary, emitted at most once per interval.
    pub summary: Option<String>,
}

/// Accumulates raw lidar points into complete revolutions. A point flagged
/// `scan_completed` marks the START of a new revolution (RP-Lidar SYNCBIT),
/// so everything buffered before it forms one full sweep.
pub struct ScanAssembler {
    pending: Vec<LidarPoint>,
    last_flush: Instant,
    revolution_times: VecDeque<Instant>,
    summary_started: Instant,
    summary_sweeps: usize,
    summary_total_points: usize,
    summary_sum_avg_delta: f32,
    summary_sum_std_dev: f32,
    summary_min_delta: f32,
    summary_max_delta: f32,
}

impl ScanAssembler {
    pub fn new() -> Self {
        Self {
            pending: Vec::new(),
            last_flush: Instant::now() - MIN_FLUSH_INTERVAL * 10,
            revolution_times: VecDeque::new(),
            summary_started: Instant::now(),
            summary_sweeps: 0,
            summary_total_points: 0,
            summary_sum_avg_delta: 0.0,
            summary_sum_std_dev: 0.0,
            summary_min_delta: f32::MAX,
            summary_max_delta: f32::MIN,
        }
    }

    /// Feed a batch of points; returns any sweeps completed by this batch.
    pub fn push(&mut self, points: Vec<LidarPoint>) -> Vec<CompletedSweep> {
        let mut completed = Vec::new();
        for p in points {
            if p.scan_completed {
                if self.pending.len() >= MIN_SWEEP_POINTS && self.last_flush.elapsed() >= MIN_FLUSH_INTERVAL {
                    completed.push(self.flush());
                }
                self.pending.clear();
            }
            self.pending.push(p);
        }
        completed
    }

    pub fn reset(&mut self) {
        *self = ScanAssembler::new();
    }

    fn flush(&mut self) -> CompletedSweep {
        self.last_flush = Instant::now();
        self.accumulate_stats();

        self.revolution_times.push_back(Instant::now());
        while self.revolution_times.len() > RATE_WINDOW {
            self.revolution_times.pop_front();
        }
        let rate_hz = if self.revolution_times.len() >= 2 {
            let span = self
                .revolution_times
                .back()
                .unwrap()
                .duration_since(*self.revolution_times.front().unwrap())
                .as_secs_f32();
            Some((self.revolution_times.len() as f32 - 1.0) / span)
        } else {
            None
        };

        CompletedSweep {
            points: std::mem::take(&mut self.pending),
            rate_hz,
            summary: self.take_summary(),
        }
    }

    /// Angular-spacing statistics of the pending sweep (rotation uniformity).
    fn accumulate_stats(&mut self) {
        if self.pending.len() < 2 {
            return;
        }
        let mut angles: Vec<f32> = self.pending.iter().map(|p| p.angle_deg).collect();
        angles.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let deltas: Vec<f32> = (0..angles.len())
            .map(|i| {
                let next = (i + 1) % angles.len();
                let mut diff = if next == 0 {
                    (angles[0] + 360.0) - angles[i]
                } else {
                    angles[next] - angles[i]
                };
                if diff < 0.0 {
                    diff += 360.0;
                }
                diff
            })
            .collect();

        let n = deltas.len() as f32;
        let avg = deltas.iter().sum::<f32>() / n;
        let variance = deltas.iter().map(|&d| (d - avg) * (d - avg)).sum::<f32>() / n;

        self.summary_sweeps += 1;
        self.summary_total_points += angles.len();
        self.summary_sum_avg_delta += avg;
        self.summary_sum_std_dev += variance.sqrt();
        for &d in &deltas {
            self.summary_min_delta = self.summary_min_delta.min(d);
            self.summary_max_delta = self.summary_max_delta.max(d);
        }
    }

    fn take_summary(&mut self) -> Option<String> {
        if self.summary_started.elapsed() < SUMMARY_INTERVAL || self.summary_sweeps == 0 {
            return None;
        }
        let sweeps = self.summary_sweeps as f32;
        let summary = format!(
            "[LIDAR Summary (last 5s)] Received {} updates, total {} points | Sweep points: {:.1}, Avg delta: {:.4}°, StdDev: {:.4}°, Min: {:.4}°, Max: {:.4}°",
            self.summary_sweeps,
            self.summary_total_points,
            self.summary_total_points as f32 / sweeps,
            self.summary_sum_avg_delta / sweeps,
            self.summary_sum_std_dev / sweeps,
            self.summary_min_delta,
            self.summary_max_delta,
        );
        self.summary_sweeps = 0;
        self.summary_total_points = 0;
        self.summary_sum_avg_delta = 0.0;
        self.summary_sum_std_dev = 0.0;
        self.summary_min_delta = f32::MAX;
        self.summary_max_delta = f32::MIN;
        self.summary_started = Instant::now();
        Some(summary)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(angle_deg: f32, scan_completed: bool) -> LidarPoint {
        LidarPoint { distance_mm: 1000.0, angle_deg, quality: 15, scan_completed }
    }

    #[test]
    fn completes_a_sweep_on_sync_marker() {
        let mut asm = ScanAssembler::new();
        let points: Vec<_> = (0..40).map(|i| point(i as f32 * 9.0, false)).collect();
        assert!(asm.push(points).is_empty());

        let sweeps = asm.push(vec![point(0.0, true)]);
        assert_eq!(sweeps.len(), 1);
        assert_eq!(sweeps[0].points.len(), 40);
    }

    #[test]
    fn discards_sweeps_with_too_few_points() {
        let mut asm = ScanAssembler::new();
        let points: Vec<_> = (0..10).map(|i| point(i as f32 * 36.0, false)).collect();
        assert!(asm.push(points).is_empty());
        assert!(asm.push(vec![point(0.0, true)]).is_empty());
    }
}
