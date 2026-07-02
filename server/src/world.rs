//! Persistent world model: the SLAM map and pose estimate, owned by `main`
//! and shared with robot sessions. A reconnecting robot resumes with the
//! existing map instead of remapping from scratch, and the model is
//! periodically persisted to disk so it also survives server restarts.

use std::fs;
use std::io::{self, Read, Write};
use std::time::{Duration, Instant};

use crate::mapping::OccupancyGrid;
use crate::odometry::Pose;
use crate::slam::BasicSlam;

/// Raw log-odds snapshot (the PGM export is lossy and for humans).
pub const AUTOSAVE_PATH: &str = "house_map.bin";
const MAGIC: &[u8; 8] = b"HRWORLD1";
/// Refuse to load absurd dimensions from a corrupt file.
const MAX_CELLS: usize = 4096 * 4096;

pub struct WorldModel {
    pub slam: BasicSlam,
    /// Best pose estimate: SLAM-corrected on each sweep, dead-reckoned from
    /// encoder deltas in between.
    pub pose: Pose,
    pub pose_initialized: bool,
    dirty: bool,
    /// Lives here rather than in the session: sessions can be short-lived
    /// (reconnects), but the autosave cadence must span them.
    last_autosave: Instant,
}

impl WorldModel {
    pub fn new() -> Self {
        Self {
            slam: BasicSlam::new(),
            pose: Pose { x: 0.0, y: 0.0, theta: 0.0 },
            pose_initialized: false,
            dirty: false,
            last_autosave: Instant::now(),
        }
    }

    /// Load the persisted world, or start fresh if none/corrupt.
    pub fn load_or_new(path: &str) -> Self {
        match Self::load(path) {
            Ok(world) => {
                let (w, h, _) = {
                    use crate::slam::Slam;
                    world.slam.get_map_data()
                };
                log::info!(
                    "[WORLD] Restored map ({}x{}) and pose X={:.2}, Y={:.2} from {}",
                    w, h, world.pose.x, world.pose.y, path
                );
                world
            }
            Err(e) if e.kind() == io::ErrorKind::NotFound => {
                log::info!("[WORLD] No saved map at {}; starting fresh.", path);
                Self::new()
            }
            Err(e) => {
                log::warn!("[WORLD] Could not load {} ({}); starting fresh.", path, e);
                Self::new()
            }
        }
    }

    /// Called when a robot (re)connects. The server-side odometry restarts at
    /// zero for each connection, so SLAM must drop its odometry-frame tracking
    /// state — the map and the pose estimate survive.
    pub fn on_new_session(&mut self) {
        self.slam.on_odometry_reset();
    }

    pub fn reset(&mut self) {
        *self = WorldModel::new();
        // Make sure the next autosave overwrites the stale on-disk map.
        self.dirty = true;
    }

    pub fn mark_dirty(&mut self) {
        self.dirty = true;
    }

    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    /// True when there are unsaved changes and the autosave interval elapsed.
    pub fn autosave_due(&self, interval: Duration) -> bool {
        self.dirty && self.last_autosave.elapsed() >= interval
    }

    /// Atomically persist map + pose (write to a temp file, then rename).
    pub fn save(&mut self, path: &str) -> io::Result<()> {
        let tmp_path = format!("{}.tmp", path);
        {
            let mut file = io::BufWriter::new(fs::File::create(&tmp_path)?);
            let map = self.slam.map();
            file.write_all(MAGIC)?;
            file.write_all(&(map.width as u32).to_le_bytes())?;
            file.write_all(&(map.height as u32).to_le_bytes())?;
            file.write_all(&map.resolution.to_le_bytes())?;
            file.write_all(&map.origin_x.to_le_bytes())?;
            file.write_all(&map.origin_y.to_le_bytes())?;
            file.write_all(&self.pose.x.to_le_bytes())?;
            file.write_all(&self.pose.y.to_le_bytes())?;
            file.write_all(&self.pose.theta.to_le_bytes())?;
            file.write_all(&[self.pose_initialized as u8])?;
            let mut bytes = Vec::with_capacity(map.data.len() * 2);
            for &v in &map.data {
                bytes.extend_from_slice(&v.to_le_bytes());
            }
            file.write_all(&bytes)?;
            file.flush()?;
        }
        fs::rename(&tmp_path, path)?;
        self.dirty = false;
        self.last_autosave = Instant::now();
        Ok(())
    }

    pub fn load(path: &str) -> io::Result<Self> {
        let mut file = io::BufReader::new(fs::File::open(path)?);

        let mut magic = [0u8; 8];
        file.read_exact(&mut magic)?;
        if &magic != MAGIC {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "bad magic"));
        }

        fn read_u32(r: &mut impl Read) -> io::Result<u32> {
            let mut b = [0u8; 4];
            r.read_exact(&mut b)?;
            Ok(u32::from_le_bytes(b))
        }
        fn read_f32(r: &mut impl Read) -> io::Result<f32> {
            let mut b = [0u8; 4];
            r.read_exact(&mut b)?;
            Ok(f32::from_le_bytes(b))
        }

        let width = read_u32(&mut file)? as usize;
        let height = read_u32(&mut file)? as usize;
        if width * height == 0 || width * height > MAX_CELLS {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "bad dimensions"));
        }
        let resolution = read_f32(&mut file)?;
        let origin_x = read_f32(&mut file)?;
        let origin_y = read_f32(&mut file)?;
        let pose = Pose {
            x: read_f32(&mut file)?,
            y: read_f32(&mut file)?,
            theta: read_f32(&mut file)?,
        };
        let mut flag = [0u8; 1];
        file.read_exact(&mut flag)?;
        let pose_initialized = flag[0] != 0;

        let mut bytes = vec![0u8; width * height * 2];
        file.read_exact(&mut bytes)?;
        let data: Vec<i16> = bytes
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]))
            .collect();

        let mut map = OccupancyGrid::new(width, height, resolution);
        map.origin_x = origin_x;
        map.origin_y = origin_y;
        map.data = data;

        Ok(Self {
            slam: BasicSlam::restore(map, pose),
            pose,
            pose_initialized,
            dirty: false,
            last_autosave: Instant::now(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::slam::Slam;

    #[test]
    fn save_load_round_trips_map_and_pose() {
        let dir = std::env::temp_dir().join("homerobot_world_test");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("world.bin");
        let path = path.to_str().unwrap();

        let mut grid = OccupancyGrid::new(64, 32, 0.05);
        grid.data[100] = 40;
        grid.data[2047] = -60;
        let pose = Pose { x: 1.5, y: -0.75, theta: 0.5 };

        let mut world = WorldModel {
            slam: BasicSlam::restore(grid, pose),
            pose,
            pose_initialized: true,
            dirty: true,
            last_autosave: Instant::now(),
        };
        world.save(path).unwrap();
        assert!(!world.is_dirty());

        let loaded = WorldModel::load(path).unwrap();
        assert!(loaded.pose_initialized);
        assert!((loaded.pose.x - 1.5).abs() < 1e-6);
        assert!((loaded.pose.theta - 0.5).abs() < 1e-6);

        let (w, h, data) = loaded.slam.get_map_data();
        assert_eq!((w, h), (64, 32));
        assert_eq!(data[100], 40);
        assert_eq!(data[2047], -60);
        assert_eq!(data.iter().filter(|&&v| v != 0).count(), 2);

        std::fs::remove_file(path).ok();
    }

    #[test]
    fn corrupt_file_falls_back_to_fresh_world() {
        let dir = std::env::temp_dir().join("homerobot_world_test");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("corrupt.bin");
        std::fs::write(&path, b"not a map at all").unwrap();

        let world = WorldModel::load_or_new(path.to_str().unwrap());
        assert!(!world.pose_initialized);

        std::fs::remove_file(path).ok();
    }
}
