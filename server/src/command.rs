use std::sync::{mpsc, Arc, Mutex};

/// Level-triggered navigation mode. Set with [`Command::SetMode`] and retained
/// by the session until replaced; distinct from one-shot commands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NavMode {
    /// No autonomous control; robot only moves on explicit Drive commands.
    Manual,
    /// Frontier-based autonomous exploration.
    Exploration,
    /// Drive to a fixed goal in map coordinates, then return to Manual.
    NavigateTo { x: f32, y: f32 },
}

/// Result of an RPC round-trip to the robot.
pub type RpcResult = Result<(), String>;

/// Edge-triggered command from the GUI or the proxy. Each command is queued
/// and consumed exactly once by the robot session — unlike the previous
/// shared-cell design, commands cannot overwrite each other.
#[derive(Debug, Clone)]
pub enum Command {
    /// Direct differential-drive request (manual control). Implies Manual mode.
    Drive {
        left_power: u8,
        left_angle: f32,
        right_power: u8,
        right_angle: f32,
    },
    /// Stop the motors and drop to Manual mode.
    StopMoving,
    /// Emergency stop (robot-side StopAll). Drops to Manual mode.
    StopAll,
    LidarControl {
        active: bool,
        target_frequency_hz: f32,
    },
    UpdateConfig {
        left_kp: f32,
        left_ki: f32,
        left_kd: f32,
        right_kp: f32,
        right_ki: f32,
        right_kd: f32,
        lidar_frequency: f32,
    },
    RunDiagnostic,
    /// Closed-loop motion RPC. Implies Manual mode. Completion is reported on
    /// `reply` once the robot answers (correlated by call_id).
    ExecuteMotion {
        motion_type: i32,
        left_ticks: i32,
        right_ticks: i32,
        max_power: u32,
        reply: Option<mpsc::Sender<RpcResult>>,
    },
    /// Server-side: persist the current SLAM map to disk.
    SaveMap,
    /// Server-side: reset odometry, SLAM map and navigation state.
    Reset,
    SetMode(NavMode),
}

/// Fan-out queue connecting command producers (GUI, proxy) to robot sessions.
/// Sessions subscribe on connect and are pruned automatically on disconnect.
/// Sending never blocks.
#[derive(Clone)]
pub struct CommandBus {
    subscribers: Arc<Mutex<Vec<mpsc::Sender<Command>>>>,
}

impl CommandBus {
    pub fn new() -> Self {
        Self {
            subscribers: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// Register a new session; returns the receiving end of its command queue.
    pub fn subscribe(&self) -> mpsc::Receiver<Command> {
        let (tx, rx) = mpsc::channel();
        self.subscribers.lock().unwrap().push(tx);
        rx
    }

    pub fn send(&self, cmd: Command) {
        let mut subs = self.subscribers.lock().unwrap();
        subs.retain(|tx| tx.send(cmd.clone()).is_ok());
        if subs.is_empty() {
            log::warn!("[BUS] No active robot session; dropped command: {:?}", cmd);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bus_delivers_to_live_subscribers_and_prunes_dead_ones() {
        let bus = CommandBus::new();
        drop(bus.subscribe()); // dead session
        let rx = bus.subscribe();

        bus.send(Command::StopMoving);
        assert!(matches!(rx.try_recv(), Ok(Command::StopMoving)));
        assert_eq!(bus.subscribers.lock().unwrap().len(), 1);
    }
}
