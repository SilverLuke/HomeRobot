use std::sync::atomic::{AtomicUsize, AtomicBool};
use std::sync::Arc;

pub struct Stats {
    pub active_connections: AtomicUsize,
    pub total_rx: AtomicUsize,
    pub total_tx: AtomicUsize,
    pub running: AtomicBool,
}

impl Stats {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            active_connections: AtomicUsize::new(0),
            total_rx: AtomicUsize::new(0),
            total_tx: AtomicUsize::new(0),
            running: AtomicBool::new(true),
        })
    }

    /// Log a message
    pub fn log(&self, msg: &str) {
        log::info!("{}", msg);
    }
}
