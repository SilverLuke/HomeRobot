use std::sync::atomic::{AtomicUsize, AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use indicatif::{ProgressBar, ProgressStyle};
use std::time::Duration;

pub struct Stats {
    pub active_connections: AtomicUsize,
    pub total_rx: AtomicUsize,
    pub total_tx: AtomicUsize,
    pub running: AtomicBool,
    pb: Mutex<Option<ProgressBar>>,
}

impl Stats {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            active_connections: AtomicUsize::new(0),
            total_rx: AtomicUsize::new(0),
            total_tx: AtomicUsize::new(0),
            running: AtomicBool::new(true),
            pb: Mutex::new(None),
        })
    }

    /// Log a message without messing up the bottom bar
    pub fn log(&self, msg: &str) {
        if let Ok(pb_guard) = self.pb.lock() {
            if let Some(pb) = pb_guard.as_ref() {
                pb.println(format!("{}\r", msg));
                return;
            }
        }
        println!("{}\r", msg);
    }

    pub fn clear(&self) {
        self.running.store(false, Ordering::SeqCst);
        if let Ok(mut pb_guard) = self.pb.lock() {
            if let Some(pb) = pb_guard.take() {
                pb.finish_and_clear();
            }
        }
    }

    pub fn start_display_thread(self: Arc<Self>) {
        let stats = self.clone();
        std::thread::spawn(move || {
            let pb = ProgressBar::new_spinner();
            pb.set_style(
                ProgressStyle::with_template(
                    " {spinner:.cyan} Connections: {msg} | RX: {rx} | TX: {tx} "
                )
                .unwrap()
                .tick_chars("⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏")
            );
            
            if let Ok(mut pb_guard) = stats.pb.lock() {
                *pb_guard = Some(pb.clone());
            }

            while stats.running.load(Ordering::Relaxed) {
                let conn = stats.active_connections.load(Ordering::Relaxed);
                let rx = stats.total_rx.load(Ordering::Relaxed);
                let tx = stats.total_tx.load(Ordering::Relaxed);

                pb.set_message(format!("{}", conn));
                pb.set_style(
                    ProgressStyle::with_template(&format!(
                        " {{spinner:.cyan}} Connections: {} | RX: {} | TX: {} ",
                        conn,
                        format_bytes(rx),
                        format_bytes(tx)
                    ))
                    .unwrap()
                );
                pb.tick();
                std::thread::sleep(Duration::from_millis(200));
            }
            pb.finish_and_clear();
        });
    }
}

fn format_bytes(bytes: usize) -> String {
    if bytes < 1024 {
        format!("{} B", bytes)
    } else if bytes < 1024 * 1024 {
        format!("{:.1} KB", bytes as f64 / 1024.0)
    } else {
        format!("{:.1} MB", bytes as f64 / (1024.0 * 1024.0))
    }
}
