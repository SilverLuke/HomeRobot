mod reader;
mod input;
mod sender;
mod constants;
mod stats;
mod gui;
mod command;
mod connection;

pub mod homerobot {
    include!(concat!(env!("OUT_DIR"), "/homerobot.rs"));
}

use gtk4::prelude::*;
use std::net::TcpListener;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::thread::sleep;
use std::time::Duration;
use std::{io, thread, process};
use std::io::Write;

use crate::input::{handle_input, print_summary};
use crate::command::RobotCommand;
use crate::stats::Stats;
use crate::gui::init_gui;
use crate::connection::handle_connection;

fn main() -> io::Result<()> {
    std::env::set_var("SDL_NO_SIGNAL_HANDLERS", "1");

    let stats = Stats::new();
    let robot_command = Arc::new(Mutex::new(RobotCommand::StopAll));
    let sig_count = Arc::new(AtomicUsize::new(0));

    // 1. Initialize GTK GUI
    let (gui_app, gui_tx) = init_gui(robot_command.clone());

    // 2. Signal Handling
    let sc = sig_count.clone();
    let stats_signal = stats.clone();
    ctrlc::set_handler(move || {
        let count = sc.fetch_add(1, Ordering::SeqCst) + 1;
        if count == 1 {
            stats_signal.running.store(false, Ordering::SeqCst);
            let _ = writeln!(io::stderr(), "\r\n[CTRL+C] Shutdown initiated. Press again to force exit.\r");
        } else {
            let _ = writeln!(io::stderr(), "\r\n[CTRL+C] Force exit requested.\r");
            process::exit(0);
        }
    }).expect("Error setting Ctrl-C handler");

    println!("======================================\r");
    println!("      Robot Control Server v1.0       \r");
    println!("      (with GTK4 Dashboard)           \r");
    println!("======================================\r");

    // 3. Start Listener
    let stats_server = stats.clone();
    let rc_server = robot_command.clone();
    let sig_count_server = sig_count.clone();
    let gui_tx_server = gui_tx.clone();
    thread::spawn(move || {
        let addr = "0.0.0.0:12345";
        let listener = TcpListener::bind(addr).expect("Could not bind");
        listener.set_nonblocking(true).ok();
        println!("[SERVER] Listening on {}...\r", addr);

        while stats_server.running.load(Ordering::Relaxed) && sig_count_server.load(Ordering::Relaxed) == 0 {
            match listener.accept() {
                Ok((stream, _)) => {
                    let rc = Arc::clone(&rc_server);
                    let st = Arc::clone(&stats_server);
                    let sc = Arc::clone(&sig_count_server);
                    let gtx = gui_tx_server.clone();
                    thread::spawn(move || handle_connection(stream, rc, st, sc, gtx));
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                    sleep(Duration::from_millis(100));
                }
                _ => {}
            }
        }
    });

    // 4. Input Loop (Background Thread)
    let rc_input = robot_command.clone();
    let st_input = stats.clone();
    let sc_input = sig_count.clone();
    thread::spawn(move || {
        if let Ok(sdl_context) = sdl2::init() {
            print_summary();
            handle_input(rc_input, &sdl_context, st_input, sc_input);
        }
    });

    // 5. Run GTK GUI on Main Thread
    gui_app.run();

    println!("\n\r[SHUTDOWN] Exiting...\r");
    Ok(())
}
