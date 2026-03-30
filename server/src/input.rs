use std::collections::HashSet;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::io::{self, Write};
use std::process;

use crate::command::RobotCommand;
use crate::stats::Stats;
use sdl2::{event::Event as SdlEvent, keyboard::Keycode, JoystickSubsystem, Sdl};
use crossterm::event::{self, Event as CEvent, KeyCode as CKeyCode, KeyEventKind, KeyModifiers};

const POWER: u8 = 127;

#[derive(PartialEq, Debug, Clone, Copy)]
enum InputType {
    Keyboard,
    Joystick,
    Trigger,
    NoEvent,
}

#[repr(u8)]
enum JoystickButton {
    B = 1,
}

/* This struct is used to dump the events read by the SDL loop.*/
#[derive(Debug)]
struct InputState {
    left_stick_x: i16,
    left_stick_y: i16,
    right_trigger: u8,
    left_trigger: u8,
    b: bool,
    pressed_keys: HashSet<Keycode>,
}

pub fn print_joystick_info(joystick_subsystem: &JoystickSubsystem) {
    let num_joysticks = joystick_subsystem.num_joysticks().unwrap();
    println!("Detected {} joystick(s)\r", num_joysticks);

    for i in 0..num_joysticks {
        match joystick_subsystem.open(i) {
            Ok(joystick) => {
                println!("Name: {}\r", joystick.name());
                println!("Axes: {}\r", joystick.num_axes());
                println!("Buttons: {}\r", joystick.num_buttons());
                println!("Balls: {}\r", joystick.num_balls());
            },
            Err(e) => eprintln!("Failed to open joystick {}: {}\r", i, e),
        }
    }
}

pub fn print_summary() {
    println!("Keyboard control started. Use WASD keys to control the robot:\r");
    println!("W - Forward (both motors)\r");
    println!("A - Turn left (right motor only)\r");
    println!("D - Turn right (left motor only)\r");
    println!("S - Backward (both motors)\r");
    println!("L - Start Lidar\r");
    println!("K - Stop Lidar\r");
    println!("Space - Stop All (Motors + Lidar)\r");
    println!("T - Run Diagnostics (RPC)\r");
    println!("'q' - Quit (press twice)\r");
    println!("Keys will be detected immediately without pressing Enter!\r");
    println!("Unified input control started. Press ESC or 'q' twice to quit.\r");
}

pub fn handle_input(robot_command: Arc<Mutex<RobotCommand>>, sdl_context: &Sdl, stats: Arc<Stats>, quit_count: Arc<AtomicUsize>) {
    let joystick_subsystem = sdl_context.joystick().unwrap();
    let _controller = joystick_subsystem.open(0).ok();
    let mut event_pump = sdl_context.event_pump().unwrap();

    let mut state = InputState { 
        left_stick_x: 0, 
        left_stick_y: 0, 
        right_trigger: 0, 
        left_trigger: 0,
        b: false, 
        pressed_keys: HashSet::new() 
    };

    while stats.running.load(Ordering::Relaxed) || stats.active_connections.load(Ordering::SeqCst) > 0 {
        let mut last_read = InputType::NoEvent;
        
        // 1. Check Terminal Input (crossterm)
        if poll_crossterm_events(&mut state, stats.clone(), quit_count.clone()) {
            last_read = InputType::Keyboard;
        }

        // 2. Check SDL Input (joystick / window)
        if poll_sdl_events(&mut event_pump, &mut state, stats.clone(), quit_count.clone()) {
            last_read = InputType::Keyboard; // Or Joystick/Trigger depending on event, but Keyboard is used for WASD
            // Note: poll_sdl_events updates state for all types.
        }

        // Only process movement commands if the server is still "running"
        if stats.running.load(Ordering::Relaxed) {
            if last_read != InputType::NoEvent || state.b || state.left_stick_x != 0 || state.left_stick_y != 0 || state.left_trigger != 0 || state.right_trigger != 0 {
                 // For simplicity, we just elaborate every loop if there's any active input
                 // In a more optimized version, we'd only do this on change.
                 let command_to_send = elaborate_input(&mut state);
                 if let Some(cmd) = command_to_send {
                     if let Ok(mut rc) = robot_command.lock() {
                         *rc = cmd;
                     }
                 }
            }
        }

        std::thread::sleep(Duration::from_millis(10));
    }
}

fn poll_crossterm_events(state: &mut InputState, stats: Arc<Stats>, quit_count: Arc<AtomicUsize>) -> bool {
    let mut activity = false;
    while event::poll(Duration::from_millis(0)).unwrap_or(false) {
        if let Ok(CEvent::Key(key_event)) = event::read() {
            if key_event.kind == KeyEventKind::Press || key_event.kind == KeyEventKind::Repeat {
                let kind_str = if key_event.kind == KeyEventKind::Repeat { "Repeat" } else { "Pressed" };
                match key_event.code {
                    CKeyCode::Char('q') | CKeyCode::Esc => handle_quit(stats.clone(), quit_count.clone(), "QUIT"),
                    CKeyCode::Char('c') if key_event.modifiers.contains(KeyModifiers::CONTROL) => handle_quit(stats.clone(), quit_count.clone(), "CTRL+C"),
                    CKeyCode::Char('w') => { stats.log(&format!("[KEY] {} 'w'", kind_str)); state.pressed_keys.insert(Keycode::W); activity = true; }
                    CKeyCode::Char('a') => { stats.log(&format!("[KEY] {} 'a'", kind_str)); state.pressed_keys.insert(Keycode::A); activity = true; }
                    CKeyCode::Char('s') => { stats.log(&format!("[KEY] {} 's'", kind_str)); state.pressed_keys.insert(Keycode::S); activity = true; }
                    CKeyCode::Char('d') => { stats.log(&format!("[KEY] {} 'd'", kind_str)); state.pressed_keys.insert(Keycode::D); activity = true; }
                    CKeyCode::Char('l') => { stats.log(&format!("[KEY] {} 'l'", kind_str)); state.pressed_keys.insert(Keycode::L); activity = true; }
                    CKeyCode::Char('k') => { stats.log(&format!("[KEY] {} 'k'", kind_str)); state.pressed_keys.insert(Keycode::K); activity = true; }
                    CKeyCode::Char('x') => { stats.log(&format!("[KEY] {} 'x'", kind_str)); state.pressed_keys.insert(Keycode::X); activity = true; }
                    CKeyCode::Char(' ') => { stats.log(&format!("[KEY] {} 'Space'", kind_str)); state.pressed_keys.insert(Keycode::Space); activity = true; }
                    CKeyCode::Char('t') => { stats.log(&format!("[KEY] {} 't'", kind_str)); state.pressed_keys.insert(Keycode::T); activity = true; }
                    _ => {}
                }
            } else if key_event.kind == KeyEventKind::Release {
                match key_event.code {
                    CKeyCode::Char('w') => { stats.log("[KEY] Released 'w'"); state.pressed_keys.remove(&Keycode::W); activity = true; }
                    CKeyCode::Char('a') => { stats.log("[KEY] Released 'a'"); state.pressed_keys.remove(&Keycode::A); activity = true; }
                    CKeyCode::Char('s') => { stats.log("[KEY] Released 's'"); state.pressed_keys.remove(&Keycode::S); activity = true; }
                    CKeyCode::Char('d') => { stats.log("[KEY] Released 'd'"); state.pressed_keys.remove(&Keycode::D); activity = true; }
                    CKeyCode::Char('l') => { stats.log("[KEY] Released 'l'"); state.pressed_keys.remove(&Keycode::L); activity = true; }
                    CKeyCode::Char('k') => { stats.log("[KEY] Released 'k'"); state.pressed_keys.remove(&Keycode::K); activity = true; }
                    CKeyCode::Char('x') => { stats.log("[KEY] Released 'x'"); state.pressed_keys.remove(&Keycode::X); activity = true; }
                    CKeyCode::Char(' ') => { stats.log("[KEY] Released 'Space'"); state.pressed_keys.remove(&Keycode::Space); activity = true; }
                    CKeyCode::Char('t') => { stats.log("[KEY] Released 't'"); state.pressed_keys.remove(&Keycode::T); activity = true; }
                    _ => {}
                }
            }
        }
    }
    activity
}

fn poll_sdl_events(event_pump: &mut sdl2::EventPump, state: &mut InputState, stats: Arc<Stats>, quit_count: Arc<AtomicUsize>) -> bool {
    let mut activity = false;
    for event in event_pump.poll_iter() {
        match event {
            SdlEvent::Quit { .. } => handle_quit(stats.clone(), quit_count.clone(), "QUIT"),
            SdlEvent::KeyDown { keycode: Some(Keycode::C), keymod, .. } if keymod.contains(sdl2::keyboard::Mod::LCTRLMOD | sdl2::keyboard::Mod::RCTRLMOD) => {
                handle_quit(stats.clone(), quit_count.clone(), "CTRL+C");
            }
            SdlEvent::KeyDown { keycode: Some(Keycode::Escape), .. } | SdlEvent::KeyDown { keycode: Some(Keycode::Q), .. } => {
                handle_quit(stats.clone(), quit_count.clone(), "QUIT");
            }
            SdlEvent::KeyDown { keycode: Some(key), .. } => {
                stats.log(&format!("[KEY] Pressed '{:?}'", key));
                state.pressed_keys.insert(key);
                activity = true;
            },
            SdlEvent::KeyUp { keycode: Some(key), .. } => {
                stats.log(&format!("[KEY] Released '{:?}'", key));
                state.pressed_keys.remove(&key);
                activity = true;
            },
            SdlEvent::JoyAxisMotion { axis_idx, value, .. } => {
                match axis_idx {
                    0 => { state.left_stick_x = value.checked_neg().unwrap_or(i16::MAX); }
                    1 => { state.left_stick_y = value.checked_neg().unwrap_or(i16::MAX); }
                    2 => { state.left_trigger = ((value as f32 + i16::MAX as f32) / (2. * i16::MAX as f32) * u8::MAX as f32) as u8; }
                    5 => { state.right_trigger = ((value as f32 + i16::MAX as f32) / (2. * i16::MAX as f32) * u8::MAX as f32) as u8; }
                    _ => {}
                }
                activity = true;
            }
            SdlEvent::JoyButtonDown { button_idx, .. } => {
                if button_idx == JoystickButton::B as u8 { state.b = true; activity = true; }
            }
            SdlEvent::JoyButtonUp { button_idx, .. } => {
                if button_idx == JoystickButton::B as u8 { state.b = false; activity = true; }
            }
            _ => {}
        }
    }
    activity
}

fn handle_quit(stats: Arc<Stats>, quit_count: Arc<AtomicUsize>, reason: &str) {
    let count = quit_count.fetch_add(1, Ordering::SeqCst) + 1;
    if count == 1 {
        stats.running.store(false, Ordering::SeqCst);
        let _ = writeln!(io::stderr(), "\n\r[{}] Shutdown initiated. Press again to force exit.", reason);
    } else {
        let _ = crossterm::terminal::disable_raw_mode();
        let _ = writeln!(io::stderr(), "\n\r[{}] Force exit requested.", reason);
        process::exit(0);
    }
}

fn elaborate_input(state: &mut InputState) -> Option<RobotCommand> {
    // Priority 1: Joystick/Trigger
    if state.b {
        return Some(RobotCommand::StopAll);
    }

    if state.left_stick_x != 0 || state.left_stick_y != 0 {
        let x_norm = state.left_stick_x as f32 / i16::MAX as f32;
        let y_norm = state.left_stick_y as f32 / i16::MAX as f32;
        let left_motor_speed = (y_norm + x_norm) * u8::MAX as f32;
        let right_motor_speed = (y_norm - x_norm) * u8::MAX as f32;

        return Some(RobotCommand::MotorDirect {
            left_speed: left_motor_speed as i16,
            right_speed: right_motor_speed as i16,
        });
    }

    if state.left_trigger != 0 || state.right_trigger != 0 {
        return Some(RobotCommand::MotorDirect {
            left_speed: state.left_trigger as i16,
            right_speed: state.right_trigger as i16,
        });
    }

    // Priority 2: Keyboard
    if state.pressed_keys.is_empty() {
        return Some(RobotCommand::StopMoving);
    }

    if state.pressed_keys.contains(&Keycode::W) && state.pressed_keys.contains(&Keycode::D) {
        return Some(RobotCommand::MotorAngle { left_power: POWER, left_angle: 1.0, right_power: POWER / 2, right_angle: 1.0 });
    } else if state.pressed_keys.contains(&Keycode::W) && state.pressed_keys.contains(&Keycode::A) {
        return Some(RobotCommand::MotorAngle { left_power: POWER / 2, left_angle: 1.0, right_power: POWER, right_angle: 1.0 });
    } else if state.pressed_keys.contains(&Keycode::W) {
        return Some(RobotCommand::MotorAngle { left_power: POWER, left_angle: 1.0, right_power: POWER, right_angle: 1.0 });
    } else if state.pressed_keys.contains(&Keycode::S) {
        return Some(RobotCommand::MotorAngle { left_power: POWER, left_angle: -1.0, right_power: POWER, right_angle: -1.0 });
    } else if state.pressed_keys.contains(&Keycode::A) {
        return Some(RobotCommand::MotorAngle { left_power: 0, left_angle: 0.0, right_power: POWER, right_angle: 1.0 });
    } else if state.pressed_keys.contains(&Keycode::D) {
        return Some(RobotCommand::MotorAngle { left_power: POWER, left_angle: 1.0, right_power: 0, right_angle: 0.0 });
    } else if state.pressed_keys.contains(&Keycode::T) {
        return Some(RobotCommand::RunDiagnostic);
    } else if state.pressed_keys.contains(&Keycode::L) {
        return Some(RobotCommand::LidarControl { active: true, target_frequency_hz: 5.0 });
    } else if state.pressed_keys.contains(&Keycode::K) {
        return Some(RobotCommand::LidarControl { active: false, target_frequency_hz: 0.0 });
    } else if state.pressed_keys.contains(&Keycode::Space) {
        return Some(RobotCommand::StopAll);
    }

    None
}
