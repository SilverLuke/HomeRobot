use std::cmp::PartialEq;
use std::collections::HashSet;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::input::TYPE::{Keyboard, NoEvent};
use crate::sender::RobotCommand;
use sdl2::{event::Event as SdlEvent, keyboard::Keycode, EventPump, JoystickSubsystem, Sdl};
use crossterm::event::{self, Event as CEvent, KeyCode as CKeyCode, KeyEventKind, KeyModifiers};

const POWER: u8 = 127;

#[derive(PartialEq)]
enum TYPE {
    Keyboard,
    Joystick,
    Trigger,
    NoEvent,
}

#[repr(u8)]
enum JoystickButton {
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LB = 4,
    RB = 5,
    Back,
    Start,
    Home,
    L3,
    R3,
}

/* This struct is used to dump the events read by the SDL loop.*/
#[derive(Debug)]
struct Input {
    left_stick_x: i16,
    left_stick_y: i16,
    right_trigger: u8,
    left_trigger: u8,
    a: bool,
    b: bool,
    x: bool,
    y: bool,
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
    println!("Space - Stop\r");
    println!("'q' - Quit (press twice)\r");
    println!("Keys will be detected immediately without pressing Enter!\r");
    println!("Unified input control started. Press ESC or 'q' twice to quit.\r");
}

use crate::stats::Stats;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::io::{self, Write};
use std::process;

pub fn handle_input(robot_command: Arc<Mutex<RobotCommand>>, sdl_context: &Sdl, stats: Arc<Stats>, quit_count: Arc<AtomicUsize>) {
    let joystick_subsystem = sdl_context.joystick().unwrap();
    let _controller = joystick_subsystem.open(0).ok();
    let mut event_pump = sdl_context.event_pump().unwrap();

    let mut inputs = Input { left_stick_x: 0, left_stick_y: 0, right_trigger: 0, left_trigger: 0,
        a: false, b: false, x: false, y: false, pressed_keys: HashSet::new() };

    // Continue running as long as we are either in "running" mode 
    // or we are gracefully waiting for connections to close.
    while stats.running.load(Ordering::Relaxed) || stats.active_connections.load(Ordering::SeqCst) > 0 {
        let mut last_read = NoEvent;
        
        // 1. Check Terminal Input (crossterm)
        if event::poll(Duration::from_millis(0)).unwrap_or(false) {
            if let Ok(CEvent::Key(key_event)) = event::read() {
                if key_event.kind == KeyEventKind::Press {
                    match key_event.code {
                        CKeyCode::Char('q') | CKeyCode::Esc => {
                            let count = quit_count.fetch_add(1, Ordering::SeqCst) + 1;
                            if count == 1 {
                                stats.running.store(false, Ordering::SeqCst);
                                let _ = writeln!(io::stderr(), "\n\r[QUIT] Shutdown initiated. Press 'q' again to force exit.");
                            } else {
                                let _ = crossterm::terminal::disable_raw_mode();
                                let _ = writeln!(io::stderr(), "\n\r[QUIT] Force exit requested.");
                                process::exit(0);
                            }
                        }
                        CKeyCode::Char('c') if key_event.modifiers.contains(KeyModifiers::CONTROL) => {
                            let count = quit_count.fetch_add(1, Ordering::SeqCst) + 1;
                            if count == 1 {
                                stats.running.store(false, Ordering::SeqCst);
                                let _ = writeln!(io::stderr(), "\n\r[CTRL+C] Shutdown initiated. Press again to force exit.");
                            } else {
                                let _ = crossterm::terminal::disable_raw_mode();
                                let _ = writeln!(io::stderr(), "\n\r[CTRL+C] Force exit requested.");
                                process::exit(0);
                            }
                        }
                        CKeyCode::Char('w') => { inputs.pressed_keys.insert(Keycode::W); last_read = Keyboard; }
                        CKeyCode::Char('a') => { inputs.pressed_keys.insert(Keycode::A); last_read = Keyboard; }
                        CKeyCode::Char('s') => { inputs.pressed_keys.insert(Keycode::S); last_read = Keyboard; }
                        CKeyCode::Char('d') => { inputs.pressed_keys.insert(Keycode::D); last_read = Keyboard; }
                        CKeyCode::Char(' ') => { inputs.pressed_keys.insert(Keycode::Space); last_read = Keyboard; }
                        _ => {}
                    }
                } else if key_event.kind == KeyEventKind::Release {
                    match key_event.code {
                        CKeyCode::Char('w') => { inputs.pressed_keys.remove(&Keycode::W); last_read = Keyboard; }
                        CKeyCode::Char('a') => { inputs.pressed_keys.remove(&Keycode::A); last_read = Keyboard; }
                        CKeyCode::Char('s') => { inputs.pressed_keys.remove(&Keycode::S); last_read = Keyboard; }
                        CKeyCode::Char('d') => { inputs.pressed_keys.remove(&Keycode::D); last_read = Keyboard; }
                        CKeyCode::Char(' ') => { inputs.pressed_keys.remove(&Keycode::Space); last_read = Keyboard; }
                        _ => {}
                    }
                }
            }
        }

        // 2. Check SDL Input (joystick / window)
        for event in event_pump.poll_iter() {
            match event {
                SdlEvent::Quit { .. } => {
                    let count = quit_count.fetch_add(1, Ordering::SeqCst) + 1;
                    if count == 1 {
                        stats.running.store(false, Ordering::SeqCst);
                        let _ = writeln!(io::stderr(), "\r\n[QUIT] Shutdown initiated. Press 'q' again to force exit.\r");
                    } else {
                        let _ = crossterm::terminal::disable_raw_mode();
                        let _ = writeln!(io::stderr(), "\r\n[QUIT] Force exit requested.\r");
                        process::exit(0);
                    }
                }
                SdlEvent::KeyDown { keycode: Some(Keycode::C), keymod, .. } if keymod.contains(sdl2::keyboard::Mod::LCTRLMOD | sdl2::keyboard::Mod::RCTRLMOD) => {
                    // This is handled by the signal handler usually, but here for SDL focus
                    let count = quit_count.fetch_add(1, Ordering::SeqCst) + 1;
                    if count == 1 {
                        stats.running.store(false, Ordering::SeqCst);
                        let _ = writeln!(io::stderr(), "\r\n[CTRL+C] Shutdown initiated. Press again to force exit.\r");
                    } else {
                        let _ = crossterm::terminal::disable_raw_mode();
                        let _ = writeln!(io::stderr(), "\r\n[CTRL+C] Force exit requested.\r");
                        process::exit(0);
                    }
                }
                SdlEvent::KeyDown { keycode: Some(Keycode::Escape), .. } | SdlEvent::KeyDown { keycode: Some(Keycode::Q), .. } => {
                    let count = quit_count.fetch_add(1, Ordering::SeqCst) + 1;
                    if count == 1 {
                        stats.running.store(false, Ordering::SeqCst);
                        let _ = writeln!(io::stderr(), "\r\n[QUIT] Shutdown initiated. Press 'q' again to force exit.\r");
                    } else {
                        let _ = crossterm::terminal::disable_raw_mode();
                        let _ = writeln!(io::stderr(), "\r\n[QUIT] Force exit requested.\r");
                        process::exit(0);
                    }
                }
                SdlEvent::KeyDown { keycode: Some(key), .. } => {
                    inputs.pressed_keys.insert(key);
                    last_read = Keyboard;
                },
                SdlEvent::KeyUp { keycode: Some(key), .. } => {
                    inputs.pressed_keys.remove(&key);
                    last_read = Keyboard;
                },
                SdlEvent::JoyAxisMotion {
                    axis_idx, value, ..
                } => {
                    match axis_idx {
                        0 => {
                            inputs.left_stick_x = value.checked_neg().unwrap_or(i16::MAX);
                            last_read = TYPE::Joystick;
                        }
                        1 => {
                            inputs.left_stick_y = value.checked_neg().unwrap_or(i16::MAX);
                            last_read = TYPE::Joystick;
                        }
                        2 => {
                            inputs.left_trigger = ((value as f32 + i16::MAX as f32)
                                / (2. * i16::MAX as f32)
                                * u8::MAX as f32) as u8;
                            last_read = TYPE::Trigger;
                        }
                        5 => {
                            inputs.right_trigger = ((value as f32 + i16::MAX as f32)
                                / (2. * i16::MAX as f32)
                                * u8::MAX as f32) as u8;
                            last_read = TYPE::Trigger;
                        }
                        _ => {}
                    }
                }
                SdlEvent::JoyButtonDown { button_idx, .. } => {
                    if button_idx == JoystickButton::B as u8 {
                        inputs.b = true;
                    }
                }
                SdlEvent::JoyButtonUp { button_idx, .. } => {
                    if button_idx == JoystickButton::B as u8 {
                        inputs.b = false;
                    }
                }
                _ => {}
            }
        }

        // Only process movement commands if the server is still "running"
        if stats.running.load(Ordering::Relaxed) {
            if last_read != TYPE::NoEvent {
                let command_to_send = elaborate_input(&mut inputs, last_read);
                
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

fn elaborate_input(inputs: &mut Input, input_type: TYPE) -> Option<RobotCommand> {
    match input_type {
        TYPE::NoEvent => {}
        TYPE::Keyboard => {
            let command = if inputs.pressed_keys.contains(&Keycode::W)
                && inputs.pressed_keys.contains(&Keycode::D)
            {
                // Forward and turn right
                Some(RobotCommand::MotorAngle {
                    left_power: POWER,
                    left_angle: 1.0,
                    right_power: POWER / 2,
                    right_angle: 1.0,
                })
            } else if inputs.pressed_keys.contains(&Keycode::W)
                && inputs.pressed_keys.contains(&Keycode::A)
            {
                // Forward and turn left
                Some(RobotCommand::MotorAngle {
                    left_power: POWER / 2,
                    left_angle: 1.0,
                    right_power: POWER,
                    right_angle: 1.0,
                })
            } else if inputs.pressed_keys.contains(&Keycode::W) {
                // Forward
                Some(RobotCommand::MotorAngle {
                    left_power: POWER,
                    left_angle: 1.0,
                    right_power: POWER,
                    right_angle: 1.0,
                })
            } else if inputs.pressed_keys.contains(&Keycode::S) {
                // Backward
                Some(RobotCommand::MotorAngle {
                    left_power: POWER,
                    left_angle: -1.0,
                    right_power: POWER,
                    right_angle: -1.0,
                })
            } else if inputs.pressed_keys.contains(&Keycode::A) {
                // Turn left
                Some(RobotCommand::MotorAngle {
                    left_power: 0,
                    left_angle: 0.0,
                    right_power: POWER,
                    right_angle: 1.0,
                })
            } else if inputs.pressed_keys.contains(&Keycode::D) {
                // Turn right
                Some(RobotCommand::MotorAngle {
                    left_power: POWER,
                    left_angle: 1.0,
                    right_power: 0,
                    right_angle: 0.0,
                })
            } else if inputs.pressed_keys.contains(&Keycode::Space) {
                Some(RobotCommand::default())
            } else {
                None
            };
            return command;
        }
        TYPE::Joystick => {
            if inputs.b {
                return Some(RobotCommand::StopAll);
            }
            // X and Y give the left/right direction and forward/backward direction
            let x_norm = inputs.left_stick_x as f32 / i16::MAX as f32;
            let y_norm = inputs.left_stick_y as f32 / i16::MAX as f32;
            // Stick x, y -> motor left, right
            // (0, 0) -> (0, 0)
            // (0, 1) -> (1, 1)
            // (1, 0) -> (1, -1)
            // (0, -1) -> (-1, -1)
            // (0, -1) -> (-1, 1)
            // A = [ 1 , 1 ]
            //     [ -1 , 1 ]   = A(x, y) = (y+x, y-x)

            let left_motor_speed = (y_norm + x_norm) * u8::MAX as f32;
            let right_motor_speed = (y_norm - x_norm) * u8::MAX as f32;

            return Some(RobotCommand::MotorDirect {
                left_speed: left_motor_speed as i16,
                right_speed: right_motor_speed as i16,
            });
        }
        TYPE::Trigger => {
            return Some(RobotCommand::MotorDirect {
                left_speed: inputs.left_trigger as i16,
                right_speed: inputs.right_trigger as i16,
            });
        }
    }
    None
}
