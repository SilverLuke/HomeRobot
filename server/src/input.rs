use std::cmp::PartialEq;
use std::collections::HashSet;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::input::TYPE::{Keyboard, NoEvent};
use crate::sender::MotorCommand;
use sdl2::{event::Event, keyboard::Keycode, EventPump, JoystickSubsystem, Sdl};

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
    A = 10,
    B = 0,
    X,
    Y,
    LB,
    RB,
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
    println!("Detected {} joystick(s)", num_joysticks);

    for i in 0..num_joysticks {
        match joystick_subsystem.open(i) {
            Ok(joystick) => {
                println!("Name: {}", joystick.name());
                println!("Axes: {}", joystick.num_axes());
                println!("Buttons: {}", joystick.num_buttons());
                println!("Balls: {}", joystick.num_balls());
            },
            Err(e) => eprintln!("Failed to open joystick {}: {}", i, e),
        }
    }

}


pub fn print_summary() {
    println!("Keyboard control started. Use WASD keys to control the robot:");
    println!("W - Forward (both motors)");
    println!("A - Turn left (right motor only)");
    println!("D - Turn right (left motor only)");
    println!("S - Backward (both motors)");
    println!("Space - Stop");
    println!("'q' - Quit");
    println!("Keys will be detected immediately without pressing Enter!");
    println!("Unified input control started. Press ESC to quit.");
}

pub fn handle_input(motor_command: Arc<Mutex<MotorCommand>>, sdl_context: &Sdl) {
    let joystick_subsystem = sdl_context.joystick().unwrap();
    let _controller = joystick_subsystem.open(0).ok();
    let mut event_pump = sdl_context.event_pump().unwrap();

    let mut inputs = Input { left_stick_x: 0, left_stick_y: 0, right_trigger: 0, left_trigger: 0,
        a: false, b: false, x: false, y: false, pressed_keys: HashSet::new() };

    loop {
        let input_type = read_input(&mut event_pump, &mut inputs);
        if input_type == TYPE::NoEvent {
            std::thread::sleep(Duration::from_millis(10));
            continue;
        }
        // println!("Controller: {:?}", inputs);
        let command_to_send = elaborate_input(&mut inputs, input_type);
         
        //if command_to_send.is_some() {
        //    println!("Sending: {:?}", command_to_send);
        //}
        
        if let Some(cmd) = command_to_send {
            if let Ok(mut mc) = motor_command.lock() {
                *mc = cmd;
            }
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

/* Read input buffer some received input events.
 * To be able to compute the vector of the controller stick
 */
fn read_input(
    event_pump: &mut EventPump,
    controller: &mut Input,
) -> TYPE {
    
    let mut last_read = NoEvent;
    for event in event_pump.poll_iter() {
        match event {
            // Keyboard input suboptimal, I can't press W and D to go forward and turn right
            Event::KeyDown { keycode: Some(key), .. } => {
                controller.pressed_keys.insert(key);
                last_read = Keyboard;
            },
            Event::KeyUp { keycode: Some(key), .. } => {
                controller.pressed_keys.remove(&key);
                last_read = Keyboard;
            },
            /* Joystick analog axes:
             * 0+ DX 0- SX
             * 1+ BACK 1- FORWARD
             * 2 Left trigger
             * 5 Right trigger
             */
            Event::JoyAxisMotion {
                axis_idx, value, ..
            } => {
                match axis_idx {
                    0 => {
                        // Left/Right
                        controller.left_stick_x = value;
                        last_read = TYPE::Joystick;
                    }
                    1 => {
                        // Forward/Backward
                        controller.left_stick_y = value;
                        last_read = TYPE::Joystick;
                    }
                    2 => {
                        controller.left_trigger = ((value as f32 + i16::MAX as f32)
                            / (2. * i16::MAX as f32)
                            * u8::MAX as f32) as u8;
                        last_read = TYPE::Trigger;
                    }
                    5 => {
                        controller.right_trigger = ((value as f32 + i16::MAX as f32)
                            / (2. * i16::MAX as f32)
                            * u8::MAX as f32) as u8;
                        last_read = TYPE::Trigger;
                    }
                    _ => {}
                }
            }
            Event::JoyButtonDown { button_idx, .. } => {
                println!("Joystick button {} pressed", button_idx);
                if button_idx == JoystickButton::B as u8 {
                    controller.b = true;
                }
            }
            Event::JoyButtonUp { button_idx, .. } => {
                println!("Joystick button {} released", button_idx);
                if button_idx == JoystickButton::B as u8 {
                    controller.b = false;
                }
            }
            _ => {}
        }
    }
    last_read
}

fn elaborate_input(inputs: &mut Input, input_type: TYPE) -> Option<MotorCommand> {
    match input_type {
        TYPE::NoEvent => {}
        TYPE::Keyboard => {
            let command = if inputs.pressed_keys.contains(&Keycode::W)
                && inputs.pressed_keys.contains(&Keycode::D)
            {
                // Forward and turn right
                Some(MotorCommand {
                    left_power: POWER,
                    left_angle: 1.0,
                    right_power: POWER / 2,
                    right_angle: 1.0,
                })
            } else if inputs.pressed_keys.contains(&Keycode::W)
                && inputs.pressed_keys.contains(&Keycode::A)
            {
                // Forward and turn left
                Some(MotorCommand {
                    left_power: POWER / 2,
                    left_angle: 1.0,
                    right_power: POWER,
                    right_angle: 1.0,
                })
            } else if inputs.pressed_keys.contains(&Keycode::W) {
                // Forward
                Some(MotorCommand {
                    left_power: POWER,
                    left_angle: 1.0,
                    right_power: POWER,
                    right_angle: 1.0,
                })
            } else if inputs.pressed_keys.contains(&Keycode::S) {
                // Backward
                Some(MotorCommand {
                    left_power: POWER,
                    left_angle: -1.0,
                    right_power: POWER,
                    right_angle: -1.0,
                })
            } else if inputs.pressed_keys.contains(&Keycode::A) {
                // Turn left
                Some(MotorCommand {
                    left_power: 0,
                    left_angle: 0.0,
                    right_power: POWER,
                    right_angle: 1.0,
                })
            } else if inputs.pressed_keys.contains(&Keycode::D) {
                // Turn right
                Some(MotorCommand {
                    left_power: POWER,
                    left_angle: 1.0,
                    right_power: 0,
                    right_angle: 0.0,
                })
            } else if inputs.pressed_keys.contains(&Keycode::Space) {
                Some(MotorCommand::default())
            } else if inputs.pressed_keys.contains(&Keycode::Escape) ||
                inputs.pressed_keys.contains(&Keycode::Q) {
                println!("Exiting...");
                std::process::exit(0);
            } else {
                None
            };
            return command;
        }
        TYPE::Joystick => {
            if inputs.b {
                return Some(MotorCommand {
                    left_power: 0,
                    left_angle: 0.0,
                    right_power: 0,
                    right_angle: 0.0,
                });
            }

            let x_norm = inputs.left_stick_x as f32 / i16::MAX as f32;
            let y_norm = inputs.left_stick_y as f32 / i16::MAX as f32;
            let r = y_norm + x_norm;
            let l = y_norm - x_norm;
            let x = f32::abs(-0.2349).clamp(0.0, 3.4);
            return Some(MotorCommand {
                right_power: (r.abs() * u8::MAX as f32) as u8,
                right_angle: if f32::is_sign_positive(r) { -1. } else { 1. },
                left_power: (l.abs() * u8::MAX as f32) as u8,  
                left_angle: if f32::is_sign_positive(l) { -1. } else { 1. },
            });
        }
        TYPE::Trigger => {
            return Some(MotorCommand {
                left_power: inputs.left_trigger,
                left_angle: 1.0,
                right_power: inputs.right_trigger,
                right_angle: 1.0,
            });
        }
    }
    None
}


#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};
    use sdl2::keyboard::Keycode;

    // Helper function to create an empty MotorCommand instance
    fn create_motor_command() -> Arc<Mutex<MotorCommand>> {
        Arc::new(Mutex::new(MotorCommand::default()))
    }

    #[test]
    fn test_elaborate_input_no_event() {
        let mut inputs = Input {
            left_stick_x: 0,
            left_stick_y: 0,
            right_trigger: 0,
            left_trigger: 0,
            a: false,
            b: false,
            x: false,
            y: false,
            pressed_keys: HashSet::new(),
        };

        let command = elaborate_input(&mut inputs, TYPE::Keyboard);
        assert_eq!(command.is_some(), false);
    }
    
    #[test]
    fn test_elaborate_input_keyboard_w() {
        let mut set = HashSet::new();
        set.insert(Keycode::W);
        let mut inputs = Input {
            left_stick_x: 0,
            left_stick_y: 0,
            right_trigger: 0,
            left_trigger: 0,
            a: false,
            b: false,
            x: false,
            y: false,
            pressed_keys: set,
        };

        let command = elaborate_input(&mut inputs, TYPE::Keyboard);
        assert_eq!(command.is_some(), true);
        
        let command = command.unwrap();
        assert_eq!(command.left_power, POWER);
        assert_eq!(command.left_angle, 1.0);
        assert_eq!(command.right_power, POWER);
        assert_eq!(command.right_angle, 1.0);
    }

    // #[test]
    // fn test_elaborate_input_keyboard_escape() {
    //     let mut set = HashSet::new();
    //     set.insert(Keycode::Escape);
    //     let mut inputs = Input {
    //         left_stick_x: 0,
    //         left_stick_y: 0,
    //         right_trigger: 0,
    //         left_trigger: 0,
    //         a: false,
    //         b: false,
    //         x: false,
    //         y: false,
    //         pressed_keys: set,
    //     };
    // 
    //     // Exiting the program is not possible to test as it terminates the process, so we omit actual behavior here
    //     let command = elaborate_input(&mut inputs, TYPE::Keyboard);
    //     assert_eq!(command.is_some(), true);
    // }

    #[test]
    fn test_elaborate_input_joystick_forward() {
        let mut inputs = Input {
            left_stick_x: i16::MAX,
            left_stick_y: 0,
            right_trigger: 0,
            left_trigger: 0,
            a: false,
            b: false,
            x: false,
            y: false,
            pressed_keys: HashSet::new(),
        };

        let result = elaborate_input(&mut inputs, TYPE::Joystick);
        assert_eq!(result.is_some(), true);
        
        let mc = result.unwrap();
        assert_eq!(mc.left_power, 255); // (MAX - MAX normalized to 0)
        assert_eq!(mc.right_power, 127); // Maximum combined power
    }

    #[test]
    fn test_elaborate_input_joystick_forward_right() {
        let mut inputs = Input {
            left_stick_x: i16::MAX,
            left_stick_y: i16::MAX,
            right_trigger: 0,
            left_trigger: 0,
            a: false,
            b: false,
            x: false,
            y: false,
            pressed_keys: HashSet::new(),
        };

        let result = elaborate_input(&mut inputs, TYPE::Joystick);
        assert_eq!(result.is_some(), true);
        
        let mc = result.unwrap();
        assert_eq!(mc.left_power, 255); // (MAX - MAX normalized to 0)
        assert_eq!(mc.right_power, 127); // Maximum combined power
    }

    #[test]
    fn test_elaborate_input_joystick_right() {
        let mut inputs = Input {
            left_stick_x: i16::MAX,
            left_stick_y: i16::MAX,
            right_trigger: 0,
            left_trigger: 0,
            a: false,
            b: false,
            x: false,
            y: false,
            pressed_keys: HashSet::new(),
        };

        let result = elaborate_input(&mut inputs, TYPE::Joystick);
        assert_eq!(result.is_some(), true);
        
        let mc = result.unwrap();
        assert_eq!(mc.left_power, 255); // (MAX - MAX normalized to 0)
        assert_eq!(mc.right_power, 127); // Maximum combined power
    }

    #[test]
    fn test_elaborate_input_trigger() {
        let motor_command = create_motor_command();
        let mut inputs = Input {
            left_stick_x: 0,
            left_stick_y: 0,
            right_trigger: 127,
            left_trigger: 255,
            a: false,
            b: false,
            x: false,
            y: false,
            pressed_keys: HashSet::new(),
        };

        let result = elaborate_input(&mut inputs, TYPE::Trigger);
        assert_eq!(result.is_some(), true);
        
        let mc = result.unwrap();
        assert_eq!(mc.left_power, 255);
        assert_eq!(mc.right_power, 127);
        assert_eq!(mc.left_angle, 1.0);
        assert_eq!(mc.right_angle, 1.0);
    }

    /*
    #[test]
    fn test_read_input_keyboard_event() {
        let mut event_pump = MockEventPump::new(vec![
            Event::KeyDown {
                keycode: Some(Keycode::W),
                ..Default::default()
            },
        ]);
        let mut inputs = Input {
            left_stick_x: 0,
            left_stick_y: 0,
            right_trigger: 0,
            left_trigger: 0,
            a: false,
            b: false,
            x: false,
            y: false,
            keycode: None,
        };

        let input_type = read_input(&mut event_pump, &mut inputs);
        assert_eq!(input_type, TYPE::KEYBOARD);
        assert_eq!(inputs.keycode, Some(Keycode::W));
    }

    #[test]
    fn test_read_input_joystick_event() {
        let mut event_pump = MockEventPump::new(vec![
            Event::JoyAxisMotion {
                axis_idx: 0,
                value: i16::MAX,
                ..Default::default()
            },
        ]);
        let mut inputs = Input {
            left_stick_x: 0,
            left_stick_y: 0,
            right_trigger: 0,
            left_trigger: 0,
            a: false,
            b: false,
            x: false,
            y: false,
            keycode: None,
        };

        let input_type = read_input(&mut event_pump, &mut inputs);
        assert_eq!(input_type, TYPE::JOYSTICK);
        assert_eq!(inputs.left_stick_x, -i16::MAX);
    }

    #[test]
    fn test_read_input_no_event() {
        let mut event_pump = MockEventPump::new(vec![]);
        let mut inputs = Input {
            left_stick_x: 0,
            left_stick_y: 0,
            right_trigger: 0,
            left_trigger: 0,
            a: false,
            b: false,
            x: false,
            y: false,
            keycode: None,
        };

        let input_type = read_input(&mut event_pump, &mut inputs);
        assert_eq!(input_type, TYPE::NO_EVENT);
    }
*/
    // Mock EventPump for testing `read_input` without SDL dependencies
    // struct MockEventPump {
    //     events: Vec<Event>,
    // }
    // 
    // impl MockEventPump {
    //     fn new(events: Vec<Event>) -> Self {
    //         MockEventPump { events }
    //     }
    // }
    // 
    // impl Iterator for MockEventPump {
    //     type Item = Event;
    // 
    //     fn next(&mut self) -> Option<Self::Item> {
    //         self.events.pop()
    //     }
    // }
    // 
    // /*impl EventPump for MockEventPump {
    //     fn poll_iter(&mut self) -> MockEventPump {
    //         self.clone()
    //     }
    // }*/
    // 
    // // Dummy implementation for default cloning, compatible with MockEventPump
    // impl Clone for MockEventPump {
    //     fn clone(&self) -> Self {
    //         MockEventPump {
    //             events: self.events.clone(),
    //         }
    //     }
    // }
}