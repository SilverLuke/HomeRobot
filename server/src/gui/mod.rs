pub mod lidar;

use gtk4::prelude::*;
use gtk4::{Application, ApplicationWindow, Builder, DrawingArea, Label, Button, EventControllerKey, SpinButton};
use gdk4::Key;
use std::sync::{Arc, Mutex, mpsc};
use crate::command::RobotCommand;
use crate::homerobot::{LidarPoint, RobotConfig};
use std::time::Duration;
use crate::gui::lidar::{setup_lidar_drawing, GUI_STATE};

pub enum GuiUpdate {
    Battery { percentage: u32, voltage_mv: u32 },
    Encoders { left: i32, right: i32 },
    Lidar(Vec<LidarPoint>),
    Imu { ax: f32, ay: f32, az: f32, gx: f32, gy: f32, gz: f32 },
    Config(RobotConfig),
    Status(String),
}

pub fn init_gui(robot_command: Arc<Mutex<RobotCommand>>) -> (Application, mpsc::Sender<GuiUpdate>) {
    let (tx, rx) = mpsc::channel::<GuiUpdate>();
    let rx = Arc::new(Mutex::new(rx));

    let app = Application::builder()
        .application_id("com.homerobot.control")
        .build();

    let rc_clone = robot_command.clone();
    
    app.connect_activate(move |app| {
        println!("GUI: Initializing GTK components...");
        let builder = Builder::new();
        builder.add_from_string(include_str!("../main_window.ui")).expect("Failed to parse UI XML");
        
        let window: ApplicationWindow = builder.object("main_window").expect("Could not find object 'main_window' in UI definition");
        window.set_application(Some(app));

        let lidar_canvas: DrawingArea = builder.object("lidar_canvas").expect("Could not find lidar_canvas");
        let battery_label: Label = builder.object("battery_label").expect("Could not find battery_label");
        let enc_left_label: Label = builder.object("enc_left_val").expect("Could not find enc_left_val");
        let enc_right_label: Label = builder.object("enc_right_val").expect("Could not find enc_right_val");
        let accel_label: Label = builder.object("accel_label").expect("Could not find accel_label");
        let gyro_label: Label = builder.object("gyro_label").expect("Could not find gyro_label");
        let apply_btn: Button = builder.object("apply_config").expect("Could not find apply_config");

        // Movement Buttons
        let btn_forward: Button = builder.object("btn_forward").expect("Could not find btn_forward");
        let btn_backward: Button = builder.object("btn_backward").expect("Could not find btn_backward");
        let btn_left: Button = builder.object("btn_left").expect("Could not find btn_left");
        let btn_right: Button = builder.object("btn_right").expect("Could not find btn_right");
        let btn_stop: Button = builder.object("btn_stop").expect("Could not find btn_stop");
        let btn_lidar: gtk4::ToggleButton = builder.object("btn_lidar").expect("Could not find btn_lidar");

        // PID SpinButtons
        let kp_left: SpinButton = builder.object("kp_left").expect("Could not find kp_left");
        let ki_left: SpinButton = builder.object("ki_left").expect("Could not find ki_left");
        let kd_left: SpinButton = builder.object("kd_left").expect("Could not find kd_left");
        let kp_right: SpinButton = builder.object("kp_right").expect("Could not find kp_right");
        let ki_right: SpinButton = builder.object("ki_right").expect("Could not find ki_right");
        let kd_right: SpinButton = builder.object("kd_right").expect("Could not find kd_right");

        // Explicitly disable focus for input widgets to prevent them from stealing WASD keys
        kp_left.set_focusable(false);
        ki_left.set_focusable(false);
        kd_left.set_focusable(false);
        kp_right.set_focusable(false);
        ki_right.set_focusable(false);
        kd_right.set_focusable(false);
        apply_btn.set_focusable(false);

        #[allow(deprecated)]
        {
            kp_left.set_can_focus(false);
            ki_left.set_can_focus(false);
            kd_left.set_can_focus(false);
            kp_right.set_can_focus(false);
            ki_right.set_can_focus(false);
            kd_right.set_can_focus(false);
            apply_btn.set_can_focus(false);
        }

        // Set up LIDAR drawing
        setup_lidar_drawing(&lidar_canvas);

        // Custom CSS for Green Lidar Button
        let provider = gtk4::CssProvider::new();
        provider.load_from_data("
            .lidar-on { background-color: #2ecc71; color: white; }
            .lidar-on:checked { background-color: #27ae60; color: white; }
        ");
        gtk4::style_context_add_provider_for_display(
            &gdk4::Display::default().expect("Could not connect to a display."),
            &provider,
            gtk4::STYLE_PROVIDER_PRIORITY_APPLICATION,
        );
        btn_lidar.add_css_class("lidar-on");

        let key_controller = EventControllerKey::new();
        key_controller.set_propagation_phase(gtk4::PropagationPhase::Bubble);
        let rc_key = rc_clone.clone();
        let btn_lidar_clone = btn_lidar.clone();
        key_controller.connect_key_pressed(move |_, key, _, _| {
            let mut cmd = rc_key.lock().unwrap();
            match key {
                Key::w | Key::W => {
                    println!("GUI: Keyboard Move: Forward (Latched)");
                    *cmd = RobotCommand::MotorAngle { left_power: 127, left_angle: 1.0, right_power: 127, right_angle: 1.0 };
                },
                Key::s | Key::S => {
                    println!("GUI: Keyboard Move: Backward (Latched)");
                    *cmd = RobotCommand::MotorAngle { left_power: 127, left_angle: -1.0, right_power: 127, right_angle: -1.0 };
                },
                Key::a | Key::A => {
                    println!("GUI: Keyboard Move: Left (Latched)");
                    *cmd = RobotCommand::MotorAngle { left_power: 0, left_angle: 0.0, right_power: 127, right_angle: 1.0 };
                },
                Key::d | Key::D => {
                    println!("GUI: Keyboard Move: Right (Latched)");
                    *cmd = RobotCommand::MotorAngle { left_power: 127, left_angle: 1.0, right_power: 0, right_angle: 0.0 };
                },
                Key::l | Key::L => {
                    let new_state = !btn_lidar_clone.is_active();
                    println!("GUI: Keyboard Lidar Toggle: {}", if new_state { "ON" } else { "OFF" });
                    *cmd = RobotCommand::LidarControl { 
                        active: new_state, 
                        target_frequency_hz: if new_state { 5.0 } else { 0.0 } 
                    };
                    btn_lidar_clone.set_active(new_state);
                },
                Key::space => {
                    println!("GUI: Keyboard STOP");
                    *cmd = RobotCommand::StopMoving;
                },
                _ => {}
            }
            glib::Propagation::Proceed
        });

        // Removed key_controller.connect_key_released to keep robot moving until Stop is requested
        
        window.add_controller(key_controller);

        // Connect Movement Buttons
        let rc_fwd = rc_clone.clone();
        btn_forward.connect_clicked(move |_| {
            println!("GUI: Button Move: Forward");
            *rc_fwd.lock().unwrap() = RobotCommand::MotorAngle { left_power: 127, left_angle: 1.0, right_power: 127, right_angle: 1.0 };
        });

        let rc_bwd = rc_clone.clone();
        btn_backward.connect_clicked(move |_| {
            println!("GUI: Button Move: Backward");
            *rc_bwd.lock().unwrap() = RobotCommand::MotorAngle { left_power: 127, left_angle: -1.0, right_power: 127, right_angle: -1.0 };
        });

        let rc_left = rc_clone.clone();
        btn_left.connect_clicked(move |_| {
            println!("GUI: Button Move: Left");
            *rc_left.lock().unwrap() = RobotCommand::MotorAngle { left_power: 0, left_angle: 0.0, right_power: 127, right_angle: 1.0 };
        });

        let rc_right = rc_clone.clone();
        btn_right.connect_clicked(move |_| {
            println!("GUI: Button Move: Right");
            *rc_right.lock().unwrap() = RobotCommand::MotorAngle { left_power: 127, left_angle: 1.0, right_power: 0, right_angle: 0.0 };
        });

        let rc_stop = rc_clone.clone();
        btn_stop.connect_clicked(move |_| {
            println!("GUI: Button STOP");
            *rc_stop.lock().unwrap() = RobotCommand::StopMoving;
        });

        let rc_lidar = rc_clone.clone();
        btn_lidar.connect_toggled(move |btn| {
            let mut cmd = rc_lidar.lock().unwrap();
            let active = btn.is_active();
            *cmd = RobotCommand::LidarControl { 
                active, 
                target_frequency_hz: if active { 5.0 } else { 0.0 } 
            };
            println!("GUI: Lidar toggled: {}", active);
        });

        // Apply config button
        let rc_conf = rc_clone.clone();
        let kpl = kp_left.clone();
        let kil = ki_left.clone();
        let kdl = kd_left.clone();
        let kpr = kp_right.clone();
        let kir = ki_right.clone();
        let kdr = kd_right.clone();
        apply_btn.connect_clicked(move |_| {
            let mut cmd = rc_conf.lock().unwrap();
            *cmd = RobotCommand::UpdateConfig {
                left_kp: kpl.value() as f32,
                left_ki: kil.value() as f32,
                left_kd: kdl.value() as f32,
                right_kp: kpr.value() as f32,
                right_ki: kir.value() as f32,
                right_kd: kdr.value() as f32,
            };
            println!("GUI: Sent RobotConfig update");
        });

        // Clone widgets for the polling loop
        let battery_label_c = battery_label.clone();
        let enc_left_label_c = enc_left_label.clone();
        let enc_right_label_c = enc_right_label.clone();
        let accel_label_c = accel_label.clone();
        let gyro_label_c = gyro_label.clone();
        let lidar_canvas_c = lidar_canvas.clone();
        let window_c = window.clone();
        let rx_inner = rx.clone();

        glib::timeout_add_local(Duration::from_millis(33), move || {
            if let Ok(rx_locked) = rx_inner.try_lock() {
                while let Ok(update) = rx_locked.try_recv() {
                    match update {
                        GuiUpdate::Battery { percentage, voltage_mv } => {
                            println!("[GUI] Received Battery: {}%", percentage);
                            battery_label_c.set_text(&format!("Battery: {}% ({} mV)", percentage, voltage_mv));
                        }
                        GuiUpdate::Encoders { left, right } => {
                            println!("[GUI] Received Encoders: L={} R={}", left, right);
                            enc_left_label_c.set_text(&left.to_string());
                            enc_right_label_c.set_text(&right.to_string());
                        }
                        GuiUpdate::Imu { ax, ay, az, gx, gy, gz } => {
                            accel_label_c.set_text(&format!("Accel: X: {:.2} Y: {:.2} Z: {:.2}", ax, ay, az));
                            gyro_label_c.set_text(&format!("Gyro: X: {:.2} Y: {:.2} Z: {:.2}", gx, gy, gz));
                        }
                        GuiUpdate::Lidar(points) => {
                            println!("[GUI] Received Lidar: {} points", points.len());
                            if !points.is_empty() {
                                let mut state = GUI_STATE.lock().unwrap();
                                crate::gui::lidar::update_scan(&mut state.display_scan, points);
                                lidar_canvas_c.queue_draw();
                            }
                        }
                        GuiUpdate::Config(conf) => {
                            if let Some(left) = conf.left_motor {
                                kp_left.set_value(left.kp as f64);
                                ki_left.set_value(left.ki as f64);
                                kd_left.set_value(left.kd as f64);
                            }
                            if let Some(right) = conf.right_motor {
                                kp_right.set_value(right.kp as f64);
                                ki_right.set_value(right.ki as f64);
                                kd_right.set_value(right.kd as f64);
                            }
                        }
                        GuiUpdate::Status(msg) => {
                            window_c.set_title(Some(&format!("HomeRobot Control Center - {}", msg)));
                        }
                    }
                }
            }
            glib::ControlFlow::Continue
        });

        window.present();
    });

    (app, tx)
}
