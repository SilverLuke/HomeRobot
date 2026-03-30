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

        let key_controller = EventControllerKey::new();
        key_controller.set_propagation_phase(gtk4::PropagationPhase::Capture);
        let rc_key = rc_clone.clone();
        key_controller.connect_key_pressed(move |_, key, _, _| {
            let mut cmd = rc_key.lock().unwrap();
            match key {
                Key::w | Key::W => *cmd = RobotCommand::MotorAngle { left_power: 127, left_angle: 1.0, right_power: 127, right_angle: 1.0 },
                Key::s | Key::S => *cmd = RobotCommand::MotorAngle { left_power: 127, left_angle: -1.0, right_power: 127, right_angle: -1.0 },
                Key::a | Key::A => *cmd = RobotCommand::MotorAngle { left_power: 0, left_angle: 0.0, right_power: 127, right_angle: 1.0 },
                Key::d | Key::D => *cmd = RobotCommand::MotorAngle { left_power: 127, left_angle: 1.0, right_power: 0, right_angle: 0.0 },
                Key::l | Key::L => *cmd = RobotCommand::LidarControl { active: true, target_frequency_hz: 5.0 },
                Key::k | Key::K => *cmd = RobotCommand::LidarControl { active: false, target_frequency_hz: 0.0 },
                Key::t | Key::T => *cmd = RobotCommand::RunDiagnostic,
                Key::space => *cmd = RobotCommand::StopAll,
                _ => {}
            }
            glib::Propagation::Proceed
        });

        let rc_release = rc_clone.clone();
        key_controller.connect_key_released(move |_, key, _, _| {
             let mut cmd = rc_release.lock().unwrap();
             match key {
                Key::w | Key::W | Key::s | Key::S | Key::a | Key::A | Key::d | Key::D => *cmd = RobotCommand::StopMoving,
                _ => {}
             }
        });
        window.add_controller(key_controller);

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
                            battery_label_c.set_text(&format!("Battery: {}% ({} mV)", percentage, voltage_mv));
                        }
                        GuiUpdate::Encoders { left, right } => {
                            enc_left_label_c.set_text(&left.to_string());
                            enc_right_label_c.set_text(&right.to_string());
                        }
                        GuiUpdate::Imu { ax, ay, az, gx, gy, gz } => {
                            accel_label_c.set_text(&format!("Accel: X: {:.2} Y: {:.2} Z: {:.2}", ax, ay, az));
                            gyro_label_c.set_text(&format!("Gyro: X: {:.2} Y: {:.2} Z: {:.2}", gx, gy, gz));
                        }
                        GuiUpdate::Lidar(points) => {
                            let mut state = GUI_STATE.lock().unwrap();
                            state.lidar_points = points;
                            lidar_canvas_c.queue_draw();
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
        lidar_canvas.grab_focus();
    });

    (app, tx)
}
