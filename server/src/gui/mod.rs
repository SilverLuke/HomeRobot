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
    Pose { x: f32, y: f32, theta: f32 },
    SlamPose { x: f32, y: f32, theta: f32 },
    Map { width: usize, height: usize, data: Vec<i16> },
    Frontiers(Vec<crate::mapping::Frontier>),
    Path(Vec<(f32, f32)>),
    Lidar(Vec<LidarPoint>),
    LidarScanRate(f32),
    Imu { 
        ax: f32, ay: f32, az: f32, 
        gx: f32, gy: f32, gz: f32,
        mx: f32, my: f32, mz: f32
    },
    Config(RobotConfig),
    Status(String),
    NavigationTarget(Option<(f32, f32)>),
    Log(String),
}

pub fn init_gui(robot_command: Arc<Mutex<RobotCommand>>, rec: Arc<Mutex<rerun::RecordingStream>>) -> (Application, mpsc::Sender<GuiUpdate>) {
    let (tx, rx) = mpsc::channel::<GuiUpdate>();
    let rx = Arc::new(Mutex::new(rx));

    let app = Application::builder()
        .application_id("com.homerobot.control")
        .build();

    let rc_clone = robot_command.clone();
    let rec_clone = rec.clone();
    
    app.connect_activate(move |app| {
        let rec = rec_clone.clone();
        println!("GUI: Initializing GTK components...");
        let builder = Builder::new();
        builder.add_from_string(include_str!("../main_window.ui")).expect("Failed to parse UI XML");
        
        let window: ApplicationWindow = builder.object("main_window").expect("Could not find object 'main_window' in UI definition");
        window.set_application(Some(app));

        let sidebar_scroll: gtk4::ScrolledWindow = builder.object("sidebar_scroll").expect("Could not find sidebar_scroll");
        let btn_toggle_sidebar: gtk4::ToggleButton = builder.object("btn_toggle_sidebar").expect("Could not find btn_toggle_sidebar");
        let power_scale: gtk4::Scale = builder.object("global_power_scale").expect("Could not find global_power_scale");
        let lidar_freq_scale: gtk4::Scale = builder.object("lidar_freq_scale").expect("Could not find lidar_freq_scale");
        let lidar_canvas: DrawingArea = builder.object("lidar_canvas").expect("Could not find lidar_canvas");
        let battery_label: Label = builder.object("battery_label").expect("Could not find battery_label");
        let enc_left_label: Label = builder.object("enc_left_val").expect("Could not find enc_left_val");
        let enc_right_label: Label = builder.object("enc_right_val").expect("Could not find enc_right_val");
        let pose_x_label: Label = builder.object("pose_x_val").expect("Could not find pose_x_val");
        let pose_y_label: Label = builder.object("pose_y_val").expect("Could not find pose_y_val");
        let pose_theta_label: Label = builder.object("pose_theta_val").expect("Could not find pose_theta_val");
        let accel_label: Label = builder.object("accel_label").expect("Could not find accel_label");
        let gyro_label: Label = builder.object("gyro_label").expect("Could not find gyro_label");
        let mag_label: Label = builder.object("mag_label").expect("Could not find mag_label");
        
        let accel_canvas: DrawingArea = builder.object("accel_plot").expect("Could not find accel_plot");
        let gyro_canvas: DrawingArea = builder.object("gyro_plot").expect("Could not find gyro_plot");
        let mag_canvas: DrawingArea = builder.object("mag_plot").expect("Could not find mag_plot");
        let btn_clear_path: Button = builder.object("btn_clear_path").expect("Could not find btn_clear_path");
        let scan_rate_label: Label = builder.object("scan_rate_label").expect("Could not find scan_rate_label");

        let apply_btn: Button = builder.object("apply_config").expect("Could not find apply_config");

        // Movement Buttons
        let btn_forward: Button = builder.object("btn_forward").expect("Could not find btn_forward");
        let btn_backward: Button = builder.object("btn_backward").expect("Could not find btn_backward");
        let btn_left: Button = builder.object("btn_left").expect("Could not find btn_left");
        let btn_right: Button = builder.object("btn_right").expect("Could not find btn_right");
        let btn_stop: Button = builder.object("btn_stop").expect("Could not find btn_stop");
        let btn_save_map: Button = builder.object("btn_save_map").expect("Could not find btn_save_map");
        let btn_lidar: gtk4::ToggleButton = builder.object("btn_lidar").expect("Could not find btn_lidar");
        let btn_explore: gtk4::ToggleButton = builder.object("btn_explore").expect("Could not find btn_explore");
        let btn_reset: Button = builder.object("btn_reset").expect("Could not find btn_reset");

        // PID SpinButtons
        let kp_left: SpinButton = builder.object("kp_left").expect("Could not find kp_left");
        let ki_left: SpinButton = builder.object("ki_left").expect("Could not find ki_left");
        let kd_left: SpinButton = builder.object("kd_left").expect("Could not find kd_left");
        let kp_right: SpinButton = builder.object("kp_right").expect("Could not find kp_right");
        let ki_right: SpinButton = builder.object("ki_right").expect("Could not find ki_right");
        let kd_right: SpinButton = builder.object("kd_right").expect("Could not find kd_right");

        // New UI controls
        let btn_start_rerun: Button = builder.object("btn_start_rerun").expect("Could not find btn_start_rerun");
        let btn_lock_pid: gtk4::ToggleButton = builder.object("btn_lock_pid").expect("Could not find btn_lock_pid");
        let btn_rotate_left: Button = builder.object("btn_rotate_left").expect("Could not find btn_rotate_left");
        let btn_rotate_right: Button = builder.object("btn_rotate_right").expect("Could not find btn_rotate_right");
        let btn_zoom_in: Button = builder.object("btn_zoom_in").expect("Could not find btn_zoom_in");
        let btn_zoom_out: Button = builder.object("btn_zoom_out").expect("Could not find btn_zoom_out");
        let log_text_view: gtk4::TextView = builder.object("log_text_view").expect("Could not find log_text_view");

        let loading_overlay: gtk4::Box = builder.object("loading_overlay").expect("Could not find loading_overlay");
        let loading_spinner: gtk4::Spinner = builder.object("loading_spinner").expect("Could not find loading_spinner");
        loading_spinner.start();

        // Explicitly disable focus for input widgets to prevent them from stealing WASD keys
        kp_left.set_focusable(false);
        ki_left.set_focusable(false);
        kd_left.set_focusable(false);
        kp_right.set_focusable(false);
        ki_right.set_focusable(false);
        kd_right.set_focusable(false);
        apply_btn.set_focusable(false);
        lidar_freq_scale.set_focusable(false);
        btn_start_rerun.set_focusable(false);
        btn_lock_pid.set_focusable(false);
        btn_rotate_left.set_focusable(false);
        btn_rotate_right.set_focusable(false);
        btn_reset.set_focusable(false);
        btn_zoom_in.set_focusable(false);
        btn_zoom_out.set_focusable(false);
        log_text_view.set_focusable(false);

        #[allow(deprecated)]
        {
            kp_left.set_can_focus(false);
            ki_left.set_can_focus(false);
            kd_left.set_can_focus(false);
            kp_right.set_can_focus(false);
            ki_right.set_can_focus(false);
            kd_right.set_can_focus(false);
            apply_btn.set_can_focus(false);
            btn_start_rerun.set_can_focus(false);
            btn_lock_pid.set_can_focus(false);
            btn_rotate_left.set_can_focus(false);
            btn_rotate_right.set_can_focus(false);
            btn_reset.set_can_focus(false);
            log_text_view.set_can_focus(false);
        }

        // Set up LIDAR drawing
        setup_lidar_drawing(&lidar_canvas);
        crate::gui::lidar::setup_accel_plot(&accel_canvas);
        crate::gui::lidar::setup_gyro_plot(&gyro_canvas);
        crate::gui::lidar::setup_mag_plot(&mag_canvas);

        // Map Click Navigation gesture
        let click_controller = gtk4::GestureClick::new();
        let rc_click = robot_command.clone();
        let lidar_canvas_click = lidar_canvas.clone();
        let btn_explore_click = btn_explore.clone();
        click_controller.connect_pressed(move |_gesture, _n_press, x, y| {
            let width = lidar_canvas_click.width() as f64;
            let height = lidar_canvas_click.height() as f64;
            
            let world_center_x = width / 2.0;
            let world_center_y = height / 2.0;
            
            // Map click (x, y) to world (goal_x, goal_y)
            let goal_y = (world_center_x - x) as f32 / 50.0;
            let goal_x = (world_center_y - y) as f32 / 50.0;
            
            println!("GUI: Map clicked at pixel ({:.1}, {:.1}) -> World Goal: X={:.2}, Y={:.2}", x, y, goal_x, goal_y);
            
            // Set navigation target in GUI State immediately for visual response
            {
                let mut state = GUI_STATE.lock().unwrap();
                state.navigation_target = Some((goal_x, goal_y));
                state.current_path.clear(); // Clear old path until replanned by server
            }
            lidar_canvas_click.queue_draw();
            
            // Deactivate auto-exploration if active
            if btn_explore_click.is_active() {
                btn_explore_click.set_active(false);
            }
            
            // Send NavigateTo command
            let mut cmd = rc_click.lock().unwrap();
            *cmd = RobotCommand::NavigateTo { x: goal_x, y: goal_y };
        });
        lidar_canvas.add_controller(click_controller);

        let lidar_canvas_clear = lidar_canvas.clone();
        btn_clear_path.connect_clicked(move |_| {
            let mut state = GUI_STATE.lock().unwrap();
            state.trajectory.clear();
            lidar_canvas_clear.queue_draw();
            println!("GUI: Trajectory cleared.");
        });

        // Custom CSS for Buttons and Loading Overlay
        let provider = gtk4::CssProvider::new();
        provider.load_from_string("
            .lidar-on { background-color: #2ecc71; color: white; }
            .lidar-on:checked { background-color: #27ae60; color: white; }
            .explore-on { background-color: #e74c3c; color: white; }
            .explore-on:checked { background-color: #c0392b; color: white; }
            .loading-overlay-bg { background-color: rgba(25, 25, 25, 0.96); color: white; }
        ");
        gtk4::style_context_add_provider_for_display(
            &gdk4::Display::default().expect("Could not connect to a display."),
            &provider,
            gtk4::STYLE_PROVIDER_PRIORITY_APPLICATION,
        );
        btn_lidar.add_css_class("lidar-on");
        btn_explore.add_css_class("explore-on");

        let rc_explore = rc_clone.clone();
        btn_explore.connect_toggled(move |btn| {
            let mut cmd = rc_explore.lock().unwrap();
            let enabled = btn.is_active();
            println!("GUI: Exploration {}", if enabled { "ENABLED" } else { "DISABLED" });
            *cmd = RobotCommand::AutonomousExploration { enabled };
        });

        let key_controller = EventControllerKey::new();
        key_controller.set_propagation_phase(gtk4::PropagationPhase::Bubble);
        let rc_key = rc_clone.clone();
        let btn_lidar_clone = btn_lidar.clone();
        let btn_toggle_sidebar_clone = btn_toggle_sidebar.clone();
        let power_scale_clone = power_scale.clone();
        let lidar_freq_scale_key = lidar_freq_scale.clone();
        let lidar_canvas_key = lidar_canvas.clone();

        key_controller.connect_key_pressed(move |_, key, _, _| {
            let mut cmd = rc_key.lock().unwrap();
            let current_power = power_scale_clone.value() as u8;

            match key {
                Key::w | Key::W => {
                    println!("GUI: Keyboard Move: Forward (Power: {})", current_power);
                    *cmd = RobotCommand::MotorAngle { 
                        left_power: current_power, left_angle: 1.0, 
                        right_power: current_power, right_angle: 1.0 
                    };
                },
                Key::s | Key::S => {
                    println!("GUI: Keyboard Move: Backward (Power: {})", current_power);
                    *cmd = RobotCommand::MotorAngle { 
                        left_power: current_power, left_angle: -1.0, 
                        right_power: current_power, right_angle: -1.0 
                    };
                },
                Key::a | Key::A => {
                    println!("GUI: Keyboard Move: Left (Power: {})", current_power);
                    *cmd = RobotCommand::MotorAngle { 
                        left_power: 0, left_angle: 0.0, 
                        right_power: current_power, right_angle: 1.0 
                    };
                },
                Key::d | Key::D => {
                    println!("GUI: Keyboard Move: Right (Power: {})", current_power);
                    *cmd = RobotCommand::MotorAngle { 
                        left_power: current_power, left_angle: 1.0, 
                        right_power: 0, right_angle: 0.0 
                    };
                },
                Key::q | Key::Q => {
                    println!("GUI: Keyboard Move: Rotate Left (Power: {})", current_power);
                    *cmd = RobotCommand::MotorAngle { 
                        left_power: current_power, left_angle: -1.0, 
                        right_power: current_power, right_angle: 1.0 
                    };
                },
                Key::e | Key::E => {
                    println!("GUI: Keyboard Move: Rotate Right (Power: {})", current_power);
                    *cmd = RobotCommand::MotorAngle { 
                        left_power: current_power, left_angle: 1.0, 
                        right_power: current_power, right_angle: -1.0 
                    };
                },
                Key::h | Key::H => {
                    let new_active = !btn_toggle_sidebar_clone.is_active();
                    btn_toggle_sidebar_clone.set_active(new_active);
                    println!("GUI: Sidebar visibility toggled via H: {}", new_active);
                },
                Key::l | Key::L => {
                    let new_state = !btn_lidar_clone.is_active();
                    println!("GUI: Keyboard Lidar Toggle: {}", if new_state { "ON" } else { "OFF" });
                    *cmd = RobotCommand::LidarControl { 
                        active: new_state, 
                        target_frequency_hz: if new_state { lidar_freq_scale_key.value() as f32 } else { 0.0 } 
                    };
                    btn_lidar_clone.set_active(new_state);
                },
                Key::space => {
                    println!("GUI: Keyboard STOP");
                    {
                        let mut state = GUI_STATE.lock().unwrap();
                        state.navigation_target = None;
                        state.current_path.clear();
                    }
                    lidar_canvas_key.queue_draw();
                    *cmd = RobotCommand::StopMoving;
                },
                Key::t | Key::T => {
                    println!("GUI: Keyboard Trigger Diagnostics");
                    *cmd = RobotCommand::RunDiagnostic;
                },
                Key::plus | Key::equal | Key::KP_Add => {
                    let mut state = GUI_STATE.lock().unwrap();
                    state.zoom_factor *= 1.2;
                    println!("GUI: Zoom In via keyboard: {:.2}x", state.zoom_factor);
                    lidar_canvas_key.queue_draw();
                },
                Key::minus | Key::KP_Subtract => {
                    let mut state = GUI_STATE.lock().unwrap();
                    state.zoom_factor = (state.zoom_factor / 1.2).max(0.1);
                    println!("GUI: Zoom Out via keyboard: {:.2}x", state.zoom_factor);
                    lidar_canvas_key.queue_draw();
                },
                _ => {}
            }
            glib::Propagation::Proceed
        });

        // Removed key_controller.connect_key_released to keep robot moving until Stop is requested
        
        let sidebar_scroll_btn = sidebar_scroll.clone();
        btn_toggle_sidebar.connect_toggled(move |btn| {
            let active = btn.is_active();
            sidebar_scroll_btn.set_visible(active);
            println!("GUI: Sidebar visibility changed: {}", active);
        });

        window.add_controller(key_controller);

        // Connect Movement Buttons
        let rc_fwd = rc_clone.clone();
        let ps_fwd = power_scale.clone();
        btn_forward.connect_clicked(move |_| {
            let p = ps_fwd.value() as u8;
            println!("GUI: Button Move: Forward (Power: {})", p);
            *rc_fwd.lock().unwrap() = RobotCommand::MotorAngle { left_power: p, left_angle: 1.0, right_power: p, right_angle: 1.0 };
        });

        let rc_bwd = rc_clone.clone();
        let ps_bwd = power_scale.clone();
        btn_backward.connect_clicked(move |_| {
            let p = ps_bwd.value() as u8;
            println!("GUI: Button Move: Backward (Power: {})", p);
            *rc_bwd.lock().unwrap() = RobotCommand::MotorAngle { left_power: p, left_angle: -1.0, right_power: p, right_angle: -1.0 };
        });

        let rc_left = rc_clone.clone();
        let ps_left = power_scale.clone();
        btn_left.connect_clicked(move |_| {
            let p = ps_left.value() as u8;
            println!("GUI: Button Move: Left (Power: {})", p);
            *rc_left.lock().unwrap() = RobotCommand::MotorAngle { left_power: 0, left_angle: 0.0, right_power: p, right_angle: 1.0 };
        });

        let rc_right = rc_clone.clone();
        let ps_right = power_scale.clone();
        btn_right.connect_clicked(move |_| {
            let p = ps_right.value() as u8;
            println!("GUI: Button Move: Right (Power: {})", p);
            *rc_right.lock().unwrap() = RobotCommand::MotorAngle { left_power: p, left_angle: 1.0, right_power: 0, right_angle: 0.0 };
        });

        let rc_rot_left = rc_clone.clone();
        let ps_rot_left = power_scale.clone();
        btn_rotate_left.connect_clicked(move |_| {
            let p = ps_rot_left.value() as u8;
            println!("GUI: Button Move: Rotate Left (Power: {})", p);
            *rc_rot_left.lock().unwrap() = RobotCommand::MotorAngle { left_power: p, left_angle: -1.0, right_power: p, right_angle: 1.0 };
        });

        let rc_rot_right = rc_clone.clone();
        let ps_rot_right = power_scale.clone();
        btn_rotate_right.connect_clicked(move |_| {
            let p = ps_rot_right.value() as u8;
            println!("GUI: Button Move: Rotate Right (Power: {})", p);
            *rc_rot_right.lock().unwrap() = RobotCommand::MotorAngle { left_power: p, left_angle: 1.0, right_power: p, right_angle: -1.0 };
        });

        let rc_stop = rc_clone.clone();
        let lidar_canvas_stop = lidar_canvas.clone();
        btn_stop.connect_clicked(move |_| {
            println!("GUI: Button STOP");
            {
                let mut state = GUI_STATE.lock().unwrap();
                state.navigation_target = None;
                state.current_path.clear();
            }
            lidar_canvas_stop.queue_draw();
            *rc_stop.lock().unwrap() = RobotCommand::StopMoving;
        });

        let rc_save = rc_clone.clone();
        btn_save_map.connect_clicked(move |_| {
            println!("GUI: Button Save Map");
            *rc_save.lock().unwrap() = RobotCommand::SaveMap;
        });

        let rc_reset = rc_clone.clone();
        let lidar_canvas_reset = lidar_canvas.clone();
        btn_reset.connect_clicked(move |_| {
            println!("GUI: Button Reset Triggered");
            {
                let mut state = GUI_STATE.lock().unwrap();
                state.navigation_target = None;
                state.current_path.clear();
            }
            lidar_canvas_reset.queue_draw();
            *rc_reset.lock().unwrap() = RobotCommand::Reset;
        });

        let rc_lidar = rc_clone.clone();
        let lidar_freq_scale_btn = lidar_freq_scale.clone();
        btn_lidar.connect_toggled(move |btn| {
            let mut cmd = rc_lidar.lock().unwrap();
            let active = btn.is_active();
            *cmd = RobotCommand::LidarControl { 
                active, 
                target_frequency_hz: if active { lidar_freq_scale_btn.value() as f32 } else { 0.0 } 
            };
            println!("GUI: Lidar toggled: {}", active);
        });
        btn_lidar.set_active(true);

        let lidar_canvas_zoom_in = lidar_canvas.clone();
        btn_zoom_in.connect_clicked(move |_| {
            let mut state = GUI_STATE.lock().unwrap();
            state.zoom_factor *= 1.2;
            println!("GUI: Zoom In clicked: {:.2}x", state.zoom_factor);
            lidar_canvas_zoom_in.queue_draw();
        });

        let lidar_canvas_zoom_out = lidar_canvas.clone();
        btn_zoom_out.connect_clicked(move |_| {
            let mut state = GUI_STATE.lock().unwrap();
            state.zoom_factor = (state.zoom_factor / 1.2).max(0.1);
            println!("GUI: Zoom Out clicked: {:.2}x", state.zoom_factor);
            lidar_canvas_zoom_out.queue_draw();
        });

        let scroll_controller = gtk4::EventControllerScroll::new(gtk4::EventControllerScrollFlags::VERTICAL);
        let lidar_canvas_scroll = lidar_canvas.clone();
        scroll_controller.connect_scroll(move |_, _, dy| {
            let mut state = GUI_STATE.lock().unwrap();
            if dy < 0.0 {
                state.zoom_factor *= 1.15;
            } else if dy > 0.0 {
                state.zoom_factor = (state.zoom_factor / 1.15).max(0.1);
            }
            lidar_canvas_scroll.queue_draw();
            glib::Propagation::Stop
        });
        lidar_canvas.add_controller(scroll_controller);

        // Start Rerun on-demand button
        let rec_btn = rec.clone();
        let btn_start_rerun_clone = btn_start_rerun.clone();
        btn_start_rerun.connect_clicked(move |_| {
            let mut stream = rec_btn.lock().unwrap();
            if stream.is_enabled() {
                println!("GUI: Rerun is already running!");
                return;
            }
            println!("GUI: Starting Rerun viewer on-demand...");
            match rerun::RecordingStreamBuilder::new("home-robot").spawn() {
                Ok(new_stream) => {
                    *stream = new_stream;
                    println!("GUI: Rerun viewer spawned successfully.");
                    btn_start_rerun_clone.set_sensitive(false);
                }
                Err(e) => {
                    eprintln!("GUI: Failed to spawn Rerun: {:?}", e);
                }
            }
        });

        // PID Lock Logic and Apply Config Button
        let is_updating = std::rc::Rc::new(std::cell::Cell::new(false));

        let rc_lidar_freq = rc_clone.clone();
        let btn_lidar_freq_ref = btn_lidar.clone();
        let is_updating_freq = is_updating.clone();
        let lidar_freq_scale_c = lidar_freq_scale.clone();
        lidar_freq_scale.connect_value_changed(move |_| {
            if !is_updating_freq.get() {
                let freq = lidar_freq_scale_c.value() as f32;
                let active = btn_lidar_freq_ref.is_active();
                if active {
                    let mut cmd = rc_lidar_freq.lock().unwrap();
                    *cmd = RobotCommand::LidarControl {
                        active: true,
                        target_frequency_hz: freq,
                    };
                    println!("GUI: Lidar frequency changed: {} Hz", freq);
                }
            }
        });

        let kp_left_c = kp_left.clone();
        let kp_right_c = kp_right.clone();
        let btn_lock_pid_c = btn_lock_pid.clone();
        let is_updating_c = is_updating.clone();
        kp_left.connect_value_changed(move |_| {
            if btn_lock_pid_c.is_active() && !is_updating_c.get() {
                is_updating_c.set(true);
                kp_right_c.set_value(kp_left_c.value());
                is_updating_c.set(false);
            }
        });

        let kp_left_c = kp_left.clone();
        let kp_right_c = kp_right.clone();
        let btn_lock_pid_c = btn_lock_pid.clone();
        let is_updating_c = is_updating.clone();
        kp_right.connect_value_changed(move |_| {
            if btn_lock_pid_c.is_active() && !is_updating_c.get() {
                is_updating_c.set(true);
                kp_left_c.set_value(kp_right_c.value());
                is_updating_c.set(false);
            }
        });

        let ki_left_c = ki_left.clone();
        let ki_right_c = ki_right.clone();
        let btn_lock_pid_c = btn_lock_pid.clone();
        let is_updating_c = is_updating.clone();
        ki_left.connect_value_changed(move |_| {
            if btn_lock_pid_c.is_active() && !is_updating_c.get() {
                is_updating_c.set(true);
                ki_right_c.set_value(ki_left_c.value());
                is_updating_c.set(false);
            }
        });

        let ki_left_c = ki_left.clone();
        let ki_right_c = ki_right.clone();
        let btn_lock_pid_c = btn_lock_pid.clone();
        let is_updating_c = is_updating.clone();
        ki_right.connect_value_changed(move |_| {
            if btn_lock_pid_c.is_active() && !is_updating_c.get() {
                is_updating_c.set(true);
                ki_left_c.set_value(ki_right_c.value());
                is_updating_c.set(false);
            }
        });

        let kd_left_c = kd_left.clone();
        let kd_right_c = kd_right.clone();
        let btn_lock_pid_c = btn_lock_pid.clone();
        let is_updating_c = is_updating.clone();
        kd_left.connect_value_changed(move |_| {
            if btn_lock_pid_c.is_active() && !is_updating_c.get() {
                is_updating_c.set(true);
                kd_right_c.set_value(kd_left_c.value());
                is_updating_c.set(false);
            }
        });

        let kd_left_c = kd_left.clone();
        let kd_right_c = kd_right.clone();
        let btn_lock_pid_c = btn_lock_pid.clone();
        let is_updating_c = is_updating.clone();
        kd_right.connect_value_changed(move |_| {
            if btn_lock_pid_c.is_active() && !is_updating_c.get() {
                is_updating_c.set(true);
                kd_left_c.set_value(kd_right_c.value());
                is_updating_c.set(false);
            }
        });

        let kp_left_c = kp_left.clone();
        let kp_right_c = kp_right.clone();
        let ki_left_c = ki_left.clone();
        let ki_right_c = ki_right.clone();
        let kd_left_c = kd_left.clone();
        let kd_right_c = kd_right.clone();
        let is_updating_c = is_updating.clone();
        btn_lock_pid.connect_toggled(move |btn| {
            if btn.is_active() {
                is_updating_c.set(true);
                kp_right_c.set_value(kp_left_c.value());
                ki_right_c.set_value(ki_left_c.value());
                kd_right_c.set_value(kd_left_c.value());
                is_updating_c.set(false);
                btn.set_label("🔒");
                println!("GUI: PID configuration locked. Parameters synchronized.");
            } else {
                btn.set_label("🔓");
                println!("GUI: PID configuration unlocked.");
            }
        });

        let rc_conf = rc_clone.clone();
        let kpl = kp_left.clone();
        let kil = ki_left.clone();
        let kdl = kd_left.clone();
        let kpr = kp_right.clone();
        let kir = ki_right.clone();
        let kdr = kd_right.clone();
        let lfs = lidar_freq_scale.clone();
        apply_btn.connect_clicked(move |_| {
            let mut cmd = rc_conf.lock().unwrap();
            *cmd = RobotCommand::UpdateConfig {
                left_kp: kpl.value() as f32,
                left_ki: kil.value() as f32,
                left_kd: kdl.value() as f32,
                right_kp: kpr.value() as f32,
                right_ki: kir.value() as f32,
                right_kd: kdr.value() as f32,
                lidar_frequency: lfs.value() as f32,
            };
            println!("GUI: Sent RobotConfig update");
        });

        // Clone widgets for the polling loop
        let battery_label_c = battery_label.clone();
        let enc_left_label_c = enc_left_label.clone();
        let enc_right_label_c = enc_right_label.clone();
        let pose_x_label_c = pose_x_label.clone();
        let pose_y_label_c = pose_y_label.clone();
        let pose_theta_label_c = pose_theta_label.clone();
        let accel_label_c = accel_label.clone();
        let gyro_label_c = gyro_label.clone();
        let mag_label_c = mag_label.clone();
        let scan_rate_label_c = scan_rate_label.clone();
        let lidar_canvas_c = lidar_canvas.clone();
        let accel_canvas_c = accel_canvas.clone();
        let gyro_canvas_c = gyro_canvas.clone();
        let mag_canvas_c = mag_canvas.clone();
        let window_c = window.clone();
        let rx_inner = rx.clone();
        let loading_overlay_c = loading_overlay.clone();
        let loading_spinner_c = loading_spinner.clone();
        let lidar_freq_scale_c_loop = lidar_freq_scale.clone();
        let log_text_view_c = log_text_view.clone();

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
                        GuiUpdate::Pose { x, y, theta } => {
                            pose_x_label_c.set_text(&format!("{:.2}", x));
                            pose_y_label_c.set_text(&format!("{:.2}", y));
                            pose_theta_label_c.set_text(&format!("{:.2}", theta));
                            
                            let mut state = GUI_STATE.lock().unwrap();
                            state.robot_x = x;
                            state.robot_y = y;
                            state.robot_theta = theta;
                            
                            // Add to trajectory if moved enough
                            let last = state.trajectory.last().cloned();
                            if let Some((lx, ly)) = last {
                                let dist = ((lx - x).powi(2) + (ly - y).powi(2)).sqrt();
                                if dist > 0.02 {
                                    state.trajectory.push((x, y));
                                }
                            } else {
                                state.trajectory.push((x, y));
                            }
                            
                            lidar_canvas_c.queue_draw();
                        }
                        GuiUpdate::SlamPose { x, y, theta } => {
                            let mut state = GUI_STATE.lock().unwrap();
                            state.slam_x = x;
                            state.slam_y = y;
                            state.slam_theta = theta;
                            // For now, we don't update the UI labels, just the internal state
                        }
                        GuiUpdate::Map { width, height, data } => {
                            let mut state = GUI_STATE.lock().unwrap();
                            state.map_width = width;
                            state.map_height = height;
                            state.map_data = data;
                        }
                        GuiUpdate::Frontiers(f) => {
                            let mut state = GUI_STATE.lock().unwrap();
                            state.frontiers = f;
                        }
                        GuiUpdate::Path(p) => {
                            let mut state = GUI_STATE.lock().unwrap();
                            state.current_path = p;
                            lidar_canvas_c.queue_draw();
                        }
                        GuiUpdate::Imu { ax, ay, az, gx, gy, gz, mx, my, mz } => {
                            accel_label_c.set_text(&format!("Accel: X: {:.2} Y: {:.2} Z: {:.2}", ax, ay, az));
                            gyro_label_c.set_text(&format!("Gyro: X: {:.2} Y: {:.2} Z: {:.2}", gx, gy, gz));
                            mag_label_c.set_text(&format!("Mag: X: {:.2} Y: {:.2} Z: {:.2}", mx, my, mz));

                            let mut state = GUI_STATE.lock().unwrap();
                            
                            let update_buffer = |buffer: &mut std::collections::VecDeque<(f32, f32, f32)>, val: (f32, f32, f32)| {
                                if buffer.len() >= 100 { buffer.pop_front(); }
                                buffer.push_back(val);
                            };

                            update_buffer(&mut state.accel_history, (ax, ay, az));
                            update_buffer(&mut state.gyro_history, (gx, gy, gz));
                            update_buffer(&mut state.mag_history, (mx, my, mz));

                            accel_canvas_c.queue_draw();
                            gyro_canvas_c.queue_draw();
                            mag_canvas_c.queue_draw();
                        }
                        GuiUpdate::Lidar(points) => {
                            println!("[GUI] Received Lidar: {} points", points.len());
                            if !points.is_empty() {
                                let mut state = GUI_STATE.lock().unwrap();
                                crate::gui::lidar::update_scan(&mut state.display_scan, points);
                                lidar_canvas_c.queue_draw();
                            }
                        }
                        GuiUpdate::LidarScanRate(hz) => {
                            scan_rate_label_c.set_text(&format!("LiDAR: {:.1} Hz", hz));
                        }
                        GuiUpdate::Config(conf) => {
                            let is_updating_recv = is_updating.clone();
                            is_updating_recv.set(true);
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
                            lidar_freq_scale_c_loop.set_value(conf.lidar_frequency as f64);
                            is_updating_recv.set(false);
                        }
                        GuiUpdate::NavigationTarget(target) => {
                            let mut state = GUI_STATE.lock().unwrap();
                            state.navigation_target = target;
                            lidar_canvas_c.queue_draw();
                        }
                        GuiUpdate::Status(msg) => {
                            window_c.set_title(Some(&format!("HomeRobot Control Center - {}", msg)));
                            if msg.contains("Connected") {
                                loading_overlay_c.set_visible(false);
                                loading_spinner_c.stop();
                            } else if msg == "Idle" || msg == "Disconnected" {
                                loading_overlay_c.set_visible(true);
                                loading_spinner_c.start();
                            }
                        }
                        GuiUpdate::Log(msg) => {
                            let buffer = log_text_view_c.buffer();
                            let mut end_iter = buffer.end_iter();
                            buffer.insert(&mut end_iter, &format!("{}\n", msg));
                            let line_count = buffer.line_count();
                            if line_count > 500 {
                                if let Some(mut limit_iter) = buffer.iter_at_line(line_count - 500) {
                                    let mut start_iter = buffer.start_iter();
                                    buffer.delete(&mut start_iter, &mut limit_iter);
                                }
                            }
                            let mark = buffer.create_mark(None, &buffer.end_iter(), false);
                            log_text_view_c.scroll_to_mark(&mark, 0.0, true, 0.0, 1.0);
                            buffer.delete_mark(&mark);
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

#[cfg(test)]
mod tests {
    use gtk4::Builder;
    use glib::Object;

    #[test]
    fn test_ui_xml_loads_correctly() {
        // Initialize GTK for the test
        if let Err(e) = gtk4::init() {
            println!("Skipping GTK test: No display found ({})", e);
            return;
        }

        let builder = Builder::new();
        let ui_str = include_str!("../main_window.ui");
        
        // This will panic if XML is invalid
        builder.add_from_string(ui_str).expect("Failed to parse UI XML in tests");

        // Verify critical objects exist
        let objects = [
            "main_window",
            "sidebar_scroll",
            "btn_toggle_sidebar",
            "btn_clear_path",
            "global_power_scale",
            "lidar_freq_scale",
            "lidar_canvas",
            "accel_plot",
            "gyro_plot",
            "mag_plot",
            "btn_forward",
            "btn_stop",
            "pose_x_val",
            "btn_rotate_left",
            "btn_rotate_right",
            "btn_lock_pid",
            "btn_start_rerun",
            "btn_lidar",
            "btn_zoom_in",
            "btn_zoom_out",
            "btn_reset",
            "scan_rate_label",
            "log_text_view",
        ];

        for id in objects {
            assert!(builder.object::<Object>(id).is_some(), "Object '{}' missing from UI definition", id);
        }
    }
}
