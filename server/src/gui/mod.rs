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
    Capabilities {
        _has_accelerometer: bool,
        _has_gyroscope: bool,
        has_magnetometer: bool,
        _wheel_diameter_mm: f32,
        _wheel_track_mm: f32,
        _encoder_ticks_per_rev: u32,
    },
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
        log::info!("GUI: Initializing GTK components...");
        let builder = Builder::new();
        builder.add_from_string(include_str!("../../resources/main_window.ui")).expect("Failed to parse UI XML");
        
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
        let btn_center_robot: Button = builder.object("btn_center_robot").expect("Could not find btn_center_robot");
        let btn_zoom_fit: Button = builder.object("btn_zoom_fit").expect("Could not find btn_zoom_fit");
        let btn_show_map: gtk4::ToggleButton = builder.object("btn_show_map").expect("Could not find btn_show_map");
        let btn_show_sensor: gtk4::ToggleButton = builder.object("btn_show_sensor").expect("Could not find btn_show_sensor");
        let log_text_view: gtk4::TextView = builder.object("log_text_view").expect("Could not find log_text_view");

        // Motion Debugging controls
        let btn_rotate_90_left: Button = builder.object("btn_rotate_90_left").expect("Could not find btn_rotate_90_left");
        let btn_rotate_90_right: Button = builder.object("btn_rotate_90_right").expect("Could not find btn_rotate_90_right");
        let spin_rotate_angle: SpinButton = builder.object("spin_rotate_angle").expect("Could not find spin_rotate_angle");
        let btn_rotate_x_left: Button = builder.object("btn_rotate_x_left").expect("Could not find btn_rotate_x_left");
        let btn_rotate_x_right: Button = builder.object("btn_rotate_x_right").expect("Could not find btn_rotate_x_right");
        let spin_move_distance: SpinButton = builder.object("spin_move_distance").expect("Could not find spin_move_distance");
        let btn_move_x_forward: Button = builder.object("btn_move_x_forward").expect("Could not find btn_move_x_forward");
        let btn_move_x_backward: Button = builder.object("btn_move_x_backward").expect("Could not find btn_move_x_backward");
        let spin_arc_radius: SpinButton = builder.object("spin_arc_radius").expect("Could not find spin_arc_radius");
        let spin_arc_distance: SpinButton = builder.object("spin_arc_distance").expect("Could not find spin_arc_distance");
        let btn_arc_left: Button = builder.object("btn_arc_left").expect("Could not find btn_arc_left");
        let btn_arc_right: Button = builder.object("btn_arc_right").expect("Could not find btn_arc_right");

        let loading_overlay: gtk4::Box = builder.object("loading_overlay").expect("Could not find loading_overlay");
        let loading_spinner: gtk4::Spinner = builder.object("loading_spinner").expect("Could not find loading_spinner");
        loading_spinner.start();
        loading_overlay.set_visible(true);

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
        btn_center_robot.set_focusable(false);
        btn_zoom_fit.set_focusable(false);
        btn_show_map.set_focusable(false);
        btn_show_sensor.set_focusable(false);
        log_text_view.set_focusable(false);

        btn_rotate_90_left.set_focusable(false);
        btn_rotate_90_right.set_focusable(false);
        spin_rotate_angle.set_focusable(false);
        btn_rotate_x_left.set_focusable(false);
        btn_rotate_x_right.set_focusable(false);
        spin_move_distance.set_focusable(false);
        btn_move_x_forward.set_focusable(false);
        btn_move_x_backward.set_focusable(false);
        spin_arc_radius.set_focusable(false);
        spin_arc_distance.set_focusable(false);
        btn_arc_left.set_focusable(false);
        btn_arc_right.set_focusable(false);

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
            btn_center_robot.set_can_focus(false);
            btn_zoom_fit.set_can_focus(false);
            btn_show_map.set_can_focus(false);
            btn_show_sensor.set_can_focus(false);
            log_text_view.set_can_focus(false);

            btn_rotate_90_left.set_can_focus(false);
            btn_rotate_90_right.set_can_focus(false);
            spin_rotate_angle.set_can_focus(false);
            btn_rotate_x_left.set_can_focus(false);
            btn_rotate_x_right.set_can_focus(false);
            spin_move_distance.set_can_focus(false);
            btn_move_x_forward.set_can_focus(false);
            btn_move_x_backward.set_can_focus(false);
            spin_arc_radius.set_can_focus(false);
            spin_arc_distance.set_can_focus(false);
            btn_arc_left.set_can_focus(false);
            btn_arc_right.set_can_focus(false);
        }

        // Set up LIDAR drawing
        setup_lidar_drawing(&lidar_canvas);
        crate::gui::lidar::setup_accel_plot(&accel_canvas);
        crate::gui::lidar::setup_gyro_plot(&gyro_canvas);
        crate::gui::lidar::setup_mag_plot(&mag_canvas);

        // Map Drag & Click Navigation gesture
        let drag_controller = gtk4::GestureDrag::new();
        let rc_click = robot_command.clone();
        let btn_explore_click = btn_explore.clone();
        
        let start_pan = std::rc::Rc::new(std::cell::Cell::new((0.0f64, 0.0f64)));
        let start_pan_begin = start_pan.clone();
        
        drag_controller.connect_drag_begin(move |_, _, _| {
            let state = GUI_STATE.lock().unwrap();
            start_pan_begin.set((state.pan_x, state.pan_y));
        });
        
        let start_pan_update = start_pan.clone();
        let lidar_canvas_drag_update = lidar_canvas.clone();
        drag_controller.connect_drag_update(move |_, offset_x, offset_y| {
            let scale = {
                let state = GUI_STATE.lock().unwrap();
                0.05 * state.zoom_factor
            };
            let (spx, spy) = start_pan_update.get();
            
            // Dragging: adjust camera center (opposite of cursor movement in screen space)
            let new_pan_x = spx + offset_y / (1000.0 * scale);
            let new_pan_y = spy + offset_x / (1000.0 * scale);
            
            {
                let mut state = GUI_STATE.lock().unwrap();
                state.pan_x = new_pan_x;
                state.pan_y = new_pan_y;
            }
            lidar_canvas_drag_update.queue_draw();
        });
        
        let lidar_canvas_drag_end = lidar_canvas.clone();
        drag_controller.connect_drag_end(move |gesture, offset_x, offset_y| {
            let dist = (offset_x * offset_x + offset_y * offset_y).sqrt();
            if dist < 6.0 {
                if let Some((start_x, start_y)) = gesture.start_point() {
                    let width = lidar_canvas_drag_end.width() as f64;
                    let height = lidar_canvas_drag_end.height() as f64;
                    let world_center_x = width / 2.0;
                    let world_center_y = height / 2.0;
                    
                    let (pan_x, pan_y, scale) = {
                        let state = GUI_STATE.lock().unwrap();
                        (state.pan_x, state.pan_y, 0.05 * state.zoom_factor)
                    };
                    
                    let goal_y = (pan_y + (world_center_x - start_x) / (1000.0 * scale)) as f32;
                    let goal_x = (pan_x + (world_center_y - start_y) / (1000.0 * scale)) as f32;
                    
                    log::info!("GUI: Map clicked at pixel ({:.1}, {:.1}) -> World Goal: X={:.2}, Y={:.2}", start_x, start_y, goal_x, goal_y);
                    
                    {
                        let mut state = GUI_STATE.lock().unwrap();
                        state.navigation_target = Some((goal_x, goal_y));
                        state.current_path.clear();
                    }
                    lidar_canvas_drag_end.queue_draw();
                    
                    if btn_explore_click.is_active() {
                        btn_explore_click.set_active(false);
                    }
                    
                    let mut cmd = rc_click.lock().unwrap();
                    *cmd = RobotCommand::NavigateTo { x: goal_x, y: goal_y };
                }
            }
        });
        lidar_canvas.add_controller(drag_controller);

        let lidar_canvas_clear = lidar_canvas.clone();
        btn_clear_path.connect_clicked(move |_| {
            let mut state = GUI_STATE.lock().unwrap();
            state.trajectory.clear();
            lidar_canvas_clear.queue_draw();
            log::info!("GUI: Trajectory cleared.");
        });

        // Custom CSS for Buttons and Loading Overlay
        let provider = gtk4::CssProvider::new();
        provider.load_from_string(include_str!("../../resources/style.css"));
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
            log::info!("GUI: Exploration {}", if enabled { "ENABLED" } else { "DISABLED" });
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
                    log::info!("GUI: Keyboard Move: Forward (Power: {})", current_power);
                    *cmd = RobotCommand::MotorAngle { 
                        left_power: current_power, left_angle: 1.0, 
                        right_power: current_power, right_angle: 1.0 
                    };
                },
                Key::s | Key::S => {
                    log::info!("GUI: Keyboard Move: Backward (Power: {})", current_power);
                    *cmd = RobotCommand::MotorAngle { 
                        left_power: current_power, left_angle: -1.0, 
                        right_power: current_power, right_angle: -1.0 
                    };
                },
                Key::a | Key::A => {
                    log::info!("GUI: Keyboard Move: Left (Power: {})", current_power);
                    *cmd = RobotCommand::MotorAngle { 
                        left_power: 0, left_angle: 0.0, 
                        right_power: current_power, right_angle: 1.0 
                    };
                },
                Key::d | Key::D => {
                    log::info!("GUI: Keyboard Move: Right (Power: {})", current_power);
                    *cmd = RobotCommand::MotorAngle { 
                        left_power: current_power, left_angle: 1.0, 
                        right_power: 0, right_angle: 0.0 
                    };
                },
                Key::q | Key::Q => {
                    log::info!("GUI: Keyboard Move: Rotate Left (Power: {})", current_power);
                    *cmd = RobotCommand::MotorAngle { 
                        left_power: current_power, left_angle: -1.0, 
                        right_power: current_power, right_angle: 1.0 
                    };
                },
                Key::e | Key::E => {
                    log::info!("GUI: Keyboard Move: Rotate Right (Power: {})", current_power);
                    *cmd = RobotCommand::MotorAngle { 
                        left_power: current_power, left_angle: 1.0, 
                        right_power: current_power, right_angle: -1.0 
                    };
                },
                Key::h | Key::H => {
                    let new_active = !btn_toggle_sidebar_clone.is_active();
                    btn_toggle_sidebar_clone.set_active(new_active);
                    log::info!("GUI: Sidebar visibility toggled via H: {}", new_active);
                },
                Key::l | Key::L => {
                    let new_state = !btn_lidar_clone.is_active();
                    log::info!("GUI: Keyboard Lidar Toggle: {}", if new_state { "ON" } else { "OFF" });
                    *cmd = RobotCommand::LidarControl { 
                        active: new_state, 
                        target_frequency_hz: if new_state { lidar_freq_scale_key.value() as f32 } else { 0.0 } 
                    };
                    btn_lidar_clone.set_active(new_state);
                },
                Key::space => {
                    log::info!("GUI: Keyboard STOP");
                    {
                        let mut state = GUI_STATE.lock().unwrap();
                        state.navigation_target = None;
                        state.current_path.clear();
                    }
                    lidar_canvas_key.queue_draw();
                    *cmd = RobotCommand::StopMoving;
                },
                Key::t | Key::T => {
                    log::info!("GUI: Keyboard Trigger Diagnostics");
                    *cmd = RobotCommand::RunDiagnostic;
                },
                Key::plus | Key::equal | Key::KP_Add => {
                    let mut state = GUI_STATE.lock().unwrap();
                    state.zoom_factor *= 1.2;
                    log::info!("GUI: Zoom In via keyboard: {:.2}x", state.zoom_factor);
                    lidar_canvas_key.queue_draw();
                },
                Key::minus | Key::KP_Subtract => {
                    let mut state = GUI_STATE.lock().unwrap();
                    state.zoom_factor = (state.zoom_factor / 1.2).max(0.1);
                    log::info!("GUI: Zoom Out via keyboard: {:.2}x", state.zoom_factor);
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
            log::info!("GUI: Sidebar visibility changed: {}", active);
        });

        window.add_controller(key_controller);

        // Connect Movement Buttons
        let rc_fwd = rc_clone.clone();
        let ps_fwd = power_scale.clone();
        btn_forward.connect_clicked(move |_| {
            let p = ps_fwd.value() as u8;
            log::info!("GUI: Button Move: Forward (Power: {})", p);
            *rc_fwd.lock().unwrap() = RobotCommand::MotorAngle { left_power: p, left_angle: 1.0, right_power: p, right_angle: 1.0 };
        });

        let rc_bwd = rc_clone.clone();
        let ps_bwd = power_scale.clone();
        btn_backward.connect_clicked(move |_| {
            let p = ps_bwd.value() as u8;
            log::info!("GUI: Button Move: Backward (Power: {})", p);
            *rc_bwd.lock().unwrap() = RobotCommand::MotorAngle { left_power: p, left_angle: -1.0, right_power: p, right_angle: -1.0 };
        });

        let rc_left = rc_clone.clone();
        let ps_left = power_scale.clone();
        btn_left.connect_clicked(move |_| {
            let p = ps_left.value() as u8;
            log::info!("GUI: Button Move: Left (Power: {})", p);
            *rc_left.lock().unwrap() = RobotCommand::MotorAngle { left_power: 0, left_angle: 0.0, right_power: p, right_angle: 1.0 };
        });

        let rc_right = rc_clone.clone();
        let ps_right = power_scale.clone();
        btn_right.connect_clicked(move |_| {
            let p = ps_right.value() as u8;
            log::info!("GUI: Button Move: Right (Power: {})", p);
            *rc_right.lock().unwrap() = RobotCommand::MotorAngle { left_power: p, left_angle: 1.0, right_power: 0, right_angle: 0.0 };
        });

        let rc_rot_left = rc_clone.clone();
        let ps_rot_left = power_scale.clone();
        btn_rotate_left.connect_clicked(move |_| {
            let p = ps_rot_left.value() as u8;
            log::info!("GUI: Button Move: Rotate Left (Power: {})", p);
            *rc_rot_left.lock().unwrap() = RobotCommand::MotorAngle { left_power: p, left_angle: -1.0, right_power: p, right_angle: 1.0 };
        });

        let rc_rot_right = rc_clone.clone();
        let ps_rot_right = power_scale.clone();
        btn_rotate_right.connect_clicked(move |_| {
            let p = ps_rot_right.value() as u8;
            log::info!("GUI: Button Move: Rotate Right (Power: {})", p);
            *rc_rot_right.lock().unwrap() = RobotCommand::MotorAngle { left_power: p, left_angle: 1.0, right_power: p, right_angle: -1.0 };
        });

        let rc_stop = rc_clone.clone();
        let lidar_canvas_stop = lidar_canvas.clone();
        btn_stop.connect_clicked(move |_| {
            log::info!("GUI: Button STOP");
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
            log::info!("GUI: Button Save Map");
            *rc_save.lock().unwrap() = RobotCommand::SaveMap;
        });

        let rc_reset = rc_clone.clone();
        let lidar_canvas_reset = lidar_canvas.clone();
        btn_reset.connect_clicked(move |_| {
            log::info!("GUI: Button Reset Triggered");
            {
                let mut state = GUI_STATE.lock().unwrap();
                state.navigation_target = None;
                state.current_path.clear();
                state.pan_x = 0.0;
                state.pan_y = 0.0;
                state.zoom_factor = 1.0;
            }
            lidar_canvas_reset.queue_draw();
            *rc_reset.lock().unwrap() = RobotCommand::Reset;
        });

        // btn_rotate_90_left
        let rc_rot_90_l = rc_clone.clone();
        btn_rotate_90_left.connect_clicked(move |_| {
            let (ticks_per_meter, wheel_base) = {
                let sizes = crate::constants::ROBOT_SIZES.lock().unwrap();
                (sizes.ticks_per_meter, sizes.wheel_base)
            };
            let angle_rad = 90.0_f32.to_radians();
            let dist = angle_rad * (wheel_base / 2.0);
            let ticks = (dist * ticks_per_meter) as i32;
            let left_ticks = -ticks;
            let right_ticks = ticks;
            log::info!("GUI: Button Rotate 90° CCW (L_ticks={}, R_ticks={})", left_ticks, right_ticks);
            *rc_rot_90_l.lock().unwrap() = RobotCommand::ExecuteMotion {
                motion_type: 1,
                left_ticks,
                right_ticks,
                max_power: 150,
            };
        });

        // btn_rotate_90_right
        let rc_rot_90_r = rc_clone.clone();
        btn_rotate_90_right.connect_clicked(move |_| {
            let (ticks_per_meter, wheel_base) = {
                let sizes = crate::constants::ROBOT_SIZES.lock().unwrap();
                (sizes.ticks_per_meter, sizes.wheel_base)
            };
            let angle_rad = -90.0_f32.to_radians();
            let dist = angle_rad * (wheel_base / 2.0);
            let ticks = (dist * ticks_per_meter) as i32;
            let left_ticks = -ticks;
            let right_ticks = ticks;
            log::info!("GUI: Button Rotate 90° CW (L_ticks={}, R_ticks={})", left_ticks, right_ticks);
            *rc_rot_90_r.lock().unwrap() = RobotCommand::ExecuteMotion {
                motion_type: 1,
                left_ticks,
                right_ticks,
                max_power: 150,
            };
        });

        // btn_rotate_x_left
        let rc_rot_x_l = rc_clone.clone();
        let spin_rot_l = spin_rotate_angle.clone();
        btn_rotate_x_left.connect_clicked(move |_| {
            let angle_deg = spin_rot_l.value() as f32;
            let (ticks_per_meter, wheel_base) = {
                let sizes = crate::constants::ROBOT_SIZES.lock().unwrap();
                (sizes.ticks_per_meter, sizes.wheel_base)
            };
            let angle_rad = angle_deg.to_radians();
            let dist = angle_rad * (wheel_base / 2.0);
            let ticks = (dist * ticks_per_meter) as i32;
            let left_ticks = -ticks;
            let right_ticks = ticks;
            log::info!("GUI: Button Rotate X° CCW: {}° (L_ticks={}, R_ticks={})", angle_deg, left_ticks, right_ticks);
            *rc_rot_x_l.lock().unwrap() = RobotCommand::ExecuteMotion {
                motion_type: 1,
                left_ticks,
                right_ticks,
                max_power: 150,
            };
        });

        // btn_rotate_x_right
        let rc_rot_x_r = rc_clone.clone();
        let spin_rot_r = spin_rotate_angle.clone();
        btn_rotate_x_right.connect_clicked(move |_| {
            let angle_deg = -spin_rot_r.value() as f32;
            let (ticks_per_meter, wheel_base) = {
                let sizes = crate::constants::ROBOT_SIZES.lock().unwrap();
                (sizes.ticks_per_meter, sizes.wheel_base)
            };
            let angle_rad = angle_deg.to_radians();
            let dist = angle_rad * (wheel_base / 2.0);
            let ticks = (dist * ticks_per_meter) as i32;
            let left_ticks = -ticks;
            let right_ticks = ticks;
            log::info!("GUI: Button Rotate X° CW: {}° (L_ticks={}, R_ticks={})", -angle_deg, left_ticks, right_ticks);
            *rc_rot_x_r.lock().unwrap() = RobotCommand::ExecuteMotion {
                motion_type: 1,
                left_ticks,
                right_ticks,
                max_power: 150,
            };
        });

        // btn_move_x_forward
        let rc_move_fwd = rc_clone.clone();
        let spin_move_fwd = spin_move_distance.clone();
        btn_move_x_forward.connect_clicked(move |_| {
            let dist_cm = spin_move_fwd.value() as f32;
            let dist_m = dist_cm / 100.0;
            let ticks_per_meter = crate::constants::ROBOT_SIZES.lock().unwrap().ticks_per_meter;
            let ticks = (dist_m * ticks_per_meter) as i32;
            let left_ticks = ticks;
            let right_ticks = ticks;
            log::info!("GUI: Button Move Forward: {}cm (L_ticks={}, R_ticks={})", dist_cm, left_ticks, right_ticks);
            *rc_move_fwd.lock().unwrap() = RobotCommand::ExecuteMotion {
                motion_type: 0,
                left_ticks,
                right_ticks,
                max_power: 150,
            };
        });

        // btn_move_x_backward
        let rc_move_bwd = rc_clone.clone();
        let spin_move_bwd = spin_move_distance.clone();
        btn_move_x_backward.connect_clicked(move |_| {
            let dist_cm = spin_move_bwd.value() as f32;
            let dist_m = -dist_cm / 100.0;
            let ticks_per_meter = crate::constants::ROBOT_SIZES.lock().unwrap().ticks_per_meter;
            let ticks = (dist_m * ticks_per_meter) as i32;
            let left_ticks = ticks;
            let right_ticks = ticks;
            log::info!("GUI: Button Move Backward: {}cm (L_ticks={}, R_ticks={})", dist_cm, left_ticks, right_ticks);
            *rc_move_bwd.lock().unwrap() = RobotCommand::ExecuteMotion {
                motion_type: 0,
                left_ticks,
                right_ticks,
                max_power: 150,
            };
        });

        // btn_arc_left
        let rc_arc_l = rc_clone.clone();
        let spin_arc_r_l = spin_arc_radius.clone();
        let spin_arc_d_l = spin_arc_distance.clone();
        btn_arc_left.connect_clicked(move |_| {
            let radius_cm = spin_arc_r_l.value() as f32;
            let dist_cm = spin_arc_d_l.value() as f32;
            let radius_m = radius_cm / 100.0;
            let dist_m = dist_cm / 100.0;
            let angle_rad = dist_m / radius_m;
            let (ticks_per_meter, wheel_base) = {
                let sizes = crate::constants::ROBOT_SIZES.lock().unwrap();
                (sizes.ticks_per_meter, sizes.wheel_base)
            };
            let left_dist = angle_rad * (radius_m - wheel_base / 2.0);
            let right_dist = angle_rad * (radius_m + wheel_base / 2.0);
            let left_ticks = (left_dist * ticks_per_meter) as i32;
            let right_ticks = (right_dist * ticks_per_meter) as i32;
            log::info!("GUI: Button Arc Left: Radius={}cm, Dist={}cm (L_ticks={}, R_ticks={})", radius_cm, dist_cm, left_ticks, right_ticks);
            *rc_arc_l.lock().unwrap() = RobotCommand::ExecuteMotion {
                motion_type: 2,
                left_ticks,
                right_ticks,
                max_power: 150,
            };
        });

        // btn_arc_right
        let rc_arc_r = rc_clone.clone();
        let spin_arc_r_r = spin_arc_radius.clone();
        let spin_arc_d_r = spin_arc_distance.clone();
        btn_arc_right.connect_clicked(move |_| {
            let radius_cm = spin_arc_r_r.value() as f32;
            let dist_cm = spin_arc_d_r.value() as f32;
            let radius_m = radius_cm / 100.0;
            let dist_m = dist_cm / 100.0;
            let angle_rad = -dist_m / radius_m;
            let (ticks_per_meter, wheel_base) = {
                let sizes = crate::constants::ROBOT_SIZES.lock().unwrap();
                (sizes.ticks_per_meter, sizes.wheel_base)
            };
            let left_dist = angle_rad * (radius_m - wheel_base / 2.0);
            let right_dist = angle_rad * (radius_m + wheel_base / 2.0);
            let left_ticks = (left_dist * ticks_per_meter) as i32;
            let right_ticks = (right_dist * ticks_per_meter) as i32;
            log::info!("GUI: Button Arc Right: Radius={}cm, Dist={}cm (L_ticks={}, R_ticks={})", radius_cm, dist_cm, left_ticks, right_ticks);
            *rc_arc_r.lock().unwrap() = RobotCommand::ExecuteMotion {
                motion_type: 2,
                left_ticks,
                right_ticks,
                max_power: 150,
            };
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
            log::info!("GUI: Lidar toggled: {}", active);
        });
        btn_lidar.set_active(true);

        let lidar_canvas_show_map = lidar_canvas.clone();
        btn_show_map.connect_toggled(move |btn| {
            let active = btn.is_active();
            {
                let mut state = GUI_STATE.lock().unwrap();
                state.show_map = active;
            }
            lidar_canvas_show_map.queue_draw();
            log::info!("GUI: Show Map toggled: {}", active);
        });

        let lidar_canvas_show_sensor = lidar_canvas.clone();
        btn_show_sensor.connect_toggled(move |btn| {
            let active = btn.is_active();
            {
                let mut state = GUI_STATE.lock().unwrap();
                state.show_lidar = active;
            }
            lidar_canvas_show_sensor.queue_draw();
            log::info!("GUI: Plot Sensor Reads toggled: {}", active);
        });

        let lidar_canvas_zoom_in = lidar_canvas.clone();
        btn_zoom_in.connect_clicked(move |_| {
            let mut state = GUI_STATE.lock().unwrap();
            state.zoom_factor *= 1.2;
            log::info!("GUI: Zoom In clicked: {:.2}x", state.zoom_factor);
            lidar_canvas_zoom_in.queue_draw();
        });

        let lidar_canvas_zoom_out = lidar_canvas.clone();
        btn_zoom_out.connect_clicked(move |_| {
            let mut state = GUI_STATE.lock().unwrap();
            state.zoom_factor = (state.zoom_factor / 1.2).max(0.1);
            log::info!("GUI: Zoom Out clicked: {:.2}x", state.zoom_factor);
            lidar_canvas_zoom_out.queue_draw();
        });

        let lidar_canvas_center = lidar_canvas.clone();
        btn_center_robot.connect_clicked(move |_| {
            let mut state = GUI_STATE.lock().unwrap();
            state.pan_x = state.robot_x as f64;
            state.pan_y = state.robot_y as f64;
            log::info!("GUI: Centered map on robot at ({:.2}, {:.2})", state.pan_x, state.pan_y);
            lidar_canvas_center.queue_draw();
        });

        let lidar_canvas_fit = lidar_canvas.clone();
        btn_zoom_fit.connect_clicked(move |_| {
            let mut state = GUI_STATE.lock().unwrap();
            
            let mut points = Vec::new();
            points.push((state.robot_x, state.robot_y));
            for &(tx, ty) in &state.trajectory {
                points.push((tx, ty));
            }
            
            if !state.map_data.is_empty() && state.map_width > 0 && state.map_height > 0 {
                let res = 0.05f64;
                for y in 0..state.map_height {
                    for x in 0..state.map_width {
                        let val = state.map_data[y * state.map_width + x];
                        if val != 0 {
                            let wx = (x as f64 - state.map_width as f64 / 2.0) * res;
                            let wy = (y as f64 - state.map_height as f64 / 2.0) * res;
                            points.push((wx as f32, wy as f32));
                        }
                    }
                }
            }
            
            if !points.is_empty() {
                let mut min_x = points[0].0;
                let mut max_x = points[0].0;
                let mut min_y = points[0].1;
                let mut max_y = points[0].1;
                
                for &(px, py) in points.iter().skip(1) {
                    min_x = min_x.min(px);
                    max_x = max_x.max(px);
                    min_y = min_y.min(py);
                    max_y = max_y.max(py);
                }
                
                state.pan_x = ((min_x + max_x) / 2.0) as f64;
                state.pan_y = ((min_y + max_y) / 2.0) as f64;
                
                let size_x = (max_x - min_x).max(2.0) as f64;
                let size_y = (max_y - min_y).max(2.0) as f64;
                
                let width = lidar_canvas_fit.width() as f64;
                let height = lidar_canvas_fit.height() as f64;
                
                let zoom_x = (width * 0.9) / (size_y * 50.0);
                let zoom_y = (height * 0.9) / (size_x * 50.0);
                
                state.zoom_factor = zoom_x.min(zoom_y).clamp(0.1, 10.0);
                log::info!(
                    "GUI: Zoomed to fit bounds X: [{:.2}, {:.2}], Y: [{:.2}, {:.2}], zoom_factor: {:.2}x",
                    min_x, max_x, min_y, max_y, state.zoom_factor
                );
            }
            lidar_canvas_fit.queue_draw();
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
                log::info!("GUI: Rerun is already running!");
                return;
            }
            log::info!("GUI: Starting Rerun viewer on-demand...");
            match rerun::RecordingStreamBuilder::new("home-robot").spawn() {
                Ok(new_stream) => {
                    *stream = new_stream;
                    log::info!("GUI: Rerun viewer spawned successfully.");
                    btn_start_rerun_clone.set_sensitive(false);
                }
                Err(e) => {
                    log::error!("GUI: Failed to spawn Rerun: {:?}", e);
                }
            }
        });

        // PID Lock Logic and Apply Config Button
        let is_updating = std::rc::Rc::new(std::cell::Cell::new(false));

        let rc_lidar_freq = rc_clone.clone();
        let btn_lidar_freq_ref = btn_lidar.clone();
        let is_updating_freq = is_updating.clone();
        let lidar_freq_scale_c = lidar_freq_scale.clone();

        let drag_controller = gtk4::GestureDrag::new();
        let rc_lidar_drag = rc_lidar_freq.clone();
        let btn_lidar_drag_ref = btn_lidar_freq_ref.clone();
        let scale_drag = lidar_freq_scale_c.clone();
        let is_updating_drag = is_updating_freq.clone();

        drag_controller.connect_drag_end(move |_, _, _| {
            if !is_updating_drag.get() {
                let freq = scale_drag.value() as f32;
                let active = btn_lidar_drag_ref.is_active();
                if active {
                    let mut cmd = rc_lidar_drag.lock().unwrap();
                    *cmd = RobotCommand::LidarControl {
                        active: true,
                        target_frequency_hz: freq,
                    };
                    log::info!("GUI: Lidar frequency changed on release: {} Hz", freq);
                }
            }
        });
        lidar_freq_scale.add_controller(drag_controller);

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
                log::info!("GUI: PID configuration locked. Parameters synchronized.");
            } else {
                btn.set_label("🔓");
                log::info!("GUI: PID configuration unlocked.");
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
            log::info!("GUI: Sent RobotConfig update");
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
                            log::info!("[GUI] Received Battery: {}%", percentage);
                            battery_label_c.set_text(&format!("Battery: {}% ({} mV)", percentage, voltage_mv));
                        }
                        GuiUpdate::Encoders { left, right } => {
                            log::info!("[GUI] Received Encoders: L={} R={}", left, right);
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
                        GuiUpdate::Capabilities {
                            _has_accelerometer,
                            _has_gyroscope,
                            has_magnetometer,
                            _wheel_diameter_mm,
                            _wheel_track_mm,
                            _encoder_ticks_per_rev,
                        } => {
                            log::info!("[GUI] Received capabilities: magnetometer={}", has_magnetometer);
                            mag_label_c.set_visible(has_magnetometer);
                            mag_canvas_c.set_visible(has_magnetometer);
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
            log::info!("Skipping GTK test: No display found ({})", e);
            return;
        }

        let builder = Builder::new();
        let ui_str = include_str!("../../resources/main_window.ui");
        
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
            "btn_center_robot",
            "btn_zoom_fit",
            "btn_reset",
            "scan_rate_label",
            "log_text_view",
        ];

        for id in objects {
            assert!(builder.object::<Object>(id).is_some(), "Object '{}' missing from UI definition", id);
        }
    }
}
