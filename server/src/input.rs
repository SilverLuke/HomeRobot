use gtk::prelude::{ApplicationExt, ApplicationExtManual, GtkWindowExt, WidgetExt};
use gtk::ApplicationWindow;

// use sdl2::joystick::Joystick;
// use sdl2::EventPump;
//
// fn main() {
//     let sdl_context = sdl2::init().unwrap();
//     let joystick_subsystem = sdl_context.joystick().unwrap();
//
//     if joystick_subsystem.num_joysticks().unwrap() > 0 {
//         let joystick = Joystick::open(0).unwrap();
//         println!("Joystick connected: {}", joystick.name());
//     } else {
//         println!("No joystick found!");
//     }
// }
//
/** GTK */

pub(crate) fn window() {
    let application = gtk::Application::new(
        Some("com.example.keyboard"),
        Default::default(),
    );

    application.connect_activate(|app| {
        let window = ApplicationWindow::new(app);
        window.set_title(Option::from("Keyboard Input Example"));
        window.set_default_size(300, 100);

        window.show();
    });

    application.run();
}
//
//
// use gilrs::{Gilrs, Event, EventType};
//
// fn main() {
//     let mut gilrs = Gilrs::new().unwrap();
//
//     println!("Waiting for gamepad input...");
//
//     loop {
//         while let Some(Event { event, id, .. }) = gilrs.next_event() {
//             match event {
//                 EventType::ButtonPressed(btn, _) => {
//                     println!("Gamepad {}: Button {:?} pressed", id, btn);
//                 }
//                 EventType::ButtonReleased(btn, _) => {
//                     println!("Gamepad {}: Button {:?} released", id, btn);
//                 }
//                 EventType::AxisChanged(axis, value, _) => {
//                     println!("Gamepad {}: Axis {:?} moved to {}", id, axis, value);
//                 }
//                 _ => {}
//             }
//         }
//     }
// }
