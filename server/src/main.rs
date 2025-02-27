use gtk::prelude::*;
use gtk::{Application, ApplicationWindow, Notebook, Label, gdk};
use std::env;
use std::net::{TcpListener, TcpStream};
use std::io::{Read, Write};
use std::sync::{Arc, Mutex};
use std::thread;
use gdk::glib;

struct Gui {
    app: Application,
    notebook: Notebook
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let use_gui = args.contains(&"--gui".to_string());

    let listener = TcpListener::bind("127.0.0.1:12345").expect("Failed to bind to port");
    println!("Listening for connections...");

    let mut gui: Option<Gui> = None;
    if use_gui {
        let mut gui: Gui;
        gui.app = Application::builder()
            .application_id("com.example.KeyboardToSocket")
            .build();

        gui.app.connect_activate(move |app| build_ui(app));
        gui.app.run();
    }

    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                thread::spawn(move || read_stream(stream));
            }
            Err(e) => eprintln!("Connection failed: {}", e),
        }
    }

}

fn build_ui(app: &Application) {
    let window = ApplicationWindow::builder()
        .application(app)
        .title("Keyboard to Socket")
        .default_width(400)
        .default_height(300)
        .build();

    let notebook = Notebook::new();
    window.set_child(Some(&notebook));
    window.show();
}

fn add_client_ui(app: &Application) {
    println!("Do gui stuff");
}

fn handle_client(mut stream: TcpStream, notebook: Arc<Mutex<Notebook>>) {
    let mut buffer = [0; 512];
    let tab_label = Label::new(Some("New Connection"));
    let tab_content = Label::new(Some("Listening for messages..."));

    let notebook_clone = Arc::clone(&notebook);
    glib::MainContext::default().spawn_local(async move {
        let mut notebook = notebook_clone.lock().unwrap();
        notebook.append_page(&tab_content, Some(&tab_label));
    });

    while match stream.read(&mut buffer) {
        Ok(size) if size > 0 => {
            let message = String::from_utf8_lossy(&buffer[..size]).to_string();
            glib::MainContext::default().spawn_local(async move {
                tab_content.set_label(&message);
            });
            true
        }
        _ => false,
    } {}
}

fn read_stream(mut stream: TcpStream) {
    let mut buffer = [0; 512];
    while match stream.read(&mut buffer) {
        Ok(size) if size > 0 => {
            let message = String::from_utf8_lossy(&buffer[..size]);
            println!("Received: {}", message);
            true
        }
        _ => false,
    } {}

}
