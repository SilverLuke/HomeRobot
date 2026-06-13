use tokio::net::{TcpListener, TcpStream};
use tokio::io::{self, AsyncReadExt, AsyncWriteExt};
use std::sync::Arc;
use tokio::sync::Mutex;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let robot_listener = TcpListener::bind("0.0.0.0:12345").await?;
    let controller_listener = TcpListener::bind("127.0.0.1:12346").await?;

    println!("ROBOT PROXY STARTED");
    println!("Robot: 12345, Controller: 12346");

    let current_controller: Arc<Mutex<Option<TcpStream>>> = Arc::new(Mutex::new(None));

    // Task 1: Accept Controllers
    let ctrl_accept = current_controller.clone();
    tokio::spawn(async move {
        loop {
            if let Ok((stream, _)) = controller_listener.accept().await {
                println!("[PROXY] Controller connected");
                let mut lock = ctrl_accept.lock().await;
                *lock = Some(stream);
            }
        }
    });

    // Task 2: Accept Robot
    loop {
        let (robot_stream, _) = robot_listener.accept().await?;
        println!("[PROXY] Robot connected");

        let (mut robot_read, mut robot_write) = io::split(robot_stream);
        let mut buf = [0u8; 4096];
        let ctrl_ref = current_controller.clone();

        loop {
            tokio::select! {
                // From Robot -> Controller
                res = robot_read.read(&mut buf) => {
                    match res {
                        Ok(0) => break,
                        Ok(n) => {
                            let mut lock = ctrl_ref.lock().await;
                            if let Some(c) = lock.as_mut() {
                                if let Err(_) = c.write_all(&buf[..n]).await {
                                    *lock = None;
                                }
                            }
                        }
                        Err(_) => break,
                    }
                }
                // From Controller -> Robot
                _ = tokio::time::sleep(std::time::Duration::from_millis(5)) => {
                    let mut lock = ctrl_ref.lock().await;
                    if let Some(c) = lock.as_mut() {
                        let mut cbuf = [0u8; 1024];
                        match c.try_read(&mut cbuf) {
                            Ok(0) => *lock = None,
                            Ok(n) => {
                                let _ = robot_write.write_all(&cbuf[..n]).await;
                            }
                            _ => {}
                        }
                    }
                }
            }
        }
        println!("[PROXY] Robot disconnected");
    }
}
