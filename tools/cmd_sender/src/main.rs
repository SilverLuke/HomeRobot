use std::io::Write;
use std::net::{TcpListener, TcpStream};
use std::time::{Duration, Instant};
use clap::{Parser, Subcommand};
use prost::Message;
use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};

pub mod homerobot {
    include!(concat!(env!("OUT_DIR"), "/homerobot.rs"));
}

use homerobot::{ServerToRobotMessage, MotorMoveCommand, server_to_robot_message};

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
    
    /// Connect to proxy instead of listening for robot
    #[arg(short, long)]
    proxy: bool,

    /// Server host to connect to in proxy mode
    #[arg(long, default_value = "127.0.0.1")]
    host: String,
}

#[derive(Subcommand)]
enum Commands {
    /// Move the robot once
    Move {
        #[arg(short, long, default_value_t = 100)]
        left: u32,
        #[arg(short, long, default_value_t = 100)]
        right: u32,
        #[arg(long, default_value_t = 1.0)]
        l_angle: f32,
        #[arg(long, default_value_t = 1.0)]
        r_angle: f32,
    },
    /// Stop the robot
    Stop,
    /// Run diagnostic
    Diag,
    /// Send a burst of 5 commands rapidly
    Burst,
    /// Lidar control
    Lidar {
        #[arg(short, long)]
        active: bool,
    },
    /// Send constant move commands for a duration
    Constant {
        #[arg(short, long, default_value_t = 100)]
        left: u32,
        #[arg(short, long, default_value_t = 100)]
        right: u32,
        #[arg(long, default_value_t = 1.0)]
        l_angle: f32,
        #[arg(long, default_value_t = 1.0)]
        r_angle: f32,
        #[arg(short, long, default_value_t = 5)]
        duration_secs: u64,
    },
    /// Interactive WASD mode (Key Down = Move, Key Up = Stop)
    Interactive,
    /// Save the current map to house_map.pgm
    SaveMap,
    /// Enable/Disable Autonomous Exploration
    Explore {
        #[arg(short, long)]
        enabled: bool,
    },
    /// Navigate to coordinate X, Y
    GoTo {
        #[arg(allow_negative_numbers = true)]
        x: f32,
        #[arg(allow_negative_numbers = true)]
        y: f32,
    },
    /// Execute high-level G-code style motion command
    Motion {
        #[arg(short, long)]
        action: String,
        #[arg(short, long, default_value_t = 0.0)]
        distance: f32,
        #[arg(long, default_value_t = 0.0)]
        angle: f32,
        #[arg(short, long, default_value_t = 0.0)]
        radius: f32,
        #[arg(short, long, default_value_t = 150)]
        power: u32,
    },
}

fn send_msg(stream: &mut TcpStream, payload: server_to_robot_message::Payload) -> anyhow::Result<()> {
    let msg = ServerToRobotMessage {
        sequence_millis: 0,
        payload: Some(payload),
    };

    let mut buf = Vec::new();
    msg.encode(&mut buf)?;

    let len = buf.len() as u16;
    stream.write_all(&len.to_be_bytes())?;
    stream.write_all(&buf)?;
    stream.flush()?;
    Ok(())
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    let mut stream = if cli.proxy {
        let addr = format!("{}:12346", cli.host);
        println!("Connecting to Proxy on {}...", addr);
        TcpStream::connect(addr.as_str())?
    } else {
        println!("Listening for robot connection on 0.0.0.0:12345...");
        let listener = TcpListener::bind("0.0.0.0:12345")?;
        let (stream, addr) = listener.accept()?;
        println!("Robot connected from {}", addr);
        stream
    };

    match &cli.command {
        Commands::Move { left, right, l_angle, r_angle } => {
            println!("Sending Move: L={} R={}", left, right);
            send_msg(&mut stream, server_to_robot_message::Payload::MotorMove(MotorMoveCommand {
                left_power: *left,
                left_angle: *l_angle,
                right_power: *right,
                right_angle: *r_angle,
            }))?;
            println!("Waiting 2 seconds to ensure message delivery...");
            std::thread::sleep(Duration::from_secs(2));
        }
        Commands::Constant { left, right, l_angle, r_angle, duration_secs } => {
            println!("Sending Constant Move: L={} R={} for {}s", left, right, duration_secs);
            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(*duration_secs) {
                send_msg(&mut stream, server_to_robot_message::Payload::MotorMove(MotorMoveCommand {
                    left_power: *left,
                    left_angle: *l_angle,
                    right_power: *right,
                    right_angle: *r_angle,
                }))?;
                std::thread::sleep(Duration::from_millis(50));
            }
            println!("Constant move complete.");
            // Send stop at the end
            send_msg(&mut stream, server_to_robot_message::Payload::StopAll(true))?;
        }
        Commands::Stop => {
            println!("Sending Stop");
            send_msg(&mut stream, server_to_robot_message::Payload::StopAll(true))?;
        }
        Commands::Diag => {
            println!("Sending Diagnostic Request");
            send_msg(&mut stream, server_to_robot_message::Payload::RpcRequest(homerobot::RpcRequest {
                call_id: 1,
                method: "RunDiagnostic".to_string(),
                payload: vec![],
            }))?;
        }
        Commands::Burst => {
            println!("Sending BURST of 5 commands...");
            for i in 1..=5 {
                let pwr = i * 20;
                println!("Burst {}: Pwr={}", i, pwr);
                send_msg(&mut stream, server_to_robot_message::Payload::MotorMove(MotorMoveCommand {
                    left_power: pwr,
                    left_angle: 1.0,
                    right_power: pwr,
                    right_angle: 1.0,
                }))?;
                std::thread::sleep(Duration::from_millis(10));
            }
            println!("Burst complete.");
        }
        Commands::Lidar { active } => {
            println!("Sending Lidar Control: active={}", active);
            send_msg(&mut stream, server_to_robot_message::Payload::LidarControl(homerobot::LidarControlCommand {
                target_frequency_hz: 5.0,
                active: *active,
            }))?;
            std::thread::sleep(Duration::from_secs(2));
        }
        Commands::Interactive => {
            println!("Entering Interactive Mode (WASD). Press ESC to quit.");
            enable_raw_mode()?;
            
            let mut last_move = Instant::now();
            let mut current_keys = std::collections::HashSet::new();

            loop {
                if event::poll(Duration::from_millis(10))? {
                    if let Event::Key(key) = event::read()? {
                        match key.code {
                            KeyCode::Char('w') | KeyCode::Char('W') => {
                                if key.kind == KeyEventKind::Press { current_keys.insert('w'); }
                                else if key.kind == KeyEventKind::Release { current_keys.remove(&'w'); }
                            }
                            KeyCode::Char('s') | KeyCode::Char('S') => {
                                if key.kind == KeyEventKind::Press { current_keys.insert('s'); }
                                else if key.kind == KeyEventKind::Release { current_keys.remove(&'s'); }
                            }
                            KeyCode::Char('a') | KeyCode::Char('A') => {
                                if key.kind == KeyEventKind::Press { current_keys.insert('a'); }
                                else if key.kind == KeyEventKind::Release { current_keys.remove(&'a'); }
                            }
                            KeyCode::Char('d') | KeyCode::Char('D') => {
                                if key.kind == KeyEventKind::Press { current_keys.insert('d'); }
                                else if key.kind == KeyEventKind::Release { current_keys.remove(&'d'); }
                            }
                            KeyCode::Esc => break,
                            _ => {}
                        }
                    }
                }

                // If keys changed or timeout, send command
                if last_move.elapsed() > Duration::from_millis(50) {
                    let (l, r, la, ra) = if current_keys.contains(&'w') {
                        (100, 100, 1.0, 1.0)
                    } else if current_keys.contains(&'s') {
                        (100, 100, -1.0, -1.0)
                    } else if current_keys.contains(&'a') {
                        (100, 100, -1.0, 1.0)
                    } else if current_keys.contains(&'d') {
                        (100, 100, 1.0, -1.0)
                    } else {
                        (0, 0, 0.0, 0.0)
                    };

                    send_msg(&mut stream, server_to_robot_message::Payload::MotorMove(MotorMoveCommand {
                        left_power: l,
                        left_angle: la,
                        right_power: r,
                        right_angle: ra,
                    }))?;
                    last_move = Instant::now();
                }
            }
            disable_raw_mode()?;
        }
        Commands::SaveMap => {
            println!("Sending SaveMap Request");
            send_msg(&mut stream, server_to_robot_message::Payload::RpcRequest(homerobot::RpcRequest {
                call_id: 2,
                method: "SaveMap".to_string(),
                payload: vec![],
            }))?;
        }
        Commands::Explore { enabled } => {
            println!("Sending Explore Request: enabled={}", enabled);
            send_msg(&mut stream, server_to_robot_message::Payload::RpcRequest(homerobot::RpcRequest {
                call_id: 3,
                method: if *enabled { "StartExplore".to_string() } else { "StopExplore".to_string() },
                payload: vec![],
            }))?;
        }
        Commands::GoTo { x, y } => {
            println!("Sending GoTo Request: X={}, Y={}", x, y);
            let mut payload = Vec::new();
            payload.extend_from_slice(&x.to_le_bytes());
            payload.extend_from_slice(&y.to_le_bytes());
            send_msg(&mut stream, server_to_robot_message::Payload::RpcRequest(homerobot::RpcRequest {
                call_id: 4,
                method: "NavigateTo".to_string(),
                payload,
            }))?;
        }
        Commands::Motion { action, distance, angle, radius, power } => {
            println!("Sending Motion command: Action={}, Dist={}, Angle={}, Radius={}, Power={}", action, distance, angle, radius, power);
            
            let motion_type = match action.to_lowercase().as_str() {
                "straight" => 0,
                "rotate" => 1,
                "arc" => 2,
                _ => anyhow::bail!("Unknown action type: {}", action),
            };
            
            let req = homerobot::MotionRequest {
                r#type: motion_type,
                distance: *distance,
                angle: *angle,
                radius: *radius,
                max_power: *power,
                left_ticks: 0,
                right_ticks: 0,
            };
            
            let mut payload = Vec::new();
            req.encode(&mut payload)?;
            
            send_msg(&mut stream, server_to_robot_message::Payload::RpcRequest(homerobot::RpcRequest {
                call_id: 5,
                method: "ExecuteMotion".to_string(),
                payload,
            }))?;
            
            println!("Motion command sent. Waiting for completion response from server...");
            
            use std::io::Read;
            let mut response = [0u8; 1];
            match stream.read_exact(&mut response) {
                Ok(_) => {
                    if response[0] == 1 {
                        println!("SUCCESS: Motion completed successfully.");
                    } else {
                        println!("FAILURE: Motion failed or timed out.");
                    }
                }
                Err(e) => {
                    println!("Error reading completion response: {}", e);
                }
            }
        }
    }

    Ok(())
}
