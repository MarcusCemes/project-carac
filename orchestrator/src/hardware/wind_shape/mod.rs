use std::{
    iter,
    net::{IpAddr, Ipv4Addr, SocketAddr},
    str,
    sync::Arc,
    time::Duration,
};

use eyre::{eyre, Context, Result};
use tokio::{io, net::UdpSocket, select, sync::watch, task::JoinSet, time::interval};

use crate::config::WindShapeConfig;

const MODULE_COUNT: usize = 56;
const MODULE_FANS: usize = 18;

const LOCAL_PORT: u16 = 60333;
const REMOTE_PORT: u16 = 60334;

const STATUS_INTERVAL: Duration = Duration::from_millis(40);

const RX_BUFFER_SIZE: usize = 65536;

pub struct WindShape {
    client_id: u8,
    link: Link,

    config: Config,
    request_control: watch::Sender<bool>,
    status: watch::Receiver<Status>,

    #[allow(dead_code)]
    task_handle: JoinSet<()>,
}

#[derive(Clone)]
struct Link {
    addr: SocketAddr,
    socket: Arc<UdpSocket>,
}

#[derive(Default)]
struct Config {
    enable_power: bool,
    fan_speed: u8,
}

#[derive(Default)]
struct Status {
    in_control: bool,
}

enum Request {
    InitiateConnection,
    Module { enable_power: bool, fan_speed: u8 },
    Status { request_control: bool },
}

struct Response {
    client_id: u8,
    payload: ResponsePayload,
}

enum ResponsePayload {
    Address,
    Module,
    Status { in_control: bool },
}

impl WindShape {
    pub async fn connect_from_config(config: &WindShapeConfig) -> Result<Self> {
        Self::connect(config.ip).await
    }

    pub async fn connect(ip: IpAddr) -> Result<WindShape> {
        let link = Link::connect(ip).await;

        tracing::debug!("Handshaking with WindShape...");

        link.send_request(Request::InitiateConnection, 0).await?;
        let client_id = link.recv_handshake().await?;

        tracing::info!("Handshake successful!");

        // Spawn background tasks to send and receive status messages
        let (status, request_control, task_handle) = Self::spawn_tasks(client_id, link.clone());

        Ok(WindShape {
            client_id,
            link,

            config: Default::default(),
            request_control,

            status,
            task_handle,
        })
    }

    /* == Public API == */

    pub async fn request_control(&mut self) {
        self.update_control(true).await
    }

    pub async fn release_control(&mut self) {
        self.update_control(false).await
    }

    pub async fn enable_power(&mut self) -> Result<()> {
        self.config.enable_power = true;
        self.send_module_state().await
    }

    pub async fn disable_power(&mut self) -> Result<()> {
        self.config.enable_power = false;
        self.config.fan_speed = 0;
        self.send_module_state().await
    }

    pub async fn set_fan_speed(&mut self, fan_speed: u8) -> Result<()> {
        self.config.fan_speed = fan_speed;
        self.send_module_state().await
    }

    async fn update_control(&mut self, value: bool) {
        self.request_control
            .send(value)
            .expect("WindShape sender died");

        loop {
            self.status.changed().await.expect("Status receiver died");

            if self.status.borrow().in_control == value {
                break;
            }
        }
    }

    /* == Socket == */

    async fn send_module_state(&self) -> Result<()> {
        let request = Request::Module {
            enable_power: self.config.enable_power,
            fan_speed: self.config.fan_speed,
        };

        self.link
            .send_request(request, self.client_id)
            .await
            .wrap_err("Failed to send module state")
    }

    /* == Background tasks == */

    fn spawn_tasks(
        client_id: u8,
        socket: Link,
    ) -> (watch::Receiver<Status>, watch::Sender<bool>, JoinSet<()>) {
        let (status_tx, status_rx) = watch::channel(Status::default());
        let (state_tx, state_rx) = watch::channel(false);

        let sender_task = Self::send_status(client_id, socket.clone(), state_rx);
        let receiver_task = Self::receive_status(client_id, socket, status_tx);

        let mut set = JoinSet::new();
        set.spawn(sender_task);
        set.spawn(receiver_task);

        (status_rx, state_tx, set)
    }

    async fn send_status(client_id: u8, socket: Link, mut control: watch::Receiver<bool>) {
        let mut timer = interval(STATUS_INTERVAL);

        loop {
            // Wait for either the control state to change or the timer to tick
            select! {
                _ = control.changed() => {}
                _ = timer.tick() => {}
            }

            // Send a status message to the device
            let request_control = *control.borrow();
            let request = Request::Status { request_control };

            if socket.send_request(request, client_id).await.is_err() {
                break;
            }
        }
    }

    async fn receive_status(client_id: u8, socket: Link, status: watch::Sender<Status>) {
        let mut buffer = vec![0u8; RX_BUFFER_SIZE];

        loop {
            let (size, _) = socket
                .socket
                .recv_from(&mut buffer)
                .await
                .expect("Error reading from socket");

            if let Some(response) = Response::parse_bytes(&buffer[..size]) {
                if response.client_id != client_id {
                    eprintln!("Invalid client ID: {}", response.client_id);
                    continue;
                }

                if let ResponsePayload::Status { in_control } = response.payload {
                    status.send_modify(|s| s.in_control = in_control);
                }
            }
        }
    }
}

impl Link {
    async fn connect(addr: IpAddr) -> Self {
        let addr = SocketAddr::new(addr, REMOTE_PORT);

        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, LOCAL_PORT))
            .await
            .unwrap()
            .into();

        Link { addr, socket }
    }

    async fn send_request(&self, request: Request, client_id: u8) -> io::Result<()> {
        let (tag, payload) = request.pack();
        let message = format!("{tag}@{client_id}:{payload}\0");

        self.socket.send_to(message.as_bytes(), self.addr).await?;

        Ok(())
    }

    async fn recv_response(&self) -> Result<Response> {
        let mut buffer = vec![0u8; RX_BUFFER_SIZE];
        let (size, _) = self.socket.recv_from(&mut buffer).await?;
        Response::parse_bytes(&buffer[..size]).ok_or(eyre!("Invalid response"))
    }

    async fn recv_handshake(&self) -> Result<u8> {
        loop {
            let response = self.recv_response().await?;

            if let ResponsePayload::Address = response.payload {
                return Ok(response.client_id);
            }
        }
    }
}

impl Request {
    fn pack(&self) -> (&'static str, String) {
        match self {
            Request::Module {
                enable_power,
                fan_speed,
            } => {
                let fan_speed = (*fan_speed).min(100).to_string();
                let enable_power = *enable_power as u8;

                let fan_power_str = iter::repeat_n(fan_speed, MODULE_FANS)
                    .collect::<Vec<_>>()
                    .join(",");

                let modules_str = (0..MODULE_COUNT)
                    .map(|i| format!("{};{};{};0;0;0", i + 1, fan_power_str, enable_power))
                    .collect::<Vec<_>>()
                    .join("|");

                ("MODULE", modules_str)
            }

            Request::InitiateConnection => ("REQUEST_CONNECTION", "no_message".to_string()),

            Request::Status { request_control } => {
                ("STATUS", format!("{};0", *request_control as u8))
            }
        }
    }
}

impl Response {
    fn parse_bytes(buffer: &[u8]) -> Option<Self> {
        let input = str::from_utf8(buffer).ok()?;

        let (header, payload) = input.split_once(':')?;
        let (tag, client_id_str) = header.split_once('@')?;
        let client_id = client_id_str.parse().ok()?;

        let payload = match tag {
            "ADDRESS" => ResponsePayload::Address,
            "MODULE" => ResponsePayload::Module,

            "STATUS" => {
                let in_control_str = payload.split(';').next()?;
                let in_control = in_control_str.parse::<u8>().ok()? == 1;
                ResponsePayload::Status { in_control }
            }

            _ => return None,
        };

        Some(Response { client_id, payload })
    }
}
