use std::{io, mem, net::Ipv4Addr, sync::Arc};

use bincode::{
    config::standard, decode_from_slice, encode_into_slice, error::DecodeError, Decode, Encode,
};
use reqwest::get;
use tokio::{net::UdpSocket, task::JoinHandle};

use crate::recorder::RecordHandle;

const DIMENSIONS: usize = 6;

const DEFAULT_IP: &str = "192.168.1.1";
const PORT: u16 = 49152;

const COMMAND_HEADER: u16 = 0x1234;
const DEFAULT_BUFFER_SIZE: usize = 8192; // 8KB
pub struct LoadCell {
    socket: Arc<UdpSocket>,
    task: JoinHandle<Result<(), TaskError>>,
}

#[derive(Debug)]
pub enum Error {
    ConfigFetch(reqwest::Error),
    Socket(io::Error),
    MalformedConfig(String),
}

#[derive(Debug)]
pub enum TaskError {
    MalformedPacket,
    SocketError(io::Error),
}

#[derive(Encode)]
struct Command {
    header: u16,
    command: u16,
    sample_count: u32,
}

impl LoadCell {
    pub async fn create(ip: Option<&str>, record: RecordHandle<DIMENSIONS>) -> Result<Self, Error> {
        let counts = Self::fetch_config(ip.unwrap_or(DEFAULT_IP)).await?;

        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).await?;
        let ip = ip.unwrap_or(DEFAULT_IP);
        socket.connect((ip, PORT)).await?;

        let socket = Arc::new(socket);
        let task = tokio::spawn(Self::receiver_task(socket.clone(), record, counts));

        Ok(LoadCell { socket, task })
    }

    pub async fn set_streaming(&self, stream: bool, samples: Option<u32>) -> io::Result<()> {
        const BUFFER_SIZE: usize = mem::size_of::<Command>();

        let command = Command {
            header: COMMAND_HEADER,
            command: if stream { 0x2 } else { 0x0 },
            sample_count: samples.unwrap_or(0),
        };

        let mut buf = [0; BUFFER_SIZE];
        encode_into_slice(command, &mut buf, standard()).unwrap();

        self.socket.send(&buf).await?;

        Ok(())
    }

    async fn fetch_config(ip: &str) -> Result<(u32, u32), Error> {
        let xml = get(&format!("http://{ip}/netftcalapi.xml"))
            .await?
            .text()
            .await?;

        let force_unit = find_xml_tag(&xml, "scalfu")?;
        let torque_unit = find_xml_tag(&xml, "scaltu")?;

        if force_unit != "N" {
            tracing::warn!("Force unit is not N: {force_unit}");
        }

        if torque_unit != "Nm" {
            tracing::warn!("Torque unit is not Nm: {torque_unit}");
        }

        Ok((
            find_xml_tag_u32(&xml, "calcpf")?,
            find_xml_tag_u32(&xml, "calcpt")?,
        ))
    }

    async fn receiver_task(
        socket: Arc<UdpSocket>,
        record: RecordHandle<DIMENSIONS>,
        counts: (u32, u32),
    ) -> Result<(), TaskError> {
        let (force_counts, torque_counts) = (counts.0 as f32, counts.1 as f32);

        let mut buf = Vec::with_capacity(DEFAULT_BUFFER_SIZE);

        loop {
            socket.recv(&mut buf).await?;

            let msg = parse_message(&buf)?;

            let payload = [
                msg.fx as f32 / force_counts,
                msg.fy as f32 / force_counts,
                msg.fz as f32 / force_counts,
                msg.tx as f32 / torque_counts,
                msg.ty as f32 / torque_counts,
                msg.tz as f32 / torque_counts,
            ];

            record.append(payload).await;

            buf.clear();
        }
    }
}

impl Drop for LoadCell {
    fn drop(&mut self) {
        tracing::warn!("Stopping task");
        self.task.abort();
    }
}

/* == Message == */

#[derive(Decode)]
struct Message {
    _rdt_sequence: u32,
    _ft_sequence: u32,
    _status: u32,

    fx: i32,
    fy: i32,
    fz: i32,
    tx: i32,
    ty: i32,
    tz: i32,
}

fn parse_message(buf: &[u8]) -> Result<Message, DecodeError> {
    Ok(decode_from_slice(buf, standard())?.0)
}

/* == Utils == */

fn find_xml_tag_u32(xml: &str, tag: &str) -> Result<u32, Error> {
    xml.lines()
        .find_map(|line| {
            line.contains(tag).then(|| {
                let line = line.trim();
                let start = line.find('<').unwrap() + 1;
                let end = line.rfind('>').unwrap();
                &line[start..end]
            })
        })
        .ok_or_else(|| Error::MalformedConfig(format!("Tag {tag} not found in XML")))?
        .parse()
        .map_err(|_| Error::MalformedConfig(format!("Tag {tag} is not a valid u32")))
}

fn find_xml_tag<'a>(xml: &'a str, tag: &str) -> Result<&'a str, Error> {
    xml.lines()
        .find_map(|line| {
            line.contains(tag).then(|| {
                let line = line.trim();
                let start = line.find('<').unwrap() + 1;
                let end = line.rfind('>').unwrap();
                &line[start..end]
            })
        })
        .ok_or_else(|| Error::MalformedConfig(format!("Tag {tag} not found in XML")))
}

impl From<io::Error> for Error {
    fn from(err: io::Error) -> Self {
        Error::Socket(err)
    }
}

impl From<reqwest::Error> for Error {
    fn from(err: reqwest::Error) -> Self {
        Error::ConfigFetch(err)
    }
}

impl From<DecodeError> for TaskError {
    fn from(_: DecodeError) -> Self {
        TaskError::MalformedPacket
    }
}

impl From<io::Error> for TaskError {
    fn from(err: io::Error) -> Self {
        TaskError::SocketError(err)
    }
}
