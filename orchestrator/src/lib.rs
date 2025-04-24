#![allow(dead_code)]

use std::io;

mod cli;
mod config;
mod data;
mod defs;
mod hardware;
mod misc;
mod recording;
mod train;

pub async fn launch() -> io::Result<()> {
    cli::parse().await
}
