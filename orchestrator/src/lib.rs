#![allow(async_fn_in_trait, dead_code)]

use std::io;

use eyre::Result;
use tracing_subscriber::EnvFilter;

pub mod cli;
pub mod config;
pub mod data;
pub mod defs;
pub mod hardware;
pub mod misc;
pub mod server;

pub fn init() -> Result<()> {
    color_eyre::install()?;

    let env_filter = EnvFilter::builder()
        .with_default_directive("orchestrator=debug".parse()?)
        .from_env_lossy();

    tracing_subscriber::fmt()
        .with_env_filter(env_filter)
        .with_writer(io::stderr)
        .init();

    Ok(())
}
