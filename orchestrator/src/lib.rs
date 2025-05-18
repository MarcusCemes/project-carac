#![allow(dead_code)]

use std::io;

use eyre::Result;
use tracing_subscriber::EnvFilter;

pub mod cli;
pub mod config;
pub mod data;
pub mod defs;
pub mod hardware;
pub mod learn;
pub mod misc;

const CRATE: &str = env!("CARGO_CRATE_NAME");
const NAME: &str = env!("CARGO_PKG_NAME");
const VERSION: &str = env!("CARGO_PKG_VERSION");

const TARGET_ARCH: &str = env!("CARGO_CFG_TARGET_ARCH");
const TARGET_ENV: &str = env!("CARGO_CFG_TARGET_ENV");
const TARGET_OS: &str = env!("CARGO_CFG_TARGET_OS");
const TARGET_VENDOR: &str = env!("CARGO_CFG_TARGET_VENDOR");

pub fn init() -> Result<()> {
    color_eyre::install()?;

    let env_filter = EnvFilter::builder()
        .with_default_directive("drone_lab=debug".parse()?)
        .from_env_lossy();

    tracing_subscriber::fmt()
        .with_env_filter(env_filter)
        .with_writer(io::stderr)
        .init();

    Ok(())
}

pub fn banner() {
    eprintln!("\n{NAME} {VERSION} ({TARGET_ARCH}-{TARGET_VENDOR}-{TARGET_OS}-{TARGET_ENV})");
    eprintln!("Designed by Marcus Cemes <marcus.cemes@epfl.ch>\n");
}
