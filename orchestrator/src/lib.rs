use std::{io, time::Duration};

use eyre::Result;
use tokio::runtime;
use tracing_subscriber::EnvFilter;

mod audio;
pub mod cli;
pub mod config;
pub mod data;
pub mod defs;
pub mod hardware;
pub mod misc;

pub use cli::{cli, kit};

const HARDWARE_TIMEOUT: Duration = Duration::from_secs(1);
const WORKER_THREADS: usize = 4;

const NAME: &str = env!("CARGO_PKG_NAME");
const VERSION: &str = env!("CARGO_PKG_VERSION");

const TARGET_ARCH: &str = env!("CARGO_CFG_TARGET_ARCH");
const TARGET_ENV: &str = env!("CARGO_CFG_TARGET_ENV");
const TARGET_OS: &str = env!("CARGO_CFG_TARGET_OS");
const TARGET_VENDOR: &str = env!("CARGO_CFG_TARGET_VENDOR");

pub fn init() -> Result<()> {
    color_eyre::install()?;

    let env_filter = EnvFilter::builder()
        .with_default_directive(format!("{NAME}=info").parse()?)
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

pub fn create_runtime() -> runtime::Runtime {
    runtime::Builder::new_multi_thread()
        .worker_threads(WORKER_THREADS)
        .enable_all()
        .build()
        .unwrap()
}
