use std::io;

use eyre::Result;

use orchestrator::cli::run;
use tracing_subscriber::EnvFilter;

fn main() -> Result<()> {
    init()?;
    run()
}

fn init() -> Result<()> {
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
