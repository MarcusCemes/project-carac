use std::io;

use eyre::Result;

use orchestrator::cli::run;

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() -> Result<()> {
    init()?;
    run().await
}

fn init() -> Result<()> {
    color_eyre::install()?;

    tracing_subscriber::fmt()
        .with_env_filter("orchestrator=debug")
        .with_writer(io::stderr)
        .init();

    Ok(())
}
