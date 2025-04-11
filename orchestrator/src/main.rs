use std::io;

use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(version, about)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    Test,
    Run,
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> io::Result<()> {
    tracing_subscriber::fmt().init();

    let cli = Cli::parse();

    match cli.command {
        Command::Test => run_test().await,
        Command::Run => orchestrator::run().await?,
    }

    Ok(())
}

async fn run_test() {
    orchestrator::test_robot_arm().await;
    orchestrator::test_wind_shape().await;
}
