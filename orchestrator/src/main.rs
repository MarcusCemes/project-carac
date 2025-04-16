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
    Convert {
        #[arg(short, long, default_value_t = 100)]
        divisions: u32,
    },
    Counter,
    PlotJugglerDemo,
    Run,
    Test,
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> io::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter("orchestrator=debug")
        .with_writer(io::stderr)
        .init();

    let cli = Cli::parse();

    match cli.command {
        Command::Convert { divisions } => orchestrator::convert(divisions).await,
        Command::Counter => orchestrator::counter().await,
        Command::PlotJugglerDemo => orchestrator::plot_juggler_demo().await,
        Command::Run => orchestrator::run().await,
        Command::Test => run_test().await,
    }
}

async fn run_test() -> io::Result<()> {
    orchestrator::test_robot_arm().await;
    orchestrator::test_wind_shape().await;
    Ok(())
}
