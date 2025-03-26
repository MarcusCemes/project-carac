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
}

#[tokio::main(flavor = "current_thread")]
async fn main() {
    let cli = Cli::parse();

    match cli.command {
        Command::Test => run_test().await,
    }
}

async fn run_test() {
    orchestrator::test_robot_arm().await;
    orchestrator::test_wind_shape().await;
}
