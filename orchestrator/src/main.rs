use std::{io, net::IpAddr};

use clap::{Parser, Subcommand};
use tracing_subscriber::fmt::format::FmtSpan;

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
    Test {
        #[arg(long)]
        robot_ip: IpAddr,
        #[arg(long)]
        robot_port: u16,
        #[arg(long)]
        windshape_ip: IpAddr,
    },
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> io::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter("orchestrator=debug")
        .with_writer(io::stderr)
        .with_span_events(FmtSpan::ACTIVE)
        .init();

    let cli = Cli::parse();

    match cli.command {
        Command::Convert { divisions } => orchestrator::convert(divisions).await,
        Command::Counter => orchestrator::counter().await,
        Command::PlotJugglerDemo => orchestrator::plot_juggler_demo().await,
        Command::Run => orchestrator::run().await,

        Command::Test {
            robot_ip,
            robot_port,
            windshape_ip,
        } => run_test(robot_ip, robot_port, windshape_ip).await,
    }
}

async fn run_test(robot_ip: IpAddr, robot_port: u16, windshape_ip: IpAddr) -> io::Result<()> {
    orchestrator::test_robot_arm(robot_ip, robot_port).await;
    orchestrator::test_wind_shape(windshape_ip).await;
    Ok(())
}
