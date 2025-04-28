use std::net::IpAddr;

use clap::{Parser, Subcommand};
use eyre::Result;

mod config;
mod convert;
mod examples;
mod run;
mod server;
mod test;
mod view;

#[derive(Parser)]
#[command(version, about)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    Config {
        #[arg(short, long, default_value = "config.yaml")]
        config: String,
    },

    Convert {
        #[arg(short, long, default_value_t = 100)]
        divisions: u32,
    },

    Counter,

    PlotJugglerDemo,

    Run {
        #[arg(short, long, default_value = "config.yaml")]
        config: String,
    },

    Server {
        #[arg(short, long, default_value_t = 8080)]
        port: u16,
    },

    Test {
        #[arg(long)]
        robot_ip: IpAddr,
        #[arg(long)]
        robot_port: u16,
        #[arg(long)]
        windshape_ip: IpAddr,
    },

    View,
}

#[derive(Subcommand)]
enum ConfigCommand {
    Test {
        #[arg(short, long, default_value = "config.yaml")]
        config: String,
    },
}

pub async fn run() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Command::Config { config } => self::config::read_and_print(&config).await,
        Command::Convert { divisions } => self::convert::segment(divisions).await,
        Command::Counter => self::examples::counter().await,
        Command::PlotJugglerDemo => self::examples::plot_juggler().await,
        Command::Run { config } => self::run::launch(&config).await,
        Command::Server { port } => self::server::start(port).await,

        Command::Test {
            robot_ip,
            robot_port,
            windshape_ip,
        } => self::test::run(robot_ip, robot_port, windshape_ip).await,

        Command::View => self::view::display_data(),
    }
}
