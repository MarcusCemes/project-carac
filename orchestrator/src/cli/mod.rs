use std::{io, net::IpAddr};

use clap::{Parser, Subcommand};

mod config;
mod convert;
mod examples;
mod run;
mod test;

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

#[derive(Subcommand)]
enum ConfigCommand {
    Test {
        #[arg(short, long, default_value = "config.yaml")]
        config: String,
    },
}

pub async fn parse() -> io::Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Command::Config { config } => self::config::read_and_print(&config).await,
        Command::Convert { divisions } => self::convert::segment(divisions).await,
        Command::Counter => self::examples::counter().await,
        Command::PlotJugglerDemo => self::examples::plot_juggler().await,
        Command::Run => self::run::launch().await,

        Command::Test {
            robot_ip,
            robot_port,
            windshape_ip,
        } => self::test::run(robot_ip, robot_port, windshape_ip).await,
    }
}
