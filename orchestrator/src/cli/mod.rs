use clap::{Parser, Subcommand};
use convert::ConvertOpts;
use eyre::Result;

mod convert;
mod run;
mod server;
mod view;

#[derive(Parser)]
#[command(version, about)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
pub enum Command {
    Convert(ConvertOpts),
    View,

    Run {
        #[arg(short, long, default_value = "config.yaml")]
        config: String,
    },

    Server {
        #[arg(short, long, default_value = "config.yaml")]
        config: String,
        #[arg(short, long, default_value_t = 8080)]
        port: u16,
    },
}

#[derive(Subcommand)]
enum ConfigCommand {
    Test {
        #[arg(short, long, default_value = "config.yaml")]
        config: String,
    },
}

pub fn run() -> Result<()> {
    execute_command(Cli::parse().command)
}

#[tokio::main]
pub async fn execute_command(command: Command) -> Result<()> {
    match command {
        Command::Convert(opts) => self::convert::segment(opts).await,
        Command::Run { config } => self::run::launch(&config).await,
        Command::Server { config, port } => self::server::start(&config, port).await,
        Command::View => self::view::display_data().await,
    }
}
