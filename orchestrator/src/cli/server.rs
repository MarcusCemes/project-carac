use std::{net::Ipv4Addr, time::Duration};

use axum::serve;
use eyre::Result;
use tokio::net::TcpListener;

use crate::{
    config::Config, hardware::HardwareContext, orchestrator::Orchestrator, server::create_router,
};

pub async fn start(config_path: &str, port: u16) -> Result<()> {
    let config = Config::load(config_path).await?;
    tracing::info!("{}", config.hardware);

    let context = HardwareContext::builder()
        .with_timeout(Duration::from_secs(1))
        .build(&config.hardware)
        .await?;

    let orchestrator = Orchestrator::new(context);
    let app = create_router(orchestrator);

    let socket = TcpListener::bind((Ipv4Addr::UNSPECIFIED, port)).await?;

    tracing::info!("Listening on http://0.0.0.0:{port}");
    serve(socket, app).await?;

    Ok(())
}
