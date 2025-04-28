use std::net::Ipv4Addr;

use axum::serve;
use eyre::Result;
use tokio::net::TcpListener;

use crate::server::create_router;

pub async fn start(port: u16) -> Result<()> {
    let app = create_router();
    let socket = TcpListener::bind((Ipv4Addr::UNSPECIFIED, port)).await?;

    serve(socket, app).await?;

    Ok(())
}
