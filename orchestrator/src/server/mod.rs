use std::sync::Arc;

use axum::{routing::get, Router};

use crate::orchestrator::Orchestrator;

#[derive(Default)]
struct AppState {
    orchestrator: Orchestrator,
}

pub fn create_router() -> Router {
    Router::new()
        .route("/status", get(status))
        .with_state(Arc::new(AppState::default()))
}

/* == Routes == */

async fn status() -> &'static str {
    "Healthy"
}
