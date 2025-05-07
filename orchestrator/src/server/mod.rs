use std::sync::Arc;

use axum::{
    extract,
    routing::{get, post},
    Json, Router,
};
use serde::{Deserialize, Serialize};
use tokio::sync::Mutex;

use crate::data::orchestrator::{Instruction, Orchestrator};

struct AppState {
    orchestrator: Mutex<Orchestrator>,
}

pub fn create_router(orchestrator: Orchestrator) -> Router {
    Router::new()
        .route("/complete", get(complete))
        .route("/execute", post(execute))
        .route("/reset", post(reset))
        .route("/status", get(status))
        .with_state(Arc::new(AppState {
            orchestrator: Mutex::new(orchestrator),
        }))
}

/* === Routes === */

/* == Execute == */

#[derive(Deserialize)]
struct ExecutePayload {
    instructions: Vec<Instruction>,
}

#[derive(Serialize)]
enum ExecuteResponse {
    Success,
    Error { message: String },
}

async fn execute(
    extract::State(state): extract::State<Arc<AppState>>,
    extract::Json(payload): extract::Json<ExecutePayload>,
) -> Json<ExecuteResponse> {
    let mut orchestrator = state.orchestrator.lock().await;

    for instruction in payload.instructions {
        if let Err(error) = orchestrator.execute(instruction).await {
            return Json(ExecuteResponse::Error {
                message: error.to_string(),
            });
        }
    }

    Json(ExecuteResponse::Success)
}

/* == Complete ==  */

async fn complete(extract::State(_state): extract::State<Arc<AppState>>) -> Json<()> {
    todo!()

    // let orchestrator = state.orchestrator.lock().await;
    // let recording = orchestrator.complete().await;

    // Json(recording)
}

/* == Reset == */

async fn reset(extract::State(_state): extract::State<Arc<AppState>>) {
    todo!()

    // let mut orchestrator = state.orchestrator.lock().await;
    // orchestrator.reset().await;
}

/* == Status == */

async fn status() -> &'static str {
    "Healthy"
}
