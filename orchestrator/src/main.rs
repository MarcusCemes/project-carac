use std::io;

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() -> io::Result<()> {
    init();
    orchestrator::launch().await
}

fn init() {
    tracing_subscriber::fmt()
        .with_env_filter("orchestrator=debug")
        .with_writer(io::stderr)
        .init();
}
