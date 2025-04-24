use std::io;

use tokio::fs;

use crate::config::Config;

pub async fn read_and_print(path: &str) -> io::Result<()> {
    let data = fs::read(path).await?;

    let config: Config = serde_yaml::from_slice(&data).map_err(|e| {
        io::Error::new(
            io::ErrorKind::InvalidData,
            format!("Failed to parse config: {e}"),
        )
    })?;

    println!("{config:#?}");
    Ok(())
}
