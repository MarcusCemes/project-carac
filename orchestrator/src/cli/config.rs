use eyre::Result;
use tokio::fs;

use crate::config::Config;

pub async fn read_and_print(path: &str) -> Result<()> {
    let config: Config = {
        let data = fs::read(path).await?;
        Config::load(&data)?
    };

    println!("{config:#?}");
    Ok(())
}
