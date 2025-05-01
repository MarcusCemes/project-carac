use eyre::Result;

use crate::config::Config;

pub async fn read_and_print(path: &str) -> Result<()> {
    let config = Config::load(path).await?;
    println!("{config:#?}");
    Ok(())
}
