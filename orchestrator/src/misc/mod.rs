use std::{fmt::Display, time::Duration};

use bincode::config::{Config, standard};
use color_eyre::owo_colors::OwoColorize;

pub mod buf;
pub mod data;
pub mod plot_juggler;
pub mod serde;

pub fn standard_config() -> impl Config {
    standard().with_little_endian().with_fixed_int_encoding()
}

pub fn compact_config() -> impl Config {
    standard().with_little_endian().with_variable_int_encoding()
}

pub fn network_config() -> impl Config {
    standard().with_big_endian().with_fixed_int_encoding()
}

pub fn type_name<T>() -> &'static str {
    let name = std::any::type_name::<T>();
    name.split("::").last().unwrap_or(name)
}

// Small helper function to sleep for a given number of f32 seconds.
pub async fn sleep(seconds: f32) {
    tokio::time::sleep(Duration::from_secs_f32(seconds)).await;
}

pub struct ColourDot(pub bool);

impl<T> From<&Option<T>> for ColourDot {
    fn from(option: &Option<T>) -> Self {
        ColourDot(option.is_some())
    }
}

impl Display for ColourDot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let char = '●';

        if self.0 {
            write!(f, "{}", char.bright_green())
        } else {
            write!(f, "{}", char.bright_red())
        }
    }
}
