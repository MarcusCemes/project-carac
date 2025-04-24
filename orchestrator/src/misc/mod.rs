pub mod plot_juggler;
pub mod serde;

pub fn standard_config() -> impl bincode::config::Config {
    bincode::config::standard()
        .with_little_endian()
        .with_fixed_int_encoding()
}

pub fn compact_config() -> impl bincode::config::Config {
    bincode::config::standard()
        .with_little_endian()
        .with_variable_int_encoding()
}
