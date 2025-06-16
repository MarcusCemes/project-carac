use std::env::var;

const FORWARDED_ENV: [&str; 4] = [
    "CARGO_CFG_TARGET_ARCH",
    "CARGO_CFG_TARGET_ENV",
    "CARGO_CFG_TARGET_OS",
    "CARGO_CFG_TARGET_VENDOR",
];

fn main() {
    for env in FORWARDED_ENV {
        if let Ok(value) = var(env) {
            println!("cargo:rustc-env={}={}", env, value);
        }
    }
}
