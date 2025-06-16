use serde::{Deserialize, Serialize};

use crate::defs::Point;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub enum Command {
    SetBias,
    SetTransform(Point),
}
