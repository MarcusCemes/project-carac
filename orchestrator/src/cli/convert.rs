use std::io;

use eyre::{ContextCompat, Result};

use crate::recording::{Recording, StreamDefinition};

pub async fn segment(divisions: u32) -> Result<()> {
    let recording = Recording::decode(&mut io::stdin())?;

    let mut segmented_recording = recording
        .segment(divisions)
        .wrap_err("Failed to segment recording")?;

    let mut w = csv::Writer::from_writer(io::stdout());

    let header = segmented_recording
        .definitions()
        .iter()
        .copied()
        .flat_map(StreamDefinition::qualified_channel_names)
        .collect::<Vec<_>>();

    w.write_field("time")?;
    w.write_record(&header)?;

    let mut buf = Vec::new();

    while let Some(time) = segmented_recording.next(&mut buf) {
        w.write_field(time.to_string())?;
        w.write_record(buf.iter().map(f32::to_string))?;
    }

    Ok(())
}
