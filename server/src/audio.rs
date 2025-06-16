use std::{
    io::Cursor,
    sync::mpsc,
    thread::{JoinHandle, spawn},
};

use eyre::Result;
use rodio::{Decoder, OutputStream, Sink};

use crate::data::orchestrator::Event;

pub struct AudioPlayer {
    queue: mpsc::Sender<Event>,
    _thread: JoinHandle<Result<()>>,
}

impl AudioPlayer {
    pub fn try_new() -> Result<Self> {
        let (queue, rx) = mpsc::channel();
        let _thread = spawn(move || audio_thread(rx));

        Ok(AudioPlayer { queue, _thread })
    }

    pub fn queue(&self, event: Event) -> Result<()> {
        self.queue.send(event)?;
        Ok(())
    }
}

fn audio_thread(queue: mpsc::Receiver<Event>) -> Result<()> {
    let (_stream, handle) = OutputStream::try_default()?;
    let sink = Sink::try_new(&handle)?;

    while let Ok(event) = queue.recv() {
        let cursor = Cursor::new(audio_file(event));
        let sound = Decoder::new_aac(cursor)?;

        sink.append(sound);
    }

    Ok(())
}

fn audio_file(event: Event) -> &'static [u8] {
    match event {
        Event::Buzzer => include_bytes!("./assets/buzzer.aac"),
        Event::Complete => include_bytes!("./assets/complete.aac"),
        Event::End => include_bytes!("./assets/end.aac"),
        Event::Error => include_bytes!("./assets/error.aac"),
        Event::Notification => include_bytes!("./assets/notification.aac"),
    }
}
