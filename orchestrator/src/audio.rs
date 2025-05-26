use std::{
    io::Cursor,
    sync::mpsc,
    thread::{JoinHandle, spawn},
};

use eyre::Result;
use rodio::{Decoder, OutputStream, Sink};

pub struct AudioPlayer {
    queue: mpsc::Sender<AudioFile>,
    thread: JoinHandle<Result<()>>,
}

#[derive(Copy, Clone, Debug)]
pub enum AudioFile {
    Up,
    Down,
    Double,
}

impl AudioPlayer {
    pub fn try_new() -> Result<Self> {
        let (queue, rx) = mpsc::channel();
        let thread = spawn(move || audio_thread(rx));

        Ok(AudioPlayer { queue, thread })
    }

    pub fn queue(&self, audio_file: AudioFile) -> Result<()> {
        self.queue.send(audio_file)?;
        Ok(())
    }
}

fn audio_thread(queue: mpsc::Receiver<AudioFile>) -> Result<()> {
    let (_stream, handle) = OutputStream::try_default()?;
    let sink = Sink::try_new(&handle)?;

    while let Ok(audio_file) = queue.recv() {
        let cursor = Cursor::new(audio_file.get_bytes());
        let sound = Decoder::new_wav(cursor)?;

        sink.append(sound);
    }

    Ok(())
}

impl AudioFile {
    fn get_bytes(self) -> &'static [u8] {
        match self {
            AudioFile::Up => include_bytes!("./assets/beep-up.wav"),
            AudioFile::Down => include_bytes!("./assets/beep-down.wav"),
            AudioFile::Double => include_bytes!("./assets/beep-double.wav"),
        }
    }
}
