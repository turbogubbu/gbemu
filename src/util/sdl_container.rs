use sdl2::Sdl;

use crate::util::event::EventHandler;
use crate::util::video::Video;

pub struct SDLContainer {
    context: Sdl,
    pub video: Video,
    pub event: EventHandler,
}

impl SDLContainer {
    pub fn new() -> SDLContainer {
        let context = sdl2::init().unwrap();
        SDLContainer {
            video: Video::new(&context),
            event: EventHandler::new(&context),
            context,
        }
    }
}
