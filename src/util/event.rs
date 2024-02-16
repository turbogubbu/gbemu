use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::EventPump;
use sdl2::Sdl;

use std::process::exit;

use crate::gameboy::joypad_input::JoypadInput;

pub struct EventHandler {
    // event_subsystem: EventSubsystem,
    event_pump: EventPump,
}

impl EventHandler {
    pub fn new(sdl: &Sdl) -> EventHandler {
        let subsys = sdl.event_pump().unwrap();

        EventHandler { event_pump: subsys }
    }

    pub fn check_events(&mut self, joypad: &mut JoypadInput) {
        for event in self.event_pump.poll_iter() {
            match event {
                Event::Quit { timestamp: _ } => {
                    println!("Terminate event received!\n");
                    exit(0);
                }
                Event::KeyDown {
                    timestamp: _,
                    window_id: _,
                    keycode,
                    scancode: _,
                    keymod: _,
                    repeat,
                } => {
                    if keycode.is_none() || repeat {
                        continue;
                    }
                    match keycode.unwrap() {
                        Keycode::K => {
                            joypad.set_a();
                        }
                        Keycode::J => {
                            joypad.set_b();
                        }
                        Keycode::W => {
                            joypad.set_up();
                        }
                        Keycode::A => {
                            joypad.set_left();
                        }
                        Keycode::S => {
                            joypad.set_down();
                        }
                        Keycode::D => {
                            joypad.set_right();
                        }
                        Keycode::Return => {
                            joypad.set_start();
                        }
                        Keycode::Escape => {
                            joypad.set_select();
                        }
                        _ => {}
                    }
                }
                Event::KeyUp {
                    timestamp: _,
                    window_id: _,
                    keycode,
                    scancode: _,
                    keymod: _,
                    repeat: _,
                } => {
                    if keycode.is_none() {
                        continue;
                    }

                    match keycode.unwrap() {
                        Keycode::K => {
                            joypad.reset_a();
                        }
                        Keycode::J => {
                            joypad.reset_b();
                        }
                        Keycode::W => {
                            joypad.reset_up();
                        }
                        Keycode::A => {
                            joypad.reset_left();
                        }
                        Keycode::S => {
                            joypad.reset_down();
                        }
                        Keycode::D => {
                            joypad.reset_right();
                        }
                        Keycode::Return => {
                            joypad.reset_start();
                        }
                        Keycode::Escape => {
                            joypad.reset_select();
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }
    }
}
