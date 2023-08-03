extern crate sdl2;

use crate::gameboy::ppu::Tile;

use crate::gameboy::ppu::Ppu;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::Color;
use sdl2::rect::{Point, Rect};
use sdl2::render::{TextureCreator, WindowCanvas};
use sdl2::VideoSubsystem;

//use super::Display::display;

pub const DIMENSIONS_X: usize = 160;
pub const DIMENSIONS_Y: usize = 144;
pub const DIMENSIONS: usize = DIMENSIONS_X * DIMENSIONS_Y;

const WIDTH: u32 = 1600;
const HEIGHT: u32 = 1000;

// LCD pixel to real pixel ratio
const PIXEL: u32 = 2;

pub struct Display {
    pub pixel_buffer: [u8; DIMENSIONS],
    lcd_control: u8,
    lcd_status: u8,
}

impl Display {
    pub fn new() -> Display {
        Display {
            pixel_buffer: [0; DIMENSIONS],
            lcd_control: 0,
            lcd_status: 0,
        }
    }

    pub fn init(&mut self, mem: &[u8; 0x10000]) {}
}
