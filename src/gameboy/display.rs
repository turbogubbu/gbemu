extern crate sdl2;

use crate::gameboy::ppu::Tile;

use crate::gameboy::ppu::Ppu;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::Color;
use sdl2::rect::{Point, Rect};
use sdl2::render::WindowCanvas;
use sdl2::VideoSubsystem;

//use super::Display::display;

pub const DIMENSIONS_X: usize = 160;
pub const DIMENSIONS_Y: usize = 160;
pub const DIMENSIONS: usize = DIMENSIONS_X * DIMENSIONS_Y;

const WIDTH: u32 = 1600;
const HEIGHT: u32 = 1000;

pub struct Display {
    pub pixel_buffer: [u8; DIMENSIONS],
    lcd_control: u8,
    lcd_status: u8,
    sdl_canvas: WindowCanvas,
}

impl Display {
    pub fn new() -> Display {
        Display {
            pixel_buffer: [0; DIMENSIONS],
            lcd_control: 0,
            lcd_status: 0,
            sdl_canvas: sdl2::init()
                .unwrap()
                .video()
                .unwrap()
                .window("gbemu", WIDTH, HEIGHT)
                .position_centered()
                .build()
                .unwrap()
                .into_canvas()
                .build()
                .unwrap(),
        }
    }

    pub fn init(&mut self, mem: &[u8; 0x10000]) {
        self.sdl_canvas.set_draw_color(Color::RGB(0, 0, 0));
        self.sdl_canvas.clear();
        self.sdl_canvas.set_draw_color(Color::RGB(255, 255, 255));
        // Lines for lcd
        self.sdl_canvas
            .draw_line(Point::new(0, 0), Point::new(DIMENSIONS_X as i32 * 2, 0))
            .unwrap();
        self.sdl_canvas
            .draw_line(
                Point::new(DIMENSIONS_X as i32 * 2, 0),
                Point::new(DIMENSIONS_X as i32 * 2, DIMENSIONS_Y as i32 * 2),
            )
            .unwrap();
        self.sdl_canvas
            .draw_line(
                Point::new(DIMENSIONS_X as i32 * 2, DIMENSIONS_Y as i32 * 2),
                Point::new(0, DIMENSIONS_Y as i32 * 2),
            )
            .unwrap();
        self.sdl_canvas
            .draw_line(Point::new(0, DIMENSIONS_Y as i32 * 2), Point::new(0, 0))
            .unwrap();
        // Line for tiles
        self.sdl_canvas
            .draw_line(
                Point::new(WIDTH as i32 - 8 * 8 * 2 - 1, 0),
                Point::new(WIDTH as i32 - 8 * 8 * 2 - 1, HEIGHT as i32),
            )
            .unwrap();

        for i in 0..256 {
            let tile = Tile::new(core::array::from_fn(|n| mem[0x8000 + i * 16 + n]));
            let pixels = tile.get_pixel();

            println!("Tile: {:?}", tile);

            let base_x = WIDTH as i32 - 8 * 8 * 2 + (i as i32 % 8) * 8;
            let base_y = (i as i32 / 8) * 8 * 2;

            for n in 0..64 {
                let pixel = pixels[n];
                self.sdl_canvas.set_draw_color(Color::RGB(
                    63 + pixel * 64,
                    63 + pixel * 64,
                    63 + pixel * 64,
                ));

                let x = base_x + (n as i32 & 8) * 2;
                let y = base_y + (n as i32 / 8) * 2;
                let rect: Rect = Rect::new(x, y, 2, 2);
                self.sdl_canvas.draw_rect(rect).unwrap();
            }
        }
        self.sdl_canvas.present();
    }

    pub fn update_frame(&mut self) {
        self.sdl_canvas.present();
    }
}
