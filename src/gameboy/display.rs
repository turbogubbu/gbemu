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
            .draw_line(
                Point::new(0, 0),
                Point::new(DIMENSIONS_X as i32 * PIXEL as i32, 0),
            )
            .unwrap();
        self.sdl_canvas
            .draw_line(
                Point::new(DIMENSIONS_X as i32 * PIXEL as i32, 0),
                Point::new(
                    DIMENSIONS_X as i32 * PIXEL as i32,
                    DIMENSIONS_Y as i32 * PIXEL as i32,
                ),
            )
            .unwrap();
        self.sdl_canvas
            .draw_line(
                Point::new(
                    DIMENSIONS_X as i32 * PIXEL as i32,
                    DIMENSIONS_Y as i32 * PIXEL as i32,
                ),
                Point::new(0, DIMENSIONS_Y as i32 * PIXEL as i32),
            )
            .unwrap();
        self.sdl_canvas
            .draw_line(
                Point::new(0, DIMENSIONS_Y as i32 * PIXEL as i32),
                Point::new(0, 0),
            )
            .unwrap();
        // Line for tiles
        self.sdl_canvas
            .draw_line(
                Point::new(WIDTH as i32 - 8 * 8 * 2 - 1, 0),
                Point::new(WIDTH as i32 - 8 * 8 * 2 - 1, HEIGHT as i32),
            )
            .unwrap();

        self.sdl_canvas.present();
    }

    fn draw_lcd_pixel(&mut self, x: i32, y: i32, color: u8) {
        assert!(color < 4, "Color val needs to be smaller than 4");
        self.sdl_canvas.set_draw_color(Color::RGB(
            63 + color * 64,
            63 + color * 64,
            63 + color * 64,
        ));
        let rect: Rect = Rect::new(x, y, PIXEL, PIXEL);
        self.sdl_canvas.draw_rect(rect).unwrap();
    }

    pub fn draw_single_tile(&mut self, mem: &[u8; 0x10000], index: u8, pos_x: i32, pos_y: i32) {
        let pixels = Tile::new(core::array::from_fn(|n| mem[0x8000 + index as usize * 16 + n])).get_pixels();

        for n in 0..64 {
            let x = pos_x + (n as i32 % 8) * PIXEL as i32;
            let y = pos_y + (n as i32 / 8) * PIXEL as i32;
            self.draw_lcd_pixel(x, y, pixels[n]);
        }
    }

    pub fn draw_vram_tiles(&mut self, mem: &[u8; 0x10000]) {
        for i in 0..256 {
            let base_x = WIDTH as i32 - 8 * 8 * PIXEL as i32 + (i as i32 % 8) * 8 * PIXEL as i32;
            let base_y = (i as i32 / 8) * 8 * PIXEL as i32;

            self.draw_single_tile(mem, i as u8, base_x, base_y);
        }

        // CE ED -> F0 00 F0 00  FC 00 FC 00 FC 00 FC 00 F3 00 F3 00

        /*let pixels = Tile::new([0x3C, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x5E, 0x7E, 0x0A, 0x7C, 0x56, 0x38, 0x7C]).get_pixels();

        let pos_x = 600;
        let pos_y = 450;

        for n in 0..64 {
            let x = pos_x + (n as i32 % 8) * PIXEL as i32;
            let y = pos_y + (n as i32 / 8) * PIXEL as i32;
            self.draw_lcd_pixel(x, y, pixels[n]);
        }*/
        self.sdl_canvas.present();
    }

    pub fn update_frame(&mut self) {
        for i in 0..DIMENSIONS {
            self.draw_lcd_pixel(
                ((i % DIMENSIONS_X) * 2) as i32,
                ((i / DIMENSIONS_X) * 2) as i32,
                self.pixel_buffer[i],
            );
        }
        self.sdl_canvas.present();
    }
}
