extern crate sdl2;

use crate::gameboy::display::{DIMENSIONS, DIMENSIONS_X, DIMENSIONS_Y};
use crate::gameboy::ppu::Tile;

use std::process::exit;

use log::debug;

use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::EventPump;

use sdl2::pixels::Color;

use sdl2::rect::{Point, Rect};
use sdl2::render::WindowCanvas;
use sdl2::ttf::Sdl2TtfContext;

// LCD pixel to real pixel ratio
const PIXEL: u32 = 2;

const WIDTH: u32 = 1600;
const HEIGHT: u32 = 1000;

pub struct Video {
    canvas: WindowCanvas,
    event_pump: EventPump,
    ttf_context: Sdl2TtfContext,
}

impl Video {
    pub fn new() -> Video {
        let sdl_context = sdl2::init().unwrap();
        Video {
            canvas: sdl_context
                .video()
                .unwrap()
                .window("gbemu", WIDTH, HEIGHT)
                .position_centered()
                .build()
                .unwrap()
                .into_canvas()
                .build()
                .unwrap(),

            event_pump: sdl_context.event_pump().unwrap(),

            ttf_context: sdl2::ttf::init().map_err(|e| e.to_string()).unwrap(),
        }
    }

    pub fn init(&mut self) {
        self.canvas.set_draw_color(Color::RGB(0, 0, 0));
        self.canvas.clear();
        self.canvas.set_draw_color(Color::RGB(255, 255, 255));
        // Lines for lcd
        self.canvas
            .draw_line(
                Point::new(0, 0),
                Point::new(DIMENSIONS_X as i32 * PIXEL as i32, 0),
            )
            .unwrap();
        self.canvas
            .draw_line(
                Point::new(DIMENSIONS_X as i32 * PIXEL as i32, 0),
                Point::new(
                    DIMENSIONS_X as i32 * PIXEL as i32,
                    DIMENSIONS_Y as i32 * PIXEL as i32,
                ),
            )
            .unwrap();
        self.canvas
            .draw_line(
                Point::new(
                    DIMENSIONS_X as i32 * PIXEL as i32,
                    DIMENSIONS_Y as i32 * PIXEL as i32,
                ),
                Point::new(0, DIMENSIONS_Y as i32 * PIXEL as i32),
            )
            .unwrap();
        self.canvas
            .draw_line(
                Point::new(0, DIMENSIONS_Y as i32 * PIXEL as i32),
                Point::new(0, 0),
            )
            .unwrap();
        // Line for tiles
        self.canvas
            .draw_line(
                Point::new(WIDTH as i32 - 8 * 8 * 2 - 1, 0),
                Point::new(WIDTH as i32 - 8 * 8 * 2 - 1, HEIGHT as i32),
            )
            .unwrap();

        self.canvas.present();
    }

    fn draw_pixel(&mut self, x: i32, y: i32, color: u8) {
        self.canvas.set_draw_color(Color::RGB(
            63 + color * 64,
            63 + color * 64,
            63 + color * 64,
        ));
        let rect: Rect = Rect::new(x, y, PIXEL, PIXEL);
        self.canvas.draw_rect(rect).unwrap();
    }

    pub fn draw_single_tile(&mut self, mem: &[u8; 0x10000], index: u8, pos_x: i32, pos_y: i32) {
        let pixels = Tile::new(core::array::from_fn(|n| {
            mem[0x8000 + index as usize * 16 + n]
        }))
        .get_pixels();

        for n in 0..64 {
            let x = pos_x + (n as i32 % 8) * PIXEL as i32;
            let y = pos_y + (n as i32 / 8) * PIXEL as i32;
            self.draw_pixel(x, y, pixels[n]);
        }
    }

    pub fn draw_vram_tiles(&mut self, mem: &[u8; 0x10000]) {
        for i in 0..256 {
            let base_x = WIDTH as i32 - 8 * 8 * PIXEL as i32 + (i as i32 % 8) * 8 * PIXEL as i32;
            let base_y = (i as i32 / 8) * 8 * PIXEL as i32;

            self.draw_single_tile(mem, i as u8, base_x, base_y);
        }

        self.canvas.present();
    }

    pub fn write_smth(&mut self) {
        let font = self
            .ttf_context
            .load_font("data/RuneScape-Chat-07.ttf", 1000)
            .map_err(|e| e.to_string())
            .unwrap();

        let text = "test".to_string();
        let surface = font
            .render(&text)
            .blended(Color::RGB(255, 255, 255))
            .map_err(|e| e.to_string())
            .unwrap();

        let texture = self.canvas.texture_creator();
        let texture2 = texture
            .create_texture_from_surface(&surface)
            .map_err(|e| e.to_string())
            .unwrap();

        let target = Rect::new(800 as i32, 20 as i32, 200 as u32, 100 as u32);
        self.canvas.copy(&texture2, None, Some(target)).unwrap();

        self.canvas.present();
    }

    pub fn update_gameboy_frame(&mut self, pixels: &[u8; DIMENSIONS]) {
        for i in 0..DIMENSIONS {
            self.draw_pixel(
                ((i % DIMENSIONS_X) * 2) as i32,
                ((i / DIMENSIONS_X) * 2) as i32,
                pixels[i],
            );
        }
        self.canvas.present();
    }

    pub fn check_events(&mut self) {
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
                        Keycode::A => {
                            debug!("A was pressed!\n");
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
                        Keycode::A => {
                            debug!("A was released!\n");
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }
    }
}
