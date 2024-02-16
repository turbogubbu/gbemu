extern crate sdl2;

use crate::gameboy::display::{DIMENSIONS, DIMENSIONS_X, DIMENSIONS_Y};
use crate::gameboy::joypad_input::JoypadInput;
use crate::gameboy::ppu::Tile;

use sdl2::pixels::PixelFormatEnum;
use sdl2::rect::Rect;
use sdl2::render::WindowCanvas;
use sdl2::surface::Surface;
use sdl2::Sdl;
use sdl2::VideoSubsystem;

// LCD pixel to real pixel ratio
const PIXEL: u32 = 2;

const WIDTH: u32 = 1600;
const HEIGHT: u32 = 1000;

pub struct Video {
    video_subsys: VideoSubsystem,
    canvas: WindowCanvas,
}

impl Video {
    pub fn new(sdl: &Sdl) -> Video {
        let video_subsys = sdl.video().unwrap();
        let canvas = video_subsys
            .window("gbemu", WIDTH, HEIGHT)
            .position_centered()
            .build()
            .unwrap()
            .into_canvas()
            .build()
            .unwrap();
        Video {
            video_subsys,
            canvas,
        }
    }

    pub fn draw_single_tile(&mut self, mem: &[u8; 0x10000], index: usize, pos_x: i32, pos_y: i32) {
        // get pixels of the tile
        let pixels = Tile::new(core::array::from_fn(|n| mem[0x8000 + index * 16 + n])).get_pixels();

        // create bitmap of tile
        let mut bitmap: [u8; 8 * 8 * 3] = [0; 8 * 8 * 3];
        for (i, pixel) in pixels.iter().enumerate() {
            bitmap[3 * i + 0] = 155 - pixel * 41;
            bitmap[3 * i + 1] = 188 - pixel * 62;
            bitmap[3 * i + 2] = 15 - pixel * 5;
        }

        let surface2 =
            Surface::from_data(&mut bitmap, 8u32, 8u32, 8 * 3u32, PixelFormatEnum::RGB24).unwrap();

        self.draw_surface(&surface2, pos_x, pos_y, 2);
    }

    pub fn draw_vram_tiles(&mut self, mem: &[u8; 0x10000]) {
        for i in 0..256 {
            let base_x = WIDTH as i32 - 8 * 8 * PIXEL as i32 + (i as i32 % 8) * 8 * PIXEL as i32;
            let base_y = (i as i32 / 8) * 8 * PIXEL as i32;

            self.draw_single_tile(mem, i, base_x, base_y);
        }
    }

    pub fn draw_tile_maps(&mut self, mem: &[u8; 0x10000]) {
        for row in 0..32 {
            for column in 0..32 {
                let base_x = WIDTH as i32 - 8 * 32 as i32 + column * 8;
                let base_y = 800i32 + row * 8;

                let index = mem[0x9800 + row as usize * 32 + column as usize] as usize;

                self.draw_single_tile(
                    mem, index, // (256 + 128) + row as usize * 32 + column as usize,
                    base_x, base_y,
                );
            }
        }

        for row in 0..32 {
            for column in 0..32 {
                let base_x = WIDTH as i32 - 8 * (32 * 2 + 1) as i32 + column * 8;
                let base_y = 800i32 + row * 8;

                let index = mem[0x9C00 + row as usize * 32 + column as usize] as usize;

                self.draw_single_tile(
                    mem, index, //256 * 2 + row as usize * 32 + column as usize,
                    base_x, base_y,
                );
            }
        }
    }

    pub fn write_smth(&mut self) {
        /*let font = self
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

        self.canvas.present();*/
    }

    pub fn update_gameboy_frame(&mut self, pixels: &[u8; DIMENSIONS]) {
        // Create rgb bitmap
        let mut pixels_rgb: [u8; DIMENSIONS * 3] = [0; DIMENSIONS * 3];
        for (i, &pixel) in pixels.iter().enumerate() {
            pixels_rgb[3 * i + 0] = 155 - pixel * 41;
            pixels_rgb[3 * i + 1] = 188 - pixel * 62;
            pixels_rgb[3 * i + 2] = 15 - pixel * 5;
        }

        // Create a surface from the bitmap
        let surface2 = Surface::from_data(
            &mut pixels_rgb,
            DIMENSIONS_X as u32,
            DIMENSIONS_Y as u32,
            DIMENSIONS_X as u32 * 3,
            PixelFormatEnum::RGB24,
        )
        .unwrap();

        self.draw_surface(&surface2, 0, 0, 4);
    }

    fn draw_round_button(&mut self, pos_x: i32, pos_y: i32, pressed: bool) {
        let col = if pressed { 0 } else { 255 };
        let mut bmp = [
            0, 0, 0, 255, col, col, 255, col, col, 255, col, col, 0, 0, 0, 255, col, col, 255, col,
            col, 255, col, col, 255, col, col, 255, col, col, 255, col, col, 255, col, col, 255,
            col, col, 255, col, col, 255, col, col, 255, col, col, 255, col, col, 255, col, col,
            255, col, col, 255, col, col, 0, 0, 0, 255, col, col, 255, col, col, 255, col, col, 0,
            0, 0,
        ];

        let surface =
            Surface::from_data(&mut bmp, 5u32, 5u32, 5u32 * 3, PixelFormatEnum::RGB24).unwrap();

        self.draw_surface(&surface, pos_x, pos_y, 10);
    }

    fn draw_dpad(&mut self, pos_x: i32, pos_y: i32, up: bool, down: bool, left: bool, right: bool) {
        let up = if up { 0 } else { 255 };
        let left = if left { 0 } else { 255 };
        let right = if right { 0 } else { 255 };
        let down = if down { 0 } else { 255 };

        let mut bmp = [
            0, 0, 0, 255, up, up, 0, 0, 0, 255, left, left, 255, 255, 255, 255, right, right, 0, 0,
            0, 255, down, down, 0, 0, 0,
        ];

        let surface =
            Surface::from_data(&mut bmp, 3u32, 3u32, 3u32 * 3, PixelFormatEnum::RGB24).unwrap();

        self.draw_surface(&surface, pos_x, pos_y, 30);
    }

    fn draw_ss_button(&mut self, pos_x: i32, pos_y: i32, pressed: bool) {
        let col = if pressed { 0 } else { 255 };

        let mut bmp = [
            0, 0, 0, 0, 0, 0, 255, col, col, 0, 0, 0, 255, col, col, 0, 0, 0, 255, col, col, 0, 0,
            0, 0, 0, 0,
        ];

        let surface =
            Surface::from_data(&mut bmp, 3u32, 3u32, 3u32 * 3, PixelFormatEnum::RGB24).unwrap();

        self.draw_surface(&surface, pos_x, pos_y, 20);
    }

    fn draw_surface(&mut self, surface: &Surface, pos_x: i32, pos_y: i32, scale: u32) {
        let texture_creator = self.canvas.texture_creator();
        let texture = texture_creator
            .create_texture_from_surface(&surface)
            .expect("Failed to create texture from surface!");

        let dest_rect = Rect::new(
            pos_x,
            pos_y,
            surface.width() * scale,
            surface.height() * scale,
        );

        // Copy the texture onto the canvas and finally display it
        self.canvas
            .copy(&texture, None, dest_rect)
            .expect("Failed to copy texture onto canvas!");
    }

    pub fn draw_buttons(&mut self, joypad: &JoypadInput) {
        self.draw_round_button(500, 650, joypad.get_a());
        self.draw_round_button(400, 720, joypad.get_b());
        self.draw_dpad(
            100,
            680,
            joypad.get_up(),
            joypad.get_down(),
            joypad.get_left(),
            joypad.get_right(),
        );
        self.draw_ss_button(320, 850, joypad.get_start());
        self.draw_ss_button(220, 850, joypad.get_select());
    }

    pub fn present(&mut self) {
        self.canvas.present();
    }
}
