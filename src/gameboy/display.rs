use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::Color;
use sdl2::render::WindowCanvas;

//use super::Display::display;

pub struct Display {
    pixel: Vec<u8>,
    lcd_control: u8,
    lcd_status: u8,
}

impl Display {
    pub fn new() -> Display {
        Display {
            pixel: vec![0; 160 * 144],
            lcd_control: 0,
            lcd_status: 0,
        }
    }

    pub fn init(&mut self) {
        let sdl_context = sdl2::init().unwrap();
        let video_subsystem = sdl_context.video().unwrap();

        let window = video_subsystem
            .window("gbemu", 800, 600)
            .position_centered()
            .build()
            .expect("could not initialize video subsystem");

        let mut canvas = window.into_canvas().build().expect("could not make canvas");

        let mut event_pump = sdl_context.event_pump().unwrap();

        'running: loop {
            for event in event_pump.poll_iter() {
                match event {
                    Event::Quit { .. }
                    | Event::KeyDown {
                        keycode: Some(Keycode::Escape),
                        ..
                    } => {
                        break 'running;
                    }
                    _ => {}
                }
            }

            canvas.set_draw_color(Color::RGB(255, 32, 64));
            canvas.clear();
            canvas.present();
        }
    }
}

struct Tile {
    data: Vec<u8>,
}

impl Tile {
    fn new(bytes: Vec<u8>) -> Tile {
        if bytes.len() != 8 {
            assert!(false, "need 8 bytes of data to create tile!");
        }

        Tile {
            data: vec![
                bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[3],
                bytes[8], bytes[9], bytes[10], bytes[11], bytes[12], bytes[13], bytes[14],
                bytes[15],
            ],
        }
    }
}

struct TileMap {
    tile_indexes: Vec<u8>,
}

impl TileMap {}

struct SpriteAttribute {
    ypos: u8,
    xpos: u8,
    index: u8,
    flags: u8,
}

impl SpriteAttribute {
    fn new(bytes: Vec<u8>) -> SpriteAttribute {
        if bytes.len() != 4 {
            assert!(false, "need 4 bytes of data to create SpriteAttribute");
        }

        SpriteAttribute {
            ypos: bytes[0],
            xpos: bytes[1],
            index: bytes[2],
            flags: bytes[3],
        }
    }
}
