use crate::gameboy::display::{DIMENSIONS, DIMENSIONS_X};
use crate::gameboy::memory::ADDRESS_SPACE;

#[derive(Debug)]
pub struct Ppu {}

impl Ppu {
    pub fn new() -> Ppu {
        Ppu {}
    }

    pub fn draw_line(&mut self, mem: &mut [u8; ADDRESS_SPACE], pixel_buff: &mut [u8; DIMENSIONS]) {
        self.draw_pixels(mem, pixel_buff, self.oam_scan(mem));
        self.horizontal_blank();
        // self.increment_lcd_y(mem);
    }

    fn get_lcd_y(&self, mem: &[u8; ADDRESS_SPACE]) -> u8 {
        mem[0xff44]
    }

    fn increment_lcd_y(&self, mem: &mut [u8; ADDRESS_SPACE]) {
        mem[0xff44] += 1;
    }

    fn get_lcd_control(&self, mem: &[u8; ADDRESS_SPACE]) -> u8 {
        mem[0xff40]
    }

    fn get_oam_table(&self, mem: &[u8; ADDRESS_SPACE]) -> [u8; 0xa0] {
        core::array::from_fn(|n| mem[0xfe00 + n])
    }

    fn get_tile(&self, mem: &[u8; ADDRESS_SPACE], index: u8) -> Tile {
        assert!(index < 40);
        Tile::new(core::array::from_fn(|n| {
            mem[0x8000 + index as usize * 16 + n]
        }))
    }

    fn oam_scan(&self, mem: &[u8; ADDRESS_SPACE]) -> Vec<OAM> {
        let oam_table = self.get_oam_table(mem);
        let lcd_control = LcdControl::new(self.get_lcd_control(mem));
        let obj_size: u8 = if lcd_control.obj_size { 16 } else { 8 };
        let y = self.get_lcd_y(mem);
        let mut active_oam = Vec::new();

        for i in 0..40 {
            let oam = OAM::new(
                oam_table[0 + 4 * i],
                oam_table[1 + 4 * i],
                oam_table[2 + 4 * i],
                oam_table[3 + 4 * i],
            );

            if y >= oam.y_pos && y < (oam.y_pos + obj_size) {
                active_oam.push(oam);
            }

            // Maximum of 10 sprites can be drawn per pixel line!
            if active_oam.len() >= 10 {
                break;
            }
        }
        active_oam
    }

    fn get_scy(mem: &[u8; ADDRESS_SPACE]) -> u8 {
        mem[0xff42]
    }

    fn get_scx(mem: &[u8; ADDRESS_SPACE]) -> u8 {
        mem[0xff43]
    }

    fn draw_pixels(
        &mut self,
        mem: &[u8; ADDRESS_SPACE],
        pixel_buff: &mut [u8; DIMENSIONS],
        sprites: Vec<OAM>,
    ) {
        let mut line: [u8; DIMENSIONS_X] = [0; DIMENSIONS_X];
        let lcd_control = LcdControl::new(self.get_lcd_control(mem));
        let mut oam_fifo = [0; 8];
        let mut bg_fifo = [0; 8];

        for i in 0..DIMENSIONS_X {
            // oam fifo
            if lcd_control.obj_enable {
                let mut sprite_index = 0xff;
                'inner: for (i, sprite) in sprites.iter().enumerate() {
                    if i >= sprite.x_pos as usize && i < (sprite.x_pos as usize + 8) {
                        sprite_index = i;
                        break 'inner;
                    }
                }

                if sprite_index < 10 {
                    let oam_entry = sprites.get(sprite_index).unwrap();
                    let y = self.get_lcd_y(mem) - oam_entry.y_pos;
                    let tile = self.get_tile(mem, oam_entry.index);
                    let sprite_pixels: [u8; 0x8] =
                        core::array::from_fn(|m| tile.get_pixel(m as u8, y));
                    if oam_entry.x_pos < 7 || oam_entry.x_pos > 160 {
                        for i in 0..8 {}
                    }
                } else {
                    oam_fifo[i % 8] = 0;
                }
            }
        }
    }

    fn horizontal_blank(&mut self) {}
}

#[derive(Copy, Clone, Debug)]
struct Pixel(u8);

#[derive(Debug)]
struct OAM {
    y_pos: u8,
    x_pos: u8,
    index: u8,
    flags: u8,
}

impl OAM {
    pub fn new(y_pos: u8, x_pos: u8, index: u8, flags: u8) -> OAM {
        OAM {
            y_pos,
            x_pos,
            index,
            flags,
        }
    }
}

#[derive(Debug)]
pub struct Tile([u8; 16]);

impl Tile {
    pub fn new(data: [u8; 16]) -> Tile {
        Tile(data)
    }

    pub fn get_pixels(&self) -> [u8; 8 * 8] {
        core::array::from_fn(|i| self.get_pixel(i as u8 % 8, i as u8 / 8))
    }

    pub fn get_pixel(&self, x: u8, y: u8) -> u8 {
        assert!(x < 8 && y < 8);
        let lsbyte: u8 = self.0[y as usize * 2];
        let msbyte: u8 = self.0[y as usize * 2 + 1];
        let lsbit: bool = if (lsbyte & (1 << (7 - x))) > 0 {
            true
        } else {
            false
        };
        let msbit: bool = if (msbyte & (1 << (7 - x))) > 0 {
            true
        } else {
            false
        };
        return if msbit { 2 } else { 0 } + if lsbit { 1 } else { 0 };
    }
}

#[derive(Debug)]
struct TileMap {
    tile_indexes: Vec<u8>,
}

impl TileMap {}

#[derive(Debug)]
struct LcdControl {
    pub enable: bool,
    pub tile_map_area: bool,
    pub window_enable: bool,
    pub bg_window_tile_area: bool,
    pub bg_tile_map_area: bool,
    pub obj_size: bool,
    pub obj_enable: bool,
    pub bg_window_enable: bool,
}

impl LcdControl {
    fn new(byte: u8) -> LcdControl {
        LcdControl {
            enable: (byte & 0x80) > 0,
            tile_map_area: (byte & 0x40) > 0,
            window_enable: (byte & 0x20) > 0,
            bg_window_tile_area: (byte & 0x10) > 0,
            bg_tile_map_area: (byte & 0x08) > 0,
            obj_size: (byte & 0x04) > 0,
            obj_enable: (byte & 0x02) > 0,
            bg_window_enable: (byte & 0x01) > 0,
        }
    }
}
