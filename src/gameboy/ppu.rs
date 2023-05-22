use crate::gameboy::display::{DIMENSIONS, DIMENSIONS_X, DIMENSIONS_Y};
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
        self.increment_lcd_y(mem);
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

    fn get_scy(&self, mem: &[u8; ADDRESS_SPACE]) -> u8 {
        mem[0xff42]
    }

    fn get_scx(&self, mem: &[u8; ADDRESS_SPACE]) -> u8 {
        mem[0xff43]
    }

    fn get_tile_map_index(&self, mem: &[u8; ADDRESS_SPACE], x: u8, y: u8) -> u8 {
        assert!(x < 32 && y < 32);
        let lcd_c = LcdControl::new(self.get_lcd_control(mem));

        if lcd_c.tile_map_area {
            mem[0x9c00 + x as usize + y as usize * 32]
        } else {
            mem[0x9800 + x as usize + y as usize * 32]
        }
    }

    fn draw_pixels(
        &mut self,
        mem: &[u8; ADDRESS_SPACE],
        pixel_buff: &mut [u8; DIMENSIONS],
        sprites: Vec<OAM>,
    ) {
        let lcd_control = LcdControl::new(self.get_lcd_control(mem));
        let mut oam_fifo = [0; DIMENSIONS_X];
        let mut bg_fifo = [0; DIMENSIONS_X];

        if lcd_control.obj_enable {
            for (i, sprite) in sprites.iter().enumerate() {
                let pos = sprite.x_pos as i16 - 8;
                let y = self.get_lcd_y(mem) - sprite.y_pos;
                let tile = self.get_tile(mem, sprite.index);
                let sprite_pixels: [u8; 0x8] = core::array::from_fn(|m| tile.get_pixel(m as u8, y));

                for i in 0..8 {
                    if (pos + i) < 0 {
                        continue;
                    } else {
                        oam_fifo[(pos + i) as usize] = sprite_pixels[i as usize];
                    }
                }
            }
        }

        let scx = self.get_scx(mem);
        let scy = self.get_scy(mem);
        let ly = self.get_lcd_y(mem);

        for i in 0..160 {
            // todo: no check of any registers, just trying to get the bootscreen running
            // good reference for ppu processing: http://pixelbits.16-b.it/GBEDG/ppu/#a-word-of-warning
            let x = ((scx / 8) + i) & 0x1f;
            let y = ((ly as u16 + scy as u16) & 0xff) as u8 / 8;
            let tile = self.get_tile(mem, self.get_tile_map_index(mem, x, y));
            let y_tile = (scy.wrapping_sub(ly)) % 8;
            let x_tile = i % 8;

            bg_fifo[i as usize] = tile.get_pixel(x_tile, y_tile);
        }

        for i in 0..160 {
            if oam_fifo[i as usize] == 0 {
                pixel_buff[ly as usize * DIMENSIONS_X + i] = bg_fifo[i];
            } else {
                pixel_buff[ly as usize * DIMENSIONS_X + i] = oam_fifo[i];
            }
        }
    }

    fn horizontal_blank(&mut self) {}

    pub fn get_lcd_ppu_enable(&self, mem: &[u8; ADDRESS_SPACE]) -> bool {
        LcdControl::new(self.get_lcd_control(mem)).enable
    }
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
    tile_indexes: [u8; 32 * 32],
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
