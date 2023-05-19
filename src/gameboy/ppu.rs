use crate::gameboy::display::DIMENSIONS;

#[derive(Debug)]
pub struct Ppu {}

impl Ppu {
    pub fn new() -> Ppu {
        Ppu {}
    }

    pub fn draw_line(&mut self, mem: &mut [u8; 0x10000], pixel_buff: &mut [u8; DIMENSIONS]) {
        self.oam_scan(mem);
        self.draw_pixels();
        self.horizontal_blank();
    }

    fn get_lcd_y(&self, mem: &[u8; 0x10000]) -> u8 {
        mem[0xff44]
    }

    fn increment_lcd_y(&self, mem: &mut [u8; 0x10000]) {
        mem[0xff44] += 1;
    }

    fn get_lcd_control(&self, mem: &[u8; 0x10000]) -> u8 {
        mem[0xff40]
    }

    fn get_oam_table(&mut self, mem: &[u8; 0x10000]) -> &[u8] {
        &mem[0xfe00..0xfea0]
    }

    fn oam_scan(&mut self, mem: &[u8; 0x10000]) -> Vec<OAM> {
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

    fn draw_pixels(&mut self) {}

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
struct LcdControl {
    enable: bool,
    tile_map_area: bool,
    window_enable: bool,
    bg_window_tile_area: bool,
    bg_tile_map_area: bool,
    obj_size: bool,
    obj_enable: bool,
    bg_window_enable: bool,
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
