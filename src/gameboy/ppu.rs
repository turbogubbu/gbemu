#[derive(Debug)]
pub struct Ppu {
    frame: [Pixel; 160 * 154],
}

impl Ppu {
    pub fn new() -> Ppu {
        Ppu {
            frame: [Pixel(0); 160 * 154],
        }
    }

    pub fn draw_image(&mut self, mem: &mut [u8; 0x10000]) {
        for n in 0..154 {
            self.draw_line();
        }
    }

    fn draw_line(&mut self) {
        self.oam_scan();
        self.draw_pixels();
        self.horizontal_blank();
    }

    fn oam_scan(&mut self) {}

    fn draw_pixels(&mut self) {}

    fn horizontal_blank(&mut self) {}
}

#[derive(Copy, Clone, Debug)]
struct Pixel(u8);
