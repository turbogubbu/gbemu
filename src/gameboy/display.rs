pub const DIMENSIONS_X: usize = 160;
pub const DIMENSIONS_Y: usize = 144;
pub const DIMENSIONS: usize = DIMENSIONS_X * DIMENSIONS_Y;

pub struct Display {
    pub pixel_buffer: [u8; DIMENSIONS],
}

impl Display {
    pub fn new() -> Display {
        Display {
            pixel_buffer: [0; DIMENSIONS],
        }
    }

    pub fn init(&mut self, _mem: &[u8; 0x10000]) {}
}
