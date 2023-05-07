#[derive(Debug)]
pub struct Memory {
    data: [u8; 0x10000],
}

impl Memory {
    pub fn new() -> Memory {
        Memory { data: [0; 0x10000] }
    }

    pub fn write(&mut self, address: u16, byte: u8) {
        self.data[address as usize] = byte;
    }

    pub fn read(&self, address: u16) -> u8 {
        self.data[address as usize]
    }

    pub fn print(&self, start_address: u16, len: u16) {
        println!("        00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f");

        let start: u16 = start_address - start_address % 16;
        let end: u16 = start + start_address % 16 + len;

        for i in start..end {
            if i % 16 == 0 {
                print!("0x{:04x}  ", i);
            }

            print!("{:02x} ", self.data[i as usize]);

            if (i + 1) % 16 == 0 {
                print!("\n");
            }
        }

        println!("\n");
    }
}
