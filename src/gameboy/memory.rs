use crate::gameboy::joypad_input::JoypadInput;

#[derive(Debug, PartialEq)]
enum CartridgeType {
    RomOnly,
    NotImplemented,
}

#[derive(Debug)]
pub struct Memory {
    pub data: [u8; ADDRESS_SPACE],
    pub joypad_input: JoypadInput,
    cartridge_type: CartridgeType,
}

pub const ADDRESS_SPACE: usize = 0x10000;
pub const JOYPAD_INPUT_ADDRESS: usize = 0xff00;

impl Memory {
    pub fn new() -> Memory {
        Memory {
            data: [0; ADDRESS_SPACE],
            joypad_input: JoypadInput::new(),
            cartridge_type: CartridgeType::NotImplemented,
        }
    }

    #[allow(dead_code)]
    pub fn get_lcd_control(&self) -> u8 {
        self.data[0xff40]
    }

    #[allow(dead_code)]
    pub fn get_oam_entry(&self, index: u8) -> [u8; 4] {
        assert!(index < 40, "OAM index has to be lower than 40!");
        [
            self.data[(0xfe00 + (index as usize) * 4) as usize],
            self.data[(0xfe01 + (index as usize) * 4) as usize],
            self.data[(0xfe02 + (index as usize) * 4) as usize],
            self.data[(0xfe03 + (index as usize) * 4) as usize],
        ]
    }

    pub fn load_rom(&mut self, rom: Vec<u8>) {
        for i in 0..rom.len() {
            self.data[i] = rom[i];
        }

        if self.data[0x147] == 0x00 {
            self.cartridge_type = CartridgeType::RomOnly;
        } else {
            /*panic!(
                "Cartridge type is not ROM only! (it is: 0x{:02x})\n",
                self.data[0x147]
            );*/
        }

        // This sets all buttons to not pressed!
        self.data[JOYPAD_INPUT_ADDRESS] = 0x0f;
    }

    #[allow(dead_code)]
    fn get_input_reg(&self) -> u8 {
        let higher_nibble = self.data[0xff00 as usize] >> 4;

        match higher_nibble {
            0x3 => 0x3f,
            0x2 => self.joypad_input.get_udlr() | 0x20,
            0x1 => self.joypad_input.get_ssba() | 0x10,
            _ => 0x0f,
        }
    }

    pub fn read_mem(&self, address: u16) -> u8 {
        match address {
            // joypad input
            0xff00 => self.get_input_reg(),
            // normal memory
            _ => self.data[address as usize],
        }
    }

    pub fn write_mem(&mut self, address: u16, value: u8) {
        if self.cartridge_type == CartridgeType::RomOnly && address <= 0x3fff {
            return;
        }

        self.data[address as usize] = value;
    }

    #[allow(dead_code)]
    pub fn get_serial_transfer(&self) -> [u8; 2] {
        [self.data[0xff01], self.data[0xff02]]
    }

    #[allow(dead_code)]
    pub fn get_timer_and_divider(&self) -> [u8; 4] {
        [
            self.data[0xff04],
            self.data[0xff05],
            self.data[0xff06],
            self.data[0xff07],
        ]
    }

    #[allow(dead_code)]
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

    pub fn print_ie_register(&self) {
        println!("IE register: 0x{:02x}", self.data[0xffff]);
    }

    pub fn print_interrupt_flag_register(&self) {
        println!("Interrupt flag register: 0x{:02x}", self.data[0xff0f]);
    }
}
