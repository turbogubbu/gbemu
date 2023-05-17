use std::fs;

use crate::gameboy::cpu::Cpu;
use crate::gameboy::memory::Memory;

pub struct Gameboy {
    cpu: Cpu,
    memory: Memory,
}

impl Gameboy {
    pub fn new() -> Gameboy {
        Gameboy {
            cpu: Cpu::new(),
            memory: Memory::new(),
        }
    }

    pub fn init(&mut self) {
        self.cpu
            .load_boot_rom(fs::read("roms/boot.gb").unwrap(), &mut self.memory.data);
        self.cpu.print_status();
    }

    pub fn run(&mut self) {
        let i: u64 = 0;
        loop {
            self.cpu.execute_single_instruction(&mut self.memory.data);
            /*self.memory.print(0x9800, 0x3ff);
            //self.cpu.print_status();

            for i in 0..40 {
                if self.memory.get_oam_entry(i)[0] != 0 {
                    println!(
                        "OAM entry y-pos at index {}: {}",
                        i,
                        self.memory.get_oam_entry(0)[0]
                    );
                    panic!();
                }
                if self.memory.get_oam_entry(i)[1] != 0 {
                    println!(
                        "OAM entry y-pos at index {}: {}",
                        i,
                        self.memory.get_oam_entry(0)[0]
                    );
                    panic!();
                }
                if self.memory.get_oam_entry(i)[2] != 0 {
                    println!(
                        "OAM entry y-pos at index {}: {}",
                        i,
                        self.memory.get_oam_entry(0)[0]
                    );
                    panic!();
                }
                if self.memory.get_oam_entry(i)[3] != 0 {
                    println!(
                        "OAM entry y-pos at index {}: {}",
                        i,
                        self.memory.get_oam_entry(0)[0]
                    );
                    panic!();
                }
            }

            println!("LCDControl: {:02x}", self.memory.get_lcd_control());*/
        }
    }
}
