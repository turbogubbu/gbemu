use std::fs;

use crate::gameboy::cpu::Cpu;

pub struct Gameboy {
    cpu: Cpu,
}

impl Gameboy {
    pub fn new() -> Gameboy {
        Gameboy {
            cpu: Cpu::new(),
        }
    }

    pub fn init(&mut self) {
        self.cpu.load_boot_rom(fs::read("roms/boot.gb").unwrap());
        self.cpu.print_status();
    }

    pub fn run(&mut self) {
        let i: u64 = 0;
        loop {
            self.cpu.execute_single_instruction();
            self.cpu.print_status();
        }
    }
}
