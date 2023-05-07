use std::fs;

use crate::gameboy::cpu::Cpu;
use crate::gameboy::display::Display;
use crate::gameboy::memory::Memory;

use super::display;

pub struct Gameboy<'a> {
    cpu: Cpu<'a>,
    display: Display,
    mem: Memory,
}

impl<'a> Gameboy<'a> {
    pub fn new() -> Gameboy<'static> {
        Gameboy {
            mem: Memory::new(),
            cpu: Cpu::new(),
            display: Display::new(),
        }
    }

    pub fn init(&'a mut self) {
        self.cpu.init_mem(&mut self.mem);
        self.cpu.load_boot_rom(fs::read("roms/boot.gb").unwrap());
        self.cpu.print_status();
    }

    pub fn run(&'a mut self) {
        let i: u64 = 0;
        self.cpu.init_mem(&mut self.mem);
        self.cpu.load_boot_rom(fs::read("roms/boot.gb").unwrap());
        self.cpu.print_status();

        loop {
            self.cpu.execute_single_instruction();
            self.cpu.print_status();

            if i % 100 == 0 {
                println!("------------------ CPU ------------------");
                self.cpu.print(0xff00, 0x99);
                println!("----------------- MEMORY ----------------");
                //self.mem.print(0xff00, 0x99);
            }
        }
    }
}
