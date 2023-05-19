use std::fs;

use crate::gameboy::cpu::Cpu;
use crate::gameboy::memory::Memory;
use crate::gameboy::ppu::Ppu;

pub struct Gameboy {
    cpu: Cpu,
    memory: Memory,
    ppu: Ppu,
}

impl Gameboy {
    pub fn new() -> Gameboy {
        Gameboy {
            cpu: Cpu::new(),
            memory: Memory::new(),
            ppu: Ppu::new(),
        }
    }

    pub fn init(&mut self) {
        self.memory.load_rom(fs::read("roms/tetris.gb").unwrap());
        self.cpu
            .load_boot_rom(fs::read("roms/boot.gb").unwrap(), &mut self.memory.data);
        self.cpu.print_status();
    }

    pub fn run(&mut self) {
        loop {
            self.cpu.execute_single_instruction(&mut self.memory.data);

            if self.cpu.get_uptime() % 456 == 0 {
                self.ppu.draw_line(&mut self.memory.data);
            }
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
