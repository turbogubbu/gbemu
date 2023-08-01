use std::fs;

use crate::gameboy::cpu::Cpu;
use crate::gameboy::display::Display;
use crate::gameboy::memory::Memory;
use crate::gameboy::ppu::Ppu;

pub struct Gameboy {
    cpu: Cpu,
    memory: Memory,
    ppu: Ppu,
    display: Display,
}

impl Gameboy {
    pub fn new() -> Gameboy {
        Gameboy {
            cpu: Cpu::new(),
            memory: Memory::new(),
            ppu: Ppu::new(),
            display: Display::new(),
        }
    }

    pub fn init(&mut self) {
        self.memory.load_rom(fs::read("roms/tetris.gb").unwrap());
        self.cpu
            .load_boot_rom(fs::read("roms/boot.gb").unwrap(), &mut self.memory.data);
        self.cpu.print_status();
        self.display.init(&self.memory.data);
    }

    pub fn run(&mut self) {
        loop {
            self.cpu.execute_single_instruction(&mut self.memory.data);

            if self.cpu.loading_boot_image {
                self.display.draw_vram_tiles(&self.memory.data);
                self.cpu.loading_boot_image = false;
            }

            if self.cpu.draw_line() && self.ppu.get_lcd_ppu_enable(&self.memory.data) {
                if self.ppu
                    .draw_line(&mut self.memory.data, &mut self.display.pixel_buffer) {
                    self.display.update_frame();
                    self.display.draw_vram_tiles(&self.memory.data);
                    // println!("Drawing frame!");
                }
            }
        }
    }
}
