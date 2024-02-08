use std::fs;

use std::time::{Duration, Instant};

use crate::gameboy::cpu::Cpu;
use crate::gameboy::display::Display;
use crate::gameboy::memory::Memory;
use crate::gameboy::ppu::Ppu;
use crate::util::video::Video;

use super::joypad_input::JoypadInput;

pub struct Gameboy {
    cpu: Cpu,
    memory: Memory,
    ppu: Ppu,
    display: Display,
    video: Video,
}

impl Gameboy {
    pub fn new() -> Gameboy {
        Gameboy {
            cpu: Cpu::new(),
            memory: Memory::new(),
            ppu: Ppu::new(),
            display: Display::new(),
            video: Video::new(),
        }
    }

    pub fn init(&mut self) {
        self.memory.load_rom(fs::read("roms/tetris.gb").unwrap());
        self.cpu
            .load_boot_rom(fs::read("roms/boot.gb").unwrap(), &mut self.memory);
        self.cpu.print_status();
        self.display.init(&self.memory.data);
    }

    pub fn run(&mut self) {
        let event_interval = Duration::from_millis(10);
        let vram_interval = Duration::from_millis(1000);
        let mut last_event_check = Instant::now();
        let mut last_vram_update = Instant::now();

        loop {
            self.cpu.execute_single_instruction(&mut self.memory);

            if self.cpu.loading_boot_image {
                //self.display.draw_vram_tiles(&self.memory.data);
                self.video.draw_vram_tiles(&self.memory.data);
                self.cpu.loading_boot_image = false;
            }

            if self.cpu.load_rom_boot_section {
                println!("boot section before loading new file");
                self.memory.print(0x0000, 0x100);
                self.cpu
                    .load_boot_rom(fs::read("roms/tetris.gb").unwrap(), &mut self.memory);
                println!("boot section after loading new file");
                self.memory.print(0x0000, 0x100);
                self.cpu.load_rom_boot_section = false;
            }

            if self.cpu.draw_line() && self.ppu.get_lcd_ppu_enable(&self.memory) {
                if self
                    .ppu
                    .draw_line(&mut self.memory, &mut self.display.pixel_buffer)
                {
                    self.video.update_gameboy_frame(&self.display.pixel_buffer);
                    // self.video.draw_vram_tiles(&self.memory.data);

                    self.video.write_smth();
                }
            }

            if last_event_check.elapsed() >= event_interval {
                self.video.check_events();
                last_event_check = Instant::now();
            }

            if last_vram_update.elapsed() >= vram_interval {
                self.video.draw_vram_tiles(&self.memory.data);
                self.cpu.print_status();
                self.memory.print_ie_register();
                self.memory.print_interrupt_flag_register();
                last_vram_update = Instant::now();
            }
        }
    }
}
