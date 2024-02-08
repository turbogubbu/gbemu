use std::fs;

use std::time::{Duration, Instant};

use crate::gameboy::cpu::Cpu;
use crate::gameboy::display::Display;
use crate::gameboy::memory::Memory;
use crate::gameboy::ppu::Ppu;
use crate::util::video::Video;

use core::arch::x86_64::_rdtsc;

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

        let mut start: u64 = 0;
        let mut end: u64 = 0;
        let mut execute_instruction_time: u64 = 0;
        let mut handle_interrupt_time: u64 = 0;
        let mut ppu_time: u64 = 0;
        let mut event_time: u64 = 0;
        let mut misc_stuff_time: u64 = 0;
        let mut sum_time: u64 = 0;

        loop {
            unsafe {
                start = _rdtsc();
            }

            self.cpu.execute_single_instruction(&mut self.memory);

            unsafe {
                end = _rdtsc();
            }

            execute_instruction_time += end - start;
            start = end;

            self.cpu.handle_interrupt(&mut self.memory);

            unsafe {
                end = _rdtsc();
            }
            handle_interrupt_time += end - start;

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

            unsafe {
                start = _rdtsc();
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

            unsafe {
                end = _rdtsc();
            }

            ppu_time += end - start;
            start = end;

            if last_event_check.elapsed() >= event_interval {
                self.video.check_events();
                last_event_check = Instant::now();
            }

            unsafe {
                end = _rdtsc();
            }
            event_time += end - start;
            start = end;

            if last_vram_update.elapsed() >= vram_interval {
                self.video.draw_vram_tiles(&self.memory.data);
                self.cpu.print_status();
                self.memory.print_ie_register();
                self.memory.print_interrupt_flag_register();
                last_vram_update = Instant::now();

                println!("Ppu enabled: {}", self.ppu.get_lcd_ppu_enable(&self.memory));

                print!(
                    "Benchmarking:\n  Instruction execution: {:10} ({}%)\n  Handle interrupt:      {:10} ({}%)\n  Ppu execution:         {:10} ({}%)\n  Event execution:       {:10} ({}%)\n  Misc execution:        {:10} ({}%)\n",
                    execute_instruction_time,
                    execute_instruction_time as f64 / sum_time as f64 * 100.0,
                    handle_interrupt_time,
                    handle_interrupt_time as f64 / sum_time as f64 * 100.0,
                    ppu_time,
                    ppu_time as f64 / sum_time as f64 * 100.0,
                    event_time,
                    event_time as f64 / sum_time as f64 * 100.0,
                    misc_stuff_time,
                    misc_stuff_time as f64 / sum_time as f64 * 100.0
                );
            }

            unsafe {
                end = _rdtsc();
            }
            misc_stuff_time += end - start;

            sum_time = execute_instruction_time
                + handle_interrupt_time
                + ppu_time
                + event_time
                + misc_stuff_time;
        }
    }
}
