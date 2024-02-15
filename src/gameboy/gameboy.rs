use std::fs;

use std::time::{Duration, Instant};

use crate::gameboy::cpu::Cpu;
use crate::gameboy::display::Display;
use crate::gameboy::memory::Memory;
use crate::gameboy::ppu::Ppu;
use crate::util::video::Video;

use core::arch::x86_64::_rdtsc;

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
        // for gameboy doctor
        // self.memory.data[0xff44] = 0x90;
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

        let mut start: u64;
        let mut end: u64;
        let mut handle_interrupt_time: u64;
        let mut ppu_start: u64;
        let mut event_time: u64;
        let mut misc_stuff_time: u64;
        let mut draw_line_time: u64 = 0;
        let mut update_frame_time: u64 = 0;
        let mut update_frame_time2: u64 = 0;
        let mut handle_boot_image_time: u64;
        let mut font_time: u64 = 0;
        let mut loop_count: u64 = 0;

        let mut sum_time: u64 = 0;
        #[allow(unused)]
        let mut sum_handle_instruction_time: u64 = 0;
        #[allow(unused)]
        let mut sum_handle_interrupt_time: u64 = 0;
        #[allow(unused)]
        let mut sum_handle_boot_image_time: u64 = 0;
        #[allow(unused)]
        let mut sum_handle_ppu: u64 = 0;
        #[allow(unused)]
        let mut sum_handle_event: u64 = 0;
        #[allow(unused)]
        let mut sum_misc_time: u64 = 0;
        #[allow(unused)]
        let mut sum_draw_line_time: u64 = 0;
        #[allow(unused)]
        let mut sum_update_frame_time: u64 = 0;
        #[allow(unused)]
        let mut sum_draw_text_time: u64 = 0;
        #[allow(unused)]
        let mut sum_draw_display_time: u64 = 0;

        // For gameboy doc
        /*
        let file = std::fs::File::create("log").unwrap();
        let mut writer = BufWriter::new(&file);*/

        loop {
            // ----------------- Benchmarking ------------------- //
            unsafe {
                start = _rdtsc();
            }
            // ----------------- Benchmarking ------------------- //

            // For gameboy doc
            /*if !self.cpu.prefixed {
                self.cpu.log_state_to_file(&self.memory, &mut writer);
            }*/

            self.cpu.execute_single_instruction(&mut self.memory);
            /*if self.cpu.prefixed {
                self.cpu.execute_single_instruction(&mut self.memory);
            }*/

            self.memory.handle_timer(self.cpu.get_uptime() / 4);

            // ----------------- Benchmarking ------------------- //
            unsafe {
                handle_interrupt_time = _rdtsc();
            }
            // ----------------- Benchmarking ------------------- //

            self.cpu.handle_interrupt(&mut self.memory);

            // ----------------- Benchmarking ------------------- //
            unsafe {
                handle_boot_image_time = _rdtsc();
            }
            // ----------------- Benchmarking ------------------- //

            if self.cpu.loading_boot_image {
                //self.display.draw_vram_tiles(&self.memory.data);
                self.video.draw_vram_tiles(&self.memory.data);
                self.cpu.loading_boot_image = false;
            }

            if self.cpu.load_rom_boot_section {
                // println!("boot section before loading new file");
                // self.memory.print(0x0000, 0x100);
                self.cpu
                    .load_boot_rom(fs::read("roms/tetris.gb").unwrap(), &mut self.memory);
                // println!("boot section after loading new file");
                // self.memory.print(0x0000, 0x100);
                self.cpu.load_rom_boot_section = false;
            }

            // ----------------- Benchmarking ------------------- //
            unsafe {
                ppu_start = _rdtsc();
            }
            // ----------------- Benchmarking ------------------- //

            if self.cpu.draw_line() && self.ppu.get_lcd_ppu_enable(&self.memory) {
                if self
                    .ppu
                    .draw_line(&mut self.memory, &mut self.display.pixel_buffer)
                {
                    // ----------------- Benchmarking ------------------- //
                    unsafe {
                        draw_line_time = _rdtsc();
                    }
                    unsafe {
                        update_frame_time = _rdtsc();
                    }
                    // ----------------- Benchmarking ------------------- //

                    self.video
                        .update_gameboy_frame(&mut self.display.pixel_buffer);
                    self.video.draw_buttons(&self.memory.joypad_input);

                    self.video.present();

                    // ----------------- Benchmarking ------------------- //
                    unsafe {
                        font_time = _rdtsc();
                    }
                    // ----------------- Benchmarking ------------------- //

                    self.video.write_smth();

                    // ----------------- Benchmarking ------------------- //
                    unsafe {
                        update_frame_time2 = _rdtsc();
                    }
                    // ----------------- Benchmarking ------------------- //
                } else {
                    // ----------------- Benchmarking ------------------- //
                    unsafe {
                        draw_line_time = _rdtsc();
                    }
                    // ----------------- Benchmarking ------------------- //
                }
            }

            // ----------------- Benchmarking ------------------- //
            unsafe {
                event_time = _rdtsc();
            }
            // ----------------- Benchmarking ------------------- //

            if last_event_check.elapsed() >= event_interval {
                self.video.check_events(&mut self.memory.joypad_input);
                last_event_check = Instant::now();
            }

            // ----------------- Benchmarking ------------------- //
            unsafe {
                misc_stuff_time = _rdtsc();
            }
            // ----------------- Benchmarking ------------------- //

            if last_vram_update.elapsed() >= vram_interval {
                self.video.draw_vram_tiles(&self.memory.data);
                self.video.draw_tile_maps(&self.memory.data);
                self.cpu.print_status();
                self.memory.print_ie_register();
                self.memory.print_interrupt_flag_register();
                last_vram_update = Instant::now();

                println!("Ppu enabled: {}", self.ppu.get_lcd_ppu_enable(&self.memory));

                /*
                print!(
                    "Benchmarking:\n\
                       Total execution:           {:15} ({:3.3}%)\n\
                       Instruction execution:     {:15} ({:3.3}%)\n\
                       Handle interrupt:          {:15} ({:3.3}%)\n\
                       Event execution:           {:15} ({:3.3}%)\n\
                       Misc execution:            {:15} ({:3.3}%)\n\
                       Handle boot execution:     {:15} ({:3.3}%)\n\
                       Ppu execution:             {:15} ({:3.3}%)\n  \
                       - Draw line execution:   {:15} ({:3.3}%)\n  \
                       - Video execution:       {:15} ({:3.3}%)\n    \
                       - Display screen:      {:15} ({:3.3}%)\n    \
                       - Font display:        {:15} ({:3.3}%)\n",
                    sum_time,
                    sum_time as f64 / sum_time as f64 * 100.0,
                    sum_handle_instruction_time,
                    sum_handle_instruction_time as f64 / sum_time as f64 * 100.0,
                    sum_handle_interrupt_time,
                    sum_handle_interrupt_time as f64 / sum_time as f64 * 100.0,
                    sum_handle_event,
                    sum_handle_event as f64 / sum_time as f64 * 100.0,
                    sum_misc_time,
                    sum_misc_time as f64 / sum_time as f64 * 100.0,
                    sum_handle_boot_image_time,
                    sum_handle_boot_image_time as f64 / sum_time as f64 * 100.0,
                    sum_handle_ppu,
                    sum_handle_ppu as f64 / sum_time as f64 * 100.0,
                    sum_draw_line_time,
                    sum_draw_line_time as f64 / sum_time as f64 * 100.0,
                    sum_update_frame_time,
                    sum_update_frame_time as f64 / sum_time as f64 * 100.0,
                    sum_draw_display_time,
                    sum_draw_display_time as f64 / sum_time as f64 * 100.0,
                    sum_draw_text_time,
                    sum_draw_text_time as f64 / sum_time as f64 * 100.0
                );*/

                println!("Average loop time: {:15}", sum_time / loop_count);
            }

            // ----------------- Benchmarking ------------------- //
            unsafe {
                end = _rdtsc();
            }

            loop_count += 1;
            sum_time += end - start;
            sum_handle_instruction_time += handle_interrupt_time - start;
            sum_handle_interrupt_time += handle_boot_image_time - handle_interrupt_time;
            sum_handle_boot_image_time += ppu_start - handle_boot_image_time;
            sum_handle_ppu += event_time - ppu_start;
            sum_handle_event += misc_stuff_time - event_time;
            sum_misc_time += end - misc_stuff_time;

            if draw_line_time != 0 {
                sum_draw_line_time += draw_line_time - ppu_start;
                draw_line_time = 0;
            }

            if update_frame_time != 0 {
                sum_update_frame_time += update_frame_time2 - update_frame_time;
                sum_draw_display_time += font_time - update_frame_time;
                sum_draw_text_time += update_frame_time2 - font_time;
                update_frame_time = 0;
            }
            // ----------------- Benchmarking ------------------- //
        }
    }
}
