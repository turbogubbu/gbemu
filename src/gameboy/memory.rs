use crate::gameboy::apu::Apu;
use crate::gameboy::joypad_input::JoypadInput;
use crate::gameboy::timer::Timer;

#[derive(Debug, PartialEq)]
enum CartridgeType {
    RomOnly,
    NotImplemented,
}

pub const JOYPAD_INPUT_ADDRESS: u16 = 0xff00;
const TIM_DIV_COUNTER: u16 = 0xff04;
const TIM_A_COUNTER: u16 = 0xff05;
const CPU_FREQ: u32 = 1024 * 1024;
const TIM_A_MODULO: u16 = 0xff06;
const TIM_A_CONTROL: u16 = 0xff07;

const INTERRUPT_FLAG_REG: u16 = 0xff0f;

const NR10_CHANNEL1_SWEEP: u16 = 0xff10;
const NR11_CHANNEL1_LENGTH_TIMER_DUTY_CYCLE: u16 = 0xff11;
const NR12_CHANNEL1_VOLUME_ENVELOPE: u16 = 0xff12;
const NR13_CHANNEL1_PERIOD_LOW: u16 = 0xff13;
const NR14_CHANNEL1_PERIOD_HIGH_CONTORL: u16 = 0xff14;
const NR50_MASTER_VOLUME: u16 = 0xff24;
const NR51_SOUND_PANNING: u16 = 0xff25;
const NR52_AUDIO_MASTER_CONTROL: u16 = 0xff26;

const OAM_DMA_ADDRESS: u16 = 0xff46;

const TIM_DIV_FREQ: u32 = 16384;

#[derive(Debug)]
pub struct Memory {
    pub data: [u8; ADDRESS_SPACE],
    pub joypad_input: JoypadInput,
    cartridge_type: CartridgeType,
    timer_div: Timer,
    timer_a: Timer,
    apu: Apu,
}

pub const ADDRESS_SPACE: usize = 0x10000;

impl Memory {
    pub fn new() -> Memory {
        Memory {
            data: [0; ADDRESS_SPACE],
            joypad_input: JoypadInput::new(),
            cartridge_type: CartridgeType::NotImplemented,
            timer_div: Timer::new(CPU_FREQ / TIM_DIV_FREQ, true),
            timer_a: Timer::new(0, false),
            apu: Apu::new(),
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
        self.data[JOYPAD_INPUT_ADDRESS as usize] = 0x0f;
    }

    #[allow(dead_code)]
    fn get_input_reg(&self) -> u8 {
        let higher_nibble = self.data[0xff00_usize] >> 4;

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
            JOYPAD_INPUT_ADDRESS => self.get_input_reg(),
            // normal memory
            _ => self.data[address as usize],
        }
    }

    fn oam_dma(&mut self, start_addr: u8) {
        assert!(
            start_addr < 0xe0,
            "OAM dma only available from address 0x0000 to 0xdf00"
        );

        let real_start_address: usize = (start_addr as usize) << 8;

        for i in 0..0xa0 {
            self.data[0xfe00 + i] = self.data[real_start_address + i];
        }
    }

    fn check_channel1_trigger(&mut self, val: u8) {
        if self.data[NR52_AUDIO_MASTER_CONTROL as usize] & 0x80 != 0x80 {
            return;
        }

        if val & 0x80 == 0x80 {
            self.apu.trigger_channel_1(
                self.data[NR14_CHANNEL1_PERIOD_HIGH_CONTORL as usize],
                self.data[NR13_CHANNEL1_PERIOD_LOW as usize],
                self.data[NR12_CHANNEL1_VOLUME_ENVELOPE as usize],
                self.data[NR11_CHANNEL1_LENGTH_TIMER_DUTY_CYCLE as usize],
                self.data[NR10_CHANNEL1_SWEEP as usize],
                self.data[TIM_DIV_COUNTER as usize],
            );
        }
    }

    pub fn write_mem(&mut self, address: u16, value: u8) {
        if self.cartridge_type == CartridgeType::RomOnly && address <= 0x3fff {
            return;
        }

        match address {
            OAM_DMA_ADDRESS => self.oam_dma(value),
            TIM_DIV_COUNTER => {
                self.data[TIM_DIV_COUNTER as usize] = 0;
                return;
            } // Writing to this
            // registers resets it
            TIM_A_MODULO => {
                self.timer_a.update_overflow_val(value);
            }
            TIM_A_CONTROL => {
                self.config_tima(value & 0x40 == 0x40, value & 0x03);
            }
            NR14_CHANNEL1_PERIOD_HIGH_CONTORL => {
                self.check_channel1_trigger(value);
            }

            _ => {}
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
                println!();
            }
        }

        println!("\n");
    }

    fn config_tima(&mut self, enable: bool, clock_select: u8) {
        if enable {
            self.timer_a.enable();
        } else {
            self.timer_a.disable();
        }

        // https://gbdev.io/pandocs/Timer_and_Divider_Registers.html#timer-and-divider-registers
        let period = match clock_select {
            0b00 => 256,
            0b01 => 4,
            0b10 => 16,
            0b11 => 64,
            _ => panic!("clock select should only be 3 bit!"),
        };

        self.timer_a.update_period(period);
    }

    pub fn print_ie_register(&self) {
        println!("IE register: 0x{:02x}", self.data[0xffff]);
    }

    pub fn print_interrupt_flag_register(&self) {
        println!("Interrupt flag register: 0x{:02x}", self.data[0xff0f]);
    }

    pub fn handle_timer(&mut self, current_cycle: u64) {
        self.timer_div
            .increment(current_cycle, &mut self.data[TIM_DIV_COUNTER as usize]);

        let before = self.data[TIM_DIV_COUNTER as usize];

        if self
            .timer_div
            .increment(current_cycle, &mut self.data[TIM_A_COUNTER as usize])
        {
            self.data[INTERRUPT_FLAG_REG as usize] |= 0x04;
        }

        // When div tim incremented, update the audio stuff
        if before != self.data[TIM_DIV_COUNTER as usize] {
            self.apu
                .update_channel_1(self.data[TIM_DIV_COUNTER as usize]);
        }
    }
}
