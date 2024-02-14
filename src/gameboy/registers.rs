use std::fmt::Display;

use crate::gameboy::instructions;

#[derive(Debug)]
pub struct Registers {
    pub a: u8,
    pub f: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub e: u8,
    pub h: u8,
    pub l: u8,
    pub sp: u16,
    pub pc: u16,
}

impl Display for Registers {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Registers {{\n\ta:  0x{:02x} f: 0x{:02x}    Flags(Zero: {}, Subtraction: {}, HalfCarry: {}, Carry: {})\n\
            \tb:  0x{:02x} c: 0x{:02x}\n\
            \td:  0x{:02x} e: 0x{:02x}\n\
            \th:  0x{:02x} l: 0x{:02x}\n\
            \tsp: 0x{:04x}\n\
            \tpc: 0x{:04x}\n\
            }}",
            self.a, self.f, self.get_flag(Flag::Zero), self.get_flag(Flag::Subtraction), self.get_flag(Flag::HalfCarry), self.get_flag(Flag::Carry), self.b, self.c, self.d, self.e, self.h, self.l, self.sp, self.pc
        )
    }
}

#[derive(Debug)]
pub enum Flag {
    Zero,
    Subtraction,
    HalfCarry,
    Carry,
}

impl Registers {
    pub fn new() -> Registers {
        Registers {
            a: 0,
            f: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            l: 0,
            h: 0,
            sp: 0,
            pc: 0,
        }
    }

    pub fn new_after_boot_rom() -> Registers {
        Registers {
            a: 0x01,
            f: 0xb0,
            b: 0x00,
            c: 0x13,
            d: 0x00,
            e: 0xd8,
            h: 0x01,
            l: 0x4d,
            sp: 0xfffe,
            pc: 0x0100,
        }
    }

    pub fn store_16bit_reg(&mut self, reg: &instructions::Registers, value: u16) {
        match reg {
            instructions::Registers::AF => self.store_af(value),
            instructions::Registers::BC => self.store_bc(value),
            instructions::Registers::DE => self.store_de(value),
            instructions::Registers::HL => self.store_hl(value),
            instructions::Registers::SP => self.sp = value,
            _ => assert!(false, "Wrong register to store 16 bit value!"),
        }
    }

    #[allow(dead_code)]
    pub fn get_af(&self) -> u16 {
        ((self.a as u16) << 8) | (self.f as u16)
    }

    pub fn get_bc(&self) -> u16 {
        ((self.b as u16) << 8) | (self.c as u16)
    }

    pub fn get_de(&self) -> u16 {
        ((self.d as u16) << 8) | (self.e as u16)
    }

    pub fn get_hl(&self) -> u16 {
        ((self.h as u16) << 8) | (self.l as u16)
    }

    pub fn store_af(&mut self, value: u16) {
        self.a = (value >> 8) as u8;
        self.f = (value & 0x00ff) as u8;
    }

    pub fn store_bc(&mut self, value: u16) {
        self.b = (value >> 8) as u8;
        self.c = (value & 0x00ff) as u8;
    }

    pub fn store_de(&mut self, value: u16) {
        self.d = (value >> 8) as u8;
        self.e = (value & 0x00ff) as u8;
    }

    pub fn store_hl(&mut self, value: u16) {
        self.h = (value >> 8) as u8;
        self.l = (value & 0x00ff) as u8;
    }

    pub fn set_flag(&mut self, flag: Flag) {
        match flag {
            Flag::Zero => {
                self.f |= 1 << 7;
            }
            Flag::Subtraction => {
                self.f |= 1 << 6;
            }
            Flag::HalfCarry => {
                self.f |= 1 << 5;
            }
            Flag::Carry => {
                self.f |= 1 << 4;
            }
        }
    }

    pub fn get_reg_val(&self, reg: instructions::Registers) -> u8 {
        match reg {
            instructions::Registers::A => self.a,
            instructions::Registers::B => self.b,
            instructions::Registers::C => self.c,
            instructions::Registers::D => self.d,
            instructions::Registers::E => self.e,
            instructions::Registers::H => self.h,
            instructions::Registers::L => self.l,
            _ => {
                panic!("Cannot get reg ref");
            }
        }
    }

    pub fn get_reg_ref(&mut self, reg: instructions::Registers) -> &mut u8 {
        match reg {
            instructions::Registers::A => &mut self.a,
            instructions::Registers::B => &mut self.b,
            instructions::Registers::C => &mut self.c,
            instructions::Registers::D => &mut self.d,
            instructions::Registers::E => &mut self.e,
            instructions::Registers::H => &mut self.h,
            instructions::Registers::L => &mut self.l,
            _ => {
                assert!(false, "Cannot get reg ref");
                &mut self.a
            }
        }
    }

    pub fn reset_flag(&mut self, flag: Flag) {
        match flag {
            Flag::Zero => {
                self.f &= 0b0111_1111;
            }
            Flag::Subtraction => {
                self.f &= 0b1011_1111;
            }
            Flag::HalfCarry => {
                self.f &= 0b1101_1111;
            }
            Flag::Carry => {
                self.f &= 0b1110_1111;
            }
        }
    }

    pub fn get_flag(&self, flag: Flag) -> bool {
        match flag {
            Flag::Zero => {
                if (self.f & 1 << 7) == 1 << 7 {
                    true
                } else {
                    false
                }
            }
            Flag::Subtraction => {
                if (self.f & 1 << 6) == 1 << 6 {
                    true
                } else {
                    false
                }
            }
            Flag::HalfCarry => {
                if (self.f & 1 << 5) == 1 << 5 {
                    true
                } else {
                    false
                }
            }
            Flag::Carry => {
                if self.f & 1 << 4 == 1 << 4 {
                    true
                } else {
                    false
                }
            }
        }
    }

    pub fn dec16(&mut self, reg: &instructions::Registers) {
        match reg {
            instructions::Registers::BC => {
                if self.c == 0 {
                    self.b = self.b.wrapping_sub(1);
                    self.c = 255;
                } else {
                    self.c -= 1;
                }
            }
            instructions::Registers::DE => {
                if self.e == 0 {
                    self.d = self.d.wrapping_sub(1);
                    self.e = 255;
                } else {
                    self.e -= 1;
                }
            }
            instructions::Registers::HL => {
                if self.l == 0 {
                    self.h = self.h.wrapping_sub(1);
                    self.l = 255;
                } else {
                    self.l -= 1;
                }
            }
            instructions::Registers::SP => {
                self.sp = self.sp.wrapping_sub(1);
            }
            _ => panic!("Dec 16 of register {:?} not available", reg),
        }
    }

    pub fn inc16(&mut self, reg: &instructions::Registers) {
        match reg {
            instructions::Registers::BC => {
                if self.c == 255 {
                    self.b = self.b.wrapping_add(1);
                    self.c = 0;
                } else {
                    self.c += 1;
                }
            }
            instructions::Registers::DE => {
                if self.e == 255 {
                    self.d = self.d.wrapping_add(1);
                    self.e = 0;
                } else {
                    self.e += 1;
                }
            }
            instructions::Registers::HL => {
                if self.l == 255 {
                    self.h = self.h.wrapping_add(1);
                    self.l = 0;
                } else {
                    self.l += 1;
                }
            }
            instructions::Registers::SP => {
                self.sp = self.sp.wrapping_add(1);
            }
            _ => assert!(false, "Dec 16 of register not available"),
        }
    }

    pub fn check_bit(&self, reg: &instructions::Registers, bit: &u8) -> bool {
        match reg {
            instructions::Registers::A => self.a & (1 << bit) != 0,
            instructions::Registers::B => self.b & (1 << bit) != 0,
            instructions::Registers::C => self.c & (1 << bit) != 0,
            instructions::Registers::D => self.d & (1 << bit) != 0,
            instructions::Registers::E => self.e & (1 << bit) != 0,
            instructions::Registers::H => self.h & (1 << bit) != 0,
            instructions::Registers::L => self.l & (1 << bit) != 0,
            _ => {
                assert!(false, "Register not implemented for Bit");
                false
            }
        }
    }

    pub fn rl_reg(&mut self, reg: &instructions::Registers, carry: bool) -> bool {
        let val: &mut u8 = self.get_reg_ref(*reg);
        let tmp: u8 = *val;
        *val <<= 1;

        if carry {
            *val |= 0x1;
        }

        if tmp & 0x80 != 0 {
            true
        } else {
            false
        }
    }

    pub fn rr_reg(&mut self, reg: &instructions::Registers, carry: bool) -> bool {
        let val: &mut u8 = self.get_reg_ref(*reg);
        let tmp: u8 = *val;
        *val >>= 1;

        if carry {
            *val |= 0x80;
        }

        if tmp & 0x01 != 0 {
            true
        } else {
            false
        }
    }

    // shift register left arithmtically
    // retval: if zero flag should be set
    pub fn sla_reg(&mut self, reg: &instructions::Registers) -> bool {
        let val: &mut u8 = self.get_reg_ref(*reg);

        *val <<= 1;

        if *val == 0 {
            true
        } else {
            false
        }
    }

    pub fn srl_reg(&mut self, reg: &instructions::Registers) -> bool {
        let val: &mut u8 = self.get_reg_ref(*reg);

        *val >>= 1;

        if *val == 0 {
            true
        } else {
            false
        }
    }

    pub fn reset_bit(&mut self, reg: &instructions::Registers, bit: &u8) {
        match reg {
            instructions::Registers::A => self.a &= !(1 << bit),
            instructions::Registers::B => self.b &= !(1 << bit),
            instructions::Registers::C => self.c &= !(1 << bit),
            instructions::Registers::D => self.d &= !(1 << bit),
            instructions::Registers::E => self.e &= !(1 << bit),
            instructions::Registers::H => self.h &= !(1 << bit),
            instructions::Registers::L => self.l &= !(1 << bit),
            _ => {
                panic!("Register not implemented to reset bit!");
            }
        };
    }

    pub fn set_bit(&mut self, reg: &instructions::Registers, bit: &u8) {
        match reg {
            instructions::Registers::A => self.a |= 1 << bit,
            instructions::Registers::B => self.b |= 1 << bit,
            instructions::Registers::C => self.c |= 1 << bit,
            instructions::Registers::D => self.d |= 1 << bit,
            instructions::Registers::E => self.e |= 1 << bit,
            instructions::Registers::H => self.h |= 1 << bit,
            instructions::Registers::L => self.l |= 1 << bit,
            _ => {
                panic!("Register not implemented to reset bit!");
            }
        };
    }
}
