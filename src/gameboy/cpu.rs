use crate::gameboy::instructions;
use crate::gameboy::instructions::Addressing::Register;
use crate::gameboy::instructions::INSTRUCTIONS;
use crate::gameboy::memory::ADDRESS_SPACE;
use crate::gameboy::registers::{Flag, Registers};

use super::instructions::Instruction;

#[derive(Debug)]
pub struct Cpu {
    registers: Registers,
    uptime: u64,
    ime: bool,
    prefixed: bool,
    draw_line: bool,
    draw_image: bool,
    pub loading_boot_image: bool,
    pub load_rom_boot_section: bool,
}

impl Cpu {
    pub fn new() -> Cpu {
        Cpu {
            registers: Registers::new(),
            uptime: 0,
            ime: false,
            prefixed: false,
            draw_line: false,
            draw_image: false,
            loading_boot_image: false,
            load_rom_boot_section: false,
        }
    }

    pub fn get_uptime(&self) -> u64 {
        self.uptime
    }

    pub fn draw_line(&self) -> bool {
        self.draw_line
    }

    pub fn print_status(&self) {
        print!("Cpu status:\n {}\n", self.registers);
        print!(" Cycles: {}\n", self.uptime);
    }

    pub fn read_mem(&self, mem: &[u8; 0x10000], address: u16) -> u8 {
        mem[address as usize]
    }

    pub fn write_mem(&mut self, mem: &mut [u8; 0x10000], address: u16, data: u8) {
        mem[address as usize] = data;
    }

    pub fn execute_single_instruction(&mut self, mem: &mut [u8; 0x10000]) {
        let opcode = self.get_opcode(mem);
        let instruction = self.get_instruction(opcode);
        assert_eq!(
            instruction.opcode,
            opcode,
            "Opcode of instruction does not match array index! {}, instruction.opcode: 0x{:02x}, opcode: 0x{:02x}",
            instruction.name,
            instruction.opcode, opcode
        );
        self.execute_instruction(instruction, mem);
        self.increment_pc(&instruction.op_type, instruction.bytes);
        self.set_flags(&instruction.flags);

        self.draw_line = if ((self.get_uptime() % 456) + instruction.cycles as u64) > 456 {
            true
        } else {
            false
        };

        self.draw_image = if ((self.get_uptime() % 70224) + instruction.cycles as u64) > 70224 {
            true
        } else {
            false
        };

        if self.registers.pc == 0x40 {
            self.loading_boot_image = true;
        }

        if self.registers.pc == 0x100 {
            self.load_rom_boot_section = true;
        }

        self.increase_uptime(instruction.cycles);
    }

    pub fn load_boot_rom(&mut self, boot_rom: Vec<u8>, mem: &mut [u8; 0x10000]) {
        for i in 0..0x100 {
            self.write_mem(mem, i as u16, boot_rom[i]);
        }
    }

    fn get_opcode(&self, mem: &[u8; 0x10000]) -> u8 {
        self.read_mem(mem, self.registers.pc)
    }

    fn get_instruction(&mut self, opcode: u8) -> &'static instructions::Instruction {
        if self.prefixed {
            self.prefixed = false;
            &instructions::PREFIXED_INSTRUCTIONS[opcode as usize]
        } else {
            &instructions::INSTRUCTIONS[opcode as usize]
        }
    }

    fn execute_instruction(&mut self, instruction: &Instruction, mem: &mut [u8; 0x10000]) {
        println!(
            "Executing instruction {} (pc: {:04x}, opcode. {:02x})",
            instruction.name, self.registers.pc, instruction.opcode
        );
        match instruction.op_type {
            instructions::OpType::Load16 => self.load16(instruction, mem),
            instructions::OpType::Load8 => self.load8(instruction, mem),
            instructions::OpType::Xor => self.xor(instruction, mem),
            instructions::OpType::Prefix => self.prefix(instruction),
            instructions::OpType::Bit => self.bit(instruction),
            instructions::OpType::JumpRelative => self.jump_relative(instruction, mem),
            instructions::OpType::Inc8 => self.inc8(instruction),
            instructions::OpType::Dec8 => self.dec8(instruction),
            instructions::OpType::Inc16 => self.inc16(instruction),
            instructions::OpType::Dec16 => self.dec16(instruction),
            instructions::OpType::Call => self.call(instruction, mem),
            instructions::OpType::Ret => self.ret(instruction, mem),
            instructions::OpType::Push => self.push(instruction, mem),
            instructions::OpType::Pop => self.pop(instruction, mem),
            instructions::OpType::RL => self.rl(instruction),
            instructions::OpType::RLA => self.rla(instruction),
            instructions::OpType::Cp => self.cp(instruction, mem),
            instructions::OpType::Sub8 => self.sub8(instruction, mem),
            instructions::OpType::Add8 => self.add8(instruction, mem),
            instructions::OpType::Nop => {
                return;
            }
            instructions::OpType::Jump => self.jump(instruction, mem),
            instructions::OpType::DI => self.di(instruction),
            instructions::OpType::Or => self.or(instruction, mem),
            instructions::OpType::EI => self.ei(instruction),
            instructions::OpType::CPL => self.cpl(instruction),
            instructions::OpType::And => self.and(instruction, mem),
            instructions::OpType::Swap => self.swap(instruction, mem),
            instructions::OpType::RST => self.rst(instruction, mem),
            instructions::OpType::Add16 => self.add16(instruction, mem),
            _ => panic!("Instruction not implemented: {:02x}", instruction.opcode),
        }
    }

    fn increment_pc(&mut self, _optype: &instructions::OpType, value: u8) {
        // For now just increase pc, jmp and call instructions need special treatment!
        self.registers.pc += value as u16;
    }

    fn set_flags(&mut self, flags: &instructions::Flags) {
        match flags.carry {
            instructions::FlagAffected::Set => self.registers.set_flag(Flag::Carry),
            instructions::FlagAffected::Reset => self.registers.reset_flag(Flag::Carry),
            _ => (),
        }
        match flags.half_carry {
            instructions::FlagAffected::Set => self.registers.set_flag(Flag::HalfCarry),
            instructions::FlagAffected::Reset => self.registers.reset_flag(Flag::HalfCarry),
            _ => (),
        }
        match flags.zero {
            instructions::FlagAffected::Set => self.registers.set_flag(Flag::Zero),
            instructions::FlagAffected::Reset => self.registers.reset_flag(Flag::Zero),
            _ => (),
        }
        match flags.negative {
            instructions::FlagAffected::Set => self.registers.set_flag(Flag::Subtraction),
            instructions::FlagAffected::Reset => self.registers.reset_flag(Flag::Subtraction),
            _ => (),
        }
    }

    fn increase_uptime(&mut self, value: u8) {
        self.uptime += value as u64;
    }

    fn get_value16_immediate(&self, mem: &[u8; 0x10000]) -> u16 {
        let upper_byte = self.read_mem(mem, self.registers.pc + 2);
        let lower_byte = self.read_mem(mem, self.registers.pc + 1);
        (upper_byte as u16) << 8 | lower_byte as u16
    }

    fn get_value16(&self, addressing: &instructions::Addressing, mem: &[u8; 0x10000]) -> u16 {
        match addressing {
            instructions::Addressing::Immediate16 => self.get_value16_immediate(mem),
            instructions::Addressing::Register(reg) => match reg {
                instructions::Registers::BC => {
                    ((self.registers.b as u16) << 8) | self.registers.c as u16
                }
                instructions::Registers::DE => {
                    ((self.registers.d as u16) << 8) | self.registers.e as u16
                }
                instructions::Registers::HL => {
                    ((self.registers.h as u16) << 8) | self.registers.l as u16
                }
                instructions::Registers::SP => self.registers.sp,
                _ => panic!("Must be double reg!"),
            },
            _ => panic!("Addressing mode for getValue16 not implemented yet"),
        }
    }

    fn store_value16(&mut self, addressing: &instructions::Addressing, value: u16) {
        match addressing {
            instructions::Addressing::Register(reg) => self.registers.store_16bit_reg(reg, value),
            _ => panic!("Addressing mode for store_value16 not implemented yet"),
        }
    }

    fn get_value8(&mut self, addressing: &instructions::Addressing, mem: &[u8; 0x10000]) -> u8 {
        match addressing {
            instructions::Addressing::Immediate8 => self.read_mem(mem, self.registers.pc + 1),
            instructions::Addressing::Register(reg) => match reg {
                instructions::Registers::A => self.registers.a,
                instructions::Registers::B => self.registers.b,
                instructions::Registers::C => self.registers.c,
                instructions::Registers::D => self.registers.d,
                instructions::Registers::E => self.registers.e,
                instructions::Registers::H => self.registers.h,
                instructions::Registers::L => self.registers.l,
                _ => {
                    panic!("Addressing of register for get_value8 not possible");
                }
            },
            instructions::Addressing::RelativeRegister(reg) => match reg {
                instructions::Registers::C => self.read_mem(mem, 0xff00 + self.registers.c as u16),
                instructions::Registers::BC => self.read_mem(mem, self.registers.get_bc()),
                instructions::Registers::DE => self.read_mem(mem, self.registers.get_de()),
                instructions::Registers::HL => self.read_mem(mem, self.registers.get_hl()),
                instructions::Registers::HlPlus => {
                    let retval = self.read_mem(mem, self.registers.get_hl());
                    self.inc16(&instructions::INSTRUCTIONS[0x23]);
                    retval
                }
                _ => {
                    panic!("get_Value8 for this relative register not implemented");
                }
            },
            instructions::Addressing::RelativeAddress8 => {
                let pc_val: u16 = self.read_mem(mem, self.registers.pc + 1u16) as u16;
                self.read_mem(mem, 0xff00 + pc_val)
            }
            _ => {
                panic!("Addressing mod for getValue16 not implemented yet")
            }
        }
    }

    fn store_value8(
        &mut self,
        addressing: &instructions::Addressing,
        mem: &mut [u8; 0x10000],
        value: u8,
    ) {
        match addressing {
            instructions::Addressing::RelativeRegister(reg) => match reg {
                instructions::Registers::HlMinus => {
                    self.write_mem(mem, self.registers.get_hl(), value);
                    self.dec16(&instructions::INSTRUCTIONS[0x2B]);
                }
                instructions::Registers::HlPlus => {
                    self.write_mem(mem, self.registers.get_hl(), value);
                    self.inc16(&instructions::INSTRUCTIONS[0x23]);
                }
                instructions::Registers::HL => {
                    self.write_mem(mem, self.registers.get_hl(), value);
                }
                instructions::Registers::C => {
                    self.write_mem(mem, self.registers.c as u16 + 0xff00, value);
                }
                _ => panic!("Addressing of register for store_value8 not possible"),
            },
            instructions::Addressing::Register(reg) => match reg {
                instructions::Registers::A => self.registers.a = value,
                instructions::Registers::B => self.registers.b = value,
                instructions::Registers::C => self.registers.c = value,
                instructions::Registers::D => self.registers.d = value,
                instructions::Registers::E => self.registers.e = value,
                instructions::Registers::H => self.registers.h = value,
                instructions::Registers::L => self.registers.l = value,
                _ => panic!("Addressing of register for store_value8 not possible"),
            },
            instructions::Addressing::RelativeAddress8 => {
                let pc_val: u16 = self.read_mem(mem, self.registers.pc + 1) as u16;
                self.write_mem(mem, 0xff00 + pc_val, value);
            }
            instructions::Addressing::RelativeAddress16 => {
                let address: u16 = self.get_value16_immediate(mem);

                self.write_mem(mem, address, value);
            }
            _ => panic!("Addressing mode for store_value16 not implemented yet"),
        }
    }

    fn load16(&mut self, instruction: &Instruction, mem: &mut [u8; 0x10000]) {
        self.store_value16(&instruction.dst, self.get_value16(&instruction.src, mem));
    }

    fn load8(&mut self, instruction: &Instruction, mem: &mut [u8; 0x10000]) {
        let val = self.get_value8(&instruction.src, mem);
        self.store_value8(&instruction.dst, mem, val);
    }

    fn dec16(&mut self, instruction: &Instruction) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => self.registers.dec16(reg),
            _ => panic!("Register not available for dec16!"),
        }
    }

    fn inc16(&mut self, instruction: &Instruction) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => self.registers.inc16(reg),
            _ => panic!("Register not available for dec16!"),
        }
    }

    fn xor(&mut self, instruction: &Instruction, mem: &[u8; 0x10000]) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => match reg {
                instructions::Registers::A => self.registers.a ^= self.registers.a,
                instructions::Registers::B => self.registers.a ^= self.registers.b,
                instructions::Registers::C => self.registers.a ^= self.registers.c,
                instructions::Registers::D => self.registers.a ^= self.registers.d,
                instructions::Registers::E => self.registers.a ^= self.registers.e,
                instructions::Registers::H => self.registers.a ^= self.registers.h,
                instructions::Registers::L => self.registers.a ^= self.registers.l,
                _ => panic!("Addressing of register for xor not possible"),
            },
            instructions::Addressing::Immediate8 => {
                self.registers.a ^= self.read_mem(mem, self.registers.pc + 1);
            }
            _ => panic!("Addressing mode for xor not implemented yet"),
        }

        self.registers.reset_flag(Flag::Carry);
        self.registers.reset_flag(Flag::HalfCarry);
        self.registers.reset_flag(Flag::Subtraction);
        if self.registers.a == 0 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }
    }

    fn prefix(&mut self, _instruction: &Instruction) {
        self.prefixed = true;
    }

    // Tests if bit is set
    fn bit(&mut self, instruction: &Instruction) {
        let mut set: bool = false;
        match &instruction.src {
            instructions::Addressing::Register(reg) => match &instruction.dst {
                instructions::Addressing::Bit(bit) => set = self.registers.check_bit(reg, bit),
                _ => panic!("Destination of bit() function must be a bit!"),
            },
            _ => panic!("Must be register"),
        }

        if set {
            self.registers.reset_flag(Flag::Zero);
        } else {
            self.registers.set_flag(Flag::Zero);
        }
        self.registers.reset_flag(Flag::Subtraction);
        self.registers.set_flag(Flag::HalfCarry);
    }

    fn jump_relative(&mut self, instruction: &Instruction, mem: &[u8; 0x10000]) {
        match instruction.src {
            instructions::Addressing::RelativeAddress8 => match &instruction.dst {
                instructions::Addressing::Flag(flag) => match flag {
                    instructions::FlagsEnum::NonZero => {
                        if !self.registers.get_flag(Flag::Zero) {
                            let signed: i8 = self.read_mem(mem, self.registers.pc + 1) as i8;
                            if signed < 0 {
                                self.registers.pc -= signed.abs() as u16;
                            } else {
                                self.registers.pc += signed.abs() as u16;
                            }
                        }
                    }
                    instructions::FlagsEnum::Zero => {
                        if self.registers.get_flag(Flag::Zero) {
                            let signed: i8 = self.read_mem(mem, self.registers.pc + 1) as i8;
                            if signed < 0 {
                                self.registers.pc -= signed.abs() as u16;
                            } else {
                                self.registers.pc += signed.abs() as u16;
                            }
                        }
                    }
                    _ => panic!("Flag for JR not implemented yet!"),
                },
                instructions::Addressing::None => {
                    let signed: i8 = self.read_mem(mem, self.registers.pc + 1) as i8;
                    if signed < 0 {
                        self.registers.pc -= signed.abs() as u16;
                    } else {
                        self.registers.pc += signed.abs() as u16;
                    }
                }
                _ => panic!("addressing mode dst of JR instructions has to be flag"),
            },
            _ => panic!("addressing mode of source for JR instructions has to be RelativeAddress8"),
        }
    }

    fn inc8(&mut self, instruction: &Instruction) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => {
                if (((*self.registers.get_reg_ref(*reg) & 0xf) + (1 & 0xf)) & 0x10) == 0x10 {
                    self.registers.set_flag(Flag::HalfCarry);
                } else {
                    self.registers.reset_flag(Flag::HalfCarry);
                }

                *self.registers.get_reg_ref(*reg) =
                    (*self.registers.get_reg_ref(*reg)).wrapping_add(1);

                if *self.registers.get_reg_ref(*reg) == 0 {
                    self.registers.set_flag(Flag::Zero);
                } else {
                    self.registers.reset_flag(Flag::Zero);
                }
            }
            _ => panic!(),
        }
    }

    fn dec8(&mut self, instruction: &Instruction) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => {
                if (((*self.registers.get_reg_ref(*reg) & 0xf) + (1 & 0xf)) & 0x10) == 0x10 {
                    self.registers.set_flag(Flag::HalfCarry);
                } else {
                    self.registers.reset_flag(Flag::HalfCarry);
                }

                *self.registers.get_reg_ref(*reg) =
                    (*self.registers.get_reg_ref(*reg)).wrapping_sub(1);

                if *self.registers.get_reg_ref(*reg) == 0 {
                    self.registers.set_flag(Flag::Zero);
                } else {
                    self.registers.reset_flag(Flag::Zero);
                }
            }
            _ => panic!(),
        }
    }

    fn make_call(&mut self, mem: &mut [u8; 0x10000]) {
        self.registers.sp -= 2;
        self.write_mem(mem, self.registers.sp, (self.registers.pc & 0xff) as u8);
        self.write_mem(mem, self.registers.sp + 1, (self.registers.pc >> 8) as u8);
        // -3 because this gets added when pc is defaultly increased
        self.registers.pc = self.get_value16_immediate(mem) - 3;
    }

    fn call(&mut self, instruction: &Instruction, mem: &mut [u8; 0x10000]) {
        let mut make_call: bool = false;
        match instruction.src {
            instructions::Addressing::Address16 => match instruction.dst {
                instructions::Addressing::None => {
                    make_call = true;
                }
                _ => panic!("addressing mode not implemented"),
            },
            _ => panic!("addressing mode not implemented"),
        }

        if make_call {
            self.make_call(mem);
        }
    }

    fn ret(&mut self, instruction: &Instruction, mem: &[u8; 0x10000]) {
        match instruction.dst {
            instructions::Addressing::None => {
                if instruction.opcode != 0xc9 as u8 {
                    panic!("With this addressing function hast to be 0xc9");
                }

                let upper_byte = self.read_mem(mem, self.registers.sp + 1);
                let lower_byte = self.read_mem(mem, self.registers.sp);
                self.registers.pc = (upper_byte as u16) << 8 | lower_byte as u16;
                // Prev instruction also has to be skipped
                // TODO: look into this
                self.registers.pc += 2;
                self.registers.sp += 2;
            }
            _ => panic!("Not implemented yet"),
        }
    }

    fn push(&mut self, instruction: &Instruction, mem: &mut [u8; 0x10000]) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => match reg {
                instructions::Registers::BC => {
                    self.registers.sp -= 2;
                    self.write_mem(mem, self.registers.sp + 1, self.registers.b);
                    self.write_mem(mem, self.registers.sp, self.registers.c);
                }
                instructions::Registers::DE => {
                    self.registers.sp -= 2;
                    self.write_mem(mem, self.registers.sp + 1, self.registers.d);
                    self.write_mem(mem, self.registers.sp, self.registers.e);
                }
                instructions::Registers::HL => {
                    self.registers.sp -= 2;
                    self.write_mem(mem, self.registers.sp + 1, self.registers.h);
                    self.write_mem(mem, self.registers.sp, self.registers.l);
                }
                instructions::Registers::AF => {
                    self.registers.sp -= 2;
                    self.write_mem(mem, self.registers.sp + 1, self.registers.a);
                    self.write_mem(mem, self.registers.sp, self.registers.f);
                }
                _ => panic!("Destination hast to be Double Register"),
            },
            _ => panic!("Destination hast to be Register"),
        }
    }

    fn pop(&mut self, instruction: &Instruction, mem: &[u8; 0x10000]) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => match reg {
                instructions::Registers::BC => {
                    self.registers.b = self.read_mem(mem, self.registers.sp + 1);
                    self.registers.c = self.read_mem(mem, self.registers.sp);
                    self.registers.sp += 2;
                }
                instructions::Registers::DE => {
                    self.registers.d = self.read_mem(mem, self.registers.sp + 1);
                    self.registers.e = self.read_mem(mem, self.registers.sp);
                    self.registers.sp += 2;
                }
                instructions::Registers::HL => {
                    self.registers.h = self.read_mem(mem, self.registers.sp + 1);
                    self.registers.l = self.read_mem(mem, self.registers.sp);
                    self.registers.sp += 2;
                }
                instructions::Registers::AF => {
                    self.registers.a = self.read_mem(mem, self.registers.sp + 1);
                    self.registers.f = self.read_mem(mem, self.registers.sp);
                    self.registers.sp += 2;
                }
                _ => panic!("Destination hast to be Double Register"),
            },
            _ => panic!("destination has to be Register!"),
        }
    }

    fn rl(&mut self, instruction: &Instruction) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => {
                if self
                    .registers
                    .rl_reg(reg, self.registers.get_flag(Flag::Carry))
                {
                    self.registers.set_flag(Flag::Carry)
                } else {
                    self.registers.reset_flag(Flag::Carry);
                }

                if *self.registers.get_reg_ref(*reg) == 0 {
                    self.registers.set_flag(Flag::Zero);
                } else {
                    self.registers.reset_flag(Flag::Zero);
                }
            }
            _ => panic!("Rotate left only available for Registers"),
        }
    }

    fn rla(&mut self, instruction: &Instruction) {
        assert_eq!(
            instruction.opcode, 0x17,
            "rla instruction has to be opcode 0x17"
        );
        self.rl(&instructions::PREFIXED_INSTRUCTIONS[0x17 as usize]);
    }

    fn cp(&mut self, instruction: &Instruction, mem: &[u8; 0x10000]) {
        let val: u8;
        match instruction.dst {
            instructions::Addressing::Immediate8
            | instructions::Addressing::Register(_)
            | instructions::Addressing::RelativeRegister(instructions::Registers::HL) => {
                val = self.get_value8(&instruction.dst, mem)
            }
            _ => {
                panic!("addressing not implemented for compare");
            }
        }

        if val > self.registers.a {
            self.registers.set_flag(Flag::Carry);
        } else {
            self.registers.reset_flag(Flag::Carry);
        }

        // TODO: Check if this is the right operation for half carry
        if (((val & 0xf) + (self.registers.a & 0xf)) & 0x10) == 0x10 {
            self.registers.set_flag(Flag::HalfCarry);
        } else {
            self.registers.reset_flag(Flag::HalfCarry);
        }

        let res = self.registers.a.wrapping_sub(val);

        if res == 0 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }
    }

    fn sub8(&mut self, instruction: &Instruction, mem: &[u8; ADDRESS_SPACE]) {
        let val = self.get_value8(&instruction.dst, mem);

        if val > self.registers.a {
            self.registers.set_flag(Flag::Carry);
        } else {
            self.registers.reset_flag(Flag::Carry);
        }

        // TODO: Check if this is the right operation for half carry
        if (((val & 0xf) + (self.registers.a & 0xf)) & 0x10) == 0x10 {
            self.registers.set_flag(Flag::HalfCarry);
        } else {
            self.registers.reset_flag(Flag::HalfCarry);
        }

        let res = self.registers.a.wrapping_sub(val);

        if res == 0 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }

        self.registers.a = res;
    }

    fn add8(&mut self, instruction: &Instruction, mem: &mut [u8; ADDRESS_SPACE]) {
        let dst = self.get_value8(&instruction.dst, mem);
        let src = self.get_value8(&instruction.src, mem);

        if dst > (255 - src) {
            self.registers.set_flag(Flag::Carry);
        } else {
            self.registers.reset_flag(Flag::Carry);
        }

        if (((src & 0xf) + (dst & 0xf)) & 0x10) == 0x10 {
            self.registers.set_flag(Flag::HalfCarry);
        } else {
            self.registers.reset_flag(Flag::HalfCarry);
        }

        let result = dst.wrapping_add(src);

        if result == 0 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }

        self.store_value8(&instruction.dst, mem, result);
    }

    fn jump(&mut self, instructions: &Instruction, mem: &[u8; ADDRESS_SPACE]) {
        /*assert_eq!(
            instructions.opcode, 0xc3,
            "Wrong opcode for jump instruction!"
        );*/

        match instructions.dst {
            instructions::Addressing::Address16 => {
                self.registers.pc = self.get_value16_immediate(mem) - instructions.bytes as u16;
            }
            instructions::Addressing::RelativeRegister(reg) => match reg {
                instructions::Registers::HL => {
                    self.registers.pc = ((self.registers.h as u16) << 8 | self.registers.l as u16)
                        - instructions.bytes as u16;
                }
                _ => panic!("Not implemented for jump"),
            },
            _ => panic!("Not implemented for jump"),
        }
    }

    fn di(&mut self, instruction: &Instruction) {
        assert_eq!(instruction.opcode, 0xf3, "Wrong opcode for DI");
        self.ime = false;
    }

    fn or(&mut self, instruction: &Instruction, mem: &[u8; ADDRESS_SPACE]) {
        self.registers.a |= self.get_value8(&instruction.dst, mem);

        if self.registers.a == 0 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }
    }

    fn ei(&mut self, instruction: &Instruction) {
        assert_eq!(instruction.opcode, 0xfb, "Wrong opcode for EI");
        self.ime = true;
    }

    fn cpl(&mut self, instruction: &Instruction) {
        assert_eq!(instruction.opcode, 0x2f, "Wrong opcode for CPL");
        self.registers.a ^= 0xff;
    }

    fn and(&mut self, instruction: &Instruction, mem: &[u8; ADDRESS_SPACE]) {
        let val = self.get_value8(&instruction.dst, mem);

        self.registers.a &= val;

        if self.registers.a == 0 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }
    }

    fn swap(&mut self, instruction: &Instruction, mem: &mut [u8; ADDRESS_SPACE]) {
        let mut val = self.get_value8(&instruction.dst, mem);
        val = ((val & 0x0f) << 4) | ((val & 0xf0) >> 4);
        self.store_value8(&instruction.dst, mem, val);
    }

    fn rst(&mut self, instruction: &Instruction, mem: &mut [u8; ADDRESS_SPACE]) {
        let mut target: u16 = 0x00;
        match &instruction.dst {
            instructions::Addressing::RstAddr(rstaddr) => target = *rstaddr,
            _ => panic!("addressing mode not implemented for rst instruction"),
        }

        self.registers.sp -= 2;
        self.write_mem(mem, self.registers.sp, (self.registers.pc & 0xff) as u8);
        self.write_mem(mem, self.registers.sp + 1, (self.registers.pc >> 8) as u8);
        // -1 for instruction size
        self.registers.pc = target - 1;

        self.print_status();
    }

    fn add16(&mut self, instruction: &Instruction, mem: &mut [u8; ADDRESS_SPACE]) {
        let src = self.get_value16(&instruction.src, mem);
        let mut dst = self.get_value16(&instruction.dst, mem);

        if (dst as u32 + src as u32) > u16::MAX as u32 {
            self.registers.set_flag(Flag::Carry);
        } else {
            self.registers.reset_flag(Flag::Carry);
        }

        if (((src & 0xf) + (dst & 0xf)) & 0x10) == 0x10 {
            self.registers.set_flag(Flag::HalfCarry);
        } else {
            self.registers.reset_flag(Flag::HalfCarry);
        }

        dst = src.wrapping_add(dst);

        self.store_value16(&instruction.dst, dst);
    }
}
