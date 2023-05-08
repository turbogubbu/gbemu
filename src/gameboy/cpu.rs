//use crate::cpu::instructions;

use crate::gameboy::instructions;
use crate::gameboy::memory::Memory;
use crate::gameboy::registers::{Registers, Flag};

use super::instructions::Instruction;

#[derive(Debug)]
pub struct Cpu {
    registers: Registers,
    address_space: Vec<u8>,
    uptime: u64,
    prefixed: bool,
    mem: Memory,
}

impl Cpu {
    pub fn new() -> Cpu {
        Cpu {
            registers: Registers::new(),
            address_space: vec![0; 0x10000], //Vec::<u8>::new(),
            uptime: 0,
            prefixed: false,
            mem: Memory::new(),
        }
    }

    pub fn print(&self, start_address: u16, len: u16) {
        println!("        00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f");

        let start: u16 = start_address - start_address % 16;
        let end: u16 = start + start_address % 16 + len;

        for i in start..end {
            if i % 16 == 0 {
                print!("0x{:04x}  ", i);
            }

            print!("{:02x} ", self.read_mem(i));

            if (i + 1) % 16 == 0 {
                print!("\n");
            }
        }

        println!("\n");
    }

    pub fn print_status(&self) {
        print!("Cpu status:\n {}\n", self.registers);
        print!(" Cycles: {}\n", self.uptime);
    }

    pub fn read_mem(&self, address: u16) -> u8 {
        self.mem.read(address)
    }

    pub fn write_mem(&mut self, address: u16, data: u8) {
        self.mem.write(address, data);
    }

    pub fn execute_single_instruction(&mut self) {
        let opcode = self.get_opcode();
        let instruction = self.get_instruction(opcode);
        assert_eq!(
            instruction.opcode,
            opcode,
            "Opcode of instruction does not match array index! {}, instruction.opcode: 0x{:02x}, opcode: 0x{:02x}",
            instruction.name,
            instruction.opcode, opcode
        );
        self.execute_instruction(instruction);
        self.increment_pc(&instruction.op_type, instruction.bytes);
        self.set_flags(&instruction.flags);
        self.increase_uptime(instruction.cycles);
    }

    pub fn load_boot_rom(&mut self, boot_rom: Vec<u8>) {
        for i in 0..boot_rom.len() {
            self.write_mem(i as u16, boot_rom[i]);
        }
    }

    fn get_opcode(&self) -> u8 {
        self.read_mem(self.registers.pc)
    }

    fn get_instruction(&mut self, opcode: u8) -> &'static instructions::Instruction {
        if self.prefixed {
            self.prefixed = false;
            &instructions::PREFIXED_INSTRUCTIONS[opcode as usize]
        } else {
            &instructions::INSTRUCTIONS[opcode as usize]
        }
    }

    fn execute_instruction(&mut self, instruction: &Instruction) {
        println!(
            "Executing instruction {} (pc: {:04x}, opcode. {:02x})",
            instruction.name, self.registers.pc, instruction.opcode
        );
        match instruction.op_type {
            instructions::OpType::Load16 => self.load16(instruction),
            instructions::OpType::Load8 => self.load8(instruction),
            instructions::OpType::Xor => self.xor(instruction),
            instructions::OpType::Prefix => self.prefix(instruction),
            instructions::OpType::Bit => self.bit(instruction),
            instructions::OpType::JumpRelative => self.jump_relative(instruction),
            instructions::OpType::Inc8 => self.inc8(instruction),
            instructions::OpType::Dec8 => self.dec8(instruction),
            instructions::OpType::Inc16 => self.inc16(instruction),
            instructions::OpType::Dec16 => self.dec16(instruction),
            instructions::OpType::Call => self.call(instruction),
            instructions::OpType::Ret => self.ret(instruction),
            instructions::OpType::Push => self.push(instruction),
            instructions::OpType::Pop => self.pop(instruction),
            instructions::OpType::RL => self.rl(instruction),
            instructions::OpType::RLA => self.rla(instruction),
            instructions::OpType::Cp => self.cp(instruction),
            _ => panic!(),
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

    fn get_value16_immediate(&self) -> u16 {
        let upper_byte = self.read_mem(self.registers.pc + 2);
        let lower_byte = self.read_mem(self.registers.pc + 1);
        (upper_byte as u16) << 8 | lower_byte as u16
    }

    fn get_value16(&self, addressing: &instructions::Addressing) -> u16 {
        match addressing {
            instructions::Addressing::Immediate16 => self.get_value16_immediate(),
            _ => {
                assert!(false, "Adressing mod for getValue16 not implemented yet");
                16u16
            }
        }
    }

    fn store_value16(&mut self, addressing: &instructions::Addressing, value: u16) {
        match addressing {
            instructions::Addressing::Register(reg) => self.registers.store_16bit_reg(reg, value),
            _ => assert!(
                false,
                "Adderssing mode for store_value16 not implemented yet"
            ),
        }
    }

    fn get_value8(&self, addressing: &instructions::Addressing) -> u8 {
        match addressing {
            instructions::Addressing::Immediate8 => {
                self.read_mem(self.registers.pc + 1)
            }
            instructions::Addressing::Register(reg) => match reg {
                instructions::Registers::A => self.registers.a,
                instructions::Registers::B => self.registers.b,
                instructions::Registers::C => self.registers.c,
                instructions::Registers::D => self.registers.d,
                instructions::Registers::E => self.registers.e,
                instructions::Registers::H => self.registers.h,
                instructions::Registers::L => self.registers.l,
                _ => {
                    assert!(false, "Addressing of register for get_value8 not possible");
                    1u8
                }
            },
            instructions::Addressing::RelativeRegister(reg) => match reg {
                instructions::Registers::C => {
                    self.read_mem(0xff00 + self.registers.c as u16)
                }
                instructions::Registers::BC => self.read_mem(self.registers.get_bc()),
                instructions::Registers::DE => self.read_mem(self.registers.get_de()),
                instructions::Registers::HL => self.read_mem(self.registers.get_hl()),
                _ => {
                    assert!(false);
                    0u8
                }
            },
            instructions::Addressing::RelativeAddress8 => {
                let pc_val: u16 = self.read_mem(self.registers.pc + 1u16) as u16;
                self.read_mem(0xff00 + pc_val)
            }
            _ => {
                panic!("Adressing mod for getValue16 not implemented yet")
            }
        }
    }

    fn store_value8(&mut self, addressing: &instructions::Addressing, value: u8) {
        match addressing {
            instructions::Addressing::RelativeRegister(reg) => match reg {
                instructions::Registers::HlMinus => {
                    self.write_mem(self.registers.get_hl(), value);

                    self.dec16(&instructions::INSTRUCTIONS[0x2B]);
                }
                instructions::Registers::HlPlus => {
                    //assert!(false);
                    self.write_mem(self.registers.get_hl(), value);

                    self.inc16(&instructions::INSTRUCTIONS[0x23]);
                }
                instructions::Registers::HL => {
                    self.write_mem(self.registers.get_hl(), value);
                }
                instructions::Registers::C => {
                    self.address_space[(self.registers.c as u16 + 0xff00) as usize] = value;
                    self.write_mem(self.registers.c as u16 + 0xff00, value);
                }
                _ => assert!(
                    false,
                    "Addressing of register for store_value8 not possible"
                ),
            },
            instructions::Addressing::Register(reg) => match reg {
                instructions::Registers::A => self.registers.a = value,
                instructions::Registers::B => self.registers.b = value,
                instructions::Registers::C => self.registers.c = value,
                instructions::Registers::D => self.registers.d = value,
                instructions::Registers::E => self.registers.e = value,
                instructions::Registers::H => self.registers.h = value,
                instructions::Registers::L => self.registers.l = value,
                _ => assert!(
                    false,
                    "Addressing of register for store_value8 not possible"
                ),
            },
            instructions::Addressing::RelativeAddress8 => {
                let pc_val: u16 = self.read_mem(self.registers.pc + 1) as u16;
                self.write_mem(0xff00 + pc_val, value);
            }
            instructions::Addressing::RelativeAddress16 => {
                let address: u16 = self.get_value16_immediate();

                self.write_mem(address, value);
            }
            _ => panic!(
                "Addressing mode for store_value16 not implemented yet"
            ),
        }
    }

    fn load16(&mut self, instruction: &Instruction) {
        self.store_value16(&instruction.dst, self.get_value16(&instruction.src));
    }

    fn load8(&mut self, instruction: &Instruction) {
        self.store_value8(&instruction.dst, self.get_value8(&instruction.src));
    }

    fn dec16(&mut self, instruction: &Instruction) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => self.registers.dec16(reg),
            _ => assert!(false, "Register not availaibale for dec16!"),
        }
    }

    fn inc16(&mut self, instruction: &Instruction) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => self.registers.inc16(reg),
            _ => assert!(false, "Register not availaibale for dec16!"),
        }
    }

    fn xor(&mut self, instruction: &Instruction) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => match reg {
                instructions::Registers::A => self.registers.a ^= self.registers.a,
                instructions::Registers::B => self.registers.a ^= self.registers.b,
                instructions::Registers::C => self.registers.a ^= self.registers.c,
                instructions::Registers::D => self.registers.a ^= self.registers.d,
                instructions::Registers::E => self.registers.a ^= self.registers.e,
                instructions::Registers::H => self.registers.a ^= self.registers.h,
                instructions::Registers::L => self.registers.a ^= self.registers.l,
                _ => assert!(false, "Addressing of register for xor not possible"),
            },
            instructions::Addressing::Immediate8 => {
                self.registers.a ^= self.read_mem(self.registers.pc + 1);
            }
            _ => assert!(false, "Addressing mode for xor not implemented yet"),
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
                _ => assert!(false, "Destination of bit function must be a bit!"),
            },
            _ => assert!(false, "Must be register"),
        }

        if set {
            self.registers.reset_flag(Flag::Zero);
        } else {
            self.registers.set_flag(Flag::Zero);
        }
        self.registers.reset_flag(Flag::Subtraction);
        self.registers.set_flag(Flag::HalfCarry);
    }

    fn jump_relative(&mut self, instruction: &Instruction) {
        match instruction.src {
            instructions::Addressing::RelativeAddress8 => match &instruction.dst {
                instructions::Addressing::Flag(flag) => match flag {
                    instructions::FlagsEnum::NonZero => {
                        if !self.registers.get_flag(Flag::Zero) {
                            let signed: i8 =
                                self.read_mem(self.registers.pc + 1) as i8;
                            if signed < 0 {
                                self.registers.pc -= signed.abs() as u16;
                            } else {
                                self.registers.pc += signed.abs() as u16;
                            }
                        }
                    }
                    instructions::FlagsEnum::Zero => {
                        if self.registers.get_flag(Flag::Zero) {
                            let signed: i8 =
                                self.read_mem(self.registers.pc + 1) as i8;
                            if signed < 0 {
                                self.registers.pc -= signed.abs() as u16;
                            } else {
                                self.registers.pc += signed.abs() as u16;
                            }
                        }
                    }
                    _ => assert!(false, "Flag for JR not implemented yet!"),
                },
                instructions::Addressing::None => {
                    let signed: i8 = self.read_mem(self.registers.pc + 1) as i8;
                    if signed < 0 {
                        self.registers.pc -= signed.abs() as u16;
                    } else {
                        self.registers.pc += signed.abs() as u16;
                    }
                }
                _ => assert!(
                    false,
                    "addressing mode dst of JR instructions has to be flag"
                ),
            },
            _ => assert!(
                false,
                "addressing mode of source for JR instructions has to be RelativeAddress8"
            ),
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
            _ => assert!(false),
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
            _ => assert!(false),
        }
    }

    fn make_call(&mut self) {
        self.registers.sp -= 2;
        self.write_mem(self.registers.sp, (self.registers.pc & 0xff) as u8);
        self.write_mem(self.registers.sp + 1, (self.registers.pc >> 8) as u8);
        // -3 because this gets added when pc is defaultly increased
        self.registers.pc = self.get_value16_immediate() - 3;
    }

    fn call(&mut self, instruction: &Instruction) {
        let mut make_call: bool = false;
        match instruction.src {
            instructions::Addressing::Address16 => match instruction.dst {
                instructions::Addressing::None => {
                    make_call = true;
                }
                _ => assert!(false, "addressing mode not implemented"),
            },
            _ => assert!(false, "addressing mode not implemented"),
        }

        if make_call {
            self.make_call();
        }
    }

    fn ret(&mut self, instruction: &Instruction) {
        match instruction.dst {
            instructions::Addressing::None => {
                if instruction.opcode != 0xc9 as u8 {
                    panic!("With this addressing function hast to be 0xc9");
                }

                let upper_byte = self.read_mem(self.registers.sp + 1);
                let lower_byte = self.read_mem(self.registers.sp);
                self.registers.pc = (upper_byte as u16) << 8 | lower_byte as u16;
                // Prev instruction also has to be skipped
                // TODO: look into this
                self.registers.pc += 2;
                self.registers.sp += 2;
            }
            _ => panic!("Not implemented yet"),
        }
    }

    fn push(&mut self, instruction: &Instruction) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => match reg {
                instructions::Registers::BC => {
                    self.registers.sp -= 2;
                    self.write_mem(self.registers.sp + 1, self.registers.b);
                    self.write_mem(self.registers.sp, self.registers.c);
                }
                instructions::Registers::DE => {
                    self.registers.sp -= 2;
                    self.write_mem(self.registers.sp + 1, self.registers.d);
                    self.write_mem(self.registers.sp, self.registers.e);
                }
                instructions::Registers::HL => {
                    self.registers.sp -= 2;
                    self.write_mem(self.registers.sp + 1, self.registers.h);
                    self.write_mem(self.registers.sp, self.registers.l);
                }
                instructions::Registers::AF => {
                    self.registers.sp -= 2;
                    self.write_mem(self.registers.sp + 1, self.registers.a);
                    self.write_mem(self.registers.sp, self.registers.f);
                }
                _ => panic!("Destination hast to be Double Register"),
            },
            _ => panic!("Destination hast to be Register"),
        }
    }

    fn pop(&mut self, instruction: &Instruction) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => match reg {
                instructions::Registers::BC => {
                    self.registers.b = self.read_mem(self.registers.sp + 1);
                    self.registers.c = self.read_mem(self.registers.sp);
                    self.registers.sp += 2;
                }
                instructions::Registers::DE => {
                    self.registers.d = self.read_mem(self.registers.sp + 1);
                    self.registers.e = self.read_mem(self.registers.sp);
                    self.registers.sp += 2;
                }
                instructions::Registers::HL => {
                    self.registers.h = self.read_mem(self.registers.sp + 1);
                    self.registers.l = self.read_mem(self.registers.sp);
                    self.registers.sp += 2;
                }
                instructions::Registers::AF => {
                    self.registers.a = self.read_mem(self.registers.sp + 1);
                    self.registers.f = self.read_mem(self.registers.sp);
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
                }
                if *self.registers.get_reg_ref(*reg) == 0 {
                    self.registers.set_flag(Flag::Zero);
                } else {
                    self.registers.reset_flag(Flag::Zero);
                }
            }
            _ => assert!(false, "Rotate left only available for Registers"),
        }
    }

    fn rla(&mut self, instruction: &Instruction) {
        if instruction.opcode != 0x17 {
            assert!(false, "rla instruction has to be opcode 0x17");
        }
        self.rl(&instructions::PREFIXED_INSTRUCTIONS[0x11 as usize])
    }

    fn cp(&mut self, instruction: &Instruction) {
        let val: u8;
        match instruction.dst {
            instructions::Addressing::Immediate8
            | instructions::Addressing::Register(_)
            | instructions::Addressing::RelativeRegister(instructions::Registers::HL) => {
                val = self.get_value8(&instruction.dst)
            }
            _ => {
                assert!(false, "addressing not implemented for compare");
                val = 0
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
}

