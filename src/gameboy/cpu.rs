//use crate::cpu::instructions;

use crate::gameboy::instructions;
use crate::gameboy::memory::Memory;

use std::fmt::Display;

use super::instructions::Instruction;

#[derive(Debug)]
pub struct Cpu<'a> {
    registers: Registers,
    address_space: Vec<u8>,
    uptime: u64,
    prefixed: bool,
    mem: Option<&'a mut Memory>,
}

impl<'a> Cpu<'a> {
    pub fn new() -> Cpu<'static> {
        Cpu {
            registers: Registers::new(),
            address_space: vec![0; 0x10000], //Vec::<u8>::new(),
            uptime: 0,
            prefixed: false,
            mem: None,
        }
    }

    pub fn init_mem(&mut self, mem: &'a mut Memory) {
        println!("value: {:?}", self.mem.replace(mem));
        /*match self.mem.replace(mem) {
            Option::<&mut Memory>::None => assert!(false, "Memory can only be initialized once"),
            _ => (),
        }*/
    }

    pub fn print(&self, start_address: u16, len: u16) {
        println!("        00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f");

        let start: u16 = start_address - start_address % 16;
        let end: u16 = start + start_address % 16 + len;

        for i in start..end {
            if i % 16 == 0 {
                print!("0x{:04x}  ", i);
            }

            print!("{:02x} ", self.address_space[i as usize]);

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
        match &self.mem {
            Some(mem) => mem.read(address),
            None => {
                assert!(false, "No memory to read from!");
                0u8
            }
        }
    }

    pub fn write_mem(&mut self, address: u16, data: u8) {
        // (*self.mem.deref())).write(address, data);
        // println!("Writing to memory!!!!!!");
        // Rc::get_mut(&mut self.mem).unwrap().write(address, data);
        // self.mem.write(address, data);
        match &mut self.mem {
            Some(mem) => mem.write(address, data),
            None => {
                assert!(false, "No memory to read from!");
            }
        }
    }

    pub fn execute_single_instruction(&mut self) {
        let opcode = self.get_opcode();
        let instruction = self.get_instruction(opcode);
        assert!(
            instruction.opcode == opcode,
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
        //self.address_space.append(&mut boot_rom);

        for i in 0..boot_rom.len() {
            self.address_space[i] = boot_rom[i];
        }
    }

    fn get_opcode(&self) -> u8 {
        self.address_space[self.registers.pc as usize]
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
            _ => assert!(false),
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
        let upper_byte = self.address_space[(self.registers.pc + 2) as usize];
        let lower_byte = self.address_space[(self.registers.pc + 1) as usize];
        (upper_byte as u16) << 8 | lower_byte as u16
    }

    fn get_value16(&self, adressing: &instructions::Addressing) -> u16 {
        match adressing {
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
                self.address_space[(self.registers.pc + 1) as usize]
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
                    self.address_space[(0xff00 + self.registers.c as u16) as usize]
                }
                instructions::Registers::BC => self.address_space[self.registers.get_bc() as usize],
                instructions::Registers::DE => self.address_space[self.registers.get_de() as usize],
                instructions::Registers::HL => self.address_space[self.registers.get_hl() as usize],
                _ => {
                    assert!(false);
                    0u8
                }
            },
            instructions::Addressing::RelativeAddress8 => {
                let pc_val: u16 = self.address_space[(self.registers.pc + 1) as usize] as u16;
                self.address_space[(0xff00 + pc_val) as usize]
            }
            _ => {
                assert!(false, "Adressing mod for getValue16 not implemented yet");
                16u8
            }
        }
    }

    fn store_value8(&mut self, addressing: &instructions::Addressing, value: u8) {
        match addressing {
            instructions::Addressing::RelativeRegister(reg) => match reg {
                instructions::Registers::HlMinus => {
                    self.address_space[self.registers.get_hl() as usize] = value;

                    self.write_mem(self.registers.get_hl(), value);

                    self.dec16(&instructions::INSTRUCTIONS[0x2B]);
                }
                instructions::Registers::HlPlus => {
                    //assert!(false);
                    self.address_space[self.registers.get_hl() as usize] = value;

                    self.write_mem(self.registers.get_hl(), value);

                    self.inc16(&instructions::INSTRUCTIONS[0x23]);
                }
                instructions::Registers::HL => {
                    self.address_space[self.registers.get_hl() as usize] = value;

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
                let pc_val: u16 = self.address_space[(self.registers.pc + 1) as usize] as u16;
                self.address_space[(0xff00 + pc_val) as usize] = value;

                self.write_mem(0xff00 + pc_val, value);
            }
            instructions::Addressing::RelativeAddress16 => {
                let address: u16 = self.get_value16_immediate();
                self.address_space[address as usize] = value;

                self.write_mem(address, value);
            }
            _ => assert!(
                false,
                "Adderssing mode for store_value16 not implemented yet"
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
                self.registers.a ^= self.address_space[(self.registers.pc + 1) as usize];
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
                                self.address_space[(self.registers.pc + 1) as usize] as i8;
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
                                self.address_space[(self.registers.pc + 1) as usize] as i8;
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
                    let signed: i8 = self.address_space[(self.registers.pc + 1) as usize] as i8;
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
        self.address_space[self.registers.sp as usize] = (self.registers.pc & 0xff) as u8;
        self.write_mem(self.registers.sp, (self.registers.pc & 0xff) as u8);
        self.address_space[(self.registers.sp + 1) as usize] = (self.registers.pc >> 8) as u8;
        self.write_mem(self.registers.sp + 1, (self.registers.pc & 0xff) as u8);
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
                    assert!(false, "With this addressing function hast to be 0xc9");
                }

                let upper_byte = self.address_space[(self.registers.sp + 1) as usize];
                let lower_byte = self.address_space[(self.registers.sp) as usize];
                self.registers.pc = (upper_byte as u16) << 8 | lower_byte as u16;
                // Prev instruction also has to be skipped
                // TODO: look into this
                self.registers.pc += 2;
                self.registers.sp += 2;
            }
            _ => assert!(false, "Not implemented yet"),
        }
    }

    fn push(&mut self, instruction: &Instruction) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => match reg {
                instructions::Registers::BC => {
                    self.registers.sp -= 2;
                    self.address_space[(self.registers.sp + 1) as usize] = self.registers.b;
                    self.address_space[self.registers.sp as usize] = self.registers.c;

                    self.write_mem(self.registers.sp + 1, self.registers.b);
                    self.write_mem(self.registers.sp, self.registers.c);
                }
                instructions::Registers::DE => {
                    self.registers.sp -= 2;
                    self.address_space[(self.registers.sp + 1) as usize] = self.registers.d;
                    self.address_space[self.registers.sp as usize] = self.registers.e;

                    self.write_mem(self.registers.sp + 1, self.registers.d);
                    self.write_mem(self.registers.sp, self.registers.e);
                }
                instructions::Registers::HL => {
                    self.registers.sp -= 2;
                    self.address_space[(self.registers.sp + 1) as usize] = self.registers.h;
                    self.address_space[self.registers.sp as usize] = self.registers.l;

                    self.write_mem(self.registers.sp + 1, self.registers.h);
                    self.write_mem(self.registers.sp, self.registers.l);
                }
                instructions::Registers::AF => {
                    self.registers.sp -= 2;
                    self.address_space[(self.registers.sp + 1) as usize] = self.registers.a;
                    self.address_space[self.registers.sp as usize] = self.registers.f;

                    self.write_mem(self.registers.sp + 1, self.registers.a);
                    self.write_mem(self.registers.sp, self.registers.f);
                }
                _ => assert!(false, "Destination hast to be Double Register"),
            },
            _ => assert!(false, "Destination hast to be Register"),
        }
    }

    fn pop(&mut self, instruction: &Instruction) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => match reg {
                instructions::Registers::BC => {
                    self.registers.b = self.address_space[(self.registers.sp + 1) as usize];
                    self.registers.c = self.address_space[self.registers.sp as usize];
                    self.registers.sp += 2;
                }
                instructions::Registers::DE => {
                    self.registers.d = self.address_space[(self.registers.sp + 1) as usize];
                    self.registers.e = self.address_space[self.registers.sp as usize];
                    self.registers.sp += 2;
                }
                instructions::Registers::HL => {
                    self.registers.h = self.address_space[(self.registers.sp + 1) as usize];
                    self.registers.l = self.address_space[self.registers.sp as usize];
                    self.registers.sp += 2;
                }
                instructions::Registers::AF => {
                    self.registers.a = self.address_space[(self.registers.sp + 1) as usize];
                    self.registers.f = self.address_space[self.registers.sp as usize];
                    self.registers.sp += 2;
                }
                _ => assert!(false, "Destination hast to be Double Register"),
            },
            _ => assert!(false, "destination has to be Register!"),
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

#[derive(Debug)]
struct Registers {
    a: u8,
    f: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    h: u8,
    l: u8,
    sp: u16,
    pc: u16,
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
enum Flag {
    Zero,
    Subtraction,
    HalfCarry,
    Carry,
}

impl Registers {
    fn new() -> Registers {
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

    fn store_16bit_reg(&mut self, reg: &instructions::Registers, value: u16) {
        match reg {
            instructions::Registers::AF => self.store_af(value),
            instructions::Registers::BC => self.store_bc(value),
            instructions::Registers::DE => self.store_de(value),
            instructions::Registers::HL => self.store_hl(value),
            instructions::Registers::SP => self.sp = value,
            _ => assert!(false, "Wrong register to store 16 bit value!"),
        }
    }

    fn get_af(&self) -> u16 {
        ((self.a as u16) << 8) | (self.f as u16)
    }

    fn get_bc(&self) -> u16 {
        ((self.b as u16) << 8) | (self.c as u16)
    }

    fn get_de(&self) -> u16 {
        ((self.d as u16) << 8) | (self.e as u16)
    }

    fn get_hl(&self) -> u16 {
        ((self.h as u16) << 8) | (self.l as u16)
    }

    fn store_af(&mut self, value: u16) {
        self.a = (value >> 8) as u8;
        self.f = (value & 0x00ff) as u8;
    }

    fn store_bc(&mut self, value: u16) {
        self.b = (value >> 8) as u8;
        self.c = (value & 0x00ff) as u8;
    }

    fn store_de(&mut self, value: u16) {
        self.d = (value >> 8) as u8;
        self.e = (value & 0x00ff) as u8;
    }

    fn store_hl(&mut self, value: u16) {
        self.h = (value >> 8) as u8;
        self.l = (value & 0x00ff) as u8;
    }

    fn set_flag(&mut self, flag: Flag) {
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

    fn get_reg_ref(&mut self, reg: instructions::Registers) -> &mut u8 {
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

    fn reset_flag(&mut self, flag: Flag) {
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

    fn get_flag(&self, flag: Flag) -> bool {
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

    fn dec16(&mut self, reg: &instructions::Registers) {
        match reg {
            instructions::Registers::BC => {
                if self.c == 0 {
                    self.b -= 1;
                    self.c = 255;
                } else {
                    self.c -= 1;
                }
            }
            instructions::Registers::DE => {
                if self.e == 0 {
                    self.d -= 1;
                    self.e = 255;
                } else {
                    self.e -= 1;
                }
            }
            instructions::Registers::HL => {
                if self.l == 0 {
                    self.h -= 1;
                    self.l = 255;
                } else {
                    self.l -= 1;
                }
            }
            instructions::Registers::SP => {
                self.sp -= 1;
            }
            _ => assert!(false, "Dec 16 of register not available"),
        }
    }

    fn inc16(&mut self, reg: &instructions::Registers) {
        match reg {
            instructions::Registers::BC => {
                if self.c == 255 {
                    self.b += 1;
                    self.c = 0;
                } else {
                    self.c += 1;
                }
            }
            instructions::Registers::DE => {
                if self.e == 255 {
                    self.d += 1;
                    self.e = 0;
                } else {
                    self.e += 1;
                }
            }
            instructions::Registers::HL => {
                if self.l == 255 {
                    self.h += 1;
                    self.l = 0;
                } else {
                    self.l += 1;
                }
            }
            instructions::Registers::SP => {
                self.sp += 1;
            }
            _ => assert!(false, "Dec 16 of register not available"),
        }
    }

    fn check_bit(&self, reg: &instructions::Registers, bit: &u8) -> bool {
        match reg {
            instructions::Registers::A => self.a & (1 << bit) != 0,
            instructions::Registers::B => self.b & (1 << bit) != 0,
            instructions::Registers::C => self.c & (1 << bit) != 0,
            instructions::Registers::D => self.d & (1 << bit) != 0,
            instructions::Registers::H => self.h & (1 << bit) != 0,
            instructions::Registers::L => self.l & (1 << bit) != 0,
            _ => {
                assert!(false, "Register not implemented for Bit");
                false
            }
        }
    }

    fn rl_reg(&mut self, reg: &instructions::Registers, carry: bool) -> bool {
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
}
