use core::time;
use std::process::exit;

use crate::gameboy::instructions;
use crate::gameboy::registers::{Flag, Registers};

use std::fs::File;
use std::io::{BufWriter, Write};

use log::{debug, info};

use super::instructions::Instruction;
use super::memory::Memory;
use super::registers;

const INTERRUPT_VECTOR_VBLANK: u16 = 0x40;

#[derive(Debug)]
pub struct Cpu {
    registers: Registers,
    uptime: u64,
    ime: bool,
    pub prefixed: bool,
    draw_line: bool,
    draw_image: bool,
    pub loading_boot_image: bool,
    pub load_rom_boot_section: bool,
}

impl Cpu {
    pub fn new() -> Cpu {
        Cpu {
            registers: Registers::new_after_boot_rom(),
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

    pub fn get_ime(&self) -> bool {
        self.ime
    }

    pub fn print_status(&self) {
        println!("Cpu status:\n {}", self.registers);
        print!(" Cycles: {}", self.uptime);
        print!(" IME: {}\n", self.ime);
    }

    pub fn log_state_to_file(&self, mem: &Memory, writer: &mut BufWriter<&File>) {
        /*let file = std::fs::File::create("log").unwrap();
        let mut writer = BufWriter::new(&file);*/

        let _ = writeln!(writer, "A:{:02X} F:{:02X} B:{:02X} C:{:02X} D:{:02X} E:{:02X} H:{:02X} L:{:02X} SP:{:04X} PC:{:04X} PCMEM:{:02X},{:02X},{:02X},{:02X}",
            self.registers.a,
            self.registers.f,
            self.registers.b,
            self.registers.c,
            self.registers.d,
            self.registers.e,
            self.registers.h,
            self.registers.l,
            self.registers.sp,
            self.registers.pc,
            mem.data[0 + self.registers.pc as usize],
            mem.data[1 + self.registers.pc as usize],
            mem.data[2 + self.registers.pc as usize],
            mem.data[3 + self.registers.pc as usize]);
    }

    pub fn execute_single_instruction(&mut self, mem: &mut Memory) {
        let opcode = self.get_opcode(mem);
        let instruction = self.get_instruction(opcode);
        assert_eq!(
            instruction.opcode,
            opcode,
            "Opcode of instruction (@pc: 0x{:04x}) does not match array index! {}, instruction.opcode: 0x{:02x}, opcode: 0x{:02x}",
            self.registers.pc,
            instruction.name,
            instruction.opcode, opcode
        );
        self.execute_instruction(instruction, mem);
        self.increment_pc(instruction.bytes);
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

    fn prepare_irq(&mut self, mem: &mut Memory) {
        // Push current pc to stack
        self.registers.sp -= 2;
        mem.write_mem(self.registers.sp, (self.registers.pc & 0xff) as u8);
        mem.write_mem(self.registers.sp + 1, (self.registers.pc >> 8) as u8);

        // Interrupt is about to be handled, add execution time for handling irq
        self.increase_uptime(20);

        // disable interrupts -> should be cleared by reti instruction at end of interrupt routine
        self.ime = false;
    }

    pub fn handle_interrupt(&mut self, mem: &mut Memory) {
        // Check if interrupt mask is set
        if !self.get_ime() {
            return;
        }

        // Check if any interrupt is enbaled
        let interrupt_enable = mem.read_mem(0xffff);
        if interrupt_enable == 0x00 {
            return;
        }

        let interrupt_flags = mem.read_mem(0xff0f);
        if interrupt_flags == 0x00 {
            // Check if any interrupt is set
            return;
        }

        if ((interrupt_flags & 0x01) & (interrupt_enable & 0x01)) == 0x01 {
            // handle vblank
            info!("Handling Vblank interrupt");
            self.prepare_irq(mem);
            mem.data[0xff0f] &= !0x1; // reset request flag
            self.registers.pc = INTERRUPT_VECTOR_VBLANK;
        } else if ((interrupt_flags & 0x02) & (interrupt_enable & 0x02)) == 0x02 {
            // handle LCD
            debug!("Handling LCD interrupt");
        } else if ((interrupt_flags & 0x04) & (interrupt_enable & 0x04)) == 0x04 {
            // handle Timer
            debug!("Handling Timer interrupt");
        } else if ((interrupt_flags & 0x08) & (interrupt_enable & 0x08)) == 0x08 {
            // handle Serial
            debug!("Handling Serial interrupt");
        } else if ((interrupt_flags & 0x10) & (interrupt_flags & 0x10)) == 0x10 {
            // handle joypad
            debug!("Handling Joypad interrupt");
        } else if interrupt_flags > 0x1f {
            panic!(
                "Value in interrupt flag register not known: 0x{:02x}!",
                interrupt_flags
            );
        }
    }

    pub fn load_boot_rom(&mut self, boot_rom: Vec<u8>, mem: &mut Memory) {
        for i in 0..0x100 {
            mem.data[i as usize] = boot_rom[i];
        }
    }

    fn get_opcode(&self, mem: &Memory) -> u8 {
        mem.read_mem(self.registers.pc)
    }

    fn get_instruction(&mut self, opcode: u8) -> &'static instructions::Instruction {
        if self.prefixed {
            self.prefixed = false;
            &instructions::PREFIXED_INSTRUCTIONS[opcode as usize]
        } else {
            &instructions::INSTRUCTIONS[opcode as usize]
        }
    }

    fn execute_instruction(&mut self, instruction: &Instruction, mem: &mut Memory) {
        match self.registers.pc {
            /*0x02ed | 0x02ef | 0x02f0 | 0x02f2 | 0x02f3 => {}
            0x02f5 => {
                debug!("Leaving Wait for VBlank interrupt routine!")
            }*/
            _ => {
                debug!(
                    "Executing instruction {} (pc: {:04x}, opcode. {:02x})",
                    instruction.name, self.registers.pc, instruction.opcode
                )
            }
        }
        /*debug!(
            "Executing instruction {} (pc: {:04x}, opcode. {:02x})",
            instruction.name, self.registers.pc, instruction.opcode
        );*/

        match instruction.op_type {
            instructions::OpType::Load16 => self.load16(instruction, mem),
            instructions::OpType::Load8 => self.load8(instruction, mem),
            instructions::OpType::Xor => self.xor(instruction, mem),
            instructions::OpType::Prefix => self.prefix(instruction),
            instructions::OpType::Bit => self.bit(instruction, mem),
            instructions::OpType::JumpRelative => self.jump_relative(instruction, mem),
            instructions::OpType::Inc8 => self.inc8(instruction, mem),
            instructions::OpType::Dec8 => self.dec8(instruction, mem),
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
            instructions::OpType::Jump => self.jump(instruction, mem),
            instructions::OpType::DI => self.di(instruction),
            instructions::OpType::Or => self.or(instruction, mem),
            instructions::OpType::EI => self.ei(instruction),
            instructions::OpType::CPL => self.cpl(instruction),
            instructions::OpType::And => self.and(instruction, mem),
            instructions::OpType::Swap => self.swap(instruction, mem),
            instructions::OpType::RST => self.rst(instruction, mem),
            instructions::OpType::Add16 => self.add16(instruction, mem),
            instructions::OpType::Res => self.res(instruction, mem),
            instructions::OpType::Set => self.set(instruction, mem),
            instructions::OpType::Reti => self.reti(instruction, mem),
            instructions::OpType::SLA => self.sla(instruction),
            instructions::OpType::SRL => self.srl(instruction, mem),
            instructions::OpType::RLCA => self.rlca(),
            instructions::OpType::ADC => self.adc(instruction, mem),
            instructions::OpType::SBC => self.sbc(instruction, mem),
            instructions::OpType::RR => self.rr(instruction, mem),
            instructions::OpType::RRA => self.rra(instruction),
            instructions::OpType::RRCA => self.rrca(),
            instructions::OpType::RLC => self.rlc(instruction, mem),
            instructions::OpType::SCF => self.scf(),
            instructions::OpType::CCF => self.ccf(),
            instructions::OpType::RRC => self.rrc(instruction, mem),
            instructions::OpType::SRA => self.sra(instruction, mem),
            instructions::OpType::Nop => return,
            instructions::OpType::Halt => {
                /*println!(
                    "Received HALT instruction at PC: 0x{:04x}, exiting program after 10 sec",
                    self.registers.pc,
                );*/

                self.registers.pc -= 1;

                /*std::thread::sleep(time::Duration::from_secs(10));

                exit(0);*/
            }
            instructions::OpType::Stop => {
                info!(
                    "Received STOP instruction at PC: 0x{:04x}, exiting program",
                    self.registers.pc
                );
                panic!(
                    "Received STOP instruction at PC: 0x{:04x}, exiting program",
                    self.registers.pc
                );
            }
            _ => panic!(
                "Instruction not implemented: {} 0x{:02x}",
                instruction.name, instruction.opcode
            ),
        }
    }

    fn increment_pc(&mut self, value: u8) {
        // For now just increase pc, jmp and call instructions need special treatment!
        self.registers.pc = self.registers.pc.wrapping_add(value as u16);
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

    // get 16bit value after opcode
    fn get_value16_immediate(&self, mem: &Memory) -> u16 {
        let upper_byte = mem.read_mem(self.registers.pc + 2);
        let lower_byte = mem.read_mem(self.registers.pc + 1);
        (upper_byte as u16) << 8 | lower_byte as u16
    }

    // Get 16bit value
    fn get_value16(&self, addressing: &instructions::Addressing, mem: &Memory) -> u16 {
        match addressing {
            instructions::Addressing::Immediate16 | instructions::Addressing::Address16 => {
                self.get_value16_immediate(mem)
            }
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

    // Store 16bit value
    fn store_value16(
        &mut self,
        addressing: &instructions::Addressing,
        value: u16,
        mem: &mut Memory,
    ) {
        match addressing {
            instructions::Addressing::Register(reg) => self.registers.store_16bit_reg(reg, value),
            instructions::Addressing::RelativeAddress16 => {
                let address = self.get_value16_immediate(mem);
                mem.write_mem(address, (value & 0xff) as u8);
                mem.write_mem(address + 1, (value >> 8) as u8);
            }
            _ => panic!(
                "Addressing mode for store_value16 not implemented yet, addressing: {:?}",
                addressing
            ),
        }
    }

    // Get 8bit value
    fn get_value8(&mut self, addressing: &instructions::Addressing, mem: &Memory) -> u8 {
        match addressing {
            instructions::Addressing::Immediate8 => mem.read_mem(self.registers.pc + 1),
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
                instructions::Registers::C => mem.read_mem(0xff00 + self.registers.c as u16),
                instructions::Registers::BC => mem.read_mem(self.registers.get_bc()),
                instructions::Registers::DE => mem.read_mem(self.registers.get_de()),
                instructions::Registers::HL => mem.read_mem(self.registers.get_hl()),
                instructions::Registers::HlPlus => {
                    let retval = mem.read_mem(self.registers.get_hl());
                    self.inc16(&instructions::INSTRUCTIONS[0x23]);
                    retval
                }
                instructions::Registers::HlMinus => {
                    let retval = mem.read_mem(self.registers.get_hl());
                    self.dec16(&instructions::INSTRUCTIONS[0x2b]);
                    retval
                }
                _ => {
                    panic!("get_Value8 for this relative register not implemented");
                }
            },
            instructions::Addressing::RelativeAddress8 => {
                let immediate = mem.read_mem(self.registers.pc + 1u16) as u16;
                mem.read_mem(0xff00 + immediate)
            }
            instructions::Addressing::RelativeAddress16 => {
                mem.read_mem(self.get_value16_immediate(mem))
            }
            _ => {
                panic!("Addressing mod for getValue8 not implemented yet")
            }
        }
    }

    // Store 8bit value
    fn store_value8(&mut self, addressing: &instructions::Addressing, mem: &mut Memory, value: u8) {
        match addressing {
            instructions::Addressing::RelativeRegister(reg) => match reg {
                instructions::Registers::HlMinus => {
                    mem.write_mem(self.registers.get_hl(), value);
                    self.dec16(&instructions::INSTRUCTIONS[0x2B]);
                }
                instructions::Registers::HlPlus => {
                    mem.write_mem(self.registers.get_hl(), value);
                    self.inc16(&instructions::INSTRUCTIONS[0x23]);
                }
                instructions::Registers::HL => {
                    mem.write_mem(self.registers.get_hl(), value);
                }
                instructions::Registers::DE => {
                    mem.write_mem(self.registers.get_de(), value);
                }
                instructions::Registers::C => {
                    mem.write_mem(self.registers.c as u16 + 0xff00, value);
                }
                _ => panic!(
                    "Addressing of register for store_value8 not possible, pc: 0x{:04x}",
                    self.registers.pc
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
                _ => panic!("Addressing of register for store_value8 not possible"),
            },
            instructions::Addressing::RelativeAddress8 => {
                let pc_val: u16 = mem.read_mem(self.registers.pc + 1) as u16;
                mem.write_mem(0xff00 + pc_val, value);
            }
            instructions::Addressing::RelativeAddress16 => {
                let address: u16 = self.get_value16_immediate(mem);

                mem.write_mem(address, value);
            }
            _ => panic!("Addressing mode for store_value16 not implemented yet"),
        }
    }

    // Load 16bit value
    fn load16(&mut self, instruction: &Instruction, mem: &mut Memory) {
        // println!("Load 16 opcode: 0x{:02x}", instruction.opcode);
        self.store_value16(
            &instruction.dst,
            self.get_value16(&instruction.src, mem),
            mem,
        );
    }

    // Load 8bit value
    fn load8(&mut self, instruction: &Instruction, mem: &mut Memory) {
        let val = self.get_value8(&instruction.src, mem);
        self.store_value8(&instruction.dst, mem, val);
    }

    // Decrement 16bit value
    fn dec16(&mut self, instruction: &Instruction) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => self.registers.dec16(reg),
            _ => panic!("Register not available for dec16!"),
        }
    }

    // Increment 16bit value
    fn inc16(&mut self, instruction: &Instruction) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => self.registers.inc16(reg),
            _ => panic!("Register not available for dec16!"),
        }
    }

    // Xor
    fn xor(&mut self, instruction: &Instruction, mem: &Memory) {
        let val = self.get_value8(&instruction.dst, mem);

        self.registers.a ^= val;

        self.registers.reset_flag(Flag::Carry);
        self.registers.reset_flag(Flag::HalfCarry);
        self.registers.reset_flag(Flag::Subtraction);
        if self.registers.a == 0 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }
    }

    // Prefix instruction
    fn prefix(&mut self, _instruction: &Instruction) {
        self.prefixed = true;
    }

    // Tests if bit is set
    fn bit(&mut self, instruction: &Instruction, mem: &Memory) {
        let set: bool;
        match &instruction.src {
            instructions::Addressing::Register(reg) => match &instruction.dst {
                instructions::Addressing::Bit(bit) => set = self.registers.check_bit(reg, bit),
                _ => panic!("Destination of bit() function must be a bit!"),
            },
            instructions::Addressing::RelativeRegister(instructions::Registers::HL) => {
                match &instruction.dst {
                    instructions::Addressing::Bit(bit) => {
                        set = (mem.data[self.registers.get_hl() as usize] & bit) > 0;
                    }
                    _ => panic!("Destination of bit() function must be a bit!"),
                }
            }
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

    fn do_rel_jump(&mut self, mem: &Memory) {
        let signed: i8 = mem.read_mem(self.registers.pc + 1) as i8;
        if signed < 0 {
            self.registers.pc -= signed.abs() as u16;
        } else {
            self.registers.pc += signed.abs() as u16;
        }
    }

    // Jump relative to current address
    fn jump_relative(&mut self, instruction: &Instruction, mem: &Memory) {
        match instruction.src {
            instructions::Addressing::RelativeAddress8 => match &instruction.dst {
                instructions::Addressing::Flag(flag) => match flag {
                    instructions::FlagsEnum::NonZero => {
                        if self.registers.get_flag(Flag::Zero) {
                            return;
                        }
                    }
                    instructions::FlagsEnum::Zero => {
                        if !self.registers.get_flag(Flag::Zero) {
                            return;
                        }
                    }
                    instructions::FlagsEnum::Carry => {
                        if !self.registers.get_flag(Flag::Carry) {
                            return;
                        }
                    }
                    instructions::FlagsEnum::NonCarry => {
                        if self.registers.get_flag(Flag::Carry) {
                            return;
                        }
                    }
                    _ => panic!("Flag for JR not implemented yet!"),
                },
                instructions::Addressing::None => {
                    self.uptime -= 4;
                }
                _ => panic!("addressing mode dst of JR instructions has to be flag"),
            },
            _ => panic!("addressing mode of source for JR instructions has to be RelativeAddress8"),
        }

        self.uptime += 4;
        self.do_rel_jump(mem);
    }

    // Increment 8bit
    fn inc8(&mut self, instruction: &Instruction, mem: &mut Memory) {
        let mut dst = self.get_value8(&instruction.dst, mem);

        if (((dst & 0xf) + (1 & 0xf)) & 0x10) == 0x10 {
            self.registers.set_flag(Flag::HalfCarry);
        } else {
            self.registers.reset_flag(Flag::HalfCarry);
        }

        dst = dst.wrapping_add(1);

        if dst == 0 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }

        self.store_value8(&instruction.dst, mem, dst);
    }

    // Decrement 8bit
    fn dec8(&mut self, instruction: &Instruction, mem: &mut Memory) {
        let mut dst = self.get_value8(&instruction.dst, mem);

        if (((dst as i16 & 0xf) - (1 & 0xf)) & 0x10) == 0x10 {
            self.registers.set_flag(Flag::HalfCarry);
        } else {
            self.registers.reset_flag(Flag::HalfCarry);
        }

        dst = dst.wrapping_sub(1);

        if dst == 0 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }

        self.store_value8(&instruction.dst, mem, dst);
    }

    // Implementation of call
    fn make_call(&mut self, mem: &mut Memory, instr_size: u16) {
        self.registers.sp -= 2;
        mem.write_mem(
            self.registers.sp,
            ((self.registers.pc + instr_size) & 0xff) as u8,
        );
        mem.write_mem(
            self.registers.sp + 1,
            ((self.registers.pc + instr_size) >> 8) as u8,
        );
        // -3 because this gets added when pc is defaultly increased (instruction.bytes)
        self.registers.pc = self.get_value16_immediate(mem) - 3;
    }

    // Call subroutine
    fn call(&mut self, instruction: &Instruction, mem: &mut Memory) {
        // let make_call: bool;
        match &instruction.dst {
            instructions::Addressing::Flag(flag) => match flag {
                instructions::FlagsEnum::NonZero => {
                    if self.registers.get_flag(Flag::Zero) {
                        return;
                    } else {
                        self.uptime += 12;
                    }
                }
                instructions::FlagsEnum::Zero => {
                    if !self.registers.get_flag(Flag::Zero) {
                        return;
                    } else {
                        self.uptime += 12;
                    }
                }
                instructions::FlagsEnum::NonCarry => {
                    if self.registers.get_flag(Flag::Carry) {
                        return;
                    } else {
                        self.uptime += 12;
                    }
                }
                instructions::FlagsEnum::Carry => {
                    if !self.registers.get_flag(Flag::Carry) {
                        return;
                    } else {
                        self.uptime += 12;
                    }
                }
                _ => panic!("Flag not implemented for call!"),
            },
            _ => {}
        }
        match &instruction.src {
            instructions::Addressing::Address16 => {}
            _ => panic!("addressing mode not implemented"),
        }

        self.make_call(mem, instruction.bytes as u16);
    }

    fn exec_ret(&mut self, mem: &Memory, instruction_size: u16) {
        let upper_byte = mem.read_mem(self.registers.sp + 1);
        let lower_byte = mem.read_mem(self.registers.sp);
        self.registers.pc = (upper_byte as u16) << 8 | lower_byte as u16;

        // the size of the current executetd instruction has to be substracted
        // since it is going to be added after this
        self.registers.pc -= instruction_size;
        self.registers.sp += 2;
    }

    // Return
    fn ret(&mut self, instruction: &Instruction, mem: &Memory) {
        match &instruction.dst {
            instructions::Addressing::None => {
                assert_eq!(
                    instruction.opcode, 0xc9,
                    "With this addressing function hast to be 0xc9",
                );
                self.uptime -= 12; // This gets added when return is invoced, in case of
                                   // non-conditional return, the execution time is constant
            }
            instructions::Addressing::Flag(flag) => match flag {
                instructions::FlagsEnum::Zero => {
                    if !self.registers.get_flag(Flag::Zero) {
                        return;
                    }
                }
                instructions::FlagsEnum::NonZero => {
                    if self.registers.get_flag(Flag::Zero) {
                        return;
                    }
                }
                instructions::FlagsEnum::NonCarry => {
                    if self.registers.get_flag(Flag::Carry) {
                        return;
                    }
                }
                instructions::FlagsEnum::Carry => {
                    if !self.registers.get_flag(Flag::Carry) {
                        return;
                    }
                }
                _ => panic!("Flag not implemented for return!"),
            },
            _ => panic!("Not implemented yet"),
        }

        // Condition is met, return is invoced!

        self.uptime += 12; // When condition is met, it takes additional 12 cycles
                           //
        self.exec_ret(mem, instruction.bytes as u16);
    }

    fn reti(&mut self, instruction: &Instruction, mem: &Memory) {
        assert_eq!(
            instruction.opcode, 0xd9,
            "Reti instruction has to have opcode 0xd9"
        );
        self.ime = true;
        self.exec_ret(mem, instruction.bytes as u16);
        // TODO: somehow smth is wrong with the call / return stuff...
        // self.registers.pc -= 1;
    }

    // Push values to stack
    fn push(&mut self, instruction: &Instruction, mem: &mut Memory) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => match reg {
                instructions::Registers::BC => {
                    self.registers.sp -= 2;
                    mem.write_mem(self.registers.sp + 1, self.registers.b);
                    mem.write_mem(self.registers.sp, self.registers.c);
                }
                instructions::Registers::DE => {
                    self.registers.sp -= 2;
                    mem.write_mem(self.registers.sp + 1, self.registers.d);
                    mem.write_mem(self.registers.sp, self.registers.e);
                }
                instructions::Registers::HL => {
                    self.registers.sp -= 2;
                    mem.write_mem(self.registers.sp + 1, self.registers.h);
                    mem.write_mem(self.registers.sp, self.registers.l);
                }
                instructions::Registers::AF => {
                    self.registers.sp -= 2;
                    mem.write_mem(self.registers.sp + 1, self.registers.a);
                    mem.write_mem(self.registers.sp, self.registers.f);
                }
                _ => panic!("Destination hast to be Double Register"),
            },
            _ => panic!("Destination hast to be Register"),
        }
    }

    // Pop values from stack
    fn pop(&mut self, instruction: &Instruction, mem: &Memory) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => match reg {
                instructions::Registers::BC => {
                    self.registers.b = mem.read_mem(self.registers.sp + 1);
                    self.registers.c = mem.read_mem(self.registers.sp);
                }
                instructions::Registers::DE => {
                    self.registers.d = mem.read_mem(self.registers.sp + 1);
                    self.registers.e = mem.read_mem(self.registers.sp);
                }
                instructions::Registers::HL => {
                    self.registers.h = mem.read_mem(self.registers.sp + 1);
                    self.registers.l = mem.read_mem(self.registers.sp);
                }
                instructions::Registers::AF => {
                    self.registers.a = mem.read_mem(self.registers.sp + 1);
                    self.registers.f = mem.read_mem(self.registers.sp) & 0xf0;
                }
                _ => panic!("Destination hast to be Double Register"),
            },
            _ => panic!("Destination has to be Register!"),
        }

        self.registers.sp += 2;
    }

    // Rotate left
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

    // Rotate left arithmetic
    fn rla(&mut self, instruction: &Instruction) {
        assert_eq!(
            instruction.opcode, 0x17,
            "rla instruction has to be opcode 0x17"
        );
        self.rl(&instructions::PREFIXED_INSTRUCTIONS[0x17 as usize]);
    }

    // Compare
    fn cp(&mut self, instruction: &Instruction, mem: &Memory) {
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

        let diff: i16 = self.registers.a as i16 - val as i16;

        if diff < 0 {
            self.registers.set_flag(Flag::Carry);
        } else {
            self.registers.reset_flag(Flag::Carry);
        }

        // TODO: Check if this is the right operation for half carry
        if ((self.registers.a as i16 & 0xf) - (val as i16 & 0xf)) < 0x0 {
            self.registers.set_flag(Flag::HalfCarry);
        } else {
            self.registers.reset_flag(Flag::HalfCarry);
        }

        // let res = self.registers.a.wrapping_sub(val);

        if diff == 0 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }
    }

    // 8bit subtraction
    fn sub8(&mut self, instruction: &Instruction, mem: &Memory) {
        let val = self.get_value8(&instruction.dst, mem);

        if val > self.registers.a {
            self.registers.set_flag(Flag::Carry);
        } else {
            self.registers.reset_flag(Flag::Carry);
        }

        if ((self.registers.a as i16 & 0xf) - (val as i16 & 0xf)) < 0x0 {
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

    // 8bit add
    fn add8(&mut self, instruction: &Instruction, mem: &mut Memory) {
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

    // Jump
    fn jump(&mut self, instructions: &Instruction, mem: &Memory) {
        match &instructions.dst {
            instructions::Addressing::Address16 => {
                self.registers.pc = self.get_value16_immediate(mem) - instructions.bytes as u16;
                return;
            }
            instructions::Addressing::RelativeRegister(reg) => match reg {
                instructions::Registers::HL => {
                    self.registers.pc = ((self.registers.h as u16) << 8 | self.registers.l as u16)
                        - instructions.bytes as u16;
                    return;
                }
                _ => panic!("Not implemented for jump"),
            },
            instructions::Addressing::Flag(flag) => match flag {
                instructions::FlagsEnum::Zero => {
                    // if flag is not set, return
                    if !self.registers.get_flag(registers::Flag::Zero) {
                        return;
                    }
                }
                instructions::FlagsEnum::NonZero => {
                    if self.registers.get_flag(registers::Flag::Zero) {
                        return;
                    }
                }
                instructions::FlagsEnum::NonCarry => {
                    if self.registers.get_flag(registers::Flag::Carry) {
                        return;
                    }
                }
                instructions::FlagsEnum::Carry => {
                    if !self.registers.get_flag(registers::Flag::Carry) {
                        return;
                    }
                }
                _ => panic!("Flag not implemented for jump instruction!\n"),
            },
            _ => panic!("Destination not implemented for jump"),
        }

        // From here on conditional jumps should be true

        match &instructions.src {
            instructions::Addressing::Address16 => {
                self.uptime += 4; // takes longer if condition is met
                self.registers.pc = self.get_value16_immediate(mem) - instructions.bytes as u16
            }
            _ => panic!("Source notimplemented for jump"),
        }
    }

    // Disable interrupts
    fn di(&mut self, instruction: &Instruction) {
        assert_eq!(instruction.opcode, 0xf3, "Wrong opcode for DI");
        self.ime = false;
    }

    // Logical 8bit or
    fn or(&mut self, instruction: &Instruction, mem: &Memory) {
        self.registers.a |= self.get_value8(&instruction.dst, mem);

        if self.registers.a == 0 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }
    }

    // Enable interupts
    fn ei(&mut self, instruction: &Instruction) {
        assert_eq!(instruction.opcode, 0xfb, "Wrong opcode for EI");
        self.ime = true;
    }

    // Complement
    fn cpl(&mut self, instruction: &Instruction) {
        assert_eq!(instruction.opcode, 0x2f, "Wrong opcode for CPL");
        self.registers.a ^= 0xff;
    }

    // Logical 8bit and
    fn and(&mut self, instruction: &Instruction, mem: &Memory) {
        let val = self.get_value8(&instruction.dst, mem);

        self.registers.a &= val;

        if self.registers.a == 0 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }
    }

    // Swap higher and lower nibble
    fn swap(&mut self, instruction: &Instruction, mem: &mut Memory) {
        let mut val = self.get_value8(&instruction.dst, mem);
        val = ((val & 0x0f) << 4) | ((val & 0xf0) >> 4);

        if val == 0x00 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }

        self.store_value8(&instruction.dst, mem, val);
    }

    // Call to rst addr
    fn rst(&mut self, instruction: &Instruction, mem: &mut Memory) {
        let /*mut*/ target: u16/* = 0x00*/;
        match &instruction.dst {
            instructions::Addressing::RstAddr(rstaddr) => target = *rstaddr,
            _ => panic!("addressing mode not implemented for rst instruction"),
        }

        self.registers.sp -= 2;
        mem.write_mem(
            self.registers.sp,
            ((self.registers.pc + instruction.bytes as u16) & 0xff) as u8,
        );
        mem.write_mem(
            self.registers.sp + 1,
            ((self.registers.pc + instruction.bytes as u16) >> 8) as u8,
        );
        // -1 for instruction size
        self.registers.pc = target.wrapping_sub(instruction.bytes as u16);

        // self.registers.sp -= 2;
        // self.write_mem(mem, self.registers.sp, (self.registers.pc & 0xff) as u8);
        // self.write_mem(mem, self.registers.sp + 1, (self.registers.pc >> 8) as u8);
        // -3 because this gets added when pc is defaultly increased
        // self.registers.pc = self.get_value16_immediate(mem) - 3;
    }

    // Add 16 bit values
    fn add16(&mut self, instruction: &Instruction, mem: &mut Memory) {
        let src = self.get_value16(&instruction.src, mem);
        let mut dst = self.get_value16(&instruction.dst, mem);

        if (dst as u32 + src as u32) > u16::MAX as u32 {
            self.registers.set_flag(Flag::Carry);
        } else {
            self.registers.reset_flag(Flag::Carry);
        }

        if ((src as u32 & 0xfff) + (dst as u32 & 0xfff)) > 0xfff {
            self.registers.set_flag(Flag::HalfCarry);
        } else {
            self.registers.reset_flag(Flag::HalfCarry);
        }

        dst = src.wrapping_add(dst);

        self.store_value16(&instruction.dst, dst, mem);
    }

    // Reset
    fn res(&mut self, instruction: &Instruction, mem: &mut Memory) {
        match &instruction.src {
            instructions::Addressing::Register(reg) => match &instruction.dst {
                instructions::Addressing::Bit(bit) => self.registers.reset_bit(reg, bit),
                _ => panic!("Destination of bit() function must be a bit!"),
            },
            instructions::Addressing::RelativeRegister(instructions::Registers::HL) => {
                match &instruction.dst {
                    instructions::Addressing::Bit(bit) => {
                        mem.data[self.registers.get_hl() as usize] &= !(1 << bit);
                    }
                    _ => panic!("Destination of bit() function must be a bit!"),
                }
            }
            _ => panic!("Must be register"),
        }
    }

    // Shift left arithmetically
    fn sla(&mut self, instruction: &Instruction) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => {
                if self.registers.get_reg_val(*reg) & 0x80 == 0x80 {
                    self.registers.set_flag(Flag::Carry);
                } else {
                    self.registers.reset_flag(Flag::Carry);
                }

                self.registers.sla_reg(reg);

                if self.registers.get_reg_val(*reg) == 0x00 {
                    self.registers.set_flag(Flag::Zero);
                } else {
                    self.registers.reset_flag(Flag::Zero);
                }
            }
            _ => panic!("SLA not implemented for this dst addressing"),
        }
    }

    fn srl(&mut self, instruction: &Instruction, _mem: &Memory) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => {
                if self.registers.get_reg_val(*reg) & 0x01 == 0x01 {
                    self.registers.set_flag(Flag::Carry);
                } else {
                    self.registers.reset_flag(Flag::Carry);
                }

                self.registers.srl_reg(reg);

                if self.registers.get_reg_val(*reg) == 0x00 {
                    self.registers.set_flag(Flag::Zero);
                } else {
                    self.registers.reset_flag(Flag::Zero);
                }
            }
            _ => panic!("SRL not implemented for this dst addressing!"),
        }
    }

    fn rlca(&mut self) {
        let carry = self.registers.a & 0x80 == 0x80;

        if carry {
            self.registers.set_flag(Flag::Carry);
        } else {
            self.registers.reset_flag(Flag::Carry);
        }

        self.registers.a <<= 1;

        if carry {
            self.registers.a |= 0x01;
        }
    }

    // Add with carry
    fn adc(&mut self, instruction: &Instruction, mem: &mut Memory) {
        let dst = self.get_value8(&instruction.dst, mem);
        let src = self.get_value8(&instruction.src, mem);
        let carry = if self.registers.get_flag(Flag::Carry) {
            1
        } else {
            0
        };

        let sum: u16 = dst as u16 + src as u16 + carry;

        if sum & 0x00ff == 0x00 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }

        if sum > 0x00ff {
            self.registers.set_flag(Flag::Carry);
        } else {
            self.registers.reset_flag(Flag::Carry);
        }

        if ((src & 0x0f) + (dst & 0x0f) + carry as u8) > 0x0f {
            self.registers.set_flag(Flag::HalfCarry);
        } else {
            self.registers.reset_flag(Flag::HalfCarry);
        }

        self.store_value8(&instruction.dst, mem, (sum & 0x00ff) as u8);
    }

    fn set(&mut self, instruction: &Instruction, mem: &mut Memory) {
        match &instruction.src {
            instructions::Addressing::Register(reg) => match &instruction.dst {
                instructions::Addressing::Bit(bit) => self.registers.set_bit(reg, bit),
                _ => panic!("Destination of bit() function must be a bit!"),
            },
            instructions::Addressing::RelativeRegister(instructions::Registers::HL) => {
                match &instruction.dst {
                    instructions::Addressing::Bit(bit) => {
                        mem.data[self.registers.get_hl() as usize] |= 1 << bit;
                    }
                    _ => panic!("Destination of bit() function must be a bit!"),
                }
            }
            _ => panic!("Must be register"),
        }
    }

    // Rotate right
    fn rr(&mut self, instruction: &Instruction, _mem: &mut Memory) {
        match &instruction.dst {
            instructions::Addressing::Register(reg) => {
                if self
                    .registers
                    .rr_reg(reg, self.registers.get_flag(Flag::Carry))
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
            _ => panic!("Rotate right only available for Registers"),
        }
    }

    // Rotate right register A
    fn rra(&mut self, _instruction: &Instruction) {
        let reg = &instructions::Registers::A;
        if self
            .registers
            .rr_reg(reg, self.registers.get_flag(Flag::Carry))
        {
            self.registers.set_flag(Flag::Carry)
        } else {
            self.registers.reset_flag(Flag::Carry);
        }
    }

    // Subtract with carry
    fn sbc(&mut self, instruction: &Instruction, mem: &mut Memory) {
        let dst = self.get_value8(&instruction.dst, mem);
        let src = self.get_value8(&instruction.src, mem);
        let carry = if self.registers.get_flag(Flag::Carry) {
            1
        } else {
            0
        };

        let diff: i16 = dst as i16 - src as i16 - carry;

        if diff & 0x00ff == 0x00 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }

        if diff < 0x0000 {
            self.registers.set_flag(Flag::Carry);
        } else {
            self.registers.reset_flag(Flag::Carry);
        }

        if ((dst as i16 & 0x0f) - (src as i16 & 0x0f) - carry as i16) < 0x00 {
            self.registers.set_flag(Flag::HalfCarry);
        } else {
            self.registers.reset_flag(Flag::HalfCarry);
        }

        self.store_value8(&instruction.dst, mem, (diff & 0x00ff) as u8);
    }

    // Set carry flag
    fn scf(&mut self) {
        // done with flags
    }

    // Complement carry flag
    fn ccf(&mut self) {
        if self.registers.get_flag(Flag::Carry) {
            self.registers.reset_flag(Flag::Carry);
        } else {
            self.registers.set_flag(Flag::Carry);
        }
    }

    // rotate right register a
    fn rrca(&mut self) {
        let lsb = self.registers.a & 0x01 == 0x01;
        if lsb {
            self.registers.set_flag(Flag::Carry);
        } else {
            self.registers.reset_flag(Flag::Carry);
        }

        self.registers.a >>= 1;

        if lsb {
            self.registers.a |= 0x80;
        }
    }

    // rotate left with carry
    fn rlc(&mut self, instruction: &Instruction, mem: &mut Memory) {
        let mut val = self.get_value8(&instruction.dst, mem);

        let msb = val & 0x80 == 0x80;

        if msb {
            self.registers.set_flag(Flag::Carry);
        } else {
            self.registers.reset_flag(Flag::Carry);
        }

        val <<= 1;

        if msb {
            val |= 0x01;
        }

        if val == 0x00 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }

        self.store_value8(&instruction.dst, mem, val);
    }

    // rotate right with carry
    fn rrc(&mut self, instruction: &Instruction, mem: &mut Memory) {
        let mut val = self.get_value8(&instruction.dst, mem);

        let lsb = val & 0x01 == 0x01;

        if lsb {
            self.registers.set_flag(Flag::Carry);
        } else {
            self.registers.reset_flag(Flag::Carry);
        }

        val >>= 1;

        if lsb {
            val |= 0x80;
        }

        if val == 0x00 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }

        self.store_value8(&instruction.dst, mem, val);
    }

    // Shift right arithmetically
    fn sra(&mut self, instruction: &Instruction, mem: &mut Memory) {
        let mut val = self.get_value8(&instruction.dst, mem);

        if val & 0x01 == 0x01 {
            self.registers.set_flag(Flag::Carry);
        } else {
            self.registers.reset_flag(Flag::Carry);
        }

        let msb = val & 0x80 == 0x80;

        val >>= 1;

        if msb {
            val |= 0x80;
        }

        if val == 0x00 {
            self.registers.set_flag(Flag::Zero);
        } else {
            self.registers.reset_flag(Flag::Zero);
        }

        self.store_value8(&instruction.dst, mem, val);
    }
}
