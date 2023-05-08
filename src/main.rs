//use crate::cpu::cpu::Cpu;
//use crate::display::display::Display;
mod gameboy;

use gameboy::gameboy::Gameboy;
//mod cpu;
//mod display;

fn main() {
    /*let mut cpu: Cpu = Cpu::new();
    cpu.print_status();
    cpu.load_boot_rom(read_boot_rom().unwrap());
    cpu.print_status();*/

    /*let mut display: Display = Display::new();
    Display::init();*/

    /*loop {
        cpu.execute_single_instruction();
        cpu.print_status();
        // let mut stdin = io::stdin();
        // stdin.read(&mut [0u8]).unwrap();
    }*/

    let mut gameboy: Gameboy = Gameboy::new();
    gameboy.init();
    gameboy.run();
}

/*fn read_boot_rom() -> Result<Vec<u8>, std::io::Error> {
    fs::read("roms/boot.gb")
}

fn print_hex(data: Vec<u8>) {
    println!("        00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f");
    for (i, elem) in data.iter().enumerate() {
        if i % 16 == 0 {
            print!("0x{:04x}  ", i);
        }

        print!("{:02x} ", elem);

        if (i + 1) % 16 == 0 {
            print!("\n");
        }
    }
}*/
