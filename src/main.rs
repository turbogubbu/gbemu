mod gameboy;
mod util;

use gameboy::gameboy::Gameboy;

fn main() {
    env_logger::init();

    let mut gameboy: Gameboy = Gameboy::new();
    gameboy.init();
    gameboy.run();
}
