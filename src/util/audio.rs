use sdl2::audio::AudioCallback;
use sdl2::audio::AudioDevice;
use sdl2::audio::AudioSpecDesired;
use sdl2::AudioSubsystem;
use sdl2::Sdl;

use std::time::Duration;

const BASE_FREQ: i32 = 48000;

pub struct Audio {
    audio_subsys: AudioSubsystem,
    channel1: AudioDevice<SquareWave>,
    channel2: AudioDevice<SquareWave>,
}

impl Audio {
    pub fn new(sdl: &Sdl) -> Audio {
        let audio_subsys = sdl.audio().unwrap();

        let desired_spec = AudioSpecDesired {
            freq: Some(BASE_FREQ),
            channels: Some(1),
            samples: None,
        };

        Audio {
            channel1: audio_subsys
                .open_playback(None, &desired_spec, |_| SquareWave::new(440, 0.2, 0.5))
                .unwrap(),

            channel2: audio_subsys
                .open_playback(None, &desired_spec, |_| SquareWave::new(440, 0.2, 0.5))
                .unwrap(),

            audio_subsys,
        }
    }

    pub fn play_sound(&mut self) {
        self.channel1.resume();
        // self.channel2.resume();
    }
}

#[derive(Debug)]
pub struct SquareWave {
    phase_inc: f32, // how far the samples are apart in a period, this depends on the device freq
    phase: f32,     // current phase
    volume: f32,    // amplitude of signal
    duty_cycle: f32, // duty cycle of SquareWave
}

impl SquareWave {
    pub fn new(frequency: u16, volume: f32, duty_cycle: f32) -> SquareWave {
        SquareWave {
            phase_inc: frequency as f32 / BASE_FREQ as f32,
            phase: 0.0,
            volume,
            duty_cycle,
        }
    }
}

impl AudioCallback for SquareWave {
    type Channel = f32;

    fn callback(&mut self, out: &mut [f32]) {
        // Generate a square wave
        for x in out.iter_mut() {
            *x = if self.phase <= self.duty_cycle {
                self.volume
            } else {
                -self.volume
            };
            self.phase = (self.phase + self.phase_inc) % 1.0;
        }
    }
}
