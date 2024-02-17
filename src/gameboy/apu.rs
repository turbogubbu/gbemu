use crate::util::audio::SquareWave;

#[derive(Debug)]
pub struct Apu {
    ch1_status: ChannelStatus,
    ch2_triggered: bool,
    ch3_triggered: bool,
    ch4_triggered: bool,
    ch1_settings: SquareWave,
}

impl Apu {
    pub fn new() -> Apu {
        Apu {
            ch1_status: ChannelStatus::new(),
            ch2_triggered: false,
            ch3_triggered: false,
            ch4_triggered: false,
            ch1_settings: SquareWave::new(0, 0.0, 0.0),
        }
    }

    pub fn trigger_channel_1(
        &mut self,
        control_period: u8,
        period: u8,
        volume_envelope: u8,
        length_duty_cycle: u8,
        sweep: u8,
        div_apu_val: u8,
    ) {
        self.ch1_status.period = (((control_period) as u16 & 0x07) << 8) | (period) as u16;
        self.ch1_status.initial_length_timer = if control_period & 0x40 == 0x40 {
            Some(length_duty_cycle & 0x1f)
        } else {
            None
        };

        // calculate the (initial) volume of the channel
        self.ch1_status.volume = volume_envelope >> 4;
        self.ch1_status.vol_sweep_pace = if volume_envelope & 0x08 == 0x08 {
            (volume_envelope & 0x7) as i8
        } else {
            -((volume_envelope & 0x7) as i8)
        };

        self.ch1_status.duty_cyle = length_duty_cycle >> 6;

        self.ch1_status.freq_sweep_pace = (sweep >> 4) & 0x7;

        // False = count down, True = count up
        self.ch1_status.freq_sweep_dir = if sweep & 0x08 == 0x08 { false } else { true };

        self.ch1_status.freq_sweep_step = sweep & 0x07;

        self.ch1_status.initial_div_apu_val = div_apu_val;
        self.ch1_status.last_channel_sweep_apu_val = div_apu_val >> 2;
        self.ch1_status.last_length_timer_apu_val = div_apu_val >> 1;
        self.ch1_status.last_volume_sweep_apu_val = div_apu_val >> 3;
        self.ch1_status.running = true;
    }

    pub fn update_channel_1(&mut self, div_apu_val: u8) {
        // Handle frequency sweep
        if self.ch1_status.freq_sweep_pace != 0 {
            if self.ch1_status.last_channel_sweep_apu_val < (div_apu_val >> 2) {
                let mut period = self.ch1_status.period;

                self.ch1_status.last_length_timer_apu_val = div_apu_val >> 2;

                for _ in 0..self.ch1_status.freq_sweep_pace {
                    if self.ch1_status.freq_sweep_dir {
                        period = period.wrapping_add(
                            period / 2u16.pow(self.ch1_status.freq_sweep_step as u32),
                        );

                        if period > 0x7ff {
                            self.ch1_status.running = false;
                            return;
                        }
                    } else {
                        period = period.wrapping_sub(
                            period / 2u16.pow(self.ch1_status.freq_sweep_step as u32),
                        );
                    }
                }
            }
        }

        // Handle length timer
        if self.ch1_status.initial_length_timer.is_some() {
            if self.ch1_status.last_length_timer_apu_val < (div_apu_val >> 1) {
                self.ch1_status.last_length_timer_apu_val = div_apu_val >> 1;

                self.ch1_status.initial_length_timer =
                    Some(self.ch1_status.initial_length_timer.unwrap() + 1);

                if self.ch1_status.initial_length_timer.unwrap() == 64 {
                    self.ch1_status.running = false;
                    return;
                }
            }
        }

        // Handle volume envelope
        if self.ch1_status.vol_sweep_pace != 0 {
            if self.ch1_status.last_volume_sweep_apu_val < (div_apu_val >> 3) {
                self.ch1_status.last_volume_sweep_apu_val = div_apu_val >> 3;

                self.ch1_status.volume = ((self.ch1_status.volume as i8)
                    .wrapping_add(self.ch1_status.vol_sweep_pace)
                    as u8)
                    & 0x0f;
            }
        }
    }
}

#[derive(Debug)]
struct ChannelStatus {
    pub running: bool,
    pub initial_length_timer: Option<u8>,
    pub vol_sweep_pace: i8,
    pub volume: u8,
    pub period: u16,
    pub duty_cyle: u8,
    pub freq_sweep_pace: u8,
    pub freq_sweep_dir: bool,
    pub freq_sweep_step: u8,
    pub initial_div_apu_val: u8,
    pub last_channel_sweep_apu_val: u8,
    pub last_length_timer_apu_val: u8,
    pub last_volume_sweep_apu_val: u8,
}

impl ChannelStatus {
    pub fn new() -> ChannelStatus {
        ChannelStatus {
            running: false,
            initial_length_timer: None,
            vol_sweep_pace: 0,
            volume: 0,
            period: 0,
            duty_cyle: 0,
            freq_sweep_pace: 0,
            freq_sweep_dir: false,
            freq_sweep_step: 0,
            initial_div_apu_val: 0,
            last_channel_sweep_apu_val: 0,
            last_length_timer_apu_val: 0,
            last_volume_sweep_apu_val: 0,
        }
    }
}
