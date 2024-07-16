#[derive(Debug)]
pub struct Timer {
    period: u32,
    enabled: bool,
    last_increment: u64,
    overflow_value: u8,
}

impl Timer {
    pub fn new(period: u32, enabled: bool) -> Timer {
        Timer {
            period,
            enabled,
            last_increment: 0,
            overflow_value: 0,
        }
    }

    // Increments the timer when necessary
    // Returns true if the timer overflowed
    pub fn increment(&mut self, current_cycle: u64, reg: &mut u8) -> bool {
        if !self.enabled {
            return false;
        }

        if current_cycle < (self.last_increment + self.period as u64) {
            return false;
        }

        self.last_increment = current_cycle;

        let mut current_val = *reg;
        let overflowed = current_val == 0xff;

        if overflowed {
            current_val = self.overflow_value;
        } else {
            current_val += 1;
        }

        *reg = current_val;

        overflowed
    }

    pub fn update_period(&mut self, new_period: u32) {
        self.period = new_period;
    }

    pub fn update_overflow_val(&mut self, new_val: u8) {
        self.overflow_value = new_val;
    }

    pub fn enable(&mut self) {
        self.enabled = true;
    }

    pub fn disable(&mut self) {
        self.enabled = false;
    }
}
