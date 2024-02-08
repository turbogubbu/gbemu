#[derive(Debug)]
pub struct JoypadInput {
    a: bool,
    b: bool,
    up: bool,
    down: bool,
    left: bool,
    right: bool,
    start: bool,
    select: bool,
}

impl JoypadInput {
    pub fn new() -> JoypadInput {
        JoypadInput {
            a: false,
            b: false,
            up: false,
            down: false,
            left: false,
            right: false,
            start: false,
            select: false,
        }
    }

    #[inline(always)]
    pub fn set_a(&mut self) {
        self.a = true;
    }

    #[inline(always)]
    pub fn reset_a(&mut self) {
        self.a = false;
    }

    #[inline(always)]
    pub fn set_b(&mut self) {
        self.b = true;
    }

    #[inline(always)]
    pub fn reset_b(&mut self) {
        self.b = false;
    }

    #[inline(always)]
    pub fn set_up(&mut self) {
        self.up = true;
    }

    #[inline(always)]
    pub fn reset_up(&mut self) {
        self.up = false;
    }

    #[inline(always)]
    pub fn set_down(&mut self) {
        self.down = true;
    }

    #[inline(always)]
    pub fn reset_down(&mut self) {
        self.down = false;
    }

    #[inline(always)]
    pub fn set_left(&mut self) {
        self.left = true;
    }

    #[inline(always)]
    pub fn reset_left(&mut self) {
        self.left = false;
    }

    #[inline(always)]
    pub fn set_right(&mut self) {
        self.right = true;
    }

    #[inline(always)]
    pub fn reset_right(&mut self) {
        self.right = false;
    }

    #[inline(always)]
    pub fn set_start(&mut self) {
        self.start = true;
    }

    #[inline(always)]
    pub fn reset_start(&mut self) {
        self.start = false;
    }

    #[inline(always)]
    pub fn set_select(&mut self) {
        self.select = true;
    }

    #[inline(always)]
    pub fn reset_select(&mut self) {
        self.select = false;
    }

    pub fn get_ssba(&self) -> u8 {
        let mut val: u8 = 0x0f;

        if self.a {
            val &= !0x1;
        }

        if self.b {
            val &= !0x2;
        }

        if self.select {
            val &= !0x4;
        }

        if self.start {
            val &= !0x8;
        }

        val
    }

    pub fn get_udlr(&self) -> u8 {
        let mut val: u8 = 0x0f;

        if self.right {
            val &= !0x1;
        }

        if self.left {
            val &= !0x2;
        }

        if self.up {
            val &= !0x4;
        }

        if self.down {
            val &= !0x8;
        }

        val
    }
}
