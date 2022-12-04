use esp32c3_hal as hal;

use hal::gpio::{Gpio3, Gpio4, Gpio5};
use hal::gpio_types::{Output, PushPull};
use hal::ledc::{
    channel::{Channel, ChannelHW},
    LowSpeed,
};

pub trait AnyChannel {
    fn set_duty(&self, duty: u32);
}

pub struct Rgb<'a, R, G, B> {
    r: &'a R,
    g: &'a G,
    b: &'a B,
}

impl<'a, R, G, B> Rgb<'a, R, G, B>
where
    R: AnyChannel,
    G: AnyChannel,
    B: AnyChannel,
{
    /// Currently a channel order of 0, 1, 2 is required, for the duty_start() & para_up() work around to function.
    pub fn new(r: &'a R, g: &'a G, b: &'a B) -> Self {
        Self { r, g, b }
    }

    pub fn set(&self, r: u32, g: u32, b: u32) {
        self.r.set_duty(r);
        self.g.set_duty(g);
        self.b.set_duty(b);
    }
}

impl AnyChannel for Channel<'_, LowSpeed, Gpio3<Output<PushPull>>> {
    fn set_duty(&self, duty: u32) {
        self.set_duty_hw(duty);
        // Workaround to make set_duty_hw() function
        // Assumes GPIO3 is paired with channel 0
        let raw_ledc = unsafe { &*hal::pac::LEDC::ptr() };
        raw_ledc.ch0_conf1.modify(|_, w| w.duty_start().set_bit());
        raw_ledc.ch0_conf0.modify(|_, w| w.para_up().set_bit());
    }
}

impl AnyChannel for Channel<'_, LowSpeed, Gpio4<Output<PushPull>>> {
    fn set_duty(&self, duty: u32) {
        self.set_duty_hw(duty);
        // Workaround to make set_duty_hw() function
        // Assumes GPIO4 is paired with channel 1
        let raw_ledc = unsafe { &*hal::pac::LEDC::ptr() };
        raw_ledc.ch1_conf1.modify(|_, w| w.duty_start().set_bit());
        raw_ledc.ch1_conf0.modify(|_, w| w.para_up().set_bit());
    }
}

impl AnyChannel for Channel<'_, LowSpeed, Gpio5<Output<PushPull>>> {
    fn set_duty(&self, duty: u32) {
        self.set_duty_hw(duty);
        // Workaround to make set_duty_hw() function
        // Assumes GPIO5 is paired with channel 2
        let raw_ledc = unsafe { &*hal::pac::LEDC::ptr() };
        raw_ledc.ch2_conf1.modify(|_, w| w.duty_start().set_bit());
        raw_ledc.ch2_conf0.modify(|_, w| w.para_up().set_bit());
    }
}
