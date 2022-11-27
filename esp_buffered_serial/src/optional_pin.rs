use esp32c3_hal as hal;
use hal::gpio_types::{InputPin, InputSignal, OutputPin, OutputSignal};

pub struct UnconnectedPin;

pub trait OptionalOutputPin {
    fn connect(&mut self, signal: OutputSignal);
}
pub trait OptionalInputPin {
    fn connect(&mut self, signal: InputSignal);
}

impl<T: OutputPin> OptionalOutputPin for T {
    fn connect(&mut self, signal: OutputSignal) {
        self.connect_peripheral_to_output(signal);
    }
}

impl<T: InputPin> OptionalInputPin for T {
    fn connect(&mut self, signal: InputSignal) {
        self.connect_input_to_peripheral(signal);
    }
}

impl OptionalOutputPin for UnconnectedPin {
    fn connect(&mut self, _: OutputSignal) {
        // NOP
    }
}

impl OptionalInputPin for UnconnectedPin {
    fn connect(&mut self, _: InputSignal) {
        // NOP
    }
}
