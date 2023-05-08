use p1_parser::*;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Default, Copy, Clone)]
pub struct Message {
    timestamp: u64,
    power: f32,
    tariff1: f32,
    tariff2: f32,
    gas: f32,
}

pub struct Builder {
    parser: Parser<128>,
    result: Message,
}

impl Builder {
    pub fn new() -> Self {
        Self {
            parser: Parser::new(),
            result: Message::default(),
        }
    }

    pub fn collect(&mut self, c: char) -> Option<&Message> {
        if let Some(event) = self.parser.parse(c) {
            match event {
                Event::Begin(_) => self.result = Message::default(),
                Event::Timestamp(DateTime::Summer(x)) => self.result.timestamp = x,
                Event::Timestamp(DateTime::Winter(x)) => self.result.timestamp = x,
                Event::MeterTariff1(x) => self.result.tariff1 = x,
                Event::MeterTariff2(x) => self.result.tariff2 = x,
                Event::Power(x) => self.result.power = x,
                Event::Gas(_, x) => self.result.gas = x,
                Event::Raw(_) => (),
                Event::Crc(_) => return Some(&self.result),
                Event::Overflow => (),
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use postcard::to_vec;

    #[test]
    fn snippet() {
        let mut ctx = Builder::new();

        let input = include_str!("../../p1_parser/test_data.txt");

        for c in input.chars() {
            if let Some(msg) = ctx.collect(c) {
                println!("{:?}", msg);
                let ser = to_vec::<_, 128>(msg);
                println!("{:?}", ser);
            }
        }
    }
}
