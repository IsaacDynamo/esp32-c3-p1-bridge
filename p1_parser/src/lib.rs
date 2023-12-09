#![cfg_attr(not(test), no_std)]

use core::option::Option;
use heapless::{String, Vec};
use chrono::{Utc, NaiveDateTime, FixedOffset, DateTime};

// Parse number with 3 fractional digits and return as milis
fn parse_unit_mili(s: &str, u: &str) -> Option<u32> {
    let (v, t) = s.split_once('*')?;
    let (i, f) = v.split_once('.')?;
    let i = i.parse::<u32>().ok()?;
    (f.len() == 3).then_some(())?;
    let f = f.parse::<u32>().ok()?;
    (t == u).then_some(i * 1000 + f)
}

fn parse_datetime(s: &str) -> Option<u64> {

    const SECONDS_PER_HOUR: i32 = 3600;
    let offset = match s.get(12..)? {
        // Assume CEST
        "W" => Some(FixedOffset::east_opt(1 * SECONDS_PER_HOUR)?), // UTC+1
        "S" => Some(FixedOffset::east_opt(2 * SECONDS_PER_HOUR)?), // UTC+2
        _ => None,
    }?;

    let local = NaiveDateTime::parse_from_str(s.get(..12)?, "%y%m%d%H%M%S").ok()?;
    let datetime = DateTime::<FixedOffset>::from_local(local, offset);
    let utc = datetime.with_timezone(&Utc);

    utc.timestamp().try_into().ok()
}

fn parse_line<'a>(line: &'a str) -> Option<Event<'a>> {
    use Event::*;

    let mut iter = line.split('(');
    let id = iter.next()?;
    let mut values = Vec::<&'a str, 2>::new();

    for v in iter {
        let v = v.strip_suffix(')')?;
        values.push(v).ok()?;
    }

    let event = match (id, values.len()) {
        ("0-0:1.0.0", 1) => Timestamp(parse_datetime(values[0])?),
        ("1-0:1.8.1", 1) => MeterTariff1(parse_unit_mili(values[0], "kWh")?),
        ("1-0:1.8.2", 1) => MeterTariff2(parse_unit_mili(values[0], "kWh")?),
        ("1-0:1.7.0", 1) => Power(parse_unit_mili(values[0], "kW")?),
        ("0-1:24.2.1", 2) => Gas(parse_datetime(values[0])?, parse_unit_mili(values[1], "m3")?),
        _ => return None,
    };

    Some(event)
}

#[derive(Default)]
struct LineBuffer<const N: usize> {
    line: String<N>,
    flush: bool,
}

impl<const N: usize> LineBuffer<N> {
    fn newline(&mut self, c: char) -> Result<Option<&str>, ()> {
        if self.flush {
            if self.line == "\r" && c == '\n' {
                self.flush = false;
                self.line.clear();
            } else {
                self.line.clear();
                self.line.push(c).unwrap();
            }
            return Ok(None);
        }

        if self.line.push(c).is_err() {
            let last = self.line.chars().last().unwrap();
            self.line.clear();
            let newline = last == '\r' && c == '\n';
            self.flush = !newline;
            if !newline {
                self.line.push(c).unwrap();
            }

            return Err(());
        }

        Ok(self.line.strip_suffix("\r\n"))
    }

    fn clear(&mut self) {
        self.line.clear()
    }
}

#[derive(Default, PartialEq, Eq)]
enum State {
    #[default]
    Scan,
    EmptyLine,
    Data,
}

#[derive(Default)]
pub struct Parser<const N: usize> {
    buffer: LineBuffer<N>,
    clear: bool,
    state: State,
}

impl<const N: usize> Parser<N> {
    pub fn new() -> Self {
        Parser::default()
    }

    pub fn parse(&mut self, c: char) -> Option<Event<'_>> {
        use Event::*;
        use State::*;

        if core::mem::take(&mut self.clear) {
            self.buffer.clear();
        }

        match self.buffer.newline(c) {
            Err(_) => Some(Overflow),
            Ok(None) => None,
            Ok(Some(line)) => {
                self.clear = true;

                let event = match self.state {
                    Scan => {
                        if line.starts_with('/') {
                            self.clear = false;
                            self.state = EmptyLine;
                        }
                        return None;
                    }
                    EmptyLine => {
                        if let Some(line) = line.strip_suffix("\r\n") {
                            self.state = Data;
                            Begin(line)
                        } else {
                            self.state = Scan;
                            return None;
                        }
                    }
                    Data => {
                        if let Some(line) = line.strip_prefix('!') {
                            self.state = Scan;
                            Crc(line)
                        } else {
                            parse_line(line).unwrap_or(Raw(line))
                        }
                    }
                };

                Some(event)
            }
        }
    }
}

#[derive(Debug)]
pub enum Event<'a> {
    Begin(&'a str),
    Timestamp(u64),
    MeterTariff1(u32),  // Wh
    MeterTariff2(u32),  // Wh
    Power(u32),         // W
    Gas(u64, u32),      // L
    Raw(&'a str),
    Crc(&'a str),
    Overflow,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snippet() {
        let mut ctx = Parser::<128>::new();

        let input = include_str!("../test_data.txt");

        for c in input.chars() {
            if let Some(event) = ctx.parse(c) {
                println!("{:?}", event);
            }
        }
    }

    #[test]
    fn timestamp() {
        let x = parse_datetime("221121223626W");
        assert!(x.is_some());
    }

    #[test]
    fn no_empty_line() {
        let mut parser = Parser::<16>::new();

        let input = "/blaat\r\nnope\r\n";

        let events: std::vec::Vec<_> = input
            .chars()
            .filter_map(|c| parser.parse(c).map(|x| format!("{:?}", x)))
            .collect();

        assert!(events.is_empty())
    }

    #[test]
    fn double_begin() {
        let mut parser = Parser::<16>::new();

        let input = "/begin\r\n\r\n/begin\r\n\r\n";

        let events: std::vec::Vec<_> = input
            .chars()
            .filter_map(|c| parser.parse(c).map(|x| format!("{:?}", x)))
            .collect();

        assert_eq!(
            events,
            vec!["Begin(\"/begin\")", "Raw(\"/begin\")", "Raw(\"\")"]
        )
    }

    #[test]
    fn overflow() {
        let mut parser = Parser::<16>::new();

        let input = "----------------\r\n/begin\r\n\r\n";

        let events: std::vec::Vec<_> = input
            .chars()
            .filter_map(|c| parser.parse(c).map(|x| format!("{:?}", x)))
            .collect();

        assert_eq!(events, vec!["Overflow", "Begin(\"/begin\")"])
    }

    #[test]
    fn overflow_corner() {
        let mut parser = Parser::<16>::new();

        let input = "---------------\r\n/begin\r\n\r\n";

        let events: std::vec::Vec<_> = input
            .chars()
            .filter_map(|c| parser.parse(c).map(|x| format!("{:?}", x)))
            .collect();

        assert_eq!(events, vec!["Overflow", "Begin(\"/begin\")"])
    }

    #[test]
    fn no_overflow() {
        let mut parser = Parser::<16>::new();

        let input = "--------------\r\n/begin\r\n\r\n";

        let events: std::vec::Vec<_> = input
            .chars()
            .filter_map(|c| parser.parse(c).map(|x| format!("{:?}", x)))
            .collect();

        assert_eq!(events, vec!["Begin(\"/begin\")"])
    }
}
