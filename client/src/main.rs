use anyhow::Result;
use std::{io::BufRead, io::BufReader, net::TcpStream};

fn main() -> Result<()> {
    let stream = TcpStream::connect("192.168.0.8:8080")?;
    let mut reader = BufReader::new(stream);

    loop {
        let b = reader.fill_buf()?;

        let msg = postcard::take_from_bytes::<message::Message>(b);
        if let Ok((msg, rem)) = msg {
            println!("{:?}", msg);
            let size = b.len() - rem.len();
            reader.consume(size);
        }
    }
}
