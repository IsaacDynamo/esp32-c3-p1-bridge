use anyhow::Result;
use tokio::io::{AsyncBufReadExt, BufReader};
use sqlx::mysql::MySqlPoolOptions;
use tokio::net::TcpStream;
use chrono::{NaiveDateTime, DateTime, FixedOffset, Utc};

const USER: &str = include_str!("../user.txt");
const PASSWORD: &str = include_str!("../password.txt");
const DATABASE: &str = include_str!("../database.txt");
const TARGET: &str = include_str!("../target.txt");

#[tokio::main]
async fn main() -> Result<()> {
    // Create a connection pool
    let pool = MySqlPoolOptions::new()
        .max_connections(5)
        .connect(&format!("mysql://{}:{}@{}", USER, PASSWORD, DATABASE)).await?;

    // Setup tables
    let create = sqlx::query("
        CREATE TABLE IF NOT EXISTS power (
            id INT AUTO_INCREMENT PRIMARY KEY,
            timestamp TIMESTAMP,
            power INT UNSIGNED
        )")
        .execute(&pool).await?;

    let create = sqlx::query("
        CREATE TABLE IF NOT EXISTS energy (
            id INT AUTO_INCREMENT PRIMARY KEY,
            timestamp TIMESTAMP,
            tariff1 INT UNSIGNED,
            tariff2 INT UNSIGNED
        )")
        .execute(&pool).await?;

    let create = sqlx::query("
        CREATE TABLE IF NOT EXISTS gas (
            id INT AUTO_INCREMENT PRIMARY KEY,
            timestamp TIMESTAMP,
            gas INT UNSIGNED
        )")
        .execute(&pool).await?;

    let stream = TcpStream::connect(TARGET).await?;
    let mut reader = BufReader::new(stream);

    let mut prev_engery = None;
    let mut prev_gas_ts = None;

    loop {
        let b = reader.fill_buf().await?;

        let msg = postcard::take_from_bytes::<message::Message>(b);
        if let Ok((msg, rem)) = msg {
            let size = b.len() - rem.len();
            reader.consume(size);

            let unix = NaiveDateTime::from_timestamp_opt(msg.timestamp as i64, 0).ok_or(anyhow::anyhow!("Bad timestamp"))?;
            let timestamp = DateTime::<Utc>::from_utc(unix, Utc);

            let unix = NaiveDateTime::from_timestamp_opt(msg.gas_timestamp as i64, 0).ok_or(anyhow::anyhow!("Bad timestamp"))?;
            let gas_timestamp = DateTime::<Utc>::from_utc(unix, Utc);

            let add = sqlx::query!("
                INSERT INTO power (
                    timestamp, power
                ) VALUES (?, ?)",
                    timestamp,
                    msg.power
                )
                .execute(&pool)
                .await;
            //println!("{:?}", add);

            if prev_engery.map_or(true, |(tariff1, tariff2)| tariff1 != msg.tariff1 || tariff2 != msg.tariff2) {
                let result = sqlx::query!("
                    INSERT INTO energy (
                        timestamp, tariff1, tariff2
                    ) VALUES (?, ?, ?)",
                        timestamp,
                        msg.tariff1,
                        msg.tariff2
                    )
                    .execute(&pool)
                    .await;

                if result.is_ok() {
                    prev_engery = Some((msg.tariff1, msg.tariff2));
                }

                //println!("{:?}", result);
            }

            if prev_gas_ts.map_or(true, |g_ts| g_ts != msg.gas_timestamp) {
                let result = sqlx::query!("
                    INSERT INTO gas (
                        timestamp, gas
                    ) VALUES (?, ?)",
                        gas_timestamp,
                        msg.gas
                    )
                    .execute(&pool)
                    .await;

                if result.is_ok() {
                    prev_gas_ts = Some(msg.gas_timestamp);
                }

                //println!("{:?}", result);
            }
        }
    }

    Ok(())
}