use anyhow::Result;
use chrono::{DateTime, NaiveDateTime, Utc};
use message::Message;
use sqlx::{mysql::MySqlPoolOptions, MySql, Pool};
use std::time::Duration;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::net::TcpStream;

const USER: &str = include_str!("../user.txt");
const PASSWORD: &str = include_str!("../password.txt");
const DATABASE: &str = include_str!("../database.txt");
const TARGET: &str = include_str!("../target.txt");

#[derive(Debug, Default)]
struct Context {
    prev_gas_ts: Option<u64>,
    prev_energy: Option<(u32, u32)>,
}

#[tokio::main]
async fn main() -> Result<()> {
    // Create a connection pool
    let pool = MySqlPoolOptions::new()
        .max_connections(5)
        .connect(&format!("mysql://{}:{}@{}", USER, PASSWORD, DATABASE))
        .await?;

    // Setup tables
    let create = sqlx::query(
        "
        CREATE TABLE IF NOT EXISTS power (
            id INT AUTO_INCREMENT PRIMARY KEY,
            timestamp TIMESTAMP,
            power INT UNSIGNED
        )",
    )
    .execute(&pool)
    .await?;
    println!("{:?}", create);

    let create = sqlx::query(
        "
        CREATE TABLE IF NOT EXISTS energy (
            id INT AUTO_INCREMENT PRIMARY KEY,
            timestamp TIMESTAMP,
            tariff1 INT UNSIGNED,
            tariff2 INT UNSIGNED
        )",
    )
    .execute(&pool)
    .await?;
    println!("{:?}", create);

    let create = sqlx::query(
        "
        CREATE TABLE IF NOT EXISTS gas (
            id INT AUTO_INCREMENT PRIMARY KEY,
            timestamp TIMESTAMP,
            gas INT UNSIGNED
        )",
    )
    .execute(&pool)
    .await?;
    println!("{:?}", create);

    // Start server loop with retries
    loop {
        match server_loop(&pool).await {
            Ok(_) => (),
            Err(e) => println!("{:?}", e),
        }

        // Retry after a 15 sec delay
        tokio::time::sleep(Duration::from_secs(15)).await;
    }
}

async fn server_loop(pool: &Pool<MySql>) -> Result<()> {
    let stream = TcpStream::connect(TARGET).await?;

    println!("Connected!");

    let mut reader = BufReader::new(stream);
    let mut context = Context::default();

    loop {
        // Receive data or timeout
        let b = tokio::time::timeout(Duration::from_secs(10), reader.fill_buf()).await??;

        // Try to parse message
        let msg = postcard::take_from_bytes::<message::Message>(b);
        if let Ok((msg, rem)) = msg {
            // Consume parsed data
            let size = b.len() - rem.len();
            reader.consume(size);

            // Insert message into database
            insert_db(&pool, &mut context, msg).await?;
        }
    }
}

fn ts_convert(timestamp: u64) -> Result<DateTime<Utc>> {
    let unix = NaiveDateTime::from_timestamp_opt(timestamp as i64, 0)
        .ok_or(anyhow::anyhow!("Bad timestamp"))?;
    Ok(DateTime::<Utc>::from_utc(unix, Utc))
}

async fn insert_db(pool: &Pool<MySql>, context: &mut Context, msg: Message) -> Result<()> {
    let timestamp = ts_convert(msg.timestamp)?;
    let gas_timestamp = ts_convert(msg.gas_timestamp)?;

    let _ = sqlx::query!(
        "
        INSERT INTO power (
            timestamp, power
        ) VALUES (?, ?)",
        timestamp,
        msg.power
    )
    .execute(pool)
    .await;

    if context.prev_energy.map_or(true, |(tariff1, tariff2)| {
        tariff1 != msg.tariff1 || tariff2 != msg.tariff2
    }) {
        let result = sqlx::query!(
            "
            INSERT INTO energy (
                timestamp, tariff1, tariff2
            ) VALUES (?, ?, ?)",
            timestamp,
            msg.tariff1,
            msg.tariff2
        )
        .execute(pool)
        .await;

        if result.is_ok() {
            context.prev_energy = Some((msg.tariff1, msg.tariff2));
        }
    }

    if context
        .prev_gas_ts
        .map_or(true, |g_ts| g_ts != msg.gas_timestamp)
    {
        let result = sqlx::query!(
            "
            INSERT INTO gas (
                timestamp, gas
            ) VALUES (?, ?)",
            gas_timestamp,
            msg.gas
        )
        .execute(pool)
        .await;

        if result.is_ok() {
            context.prev_gas_ts = Some(msg.gas_timestamp);
        }
    }

    Ok(())
}
