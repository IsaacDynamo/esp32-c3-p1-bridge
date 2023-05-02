mod wifi;
use wifi::wifi_connection;

use anyhow::Result;
use esp_idf_hal as hal;
use esp_idf_hal::gpio::*;
use esp_idf_hal::ledc::*;
use esp_idf_hal::prelude::*;
use esp_idf_hal::uart::*;
use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
                      //use std::error::Error;
                      //use std::os::fd::AsFd;
use std::os::fd::AsRawFd;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
//use std::fmt::Write;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::wifi::EspWifi;

use heapless::Vec;
use log::*;
use std::{
    io::{Read, Write},
    net::{Shutdown, TcpListener, TcpStream},
    sync::Arc,
    thread,
    thread::sleep,
    time::Duration,
};
use thiserror::Error;

#[derive(Error, Debug)]
#[error("Push overflow")]
struct PushOverflow;

pub static WIFI_STATUS: AtomicU8 = AtomicU8::new(0);
pub static CLIENTS: AtomicBool = AtomicBool::new(false);

fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_sys::link_patches();

    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    // Turn off onboard amber and white LEDs
    let mut amber = PinDriver::output(peripherals.pins.gpio18)?;
    let mut white = PinDriver::output(peripherals.pins.gpio19)?;
    amber.set_low()?;
    white.set_low()?;

    // Spawn status LED thread
    thread::spawn(|| {
        status_leds(
            peripherals.ledc,
            peripherals.pins.gpio3,
            peripherals.pins.gpio4,
            peripherals.pins.gpio5,
        )
        .unwrap()
    });

    // Create wifi object
    let sysloop = EspSystemEventLoop::take()?;
    let wifi = Box::new(EspWifi::new(peripherals.modem, sysloop.clone(), None)?);

    // Spawn wifi connection thread
    thread::Builder::new()
        .stack_size(16 * 1024)
        .spawn(move || wifi_connection(wifi, sysloop.clone()).unwrap())?;

    // Spawn server
    thread::spawn(|| server().unwrap());

    // Setup UART
    let config = hal::uart::config::Config::new().baudrate(Hertz(115_200));
    let mut uart = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio1,
        peripherals.pins.gpio2,
        Option::<Gpio0>::None,
        Option::<Gpio0>::None,
        &config,
    )?;

    // Invert RX signal
    //esp_idf_sys::esp!(unsafe{esp_idf_sys::uart_set_line_inverse(uart.port(), esp_idf_sys::uart_signal_inv_t_UART_SIGNAL_RXD_INV)})?;

    {
        use std::fmt::Write;
        uart.write_str("Hello World!             ")?;
    }

    let mut buf = [0u8; 8];
    let len = uart.read(&mut buf, hal::delay::BLOCK)?;
    println!("{:?}", std::str::from_utf8(&buf[0..len]));

    // Start P1 parsing
    loop {
        sleep(Duration::from_millis(1000));
    }
}

fn status_leds(ledc: LEDC, red: Gpio3, green: Gpio4, blue: Gpio5) -> Result<()> {
    // Setup RGB LED
    let timer_config = hal::ledc::config::TimerConfig::new().frequency(25.kHz().into());
    let timer = Arc::new(LedcTimerDriver::new(ledc.timer0, &timer_config)?);
    let mut red = LedcDriver::new(ledc.channel0, timer.clone(), red)?;
    let mut green = LedcDriver::new(ledc.channel1, timer.clone(), green)?;
    let mut blue = LedcDriver::new(ledc.channel2, timer, blue)?;

    let mut rgb = move |r, g, b| {
        red.set_duty(r).unwrap();
        green.set_duty(g).unwrap();
        blue.set_duty(b).unwrap();
    };

    loop {
        rgb(0, 0, 0);
        sleep(Duration::from_millis(900));

        let wifi_status = WIFI_STATUS.load(Ordering::Relaxed);
        let clients = CLIENTS.load(Ordering::Relaxed);
        match (wifi_status, clients) {
            (0, _) => rgb(10, 0, 0),
            (1, false) => rgb(6, 4, 0),
            (1, true) => rgb(0, 10, 0),
            _ => rgb(0, 0, 10),
        }
        sleep(Duration::from_millis(100));
    }
}

fn server() -> Result<()> {
    info!("About to bind a simple echo service to port 8080");

    const MAX_CLIENTS: usize = 3;
    const MAX_POLLFDS: usize = 1 + MAX_CLIENTS;

    let listener = TcpListener::bind("0.0.0.0:8080")?;
    let mut clients: Vec<TcpStream, MAX_CLIENTS> = Vec::new();

    loop {
        let mut pollfds: Vec<libc::pollfd, MAX_POLLFDS> = Vec::new();
        let mut remove: Vec<usize, MAX_CLIENTS> = Vec::new();

        pollfds
            .push(libc::pollfd {
                fd: listener.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            })
            .map_err(|_| PushOverflow)?;

        for client in clients.iter() {
            pollfds
                .push(libc::pollfd {
                    fd: client.as_raw_fd(),
                    events: libc::POLLIN,
                    revents: 0,
                })
                .map_err(|_| PushOverflow)?;
        }

        info!("poll(), n={}", pollfds.len());
        let rc = unsafe { libc::poll(pollfds.as_mut_ptr(), pollfds.len().try_into().unwrap(), -1) };
        info!("poll(), rc={}", rc);

        for (i, pollfd) in pollfds.into_iter().enumerate() {
            if pollfd.revents == 0 {
                continue;
            }

            if i == 0 {
                let stream = listener.accept().map(|(s, _)| s);
                match stream {
                    Ok(stream) => {
                        let peer = stream.peer_addr();

                        stream.set_nonblocking(true)?;

                        let appending = clients.push(stream);

                        match appending {
                            Ok(_) => {
                                info!("Accepted client {:?}", peer);
                            }
                            Err(stream) => {
                                info!("Reject client {:?}", peer);
                                stream.shutdown(Shutdown::Both)?;
                            }
                        }
                    }
                    Err(e) => {
                        error!("Error: {}", e);
                    }
                }
            } else {
                let client_index = i - 1;
                let stream = clients
                    .get_mut(client_index)
                    .expect("pollfds entry without client");

                if pollfd.revents & libc::POLLIN != 0 {
                    let mut read = [0; 128];
                    match stream.read(&mut read) {
                        Ok(0) => {
                            // Closed
                            remove.push(client_index).map_err(|_| PushOverflow)?;
                        }
                        Ok(n) => {
                            stream.write_all(&read[0..n]).unwrap();
                        }
                        Err(err) => {
                            info!("{}", err);
                            remove.push(client_index).map_err(|_| PushOverflow)?;
                        }
                    }
                }

                if pollfd.revents & libc::POLLHUP != 0 {
                    info!("POLLHUP");
                }
            }
        }

        // Remove clients that are marked for removal
        for &i in remove.iter().rev() {
            clients.remove(i);
        }

        CLIENTS.store(!clients.is_empty(), Ordering::Relaxed);
    }
}
