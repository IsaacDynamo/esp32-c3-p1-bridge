mod wifi;
use wifi::wifi_connection;

use esp_idf_hal as hal;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::wifi::EspWifi;
use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use hal::delay::TickType;
use hal::gpio::*;
use hal::ledc::*;
use hal::prelude::*;
use hal::uart::*;

use std::{
    io::{Read, Write},
    net::{Shutdown, TcpListener, TcpStream},
    os::fd::AsRawFd,
    sync::{
        atomic::{AtomicBool, AtomicU8, Ordering},
        Arc, Mutex,
    },
    thread,
    thread::sleep,
    time::Duration,
};

use anyhow::{bail, Result};
use heapless::Vec;
use log::*;
use postcard::to_vec;
use thiserror::Error;

#[derive(Error, Debug)]
#[error("Push overflow")]
struct PushOverflow;

pub static WIFI_STATUS: AtomicU8 = AtomicU8::new(0);
pub static CLIENTS: AtomicBool = AtomicBool::new(false);
pub static BLINK: AtomicBool = AtomicBool::new(false);
static MESSAGE: Mutex<Option<message::Message>> = Mutex::new(None);

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

    // Init task watchdog
    esp_idf_sys::esp!(unsafe { esp_idf_sys::esp_task_wdt_init(120, true) })?;

    // Spawn wifi connection thread
    thread::Builder::new()
        .stack_size(16 * 1024)
        .spawn(move || wifi_connection(wifi, sysloop.clone()).unwrap())?;

    // Spawn server
    thread::spawn(|| server().unwrap());

    // Setup UART
    let config = hal::uart::config::Config::new().baudrate(Hertz(115_200));
    let uart = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio1,
        peripherals.pins.gpio2,
        Option::<Gpio0>::None,
        Option::<Gpio0>::None,
        &config,
    )?;

    // Invert RX signal
    esp_idf_sys::esp!(unsafe {
        esp_idf_sys::uart_set_line_inverse(
            uart.port(),
            esp_idf_sys::uart_signal_inv_t_UART_SIGNAL_RXD_INV,
        )
    })?;

    // Start P1 parsing
    let mut builder = message::Builder::new();
    loop {
        let mut buf = [0u8; 8];
        let timeout: TickType = Duration::from_millis(1).into();
        let len = uart.read(&mut buf, timeout.0)?;

        for &b in &buf[0..len] {
            let c = char::from(b);
            if let Some(&msg) = builder.collect(c) {
                let mut m = MESSAGE.lock().unwrap();
                *m = Some(msg);
                BLINK.store(true, Ordering::Relaxed);
            }
        }
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
        let wifi_status = WIFI_STATUS.load(Ordering::Relaxed);
        let clients = CLIENTS.load(Ordering::Relaxed);
        let blink = BLINK.swap(false, Ordering::SeqCst);
        match (wifi_status, clients) {
            (0, _) => rgb(10, 0, 0),
            (1, false) => rgb(6, 4, 0),
            (1, true) => rgb(0, 10, 0),
            _ => rgb(0, 0, 10),
        }
        sleep(Duration::from_millis(100));
        if blink {
            rgb(0, 0, 0);
        }
        sleep(Duration::from_millis(900));
    }
}

fn server() -> Result<()> {
    const MAX_CLIENTS: usize = 3;
    const MAX_POLLFDS: usize = 1 + MAX_CLIENTS;

    // Add this task to the task watchdog list
    esp_idf_sys::esp!(unsafe { esp_idf_sys::esp_task_wdt_add(std::ptr::null_mut()) })?;

    let listener = TcpListener::bind("0.0.0.0:8080")?;
    let mut clients: Vec<TcpStream, MAX_CLIENTS> = Vec::new();

    loop {
        let mut pollfds: Vec<libc::pollfd, MAX_POLLFDS> = Vec::new();
        let mut remove: Vec<usize, MAX_CLIENTS> = Vec::new();

        fn add<const N: usize>(
            pollfds: &mut Vec<libc::pollfd, N>,
            fd: libc::c_int,
            events: libc::c_short,
        ) -> Result<()> {
            pollfds
                .push(libc::pollfd {
                    fd,
                    events,
                    revents: 0,
                })
                .map_err(|_| PushOverflow)?;
            Ok(())
        }

        // Add events
        add(&mut pollfds, listener.as_raw_fd(), libc::POLLIN)?;
        for client in clients.iter() {
            add(&mut pollfds, client.as_raw_fd(), libc::POLLIN)?;
        }

        // Poll
        let fds = pollfds.as_mut_ptr();
        let nfds = pollfds.len().try_into().unwrap();
        let rc = unsafe { libc::poll(fds, nfds, 10) };
        if rc < 0 {
            bail!("poll err");
        }

        // Process listener poll events
        if pollfds[0].revents != 0 {
            let incoming = listener.accept();
            match incoming {
                Ok((stream, addr)) => {
                    stream.set_nonblocking(true)?;

                    let appending = clients.push(stream);
                    match appending {
                        Ok(_) => {
                            info!("Accepted client {:?}", addr);
                        }
                        Err(stream) => {
                            info!("Reject client {:?}", addr);
                            stream.shutdown(Shutdown::Both)?;
                        }
                    }
                }
                Err(e) => {
                    error!("Error: {}", e);
                }
            }
        }

        // Process client poll events
        for (i, pollfd) in pollfds[1..].iter().enumerate() {
            if pollfd.revents & libc::POLLIN != 0 {
                let stream = clients.get_mut(i).expect("pollfds entry without client");

                let mut read = [0; 128];
                match stream.read(&mut read) {
                    Ok(0) => {
                        // Closed
                        info!("Close client {:?}", stream.peer_addr());
                        remove.push(i).map_err(|_| PushOverflow)?;
                    }
                    Ok(n) => {
                        stream.write_all(&read[0..n]).unwrap();
                    }
                    Err(err) => {
                        info!("{}", err);
                        remove.push(i).map_err(|_| PushOverflow)?;
                    }
                }
            }

            if pollfd.revents & libc::POLLHUP != 0 {
                info!("POLLHUP");
            }
        }

        // Remove clients that are marked for removal
        for &i in remove.iter().rev() {
            clients.remove(i);
        }

        // If there is a message, send it to all clients
        let mut msg = MESSAGE.lock().unwrap();
        if let Some(msg) = msg.take() {
            let data = to_vec::<_, 128>(&msg)?;
            for stream in clients.iter_mut() {
                stream.write_all(data.as_slice()).unwrap();

                // Reset task watchdog when sending data
                esp_idf_sys::esp!(unsafe { esp_idf_sys::esp_task_wdt_reset() })?;
            }
        }

        // Update client status
        CLIENTS.store(!clients.is_empty(), Ordering::Relaxed);
    }
}
