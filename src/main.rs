#![no_std]
#![no_main]


use esp32c3_hal as hal;

use embedded_svc::wifi::{
    ClientConfiguration, ClientConnectionStatus, ClientIpStatus, ClientStatus,
    Configuration, Status,Wifi
};

use esp_backtrace as _;
use esp_println::logger::init_logger;
use esp_println::println;

use esp_wifi::wifi::utils::create_network_interface;
use esp_wifi::{create_network_stack_storage, network_stack_storage};
use esp_wifi::wifi_interface::timestamp;


use smoltcp::socket::{TcpSocket, TcpSocketBuffer, TcpState};

use hal::system::SystemExt;
use hal::clock::{ClockControl, CpuClock};
use hal::{pac::Peripherals, Rtc};
use hal::{
    IO,
};

use enumset::enum_set;

use esp32c3_hal::{interrupt, pac, prelude::*};

use riscv_rt::entry;

use esp_buffered_serial::{DataBits, Parity, StopBits, Inverse, Pins, BufferedSerial, Config};
use esp_buffered_serial::optional_pin::UnconnectedPin;



const SSID: &str = include_str!("../ssid.txt");
const PASSWORD: &str = include_str!("../password.txt");


fn is_starting(wifi_interface: &esp_wifi::wifi_interface::Wifi) -> bool {
    matches!(wifi_interface.get_status(), Status(ClientStatus::Starting,_))
}

fn is_started(wifi_interface: &esp_wifi::wifi_interface::Wifi) -> bool {
    matches!(wifi_interface.get_status(), Status(ClientStatus::Started(_),_))
}

fn is_connected(wifi_interface: &esp_wifi::wifi_interface::Wifi) -> bool {
    matches!(wifi_interface.get_status(), Status(ClientStatus::Started(ClientConnectionStatus::Connected(ClientIpStatus::Done(_))),_))
}

fn esp_println_init(x: hal::pac::UART0) {
    let _ = x;
}

#[derive(PartialEq, Eq)]
enum UplinkState {
    Normal,
    Flush,
    Empty
}

static SERIAL: BufferedSerial<hal::pac::UART1, 512, 64> = BufferedSerial::new();

#[interrupt]
fn UART1() {
    SERIAL.isr();
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    esp_println_init(peripherals.UART0); // Implicitly used by esp-println
    init_logger(log::LevelFilter::Info);

    esp_wifi::init_heap();

    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);


    let config = Config {
        baudrate: 115200,
        data_bits: DataBits::DataBits8,
        parity: Parity::ParityNone,
        stop_bits: StopBits::STOP1,
        inverse: enum_set!(Inverse::Rx),
    };

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);


    let pins = Pins {
        tx: io.pins.gpio1.into_push_pull_output(), //UnconnectedPin,
        rx: io.pins.gpio2.into_floating_input(),
        cts: UnconnectedPin,
        rts: UnconnectedPin,
    };

    let mut serial1 = SERIAL.init(peripherals.UART1, config, pins, &clocks).unwrap();

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();

    let mut storage = create_network_stack_storage!(2, 8, 1, 1);
    let ethernet = create_network_interface(network_stack_storage!(storage));
    let mut wifi_interface = esp_wifi::wifi_interface::Wifi::new(ethernet);

    println!("status = {:?}", wifi_interface.get_status().0);

    use hal::systimer::SystemTimer;
    let syst = SystemTimer::new(peripherals.SYSTIMER);
    esp_wifi::initialize(syst.alarm0, peripherals.RNG, &clocks).unwrap();

    while !is_starting(&wifi_interface) { }

    println!("status = {:?}", wifi_interface.get_status().0);

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        password: PASSWORD.into(),
        ..Default::default()
    });

    let res = wifi_interface.set_configuration(&client_config);
    println!("set_configuration = {:?}", res);

    while !is_started(&wifi_interface) { }

    println!("status = {:?}", wifi_interface.get_status().0);

    while !is_connected(&wifi_interface) {
        wifi_interface.poll_dhcp().unwrap();

        wifi_interface
            .network_interface()
            .poll(timestamp())
            .unwrap();
    }

    println!("status = {:?}", wifi_interface.get_status().0);

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let socket = TcpSocket::new(
        TcpSocketBuffer::new(&mut rx_buffer[..]),
        TcpSocketBuffer::new(&mut tx_buffer[..]),
    );

    let tcp_handle = wifi_interface.network_interface().add_socket(socket);


    let mut socket_state = TcpState::Closing;
    let mut uplink_almost_full = false;
    let mut uplink_state = UplinkState::Normal;


    hal::interrupt::enable(hal::pac::Interrupt::UART1, hal::interrupt::Priority::Priority3).unwrap();

    loop {

        wifi_interface.poll_dhcp().unwrap();

        wifi_interface
            .network_interface()
            .poll(timestamp())
            .unwrap();

        let tcp_socket = wifi_interface.network_interface().get_socket::<TcpSocket>(tcp_handle);

        if !tcp_socket.is_open() {
            tcp_socket.listen(4000).unwrap();
        }

        if tcp_socket.can_recv() {
            tcp_socket.recv(|bytes| {

                // Pump data from TCP buffer into UART HW fifo
                let n = bytes.iter().copied().map_while(|byte| {
                    match serial1.write(byte) {
                        Ok(_)  => Some(()),
                        Err(_) => None,
                    }
                }).count();

                (n, ())
            }).unwrap();
        }


        // if serial1.rx_fifo_full_interrupt_set() {
        //     uplink_state = UplinkState::Flush;
        //     while let nb::Result::Ok(_) = serial1.read() {}
        //     serial1.reset_rx_fifo_full_interrupt()
        // }


        if tcp_socket.can_send() && uplink_state != UplinkState::Flush{

            if uplink_state == UplinkState::Empty {

                tcp_socket.send(|bytes| {
                    if bytes.len() >= 5 {

                        uplink_state = UplinkState::Normal;
                        bytes[0..5].copy_from_slice(b"---\r\n");

                        (5, ())
                    } else {
                        (0, ())
                    }

                }).unwrap();

            } else {

                tcp_socket.send(|bytes| {

                    // Fill TCP buffer with data from UART fifo
                    let n = bytes.iter_mut().map_while(|byte|{
                        match serial1.read() {
                            Ok(c) => {
                                *byte = c;
                                Some(())
                            },
                            Err(_) => None,
                        }
                    }).count();

                    (n, ())
                }).unwrap();
            }
        }

        if tcp_socket.send_queue() == 0 && uplink_state == UplinkState::Flush {
            //uplink_state = UplinkState::Empty;
        }

        if (tcp_socket.send_capacity() - tcp_socket.send_queue()) < 8 {
            //uplink_state = UplinkState::Flush;
        }

        if !tcp_socket.may_send() || uplink_state == UplinkState::Flush {
            // Drop UART data, When there is no connection
            while let nb::Result::Ok(_) = serial1.read() {}
        }

        if tcp_socket.state() == TcpState::CloseWait {
            tcp_socket.close();
        }

        {
            let prev= uplink_almost_full;
            uplink_almost_full = (tcp_socket.send_capacity() - tcp_socket.send_queue()) < 64;
            if uplink_almost_full != prev {
                println!("uplink_almost_full = {}", uplink_almost_full);
            }
        }

        {
            let prev = socket_state;
            socket_state = tcp_socket.state();
            if socket_state != prev {
                println!("socket_state = {}", socket_state);
            }
        }
    }

}
