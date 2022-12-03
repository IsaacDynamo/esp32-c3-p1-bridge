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
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource,
        LowSpeed,
        LEDC,
    },
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

    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();

    // Turn on-board amber and white LEDs off.
    peripherals.USB_DEVICE.conf0.modify(|_,w| w.usb_pad_enable().clear_bit());

    let mut leds = (
        io.pins.gpio18.into_push_pull_output(),
        io.pins.gpio19.into_push_pull_output()
    );

    leds.0.set_low().unwrap();
    leds.1.set_low().unwrap();

    // Init GPIO used by RGB LED and hook them to LEDC channels
    let rgb_gpio = (
        io.pins.gpio3.into_push_pull_output(),
        io.pins.gpio4.into_push_pull_output(),
        io.pins.gpio5.into_push_pull_output()
    );

    let mut ledc = LEDC::new(
        peripherals.LEDC,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut lstimer0 = ledc.get_timer::<LowSpeed>(timer::Number::Timer2);

    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 24u32.kHz(),
        })
        .unwrap();

    let mut channel0 = ledc.get_channel(channel::Number::Channel0, rgb_gpio.0);
    channel0
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 10,
        })
        .unwrap();

    let mut channel1 = ledc.get_channel(channel::Number::Channel1, rgb_gpio.1);
        channel1
            .configure(channel::config::Config {
                timer: &lstimer0,
                duty_pct: 10,
            })
            .unwrap();

    let mut channel2 = ledc.get_channel(channel::Number::Channel2, rgb_gpio.2);
        channel2
            .configure(channel::config::Config {
                timer: &lstimer0,
                duty_pct: 10,
            })
            .unwrap();

    // Update duty cycles. This is done directly by touching the hardware registers. It relies on the side effects of the above setup code.
    fn set_rgb(r: u32, g: u32, b: u32)
    {
        let raw_ledc = unsafe { &*hal::pac::LEDC::ptr() };

        raw_ledc.ch0_duty.write(|w| w.duty().variant(r));
        raw_ledc.ch0_conf1.modify(|_,w| w.duty_start().set_bit());  // SC, Self Clear
        raw_ledc.ch0_conf0.modify(|_,w| w.para_up().set_bit());     // TW, Write Trigger

        raw_ledc.ch1_duty.write(|w| w.duty().variant(g));
        raw_ledc.ch1_conf1.modify(|_,w| w.duty_start().set_bit());  // SC, Self Clear
        raw_ledc.ch1_conf0.modify(|_,w| w.para_up().set_bit());     // TW, Write Trigger

        raw_ledc.ch2_duty.write(|w| w.duty().variant(b));
        raw_ledc.ch2_conf1.modify(|_,w| w.duty_start().set_bit());  // SC, Self Clear
        raw_ledc.ch2_conf0.modify(|_,w| w.para_up().set_bit());     // TW, Write Trigger
    }

    // Make RGB LED red
    set_rgb(16, 0, 0);

    // Setup UART
    let config = Config {
        baudrate: 115200,
        data_bits: DataBits::DataBits8,
        parity: Parity::ParityNone,
        stop_bits: StopBits::STOP1,
        inverse: enum_set!(Inverse::Rx),
    };

    let pins = Pins {
        tx: io.pins.gpio1.into_push_pull_output(), //UnconnectedPin,
        rx: io.pins.gpio2.into_floating_input(),
        cts: UnconnectedPin,
        rts: UnconnectedPin,
    };

    let mut serial1 = SERIAL.init(peripherals.UART1, config, pins, &clocks).unwrap();


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

    // Make RGB led blue
    set_rgb(0, 0, 16);

    while !is_connected(&wifi_interface) {
        wifi_interface.poll_dhcp().unwrap();

        wifi_interface
            .network_interface()
            .poll(timestamp())
            .unwrap();
    }

    println!("status = {:?}", wifi_interface.get_status().0);

    // Make RGB led green
    set_rgb(0, 16, 0);

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let socket = TcpSocket::new(
        TcpSocketBuffer::new(rx_buffer.as_mut_slice()),
        TcpSocketBuffer::new(tx_buffer.as_mut_slice()),
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
            while serial1.read().is_ok() {}
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
