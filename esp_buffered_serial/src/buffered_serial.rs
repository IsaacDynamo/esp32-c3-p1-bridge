use heapless::spsc::Consumer;
use heapless::spsc::Producer;
use heapless::spsc::Queue;

use esp32c3_hal as hal;
use hal::{clock::Clocks, serial::Instance};

use core::cell::RefCell;
use core::ops::DerefMut;
use critical_section::Mutex;
use enumset::EnumSet;

use once_mut::OnceMut;
use crate::optional_pin::*;
use crate::serial_util::*;

const UART_FIFO_SIZE: u16 = 128;

pub use crate::serial_util::Inverse;
pub use hal::serial::config::{DataBits, Parity, StopBits};

pub struct Config {
    pub baudrate: u32,
    pub data_bits: DataBits,
    pub parity: Parity,
    pub stop_bits: StopBits,
    pub inverse: EnumSet<Inverse>,
}

pub struct Pins<TX, RX, CTS, RTS> {
    pub tx: TX,
    pub rx: RX,
    pub cts: CTS,
    pub rts: RTS,
}

#[derive(Default)]
struct Buffers<const NR: usize, const NS: usize> {
    recv: Queue<u8, NR>,
    send: Queue<u8, NS>,
}

struct IsrPart<I, const NR: usize, const NS: usize> {
    uart: I,
    recv: Producer<'static, u8, NR>,
    send: Consumer<'static, u8, NS>,
}

pub struct BufferedSerial<I, const NR: usize, const NS: usize> {
    buffers: OnceMut<Buffers<NR, NS>>,
    isr: Mutex<RefCell<Option<IsrPart<I, NR, NS>>>>,
}

impl<I, const NR: usize, const NS: usize> BufferedSerial<I, NR, NS>
where
    I: Instance,
{
    pub const fn new() -> Self {
        Self {
            buffers: OnceMut::new(),
            isr: Mutex::new(RefCell::new(None)),
        }
    }

    pub fn init<TX, RX, CTS, RTS>(
        &'static self,
        mut uart: I,
        config: Config,
        mut pins: Pins<TX, RX, CTS, RTS>,
        clocks: &Clocks,
    ) -> Option<BufferedSerialHandle<I, TX, RX, CTS, RTS, NR, NS>>
    where
        TX: OptionalOutputPin,
        RX: OptionalInputPin,
        CTS: OptionalInputPin,
        RTS: OptionalOutputPin,
    {
        uart.disable_rx_interrupts();
        uart.disable_tx_interrupts();

        pins.tx.connect(uart.tx_signal());
        pins.rx.connect(uart.rx_signal());
        pins.cts.connect(uart.cts_signal());
        pins.rts.connect(uart.rts_signal());

        change_data_bits(&mut uart, config.data_bits);
        change_parity(&mut uart, config.parity);
        change_stop_bits(&mut uart, config.stop_bits);
        change_baud(&mut uart, config.baudrate, clocks);
        change_inverse(&mut uart, config.inverse);

        set_rx_fifo_full_threshold(&mut uart, 16);
        set_tx_fifo_empty_threshold(&mut uart, 16);

        let buffers = self.buffers.take(|| Buffers {
            recv: Queue::default(),
            send: Queue::default(),
        })?;

        let (isr_recv, app_recv) = buffers.recv.split();
        let (app_send, isr_send) = buffers.send.split();

        critical_section::with(|cs| {
            self.isr.borrow(cs).replace(Some(IsrPart {
                uart,
                recv: isr_recv,
                send: isr_send,
            }));
        });

        let serial = BufferedSerialHandle {
            dev: &self,
            pins,
            recv: app_recv,
            send: app_send,
        };

        serial.with_registers(|registers| {
            registers.int_ena.modify(
                |_, w| w.rxfifo_full_int_ena().set_bit(), //.rxfifo_ovf_int_ena().set_bit()
                                                          //.txfifo_empty_int_ena().set_bit()
            );
        });

        Some(serial)
    }

    pub fn isr(&'static self) {
        critical_section::with(|cs| {
            if let Some(isr) = self.isr.borrow(cs).borrow_mut().deref_mut() {
                while isr.recv.ready() && isr.uart.get_rx_fifo_count() > 0 {
                    let val = isr
                        .uart
                        .register_block()
                        .fifo
                        .read()
                        .rxfifo_rd_byte()
                        .bits();
                    let _ = isr.recv.enqueue(val);
                }

                while isr.send.ready() && isr.uart.get_tx_fifo_count() < UART_FIFO_SIZE {
                    if let Some(val) = isr.send.dequeue() {
                        isr.uart
                            .register_block()
                            .fifo
                            .write(|w| w.rxfifo_rd_byte().variant(val));
                    }
                }

                if !isr.send.ready() {
                    isr.uart
                        .register_block()
                        .int_ena
                        .modify(|_, w| w.txfifo_empty_int_ena().clear_bit());
                }

                isr.uart.register_block().int_clr.write(|w| {
                    w.rxfifo_full_int_clr()
                        .set_bit()
                        .txfifo_empty_int_clr()
                        .set_bit()
                });
            }
        });
    }
}

pub struct BufferedSerialHandle<I: 'static, TX, RX, CTS, RTS, const NR: usize, const NS: usize> {
    dev: &'static BufferedSerial<I, NR, NS>,
    pins: Pins<TX, RX, CTS, RTS>,
    recv: Consumer<'static, u8, NR>,
    send: Producer<'static, u8, NS>,
}

impl<I, TX, RX, CTS, RTS, const NR: usize, const NS: usize>
    BufferedSerialHandle<I, TX, RX, CTS, RTS, NR, NS>
where
    I: Instance,
    TX: OptionalOutputPin,
    RX: OptionalInputPin,
    CTS: OptionalInputPin,
    RTS: OptionalOutputPin,
{
    fn with_registers<F>(&self, func: F)
    where
        F: FnOnce(&hal::pac::uart0::RegisterBlock),
    {
        critical_section::with(|cs| {
            if let Some(isr) = self.dev.isr.borrow(cs).borrow_mut().deref_mut() {
                func(isr.uart.register_block());
            }
        });
    }

    pub fn write(&mut self, word: u8) -> nb::Result<(), ()> {
        let ret = self.send.enqueue(word).map_err(|_| nb::Error::WouldBlock);

        critical_section::with(|cs| {
            if let Some(isr) = self.dev.isr.borrow(cs).borrow_mut().deref_mut() {
                isr.uart
                    .register_block()
                    .int_ena
                    .modify(|_, w| w.txfifo_empty_int_ena().set_bit());
            }
        });

        ret
    }

    pub fn read(&mut self) -> nb::Result<u8, ()> {
        self.recv.dequeue().ok_or(nb::Error::WouldBlock)
    }

    pub fn release(self) -> (Pins<TX, RX, CTS, RTS>,) {
        // TODO: deinit serial
        (self.pins,)
    }
}
