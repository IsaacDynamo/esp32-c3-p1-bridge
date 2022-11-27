// Mostly copied from esp-hal-common/src/serial.rs

use esp32c3_hal as hal;
use hal::clock::Clocks;
use hal::serial::{
    config::{DataBits, Parity, StopBits},
    Instance,
};

use enumset::{EnumSet, EnumSetType};

#[derive(EnumSetType, Debug)]
pub enum Inverse {
    Tx,
    Rx,
    Cts,
    Rts,
}

/// Change the number of stop bits
pub fn change_stop_bits<I: Instance>(uart: &mut I, stop_bits: StopBits) {
    uart.register_block()
        .conf0
        .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8) });
}

/// Change the number of data bits
pub fn change_data_bits<I: Instance>(uart: &mut I, data_bits: DataBits) {
    uart.register_block()
        .conf0
        .modify(|_, w| unsafe { w.bit_num().bits(data_bits as u8) });
}

/// Change the type of parity checking
pub fn change_parity<I: Instance>(uart: &mut I, parity: Parity) {
    uart.register_block().conf0.modify(|_, w| match parity {
        Parity::ParityNone => w.parity_en().clear_bit(),
        Parity::ParityEven => w.parity_en().set_bit().parity().clear_bit(),
        Parity::ParityOdd => w.parity_en().set_bit().parity().set_bit(),
    });
}

pub fn change_baud<I: Instance>(uart: &mut I, baudrate: u32, clocks: &Clocks) {
    // we force the clock source to be APB and don't use the decimal part of the
    // divider
    let clk = clocks.apb_clock.to_Hz();
    let max_div = 0b1111_1111_1111 - 1;
    let clk_div = ((clk) + (max_div * baudrate) - 1) / (max_div * baudrate);

    uart.register_block().clk_conf.write(|w| unsafe {
        w.sclk_sel()
            .bits(1) // APB
            .sclk_div_a()
            .bits(0)
            .sclk_div_b()
            .bits(0)
            .sclk_div_num()
            .bits(clk_div as u8 - 1)
            .rx_sclk_en()
            .bit(true)
            .tx_sclk_en()
            .bit(true)
    });

    let clk = clk / clk_div;
    let divider = clk / baudrate;
    let divider = divider as u16;

    uart.register_block()
        .clkdiv
        .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });
}

pub fn change_inverse<I: Instance>(uart: &mut I, inverse: EnumSet<Inverse>) {
    uart.register_block()
        .conf0
        .modify(|_, w| {
            w
                .txd_inv().bit(inverse.contains(Inverse::Tx))
                .rxd_inv().bit(inverse.contains(Inverse::Rx))
                .cts_inv().bit(inverse.contains(Inverse::Cts))
                .rts_inv().bit(inverse.contains(Inverse::Rts))
        });
}

pub fn set_rx_fifo_full_threshold<I: Instance>(uart: &mut I, threshold: u16) {
    uart.register_block()
        .conf1
        .modify(|_, w| w.rxfifo_full_thrhd().variant(threshold));
}

pub fn set_tx_fifo_empty_threshold<I: Instance>(uart: &mut I, threshold: u16) {
    uart.register_block()
        .conf1
        .modify(|_, w| w.txfifo_empty_thrhd().variant(threshold));
}
