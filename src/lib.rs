//! nRF24L01+ driver for use with [embedded-hal](https://crates.io/crates/embedded-hal)

#![no_std]
#[macro_use]
extern crate bitfield;

pub mod setup;

mod command;
mod config;
mod device;
mod payload;
mod registers;

pub use crate::config::{Configuration, CrcMode, DataRate};
pub use crate::payload::Payload;

use crate::command::{FlushTx, ReadRxPayload, ReadRxPayloadWidth, WriteTxPayload};
use crate::device::{Device, DeviceImpl};
use crate::registers::{FifoStatus, Status};
use core::fmt::Debug;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;

/// Number of RX pipes with configurable addresses
pub const PIPES_COUNT: usize = 6;
/// Minimum address length
pub const MIN_ADDR_BYTES: usize = 2;
/// Maximum address length
pub const MAX_ADDR_BYTES: usize = 5;

pub const NUM_PIPES: usize = 6;
pub const RX_ADDR_LEN: usize = 5;
pub const RX_ADDR_PREFIX_LEN: usize = 4;

pub struct Config {
    auto_retransmit_delay: u8,
    auto_retransmit_count: u8,
    data_rate: DataRate,
    power: u8,
    crc_mode: CrcMode,
    frequency: u8,
    rx_prefix: Option<[u8; RX_ADDR_PREFIX_LEN]>,
    rx_enabled: [bool; NUM_PIPES],
    rx_length: [Option<u8>; NUM_PIPES],
    rx_auto_ack: [bool; NUM_PIPES],
    rx_addr: [u8; NUM_PIPES],
}

impl Config {
    pub fn default() -> Self {
        Self {
            auto_retransmit_delay: 1,
            auto_retransmit_count: 10,
            data_rate: DataRate::R250Kbps,
            power: 3,
            crc_mode: CrcMode::TwoBytes,
            frequency: 42,
            rx_prefix: None,
            rx_enabled: [false; NUM_PIPES],
            rx_length: [None; NUM_PIPES],
            rx_auto_ack: [true; NUM_PIPES],
            rx_addr: [0; NUM_PIPES],
        }
    }
    pub fn auto_retransmit_delay(mut self, delay: u8) -> Self {
        self.auto_retransmit_delay = delay;
        self
    }
    pub fn auto_retransmit_count(mut self, count: u8) -> Self {
        self.auto_retransmit_count = count;
        self
    }
    pub fn data_rate(mut self, rate: DataRate) -> Self {
        self.data_rate = rate;
        self
    }
    pub fn power(mut self, power: u8) -> Self {
        self.power = power;
        self
    }
    pub fn crc_mode(mut self, mode: CrcMode) -> Self {
        self.crc_mode = mode;
        self
    }
    pub fn frequency(mut self, freq: u8) -> Self {
        self.frequency = freq;
        self
    }
    pub fn rx_prefix(mut self, prefix: [u8; RX_ADDR_PREFIX_LEN]) -> Self {
        self.rx_prefix = Some(prefix);
        self
    }
    pub fn rx_full(mut self, pipe: u8, address: u8, length: u8, auto_ack: bool) -> Self {
        assert!(pipe >= 1);
        assert!(pipe < 6);
        let pipe = pipe as usize;
        self.rx_enabled[pipe] = true;
        self.rx_addr[pipe] = address;
        self.rx_length[pipe] = Some(length);
        self.rx_auto_ack[pipe] = auto_ack;
        self
    }
    pub fn rx(mut self, pipe: u8, address: u8) -> Self {
        assert!(pipe >= 1);
        assert!(pipe < 6);
        let pipe = pipe as usize;
        self.rx_enabled[pipe] = true;
        self.rx_addr[pipe] = address;
        self
    }
    fn configure<T: Configuration>(
        self,
        device: &mut T,
    ) -> Result<(), <<T as Configuration>::Inner as Device>::Error> {
        device.set_auto_retransmit(self.auto_retransmit_delay, self.auto_retransmit_count)?;
        device.set_rf(&self.data_rate, self.power)?;
        device.set_crc(self.crc_mode)?;
        device.set_frequency(self.frequency)?;
        device.set_pipes_rx_enable(&self.rx_enabled)?;
        device.set_pipes_rx_lengths(&self.rx_length)?;
        device.set_auto_ack(&self.rx_auto_ack)?;

        // This improves the error rate, not sure why or if this is the best place for a wait
        wait(100);

        if let Some(rx_prefix) = self.rx_prefix {
            let address = [
                self.rx_addr[1],
                rx_prefix[0],
                rx_prefix[1],
                rx_prefix[2],
                rx_prefix[3],
            ];
            device.set_rx_addr(1, &address)?;

            for i in 2..NUM_PIPES {
                if self.rx_enabled[i] {
                    device.set_rx_addr(i, &[self.rx_addr[i]])?;
                }
            }
        }

        Ok(())
    }
}

pub struct Nrf24l01<Ce, Csn, Spi, E, SpiE>
where
    Ce: OutputPin<Error = E>,
    Csn: OutputPin<Error = E>,
    Spi: Transfer<u8, Error = SpiE>,
    E: Debug,
    SpiE: Debug,
{
    mode: Mode,
    device: DeviceImpl<Ce, Csn, Spi, E>,
}
impl<Ce, Csn, Spi, E, SpiE> Nrf24l01<Ce, Csn, Spi, E, SpiE>
where
    Ce: OutputPin<Error = E>,
    Csn: OutputPin<Error = E>,
    Spi: Transfer<u8, Error = SpiE>,
    E: Debug,
    SpiE: Debug,
{
    pub fn new(ce: Ce, csn: Csn, spi: Spi, config: Config) -> Result<Self, Error<SpiE>> {
        let mut result = Self {
            mode: Mode::Standby,
            device: DeviceImpl::new(ce, csn, spi)?,
        };
        config.configure(&mut result)?;
        result
            .device
            .update_config(|config| config.set_pwr_up(true))?;
        Ok(result)
    }
    pub fn config() -> Config {
        Config::default()
    }
    fn clear(&mut self, interrupts: Interrupts) -> Result<(), SpiE> {
        let mut clear = Status(0);
        clear.set_rx_dr(interrupts.rx_dr);
        clear.set_tx_ds(interrupts.tx_ds);
        clear.set_max_rt(interrupts.max_rt);
        self.device.write_register(clear)?;
        Ok(())
    }
    pub fn clear_interrupts(&mut self) -> Result<(), SpiE> {
        self.clear(Interrupts::new().set_rx_dr().set_tx_ds().set_max_rt())?;
        Ok(())
    }
    fn rx(&mut self) -> Result<(), nb::Error<SpiE>> {
        if self.mode == Mode::Rx {
            return Ok(());
        }
        self.wait_tx_empty()?;
        self.device.ce_enable();
        self.device
            .update_config(|config| config.set_prim_rx(true))?;
        self.mode = Mode::Rx;
        Ok(())
    }
    fn tx(&mut self) -> Result<(), SpiE> {
        if self.mode == Mode::Tx {
            return Ok(());
        }
        self.device.ce_disable();
        self.device
            .update_config(|config| config.set_prim_rx(false))?;
        self.mode = Mode::Tx;
        Ok(())
    }
    pub fn send(&mut self, packet: &[u8]) -> Result<(), nb::Error<SpiE>> {
        self.tx()?;
        self.wait_tx_ready()?;
        self.device.send_command(&WriteTxPayload::new(packet))?;
        self.device.ce_enable();
        Ok(())
    }
    pub fn wait_tx_ready(&mut self) -> Result<(), nb::Error<SpiE>> {
        self.tx()?;
        let (status, fifo_status) = self.device.read_register::<FifoStatus>()?;
        if status.max_rt() {
            self.device.send_command(&FlushTx)?;
            self.clear(Interrupts::new().set_tx_ds().set_max_rt())?;
        }
        match fifo_status.tx_full() {
            true => Err(nb::Error::WouldBlock),
            false => Ok(()),
        }
    }
    pub fn wait_tx_empty(&mut self) -> Result<(), nb::Error<SpiE>> {
        self.tx()?;
        let (status, fifo_status) = self.device.read_register::<FifoStatus>()?;
        if status.max_rt() {
            self.device.send_command(&FlushTx)?;
            self.clear(Interrupts::new().set_tx_ds().set_max_rt())?;
        }
        match fifo_status.tx_empty() {
            true => {
                self.device.ce_disable();
                Ok(())
            }
            false => Err(nb::Error::WouldBlock),
        }
    }
    pub fn wait_rx_ready(&mut self) -> Result<u8, nb::Error<SpiE>> {
        self.rx()?;
        let (status, fifo_status) = self.device.read_register::<FifoStatus>()?;
        match fifo_status.rx_empty() {
            true => Err(nb::Error::WouldBlock),
            false => Ok(status.rx_p_no()),
        }
    }
    pub fn read(&mut self) -> Result<Payload, nb::Error<SpiE>> {
        self.rx()?;
        let (_, payload_width) = self.device.send_command(&ReadRxPayloadWidth)?;
        let (_, payload) = self
            .device
            .send_command(&ReadRxPayload::new(payload_width as usize))?;
        Ok(payload)
    }
}

#[derive(PartialEq)]
enum Mode {
    Standby,
    Rx,
    Tx,
}

struct Interrupts {
    rx_dr: bool,
    tx_ds: bool,
    max_rt: bool,
}
impl Interrupts {
    fn new() -> Self {
        Self {
            rx_dr: false,
            tx_ds: false,
            max_rt: false,
        }
    }
    fn set_rx_dr(mut self) -> Self {
        self.rx_dr = true;
        self
    }
    fn set_tx_ds(mut self) -> Self {
        self.tx_ds = true;
        self
    }
    fn set_max_rt(mut self) -> Self {
        self.max_rt = true;
        self
    }
}
impl<Ce, Csn, Spi, E, SpiE> Configuration for Nrf24l01<Ce, Csn, Spi, E, SpiE>
where
    Ce: OutputPin<Error = E>,
    Csn: OutputPin<Error = E>,
    Spi: Transfer<u8, Error = SpiE>,
    E: Debug,
    SpiE: Debug,
{
    type Inner = DeviceImpl<Ce, Csn, Spi, E>;
    fn device(&mut self) -> &mut Self::Inner {
        &mut self.device
    }
}

fn wait(mut count: u32) {
    while count > 0 {
        unsafe { core::ptr::read_volatile(&count) };
        count -= 1;
    }
}

#[derive(Debug)]
pub enum Error<E: Debug> {
    NotConnected,
    Spi(E),
}
impl<SpiE: Debug> From<SpiE> for Error<SpiE> {
    fn from(e: SpiE) -> Self {
        Error::Spi(e)
    }
}
