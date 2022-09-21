//! nRF24L01+ driver for use with [embedded-hal](https://crates.io/crates/embedded-hal)

#![no_std]
#[macro_use]
extern crate bitfield;

pub mod setup;

mod command;
mod config;
mod device;
mod error;
mod payload;
mod registers;

pub use crate::config::{Configuration, CrcMode, DataRate};
pub use crate::error::Error;
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
    pub fn new(ce: Ce, csn: Csn, spi: Spi) -> Result<Self, Error<SpiE>> {
        let mut device = DeviceImpl::new(ce, csn, spi)?;
        device.update_config(|config| config.set_pwr_up(true))?;
        Ok(Self {
            mode: Mode::Standby,
            device,
        })
    }
    fn clear(&mut self, interrupts: Interrupts) -> Result<(), Error<SpiE>> {
        let mut clear = Status(0);
        clear.set_rx_dr(interrupts.rx_dr);
        clear.set_tx_ds(interrupts.tx_ds);
        clear.set_max_rt(interrupts.max_rt);
        self.device.write_register(clear)?;
        Ok(())
    }
    pub fn clear_interrupts(&mut self) -> Result<(), Error<SpiE>> {
        self.clear(Interrupts::new().set_rx_dr().set_tx_ds().set_max_rt())?;
        Ok(())
    }
    fn rx(&mut self) -> Result<(), nb::Error<Error<SpiE>>> {
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
    fn tx(&mut self) -> Result<(), Error<SpiE>> {
        if self.mode == Mode::Tx {
            return Ok(());
        }
        self.device.ce_disable();
        self.device
            .update_config(|config| config.set_prim_rx(false))?;
        self.mode = Mode::Tx;
        Ok(())
    }
    pub fn send(&mut self, packet: &[u8]) -> Result<(), nb::Error<Error<SpiE>>> {
        self.tx()?;
        self.device.send_command(&WriteTxPayload::new(packet))?;
        self.device.ce_enable();
        Ok(())
    }
    fn is_tx_full(&mut self) -> Result<bool, Error<SpiE>> {
        self.tx()?;
        let (_, fifo_status) = self.device.read_register::<FifoStatus>()?;
        Ok(fifo_status.tx_full())
    }
    pub fn wait_tx_ready(&mut self) -> Result<(), nb::Error<Error<SpiE>>> {
        self.tx()?;
        match self.is_tx_full()? {
            true => Err(nb::Error::WouldBlock),
            false => Ok(()),
        }
    }
    pub fn wait_tx_empty(&mut self) -> Result<(), nb::Error<Error<SpiE>>> {
        self.tx()?;
        let (status, fifo_status) = self.device.read_register::<FifoStatus>()?;
        if fifo_status.tx_empty() {
            self.device.ce_disable();
            return Ok(());
        }
        if status.max_rt() {
            self.device.send_command(&FlushTx)?;
            self.clear(Interrupts::new().set_tx_ds().set_max_rt())?;
        }
        Err(nb::Error::WouldBlock)
    }
    pub fn wait_rx_ready(&mut self) -> Result<u8, nb::Error<Error<SpiE>>> {
        self.rx()?;
        let (status, fifo_status) = self.device.read_register::<FifoStatus>()?;
        match fifo_status.rx_empty() {
            true => Err(nb::Error::WouldBlock),
            false => Ok(status.rx_p_no()),
        }
    }
    pub fn read(&mut self) -> Result<Payload, nb::Error<Error<SpiE>>> {
        self.rx()?;
        let (_, payload_width) = self.device.send_command(&ReadRxPayloadWidth)?;
        let (_, payload) = self
            .device
            .send_command(&ReadRxPayload::new(payload_width as usize))?;
        Ok(payload)
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
