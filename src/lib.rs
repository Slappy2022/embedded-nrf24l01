//! nRF24L01+ driver for use with [embedded-hal](https://crates.io/crates/embedded-hal)

#![no_std]
#[macro_use]
extern crate bitfield;

use crate::command::{FlushTx, ReadRxPayload, ReadRxPayloadWidth, WriteTxPayload};
use crate::registers::{FifoStatus, TxAddr};
use core::fmt;
use core::fmt::Debug;
use embedded_hal::blocking::spi::Transfer as SpiTransfer;
use embedded_hal::digital::v2::OutputPin;

mod config;
pub use crate::config::{Configuration, CrcMode, DataRate};
pub mod setup;

mod registers;
use crate::registers::{Config, Register, SetupAw, Status};
mod command;
use crate::command::{Command, ReadRegister, WriteRegister};
mod payload;
pub use crate::payload::Payload;
mod error;
pub use crate::error::Error;

mod device;
pub use crate::device::Device;

/// Number of RX pipes with configurable addresses
pub const PIPES_COUNT: usize = 6;
/// Minimum address length
pub const MIN_ADDR_BYTES: usize = 2;
/// Maximum address length
pub const MAX_ADDR_BYTES: usize = 5;

/// Driver for the nRF24L01+
///
pub struct NRF24L01<E: Debug, CE: OutputPin<Error = E>, CSN: OutputPin<Error = E>, SPI: SpiTransfer<u8>> {
    ce: CE,
    csn: CSN,
    spi: SPI,
    config: Config,
}

impl<E: Debug, CE: OutputPin<Error = E>, CSN: OutputPin<Error = E>, SPI: SpiTransfer<u8, Error = SPIE>, SPIE: Debug> fmt::Debug
    for NRF24L01<E, CE, CSN, SPI>
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "NRF24L01")
    }
}

impl<E: Debug, CE: OutputPin<Error = E>, CSN: OutputPin<Error = E>, SPI: SpiTransfer<u8, Error = SPIE>, SPIE: Debug>
    NRF24L01<E, CE, CSN, SPI>
{
    /// Construct a new driver instance.
    pub fn new(mut ce: CE, mut csn: CSN, spi: SPI) -> Result<Nrf24l01<Self>, Error<SPIE>> {
        ce.set_low().unwrap();
        csn.set_high().unwrap();

        // Reset value
        let mut config = Config(0b0000_1000);
        config.set_mask_rx_dr(false);
        config.set_mask_tx_ds(false);
        config.set_mask_max_rt(false);
        let mut device = NRF24L01 {
            ce,
            csn,
            spi,
            config,
        };

        match device.is_connected() {
            Err(e) => return Err(e),
            Ok(false) => return Err(Error::NotConnected),
            _ => {}
        }
        Nrf24l01::new(device)
    }

    /// Reads and validates content of the `SETUP_AW` register.
    pub fn is_connected(&mut self) -> Result<bool, Error<SPIE>> {
        let (_, setup_aw) = self.read_register::<SetupAw>()?;
        let valid = setup_aw.aw() <= 3;
        Ok(valid)
    }
}

impl<E: Debug, CE: OutputPin<Error = E>, CSN: OutputPin<Error = E>, SPI: SpiTransfer<u8, Error = SPIE>, SPIE: Debug> Device
    for NRF24L01<E, CE, CSN, SPI>
{
    type Error = Error<SPIE>;

    fn ce_enable(&mut self) {
        self.ce.set_high().unwrap();
    }

    fn ce_disable(&mut self) {
        self.ce.set_low().unwrap();
    }

    fn send_command<C: Command>(
        &mut self,
        command: &C,
    ) -> Result<(Status, C::Response), Self::Error> {
        // Allocate storage
        let mut buf_storage = [0; 33];
        let len = command.len();
        let buf = &mut buf_storage[0..len];
        // Serialize the command
        command.encode(buf);

        // SPI transaction
        self.csn.set_low().unwrap();
        let transfer_result = self.spi.transfer(buf).map(|_| {});
        self.csn.set_high().unwrap();
        // Propagate Err only after csn.set_high():
        transfer_result?;

        // Parse response
        let status = Status(buf[0]);
        let response = C::decode_response(buf);

        Ok((status, response))
    }

    fn write_register<R: Register>(&mut self, register: R) -> Result<Status, Self::Error> {
        let (status, ()) = self.send_command(&WriteRegister::new(register))?;
        Ok(status)
    }

    fn read_register<R: Register>(&mut self) -> Result<(Status, R), Self::Error> {
        self.send_command(&ReadRegister::new())
    }

    fn update_config<F, R>(&mut self, f: F) -> Result<R, Self::Error>
    where
        F: FnOnce(&mut Config) -> R,
    {
        // Mutate
        let old_config = self.config.clone();
        let result = f(&mut self.config);

        if self.config != old_config {
            let config = self.config.clone();
            self.write_register(config)?;
        }
        Ok(result)
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

pub struct Nrf24l01<D: Device> {
    mode: Mode,
    device: D,
}

impl<D: Device> Nrf24l01<D> {
    pub fn new(mut device: D) -> Result<Self, D::Error> {
        device.update_config(|config| config.set_pwr_up(true))?;
        Ok(Self {
            mode: Mode::Standby,
            device
        })
    }
    fn clear(&mut self, interrupts: Interrupts) -> Result<(), D::Error> {
        let mut clear = Status(0);
        clear.set_rx_dr(interrupts.rx_dr);
        clear.set_tx_ds(interrupts.tx_ds);
        clear.set_max_rt(interrupts.max_rt);
        self.device.write_register(clear)?;
        Ok(())
    }
    pub fn clear_interrupts(&mut self) -> Result<(), D::Error> {
        self.clear(Interrupts::new().set_rx_dr().set_tx_ds().set_max_rt())?;
        Ok(())
    }
    fn rx(&mut self) -> Result<(), nb::Error<D::Error>> {
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
    fn tx(&mut self) -> Result<(), D::Error> {
        if self.mode == Mode::Tx {
            return Ok(());
        }
        self.device.ce_disable();
        self.device
            .update_config(|config| config.set_prim_rx(false))?;
        self.mode = Mode::Tx;
        Ok(())
    }
    pub fn send(&mut self, packet: &[u8]) -> Result<(), nb::Error<D::Error>> {
        self.tx()?;
        self.device.send_command(&WriteTxPayload::new(packet))?;
        self.device.ce_enable();
        Ok(())
    }
    fn is_tx_full(&mut self) -> Result<bool, D::Error> {
        self.tx()?;
        let (_, fifo_status) = self.device.read_register::<FifoStatus>()?;
        Ok(fifo_status.tx_full())
    }
    pub fn wait_tx_ready(&mut self) -> Result<(), nb::Error<D::Error>> {
        self.tx()?;
        match self.is_tx_full()? {
            true => Err(nb::Error::WouldBlock),
            false => Ok(()),
        }
    }
    pub fn wait_tx_empty(&mut self) -> Result<(), nb::Error<D::Error>> {
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
    pub fn wait_rx_ready(&mut self) -> Result<u8, nb::Error<D::Error>> {
        self.rx()?;
        let (status, fifo_status) = self.device.read_register::<FifoStatus>()?;
        match fifo_status.rx_empty() {
            true => Err(nb::Error::WouldBlock),
            false => Ok(status.rx_p_no()),
        }
    }
    pub fn read(&mut self) -> Result<Payload, nb::Error<D::Error>> {
        self.rx()?;
        let (_, payload_width) = self.device.send_command(&ReadRxPayloadWidth)?;
        let (_, payload) = self
            .device
            .send_command(&ReadRxPayload::new(payload_width as usize))?;
        Ok(payload)
    }
    pub fn set_tx_addr(&mut self, addr: &[u8]) -> Result<(), D::Error> {
        let register = TxAddr::new(addr);
        self.device.write_register(register)?;
        Ok(())
    }
}
impl<D: Device> Configuration for Nrf24l01<D> {
    type Inner = D;
    fn device(&mut self) -> &mut Self::Inner {
        &mut self.device
    }
}
