use crate::command::{Command, ReadRegister, WriteRegister};
use crate::registers::{Config, Register, SetupAw, Status};
use crate::Error;
use core::fmt::Debug;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;

pub struct DeviceImpl<
    Ce: OutputPin<Error = E>,
    Csn: OutputPin<Error = E>,
    Spi: Transfer<u8>,
    E: Debug,
> {
    ce: Ce,
    csn: Csn,
    spi: Spi,
    config: Config,
}

impl<
        Ce: OutputPin<Error = E>,
        Csn: OutputPin<Error = E>,
        Spi: Transfer<u8, Error = SpiE>,
        E: Debug,
        SpiE: Debug,
    > DeviceImpl<Ce, Csn, Spi, E>
{
    /// Construct a new driver instance.
    pub fn new(mut ce: Ce, mut csn: Csn, spi: Spi) -> Result<Self, Error<SpiE>> {
        ce.set_low().unwrap();
        csn.set_high().unwrap();

        // Reset value
        let mut config = Config(0b0000_1000);
        config.set_mask_rx_dr(false);
        config.set_mask_tx_ds(false);
        config.set_mask_max_rt(false);
        let mut device = DeviceImpl {
            ce,
            csn,
            spi,
            config,
        };

        match device.is_connected()? {
            true => Ok(device),
            false => Err(Error::NotConnected),
        }
    }

    /// Reads and validates content of the `SETUP_AW` register.
    pub fn is_connected(&mut self) -> Result<bool, SpiE> {
        let (_, setup_aw) = self.read_register::<SetupAw>()?;
        let valid = setup_aw.aw() <= 3;
        Ok(valid)
    }
}

impl<
        Ce: OutputPin<Error = E>,
        Csn: OutputPin<Error = E>,
        Spi: Transfer<u8, Error = SpiE>,
        E: Debug,
        SpiE: Debug,
    > Device for DeviceImpl<Ce, Csn, Spi, E>
{
    type Error = SpiE;

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

        // Spi transaction
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

/// Trait that hides all the GPIO/Spi type parameters for use by the
/// operation modes
pub trait Device {
    /// Error from the Spi implementation
    type Error;

    /// Set Ce pin high
    fn ce_enable(&mut self);
    /// Set Ce pin low
    fn ce_disable(&mut self);
    /// Helper; the receiving during RX and sending during TX require `Ce`
    /// to be low.
    fn with_ce_disabled<F, R>(&mut self, f: F) -> R
    where
        F: FnOnce(&mut Self) -> R,
    {
        self.ce_disable();
        let r = f(self);
        self.ce_enable();
        r
    }

    /// Send a command via Spi
    fn send_command<C: Command>(
        &mut self,
        command: &C,
    ) -> Result<(Status, C::Response), Self::Error>;
    /// Send `W_REGISTER` command
    fn write_register<R: Register>(&mut self, register: R) -> Result<Status, Self::Error>;
    /// Send `R_REGISTER` command
    fn read_register<R: Register>(&mut self) -> Result<(Status, R), Self::Error>;

    /// Read, and modify a register, and write it back if it has been changed.
    fn update_register<Reg, F, R>(&mut self, f: F) -> Result<R, Self::Error>
    where
        Reg: Register + PartialEq + Clone,
        F: FnOnce(&mut Reg) -> R,
    {
        // Use `update_config()` for `registers::Config`
        assert!(Reg::addr() != 0x00);

        let (_, old_register) = self.read_register::<Reg>()?;
        let mut register = old_register.clone();
        let result = f(&mut register);

        if register != old_register {
            self.write_register(register)?;
        }
        Ok(result)
    }

    /// Modify the (cached) `CONFIG` register and write if it has changed.
    fn update_config<F, R>(&mut self, f: F) -> Result<R, Self::Error>
    where
        F: FnOnce(&mut Config) -> R;
}
