//! External adcs are connected to the two I2C busses (com1 and com2).
//! Each of them can service 9 adcs with unique ids. We use (ADC101C027 based sensors).

use embassy_time::{Duration, with_timeout};

use crate::index::I2cBus;

const I2C_TIMEOUT: Duration = Duration::from_millis(100);

/// A probe of an address nobody is at should NACK immediately, so it gets a much tighter budget.
const I2C_PROBE_TIMEOUT: Duration = Duration::from_millis(10);

#[derive(Copy, Clone, Debug)]
pub struct Reading {
    /// 10-bit conversion result.
    pub value: u16,
    /// The device's ALERT flag, set when its own window comparator tripped.
    pub alert: bool,
}

/// The pair of I2C buses, either of which may be absent on a partly populated board.
pub struct Buses<I0, I1> {
    pub bus0: Option<I0>,
    pub bus1: Option<I1>,
}

/// The concrete pair the firmware constructs: both buses are the same peripheral type, wired to
/// `'static` singletons the way `board::init_board` hands them out.
#[cfg(feature = "hardware")]
pub type BoardBuses = Buses<
    &'static mut embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>,
    &'static mut embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>,
>;

impl<I0: embedded_hal_async::i2c::I2c, I1: embedded_hal_async::i2c::I2c> Buses<I0, I1> {
    /// Read one amplifier's conversion register.
    ///
    /// Taking an [`I2cBus`] rather than a `usize` makes the match exhaustive: there is no longer
    /// an "invalid bus number" arm that quietly reports the amplifier as absent.
    pub async fn read(&mut self, bus: I2cBus, address: u8) -> Option<Reading> {
        match bus {
            I2cBus::Bus0 => read_conversion_opt(self.bus0.as_mut(), address, I2C_TIMEOUT).await,
            I2cBus::Bus1 => read_conversion_opt(self.bus1.as_mut(), address, I2C_TIMEOUT).await,
        }
    }

    /// Probe one address to see whether anything answers, without disturbing the sample rate.
    pub async fn probe(&mut self, bus: I2cBus, address: u8) -> bool {
        match bus {
            I2cBus::Bus0 => read_conversion_opt(self.bus0.as_mut(), address, I2C_PROBE_TIMEOUT).await.is_some(),
            I2cBus::Bus1 => read_conversion_opt(self.bus1.as_mut(), address, I2C_PROBE_TIMEOUT).await.is_some(),
        }
    }
}

/// `read_conversion`, but for a bus that might not be populated at all.
async fn read_conversion_opt<I2C: embedded_hal_async::i2c::I2c>(
    i2c: Option<&mut I2C>,
    address: u8,
    timeout: Duration,
) -> Option<Reading> {
    read_conversion(i2c?, address, timeout).await
}

/// <https://www.ti.com/lit/ds/symlink/adc101c027.pdf>: a plain 2-byte read returns the conversion
/// register, with the alert flag in bit 15 and the 10-bit result in bits 12..2.
async fn read_conversion<I2C: embedded_hal_async::i2c::I2c>(
    i2c: &mut I2C,
    address: u8,
    timeout: Duration,
) -> Option<Reading> {
    let mut buffer = [0u8; 2];
    // TODO: check cancel safety
    let result = with_timeout(timeout, i2c.read(address, &mut buffer)).await;

    match result {
        Ok(Ok(())) => {
            let register = u16::from_be_bytes(buffer);
            Some(Reading {
                value: (register >> 2) & 0x3FF,
                alert: (register >> 15) != 0,
            })
        }
        Ok(Err(e)) => {
            // A NACK is the normal answer from an address with nothing on it, so this is only
            // worth a line at trace level.
            defmt::trace!("i2c addr {=u8:#04x}: nack or bus error", address);
            let _ = e;
            None
        }
        Err(_) => {
            defmt::warn!("i2c addr {=u8:#04x}: timed out after {} ms", address, timeout.as_millis());
            None
        }
    }
}
