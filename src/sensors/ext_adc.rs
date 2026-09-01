//! The devices on the two I2C busses (com1 and com2).
//!
//! Each bus can service 9 amplifiers with unique address straps (ADC101C027 based sensors), plus
//! at most one AS5600 magnetic rotary encoder — the encoder's address is fixed in the part, so
//! the bus it is on is the whole of its identity.

use embassy_time::{Duration, with_timeout};
use embedded_hal_async::i2c::ErrorKind;

use crate::errors::{ErrorCounter, bump};
use crate::index::I2cBus;

const I2C_TIMEOUT: Duration = Duration::from_millis(100);

/// A probe of an address nobody is at should NACK immediately, so it gets a much tighter budget.
const I2C_PROBE_TIMEOUT: Duration = Duration::from_millis(10);

/// Why a transfer did not produce a reading.
///
/// The three are worth telling apart because only one of them can be normal: an address with
/// nothing on it NACKs, which is exactly what a presence probe is asking. A timeout or a bus
/// error is a fault no matter who asked — an empty address still answers a probe *immediately*,
/// so a probe that times out means the bus itself is in trouble.
#[derive(Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
pub enum I2cFault {
    /// Nobody acknowledged the address.
    Nack,
    /// Arbitration loss, a bus error, an overrun — the peripheral could not complete the
    /// transfer.
    Bus,
    /// The transfer did not finish inside its budget: a held SDA, a missing pull-up, or a device
    /// stretching the clock forever.
    Timeout,
}

impl I2cFault {
    /// Count this fault, given whether we had reason to believe a device was there.
    ///
    /// `expected` is false for a presence probe, which is the one case where a NACK is the
    /// expected answer rather than a failure. Everything else is counted either way.
    fn note(self, expected: bool) {
        match self {
            Self::Nack if expected => bump(ErrorCounter::I2cNack),
            Self::Nack => {}
            Self::Bus => bump(ErrorCounter::I2cBusError),
            Self::Timeout => bump(ErrorCounter::I2cTimeout),
        }
    }

    fn from_error<E: embedded_hal_async::i2c::Error>(e: E) -> Self {
        match e.kind() {
            ErrorKind::NoAcknowledge(_) => Self::Nack,
            _ => Self::Bus,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Reading {
    /// 10-bit conversion result.
    pub value: u16,
    /// The device's ALERT flag, set when its own window comparator tripped.
    pub alert: bool,
}

/// One AS5600 reading: the unfiltered angle plus what the part thinks of its magnet.
#[derive(Copy, Clone, Debug)]
pub struct AngleReading {
    /// 12-bit `RAW_ANGLE`, 0..4095 over a full turn.
    pub angle: u16,
    /// `MD`: a magnet is detected at all.
    pub magnet_detected: bool,
    /// `ML` or `MH`: the magnet is there but too far away or too close, so the angle is not to
    /// be trusted even though the part answers with one.
    pub magnet_out_of_range: bool,
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
        let result = match bus {
            I2cBus::Bus0 => read_conversion_opt(self.bus0.as_mut(), address, I2C_TIMEOUT).await,
            I2cBus::Bus1 => read_conversion_opt(self.bus1.as_mut(), address, I2C_TIMEOUT).await,
        };
        note(result, true)
    }

    /// Probe one address to see whether anything answers, without disturbing the sample rate.
    pub async fn probe(&mut self, bus: I2cBus, address: u8) -> bool {
        let result = match bus {
            I2cBus::Bus0 => read_conversion_opt(self.bus0.as_mut(), address, I2C_PROBE_TIMEOUT).await,
            I2cBus::Bus1 => read_conversion_opt(self.bus1.as_mut(), address, I2C_PROBE_TIMEOUT).await,
        };
        note(result, false).is_some()
    }

    /// Read the AS5600 on `bus`, if one answers.
    pub async fn read_angle(&mut self, bus: I2cBus) -> Option<AngleReading> {
        let result = match bus {
            I2cBus::Bus0 => read_angle_opt(self.bus0.as_mut(), I2C_TIMEOUT).await,
            I2cBus::Bus1 => read_angle_opt(self.bus1.as_mut(), I2C_TIMEOUT).await,
        };
        note(result, true)
    }

    /// Probe for an AS5600 on `bus`, on the tight budget the presence sweep uses.
    pub async fn probe_angle(&mut self, bus: I2cBus) -> bool {
        let result = match bus {
            I2cBus::Bus0 => read_angle_opt(self.bus0.as_mut(), I2C_PROBE_TIMEOUT).await,
            I2cBus::Bus1 => read_angle_opt(self.bus1.as_mut(), I2C_PROBE_TIMEOUT).await,
        };
        note(result, false).is_some()
    }
}

/// Count whatever went wrong and reduce to the `Option` the callers want.
///
/// The counting lives here, at the one place that knows both the fault and whether a device was
/// expected — the transfer functions below cannot tell a sample from a probe, and the sensor task
/// above cannot tell a NACK from a timeout.
fn note<T>(result: Result<T, I2cFault>, expected: bool) -> Option<T> {
    match result {
        Ok(value) => Some(value),
        Err(fault) => {
            fault.note(expected);
            None
        }
    }
}

async fn read_angle_opt<I2C: embedded_hal_async::i2c::I2c>(
    i2c: Option<&mut I2C>,
    timeout: Duration,
) -> Result<AngleReading, I2cFault> {
    // An unpopulated bus is not a fault to count — the board simply has nothing there — but it
    // has no reading either, and `Nack` is the fault that is only counted when something was
    // expected, so it is the honest answer.
    let Some(i2c) = i2c else {
        return Err(I2cFault::Nack);
    };
    read_angle(i2c, timeout).await
}

/// <https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf>: `STATUS` (0x0B),
/// `RAW_ANGLE` high (0x0C) and low (0x0D) are consecutive, so one write-then-read starting at
/// 0x0B gets the angle and the magnet diagnostics together.
///
/// `RAW_ANGLE` rather than `ANGLE` deliberately: `ANGLE` is scaled and clamped by the part's own
/// burned-in ZPOS/MPOS registers, which would put a second, invisible calibration in front of
/// the one in the object dictionary. A blank part reads the same on both; a part that has been
/// programmed on someone's bench does not.
async fn read_angle<I2C: embedded_hal_async::i2c::I2c>(
    i2c: &mut I2C,
    timeout: Duration,
) -> Result<AngleReading, I2cFault> {
    const STATUS_REGISTER: u8 = 0x0B;
    /// `MD`, bit 5 of STATUS.
    const MAGNET_DETECTED: u8 = 1 << 5;
    /// `ML` (too weak, bit 4) or `MH` (too strong, bit 3).
    const MAGNET_OUT_OF_RANGE: u8 = (1 << 4) | (1 << 3);

    let mut buffer = [0u8; 3];
    let transfer = i2c.write_read(crate::config::ENCODER_ADDRESS, &[STATUS_REGISTER], &mut buffer);

    match with_timeout(timeout, transfer).await {
        Ok(Ok(())) => Ok(AngleReading {
            angle: u16::from_be_bytes([buffer[1], buffer[2]]) & 0x0FFF,
            magnet_detected: buffer[0] & MAGNET_DETECTED != 0,
            magnet_out_of_range: buffer[0] & MAGNET_OUT_OF_RANGE != 0,
        }),
        Ok(Err(e)) => {
            defmt::trace!("as5600: nack or bus error");
            Err(I2cFault::from_error(e))
        }
        Err(_) => {
            defmt::warn!("as5600: timed out after {} ms", timeout.as_millis());
            Err(I2cFault::Timeout)
        }
    }
}

/// `read_conversion`, but for a bus that might not be populated at all.
async fn read_conversion_opt<I2C: embedded_hal_async::i2c::I2c>(
    i2c: Option<&mut I2C>,
    address: u8,
    timeout: Duration,
) -> Result<Reading, I2cFault> {
    // See `read_angle_opt` on why an unpopulated bus answers `Nack`.
    let Some(i2c) = i2c else {
        return Err(I2cFault::Nack);
    };
    read_conversion(i2c, address, timeout).await
}

/// <https://www.ti.com/lit/ds/symlink/adc101c027.pdf>: a plain 2-byte read returns the conversion
/// register, with the alert flag in bit 15 and the 10-bit result in bits 12..2.
async fn read_conversion<I2C: embedded_hal_async::i2c::I2c>(
    i2c: &mut I2C,
    address: u8,
    timeout: Duration,
) -> Result<Reading, I2cFault> {
    let mut buffer = [0u8; 2];
    // TODO: check cancel safety
    let result = with_timeout(timeout, i2c.read(address, &mut buffer)).await;

    match result {
        Ok(Ok(())) => {
            let register = u16::from_be_bytes(buffer);
            Ok(Reading {
                value: (register >> 2) & 0x3FF,
                alert: (register >> 15) != 0,
            })
        }
        Ok(Err(e)) => {
            // A NACK is the normal answer from an address with nothing on it, so this is only
            // worth a line at trace level. Whether it is worth *counting* is the caller's call —
            // see `I2cFault::note`.
            defmt::trace!("i2c addr {=u8:#04x}: nack or bus error", address);
            Err(I2cFault::from_error(e))
        }
        Err(_) => {
            defmt::warn!("i2c addr {=u8:#04x}: timed out after {} ms", address, timeout.as_millis());
            Err(I2cFault::Timeout)
        }
    }
}
