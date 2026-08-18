//! AS5600 12-bit magnetic rotary encoder, on either COM header's I2C bus.
//!
//! <https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf>
//!
//! Read-only on purpose. The chip's zero position, angular range and output stage all live in
//! registers that can be burned into OTP (via 0xFF, once and irreversibly), so a driver that never
//! writes anything cannot spoil an encoder by accident. A part as delivered already reports the
//! full 0..4095 range over a turn, which is all this needs.
//!
//! The address is fixed in silicon, so there is at most one encoder per bus — unlike the
//! amplifiers in [`super::ext_adc`], which are strapped to nine different addresses. Which bus an
//! encoder is wired to is a build-time board fact, held in
//! [`NodeSettings::encoder_bus`](crate::config::NodeSettings::encoder_bus).

use embassy_time::{Duration, with_timeout};
use embedded_hal_async::i2c::I2c;

use super::ext_adc::Buses;
use crate::index::I2cBus;

/// The fixed 7-bit slave address. Not strappable — hence one encoder per bus.
pub const ADDRESS: u8 = 0x36;

/// Same budget as an amplifier read: a burst of five registers at 100 kHz needs well under a
/// millisecond, so anything near this is a stuck bus rather than a slow device.
const I2C_TIMEOUT: Duration = Duration::from_millis(100);

/// STATUS (0x0B), then RAW ANGLE (0x0C/0x0D) and ANGLE (0x0E/0x0F): five consecutive registers,
/// which is why one burst read covers the whole measurement.
const REG_STATUS: u8 = 0x0B;

/// AGC (0x1A), then MAGNITUDE (0x1B/0x1C): the magnet-placement diagnostics, three consecutive
/// registers.
const REG_AGC: u8 = 0x1A;

/// Every angle and the magnitude are 12-bit values in a big-endian register pair.
const VALUE_MASK: u16 = 0x0FFF;

/// One full turn, in angle counts.
pub const COUNTS_PER_TURN: u32 = 4096;

/// The STATUS register (0x0B).
///
/// `too_weak` and `too_strong` are the two ends of the automatic gain control running out of
/// range, which is what a magnet mounted too far away or too close looks like. Both are worth
/// seeing during assembly: the chip still reports an angle, just not a trustworthy one.
#[derive(Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
pub struct Status {
    /// MD: a magnet is in range.
    pub magnet_detected: bool,
    /// ML: AGC maximum gain overflow — the magnet is too far away.
    pub too_weak: bool,
    /// MH: AGC minimum gain overflow — the magnet is too close.
    pub too_strong: bool,
    /// The register as read, for the bits this does not name.
    pub bits: u8,
}

impl Status {
    const MD: u8 = 1 << 5;
    const ML: u8 = 1 << 4;
    const MH: u8 = 1 << 3;

    const fn from_bits(bits: u8) -> Self {
        Self {
            magnet_detected: bits & Self::MD != 0,
            too_weak: bits & Self::ML != 0,
            too_strong: bits & Self::MH != 0,
            bits,
        }
    }

    /// True when the angle can be believed: a magnet is present and the gain is not pinned at
    /// either end of its range.
    pub const fn is_usable(&self) -> bool {
        self.magnet_detected && !self.too_weak && !self.too_strong
    }
}

/// One sample of everything the chip has to say, taken in two burst reads.
#[derive(Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
pub struct Reading {
    /// The scaled output, 0..4095 over one turn. Equal to `raw_angle` unless ZPOS/MPOS have been
    /// written, which this driver never does.
    pub angle: u16,
    /// The unscaled measurement, unaffected by the position registers.
    pub raw_angle: u16,
    pub status: Status,
    /// Automatic gain, the useful proxy for magnet distance: it rises as the magnet moves away.
    /// 0..128 at 3.3 V, 0..255 at 5 V.
    pub agc: u8,
    /// CORDIC magnitude of the internal field measurement.
    pub magnitude: u16,
}

impl Reading {
    /// The angle in hundredths of a degree, 0..35999.
    ///
    /// Integer arithmetic, like the rest of the sensor path: the STM32F105 is a Cortex-M3 with no
    /// FPU. The widest intermediate is `4095 * 36000`, which fits a u32 with room to spare.
    pub const fn centi_degrees(&self) -> u16 {
        ((self.angle & VALUE_MASK) as u32 * 36_000 / COUNTS_PER_TURN) as u16
    }
}

/// A 12-bit value from its big-endian register pair.
const fn value12(hi: u8, lo: u8) -> u16 {
    u16::from_be_bytes([hi, lo]) & VALUE_MASK
}

/// Decode the STATUS..ANGLE burst into `(status, raw_angle, angle)`.
const fn decode_measurement(buffer: [u8; 5]) -> (Status, u16, u16) {
    (Status::from_bits(buffer[0]), value12(buffer[1], buffer[2]), value12(buffer[3], buffer[4]))
}

/// Decode the AGC..MAGNITUDE burst into `(agc, magnitude)`.
const fn decode_diagnostics(buffer: [u8; 3]) -> (u8, u16) {
    (buffer[0], value12(buffer[1], buffer[2]))
}

/// Sample the encoder on `bus`, or `None` when it did not answer.
///
/// Takes the [`Buses`] pair rather than one bus because the sensor task owns both peripherals
/// outright — there is no shared-bus mutex to borrow through, and the encoder shares its bus with
/// whatever amplifiers are strapped onto the same header.
pub async fn read<I0: I2c, I1: I2c>(buses: &mut Buses<I0, I1>, bus: I2cBus) -> Option<Reading> {
    match bus {
        I2cBus::Bus0 => read_opt(buses.bus0.as_mut()).await,
        I2cBus::Bus1 => read_opt(buses.bus1.as_mut()).await,
    }
}

/// `read_one`, but for a bus that might not be populated at all.
async fn read_opt<I2C: I2c>(i2c: Option<&mut I2C>) -> Option<Reading> {
    read_one(i2c?).await
}

async fn read_one<I2C: I2c>(i2c: &mut I2C) -> Option<Reading> {
    // Two transfers rather than one 18-register read: the gap between 0x0F and 0x1A is all
    // reserved space, and reading through it would spend three quarters of the transfer on bytes
    // nothing looks at.
    let measurement: [u8; 5] = burst(i2c, REG_STATUS).await?;
    let diagnostics: [u8; 3] = burst(i2c, REG_AGC).await?;

    let (status, raw_angle, angle) = decode_measurement(measurement);
    let (agc, magnitude) = decode_diagnostics(diagnostics);

    Some(Reading {
        angle,
        raw_angle,
        status,
        agc,
        magnitude,
    })
}

/// Write the register address, then read `N` registers from it: the chip auto-increments its
/// address pointer within a read, so consecutive registers come out of one transfer and cannot be
/// torn across two.
async fn burst<I2C: I2c, const N: usize>(i2c: &mut I2C, register: u8) -> Option<[u8; N]> {
    let mut buffer = [0u8; N];
    // TODO: check cancel safety (same question as `ext_adc::read_conversion`)
    let result = with_timeout(I2C_TIMEOUT, i2c.write_read(ADDRESS, &[register], &mut buffer)).await;

    match result {
        Ok(Ok(())) => Some(buffer),
        Ok(Err(_)) => {
            // A NACK is what an unpopulated header looks like, so it is only worth a trace line;
            // the caller logs the appearing and vanishing.
            defmt::trace!("as5600 reg {=u8:#04x}: nack or bus error", register);
            None
        }
        Err(_) => {
            defmt::warn!("as5600 reg {=u8:#04x}: timed out after {} ms", register, I2C_TIMEOUT.as_millis());
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use embedded_hal_async::i2c::{Error as I2cError, ErrorKind, ErrorType, Operation};

    use super::*;

    /// The chip's read side: a register file, addressed by a one-byte write and read out with
    /// auto-increment. Anything at another address NACKs.
    struct MockAs5600 {
        registers: HashMap<u8, u8>,
        /// Register addresses the last transaction started from, so a test can assert *where* the
        /// driver looked rather than only what it decoded.
        reads: Vec<u8>,
        /// Fail every transfer, standing in for a header with no encoder on it.
        silent: bool,
    }

    #[derive(Debug)]
    struct Nack;
    impl I2cError for Nack {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    impl MockAs5600 {
        fn new() -> Self {
            Self {
                registers: HashMap::new(),
                reads: Vec::new(),
                silent: false,
            }
        }

        fn silent() -> Self {
            Self {
                silent: true,
                ..Self::new()
            }
        }

        fn set(&mut self, register: u8, value: u8) -> &mut Self {
            self.registers.insert(register, value);
            self
        }

        /// A 12-bit value across a register pair, the way the chip lays angles out.
        fn set12(&mut self, register: u8, value: u16) -> &mut Self {
            self.set(register, (value >> 8) as u8 & 0x0F);
            self.set(register + 1, value as u8)
        }
    }

    impl ErrorType for MockAs5600 {
        type Error = Nack;
    }

    impl embedded_hal_async::i2c::I2c for MockAs5600 {
        async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Nack> {
            if self.silent || address != ADDRESS {
                return Err(Nack);
            }
            let mut pointer = 0u8;
            for op in operations {
                match op {
                    Operation::Write(bytes) => {
                        pointer = *bytes.first().ok_or(Nack)?;
                        self.reads.push(pointer);
                    }
                    Operation::Read(buffer) => {
                        for byte in buffer.iter_mut() {
                            *byte = self.registers.get(&pointer).copied().unwrap_or(0);
                            pointer = pointer.wrapping_add(1);
                        }
                    }
                }
            }
            Ok(())
        }
    }

    /// No async executor is available on the host build (see the no-dev-dependencies note in
    /// Cargo.toml), so this busy-polls with a no-op waker. Every future here resolves on the first
    /// poll: `MockAs5600::transaction` never actually awaits, and `with_timeout` only needs the
    /// mock clock to advance if the wrapped future does not resolve immediately.
    fn block_on<F: core::future::Future>(fut: F) -> F::Output {
        use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

        fn noop(_: *const ()) {}
        fn clone(_: *const ()) -> RawWaker {
            RawWaker::new(core::ptr::null(), &VTABLE)
        }
        static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);

        let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
        let mut cx = Context::from_waker(&waker);
        let mut fut = core::pin::pin!(fut);
        loop {
            if let Poll::Ready(val) = fut.as_mut().poll(&mut cx) {
                return val;
            }
        }
    }

    /// A chip with a magnet in range, sitting at 90 degrees.
    fn quarter_turn() -> MockAs5600 {
        let mut chip = MockAs5600::new();
        chip.set(0x0B, Status::MD);
        chip.set12(0x0C, 1024);
        chip.set12(0x0E, 1024);
        chip.set(0x1A, 64);
        chip.set12(0x1B, 2048);
        chip
    }

    fn buses_with(bus0: MockAs5600, bus1: MockAs5600) -> Buses<MockAs5600, MockAs5600> {
        Buses {
            bus0: Some(bus0),
            bus1: Some(bus1),
        }
    }

    /// The whole point of the register constants: reading the wrong pair would still decode into a
    /// plausible-looking angle, so the addresses are asserted directly.
    #[test]
    fn a_read_bursts_from_the_status_and_agc_registers() {
        let mut buses = buses_with(quarter_turn(), MockAs5600::new());

        block_on(read(&mut buses, I2cBus::Bus0)).expect("the chip answered");

        assert_eq!(buses.bus0.unwrap().reads, vec![0x0B, 0x1A]);
    }

    #[test]
    fn a_reading_decodes_every_field() {
        let mut buses = buses_with(quarter_turn(), MockAs5600::new());

        let reading = block_on(read(&mut buses, I2cBus::Bus0)).expect("the chip answered");

        assert_eq!(reading.angle, 1024);
        assert_eq!(reading.raw_angle, 1024);
        assert_eq!(reading.agc, 64);
        assert_eq!(reading.magnitude, 2048);
        assert!(reading.status.magnet_detected);
        assert!(reading.status.is_usable());
    }

    /// Each bus has its own encoder, and asking for one must not read the other.
    #[test]
    fn each_bus_is_read_independently() {
        let mut other = quarter_turn();
        other.set12(0x0E, 3072);
        let mut buses = buses_with(quarter_turn(), other);

        let bus0 = block_on(read(&mut buses, I2cBus::Bus0)).expect("bus 0 answered");
        let bus1 = block_on(read(&mut buses, I2cBus::Bus1)).expect("bus 1 answered");

        assert_eq!(bus0.angle, 1024);
        assert_eq!(bus1.angle, 3072);
    }

    #[test]
    fn a_silent_bus_reads_as_nothing() {
        let mut buses = buses_with(MockAs5600::silent(), MockAs5600::new());
        assert!(block_on(read(&mut buses, I2cBus::Bus0)).is_none());
    }

    #[test]
    fn an_unpopulated_bus_reads_as_nothing() {
        let mut buses: Buses<MockAs5600, MockAs5600> = Buses { bus0: None, bus1: None };
        assert!(block_on(read(&mut buses, I2cBus::Bus0)).is_none());
    }

    /// The upper nibble of the high register is reserved, so a chip that leaves bits set there
    /// must not push the angle past one turn.
    #[test]
    fn the_reserved_high_bits_are_masked_off() {
        let mut chip = quarter_turn();
        chip.set(0x0E, 0xFF);
        chip.set(0x0F, 0xFF);
        let mut buses = buses_with(chip, MockAs5600::new());

        let reading = block_on(read(&mut buses, I2cBus::Bus0)).expect("the chip answered");

        assert_eq!(reading.angle, 4095);
    }

    #[test]
    fn the_status_bits_are_the_datasheet_ones() {
        assert!(Status::from_bits(0b0010_0000).magnet_detected, "MD is bit 5");
        assert!(Status::from_bits(0b0001_0000).too_weak, "ML is bit 4");
        assert!(Status::from_bits(0b0000_1000).too_strong, "MH is bit 3");

        // A magnet that is present but out of gain range is detected and unusable at the same
        // time, which is exactly the case the assembly log has to distinguish.
        let too_far = Status::from_bits(Status::MD | Status::ML);
        assert!(too_far.magnet_detected && too_far.too_weak);
        assert!(!too_far.is_usable());

        let nothing = Status::from_bits(0);
        assert!(!nothing.magnet_detected);
        assert!(!nothing.is_usable());
    }

    #[test]
    fn angle_counts_convert_to_centi_degrees() {
        let at = |angle| Reading {
            angle,
            raw_angle: angle,
            status: Status::from_bits(Status::MD),
            agc: 0,
            magnitude: 0,
        };

        assert_eq!(at(0).centi_degrees(), 0);
        assert_eq!(at(1024).centi_degrees(), 9000, "a quarter turn is 90.00 degrees");
        assert_eq!(at(2048).centi_degrees(), 18000);
        // 4095 counts is one count short of a full turn, and must not read as 360.00 degrees.
        assert_eq!(at(4095).centi_degrees(), 35991);
    }

    #[test]
    fn centi_degrees_are_monotonic_across_a_turn() {
        let mut previous = 0;
        for angle in 1..COUNTS_PER_TURN as u16 {
            let centi = Reading {
                angle,
                raw_angle: angle,
                status: Status::from_bits(Status::MD),
                agc: 0,
                magnitude: 0,
            }
            .centi_degrees();
            assert!(centi > previous, "not monotonic at angle={angle}");
            previous = centi;
        }
    }
}
