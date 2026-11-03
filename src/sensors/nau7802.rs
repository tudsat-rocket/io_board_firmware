//! The NAU7802 bridge ADC: bringing one up, and walking its input mux between its two channels.
//!
//! Nuvoton NAU7802 datasheet, Rev 2.4. One per bus, since its address is fixed; on the Adafruit
//! breakout (product 4538, rev C) channel A is A+/A- and channel B is B+/B-, both bridges are
//! excited from E+/E-, which is the chip's AVDD, and AVDD is also the ADC's reference. That is
//! what makes a reading a ratio of the excitation rather than a voltage.
//!
//! # One tick, one transfer
//!
//! Bringing the chip up takes a few hundred milliseconds — power-up, the analog side settling,
//! and an internal offset calibration per channel — and the sensor task cannot stall every other
//! device on both buses for that long. So [`Bridge`] is a state machine that the sensor task
//! advances by one [`Bridge::step`] per tick, and every step is at most **one** register transfer:
//! [`Bridge::request`] says which, and [`Bridge::advance`] takes in its result. Besides keeping each
//! tick short, that leaves one place in the task that awaits the bus, which on a board with a few
//! hundred bytes of flash to spare is worth more than the extra ticks the bring-up takes.
//!
//! # Two channels, one converter
//!
//! The chip has one PGA and one ADC behind a mux (`CTRL2.CHS`), so the channels are sampled in
//! turn, not together. After a switch the digital filter still holds the old channel, so nothing
//! is read until six conversions after it ([`Timing::settle`]). A bus with only one channel in use never
//! switches, and reads at whatever the sensor tick and the conversion rate allow.
//!
//! A sample is a read of `ADCO` without first checking `CR`: the datasheet says such a read
//! returns the previous conversion (1.29), which is never more than one conversion old. Reading the
//! same one twice when the tick outruns the converter — always, below 80 SPS — is harmless.
//!
//! The conversion rate is configurable (0x3033). 320 SPS has no mains rejection; 10 SPS has the
//! chip's full 50/60 Hz notch, at a tenth of a second per sample.

use embassy_time::{Duration, Instant};
use embedded_hal_async::i2c::I2c;

use super::ext_adc::Buses;
use crate::index::{BridgeChannel, I2cBus};
use crate::store::RAW_BRIDGE_INVALID;

const PU_CTRL: u8 = 0x00;
const CTRL1: u8 = 0x01;
const CTRL2: u8 = 0x02;
const ADCO_B2: u8 = 0x12;
const ADC_CTRL: u8 = 0x15;

// PU_CTRL.
const RR: u8 = 1 << 0;
const PUD: u8 = 1 << 1;
const PUA: u8 = 1 << 2;
const CS: u8 = 1 << 4;
const AVDDS: u8 = 1 << 7;

/// `CTRL1.VLDO` for 4.2 V. The breakout runs off 5 V, and the LDO needs 0.3 V of that as
/// headroom, so 4.5 V would drop out on a 5 V rail that sagged to 4.8 V — harmlessly, since AVDD
/// is both the bridge excitation and the reference, but 4.2 V stays regulated. A bridge then sits
/// at 2.1 V common-mode, in the middle of the 1.5 V..AVDD-1.5 V the PGA allows at gain 64 and 128.
///
/// Do not run the breakout off 3.3 V with this: the LDO would be asked for more than its supply.
/// That wants 3.0 V (`0b101`) here.
const VLDO_4V2: u8 = 0b001 << 3;

// CTRL2.
const CALS: u8 = 1 << 2;
const CAL_ERR: u8 = 1 << 3;
const CRS_SHIFT: u8 = 4;
const CHS: u8 = 1 << 7;

/// `REG_CHPS = 0b11`, which the datasheet gives as the recommended value (1.30).
const ADC_CHPS: u8 = 0x30;

// `PGA_CAP_EN` (0x1C bit 7) is deliberately left at its default of off: the breakout puts a 470 pF
// filter capacitor straight across VIN2P/VIN2N, and that bit would hang the PGA output across it
// — on top of channel B's input.

/// The register writes that bring the chip up, one per tick, in order (datasheet 1.17).
///
/// `RR` is level-triggered, so the reset is set and then cleared with the digital side powered.
/// The datasheet then asks to wait for `PUR`, which comes up "after about 200 microseconds"; a
/// tick later it has, and a chip for which it has not fails its calibration instead. The LDO
/// voltage goes in before the LDO is switched on, so it never comes up at its 4.5 V default.
/// `CTRL1` carries the gain as well, filled in at the time — [`GAIN_STEP`] is where a gain change
/// starts over.
///
/// Between [`POWER_UP_STEP`] and the rest sits a read of the revision ID ([`Phase::Identify`]):
/// the chip's digital side is off until `PUD`, and until then the ID reads as zero, so this is the
/// first point at which checking it means anything.
const INIT: [(u8, u8); 5] = [
    (PU_CTRL, RR),
    (PU_CTRL, PUD),
    (CTRL1, VLDO_4V2),
    (ADC_CTRL, ADC_CHPS),
    (PU_CTRL, AVDDS | PUA | PUD | CS),
];

/// The step of [`INIT`] that powers up the digital side, after which the chip can identify itself.
const POWER_UP_STEP: u8 = 1;

/// The step of [`INIT`] that programs the gain.
const GAIN_STEP: u8 = 2;

/// `REVISION_ID`: the low nibble reads `0xF` on a NAU7802 whose digital side is powered.
const REVISION_ID: u8 = 0x1F;

/// How long the bring-up and the channel switches wait, at one conversion rate.
///
/// Worked out at compile time for each of the five rates, rather than scaled from the conversion
/// period at run time: `Duration` arithmetic is 64-bit with overflow checks, and on this board
/// that was half a kilobyte of flash for three multiplications.
#[derive(Clone, Copy)]
struct Timing {
    /// From powering up the analog side, or changing gain or rate, to the first calibration. The
    /// datasheet gives 600 ms at 10 SPS as "5 sample times plus 100 ms" (1.3); this adds another
    /// 100 ms on top.
    warm_up: Duration,
    /// An internal offset calibration is a handful of conversions; this is a stuck one.
    calibration_timeout: Duration,
    /// From a channel switch to the first sample taken from the new channel: six conversions, the
    /// count the datasheet asks to wait out after a reset (1.14), and a little over. 20 ms at
    /// 320 SPS, 601 ms at 10 SPS.
    settle: Duration,
}

impl Timing {
    const fn for_conversion_us(us: u64) -> Self {
        Self {
            warm_up: Duration::from_micros(5 * us + 200_000),
            calibration_timeout: Duration::from_micros(20 * us + 500_000),
            settle: Duration::from_micros(6 * us + 1_250),
        }
    }

    /// At `CTRL2.CRS` code `rate_code` ([`crate::config::bridge_rate_code`]).
    fn at(rate_code: u8) -> Self {
        const TIMINGS: [Timing; 5] = [
            Timing::for_conversion_us(100_000), // 10 SPS
            Timing::for_conversion_us(50_000),  // 20 SPS
            Timing::for_conversion_us(25_000),  // 40 SPS
            Timing::for_conversion_us(12_500),  // 80 SPS
            Timing::for_conversion_us(3_125),   // 320 SPS, code 7
        ];
        TIMINGS[(rate_code as usize).min(4)]
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Phase {
    /// Not answering, or never found. The presence scan brings it back through
    /// [`Bridge::found`].
    Absent,
    /// Working through [`INIT`]; the next write is this step of it.
    Init(u8),
    /// Digital side just powered up; reading the revision ID to make sure this is a NAU7802.
    Identify,
    /// Analog side on and converting, waiting out the settling time before calibrating.
    WarmingUp { until: Instant },
    /// About to point the mux at `channel`, and either calibrate it or start converting on it.
    Select { channel: BridgeChannel, calibrate: bool },
    /// Internal offset calibration running on `channel`; polling `CTRL2` until it is done.
    Calibrating { channel: BridgeChannel, since: Instant },
    /// Converting on `channel`; sampling it from `settled_at` on.
    Converting {
        channel: BridgeChannel,
        settled_at: Instant,
    },
}

/// Why a [`Bridge::step`] did not go to plan.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Fault {
    /// A transfer failed: the chip has gone, as far as presence is concerned.
    Bus,
    /// The chip answers but would not calibrate. Worth a reset and another go.
    Init,
}

/// One register transfer: a write of `value`, or a read of `len` bytes.
struct Request {
    register: u8,
    value: Option<u8>,
    len: usize,
}

impl Request {
    const fn write(register: u8, value: u8) -> Self {
        Self {
            register,
            value: Some(value),
            len: 0,
        }
    }

    const fn read(register: u8, len: usize) -> Self {
        Self {
            register,
            value: None,
            len,
        }
    }
}

/// What a chip is programmed with, both halves of which a calibration is only valid for.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Setting {
    /// `CTRL1.GAINS`: the base-2 log of the PGA gain.
    pub gain_code: u8,
    /// `CTRL2.CRS`: the conversion rate, see [`crate::config::bridge_rate_code`].
    pub rate_code: u8,
}

/// One bus's NAU7802.
pub struct Bridge {
    phase: Phase,
    /// What the chip was last programmed with.
    setting: Setting,
}

impl Bridge {
    pub const fn new() -> Self {
        Self {
            phase: Phase::Absent,
            setting: Setting {
                gain_code: 0,
                rate_code: 0,
            },
        }
    }

    /// The presence scan found a chip: bring it up from scratch.
    pub fn found(&mut self) {
        self.phase = Phase::Init(0);
    }

    pub fn lost(&mut self) {
        self.phase = Phase::Absent;
    }

    /// Calibrated and converting, so the samples it has handed out are at [`Self::gain_code`].
    pub fn is_ready(&self) -> bool {
        matches!(self.phase, Phase::Converting { .. } | Phase::Select { calibrate: false, .. })
    }

    pub fn gain_code(&self) -> u8 {
        self.setting.gain_code
    }

    /// Advance by one transfer, leaving any fresh sample in `raw`.
    ///
    /// `setting` is the gain and rate the configuration asks for; a change re-calibrates, since
    /// the datasheet asks for that after either (1.12). `wanted` says
    /// which channels some slot reads: an unwanted channel is not switched to unless neither is
    /// wanted, in which case both are sampled so that 0x2008 has something to commission against.
    ///
    /// `raw` is only ever written with a sample; invalidating it while the chip is not
    /// [`Self::is_ready`] is the caller's job.
    pub async fn step<I0: I2c, I1: I2c>(
        &mut self,
        buses: &mut Buses<I0, I1>,
        bus: I2cBus,
        setting: Setting,
        wanted: [bool; 2],
        now: Instant,
        raw: &mut [i32; 2],
    ) -> Result<(), Fault> {
        let Some(request) = self.request(setting, now) else {
            return Ok(());
        };
        let mut reply = [0u8; 3];
        let (bytes, write_len) = match request.value {
            Some(value) => ([request.register, value], 2),
            None => ([request.register, 0], 1),
        };
        if buses.bridge_transfer(bus, &bytes[..write_len], &mut reply[..request.len]).await.is_none() {
            defmt::warn!("nau7802 bus {}: transfer at register {=u8:#04x} failed", bus.as_u8(), request.register);
            return Err(Fault::Bus);
        }
        self.advance(bus, reply, setting, wanted, now, raw)
    }

    /// The transfer this tick calls for, if any — after any transition that is due on time or on
    /// a gain change alone.
    fn request(&mut self, setting: Setting, now: Instant) -> Option<Request> {
        match self.phase {
            Phase::WarmingUp { until } if now >= until => {
                self.phase = Phase::Select {
                    channel: BridgeChannel::A,
                    calibrate: true,
                };
            }
            // A new gain or rate makes the calibrated offsets wrong, so it goes round again from the
            // gain write — and the warm-up, which the analog side does not need but which keeps
            // this to one path.
            Phase::Converting { .. } | Phase::Select { calibrate: false, .. } if setting != self.setting => {
                self.phase = Phase::Init(GAIN_STEP);
            }
            _ => {}
        }

        match self.phase {
            Phase::Absent | Phase::WarmingUp { .. } => None,
            Phase::Init(step) => {
                let (register, value) = INIT[step as usize];
                let value = if step == GAIN_STEP {
                    value | setting.gain_code
                } else {
                    value
                };
                Some(Request::write(register, value))
            }
            Phase::Select { channel, calibrate } => {
                let cals = if calibrate { CALS } else { 0 };
                Some(Request::write(CTRL2, self.ctrl2(channel) | cals))
            }
            Phase::Identify => Some(Request::read(REVISION_ID, 1)),
            Phase::Calibrating { .. } => Some(Request::read(CTRL2, 1)),
            Phase::Converting { settled_at, .. } if now >= settled_at => Some(Request::read(ADCO_B2, 3)),
            Phase::Converting { .. } => None,
        }
    }

    /// Take in the result of the transfer [`Self::request`] asked for.
    fn advance(
        &mut self,
        bus: I2cBus,
        reply: [u8; 3],
        setting: Setting,
        wanted: [bool; 2],
        now: Instant,
        raw: &mut [i32; 2],
    ) -> Result<(), Fault> {
        self.phase = match self.phase {
            Phase::Init(step) => {
                if step == GAIN_STEP {
                    self.setting = setting;
                }
                if step == POWER_UP_STEP {
                    Phase::Identify
                } else if step as usize + 1 < INIT.len() {
                    Phase::Init(step + 1)
                } else {
                    #[cfg(feature = "i2c-debug")]
                    defmt::debug!(
                        "nau7802 bus {}: powered up at gain {}, warming up",
                        bus.as_u8(),
                        1u8 << self.setting.gain_code
                    );
                    Phase::WarmingUp {
                        until: now + Timing::at(self.setting.rate_code).warm_up,
                    }
                }
            }
            Phase::Identify => {
                // Something answered at 0x2A; this is where it has to prove it is a NAU7802.
                if reply[0] & 0x0F != 0x0F {
                    defmt::warn!(
                        "nau7802 bus {}: revision id is {=u8:#04x} after power-up, not a NAU7802?",
                        bus.as_u8(),
                        reply[0]
                    );
                    return Err(Fault::Init);
                }
                Phase::Init(POWER_UP_STEP + 1)
            }
            Phase::Select {
                channel,
                calibrate: true,
            } => Phase::Calibrating { channel, since: now },
            Phase::Select {
                channel,
                calibrate: false,
            } => Phase::Converting {
                channel,
                settled_at: now + Timing::at(self.setting.rate_code).settle,
            },
            Phase::Calibrating { channel, since } => {
                if reply[0] & CALS != 0 {
                    if now - since > Timing::at(self.setting.rate_code).calibration_timeout {
                        defmt::warn!(
                            "nau7802 bus {}: channel {} calibration timed out, CTRL2 = {=u8:#04x}",
                            bus.as_u8(),
                            channel,
                            reply[0]
                        );
                        return Err(Fault::Init);
                    }
                    return Ok(());
                }
                if reply[0] & CAL_ERR != 0 {
                    defmt::warn!(
                        "nau7802 bus {}: channel {} calibration failed (CAL_ERR), CTRL2 = {=u8:#04x}",
                        bus.as_u8(),
                        channel,
                        reply[0]
                    );
                    return Err(Fault::Init);
                }
                #[cfg(feature = "i2c-debug")]
                defmt::debug!("nau7802 bus {}: channel {} calibrated", bus.as_u8(), channel);
                if channel == BridgeChannel::B {
                    defmt::info!(
                        "nau7802 bus {}: ready, converting at gain {}",
                        bus.as_u8(),
                        1u8 << self.setting.gain_code
                    );
                }
                match channel {
                    BridgeChannel::A => Phase::Select {
                        channel: BridgeChannel::B,
                        calibrate: true,
                    },
                    BridgeChannel::B => Phase::Select {
                        channel: if wanted[0] || !wanted[1] {
                            BridgeChannel::A
                        } else {
                            BridgeChannel::B
                        },
                        calibrate: false,
                    },
                }
            }
            Phase::Converting { channel, settled_at } => {
                // Big-endian 24-bit two's complement, sign-extended by shifting it down from the top.
                raw[channel.index()] = i32::from_be_bytes([reply[0], reply[1], reply[2], 0]) >> 8;
                // Every sample, so only at trace: build with DEFMT_LOG=trace as well to see them.
                #[cfg(feature = "i2c-debug")]
                defmt::trace!("nau7802 bus {} channel {}: {=i32}", bus.as_u8(), channel, raw[channel.index()]);
                let other = match channel {
                    BridgeChannel::A => BridgeChannel::B,
                    BridgeChannel::B => BridgeChannel::A,
                };
                if wanted[other.index()] || !wanted[channel.index()] {
                    Phase::Select {
                        channel: other,
                        calibrate: false,
                    }
                } else {
                    // Staying put: the other channel's last sample would only get staler.
                    raw[other.index()] = RAW_BRIDGE_INVALID;
                    Phase::Converting { channel, settled_at }
                }
            }
            // Neither asks for a transfer, so there is nothing to take in.
            phase @ (Phase::Absent | Phase::WarmingUp { .. }) => phase,
        };
        Ok(())
    }

    /// `CTRL2` for converting on `channel` at the programmed rate.
    fn ctrl2(&self, channel: BridgeChannel) -> u8 {
        let chs = match channel {
            BridgeChannel::A => 0,
            BridgeChannel::B => CHS,
        };
        self.setting.rate_code << CRS_SHIFT | chs
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal_async::i2c::{ErrorType, Operation};

    use super::*;
    use crate::sensors::tests::{Nack, block_on};

    const PUR: u8 = 1 << 3;

    /// The part of the NAU7802 the state machine touches: the registers, `PUR` coming up as soon
    /// as the digital side is powered, a calibration that finishes the moment it starts, and one
    /// fixed conversion per channel.
    struct Chip {
        regs: [u8; 0x20],
        adc: [i32; 2],
        cal_err: bool,
        /// What `REVISION_ID` reads once the digital side is up.
        revision: u8,
        pointer: u8,
    }

    impl Chip {
        fn new(adc: [i32; 2]) -> Self {
            Self {
                regs: [0; 0x20],
                adc,
                cal_err: false,
                revision: 0x0F,
                pointer: 0,
            }
        }

        fn poke(&mut self, register: u8, value: u8) {
            let value = match register {
                PU_CTRL if value & PUD != 0 => (value & !RR) | PUR,
                CTRL2 if value & CALS != 0 && self.cal_err => (value & !CALS) | CAL_ERR,
                CTRL2 => value & !CALS,
                _ => value,
            };
            self.regs[register as usize] = value;
        }

        fn peek(&self, register: u8) -> u8 {
            let channel = (self.regs[CTRL2 as usize] & CHS != 0) as usize;
            let bytes = self.adc[channel].to_be_bytes();
            match register {
                0x12..=0x14 => bytes[(register - 0x12) as usize + 1],
                // Zero until the digital side is powered: what a chip fresh out of power-on read on
                // the bench, and why the ID is not checked at probe time.
                REVISION_ID if self.regs[PU_CTRL as usize] & PUD != 0 => self.revision,
                REVISION_ID => 0,
                _ => self.regs[register as usize],
            }
        }
    }

    impl ErrorType for Chip {
        type Error = Nack;
    }

    impl I2c for Chip {
        async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Nack> {
            if address != crate::config::BRIDGE_ADDRESS {
                return Err(Nack);
            }
            for op in operations {
                match op {
                    Operation::Write(bytes) => {
                        self.pointer = bytes[0];
                        if let [register, value] = **bytes {
                            self.poke(register, value);
                        }
                    }
                    Operation::Read(buffer) => {
                        for (i, byte) in buffer.iter_mut().enumerate() {
                            *byte = self.peek(self.pointer + i as u8);
                        }
                    }
                }
            }
            Ok(())
        }
    }

    fn buses(chip: Chip) -> Buses<Chip, Chip> {
        Buses {
            bus0: Some(chip),
            bus1: None,
        }
    }

    fn chip(buses: &Buses<Chip, Chip>) -> &Chip {
        buses.bus0.as_ref().unwrap()
    }

    /// The factory default: gain 128, 320 SPS.
    const DEFAULT: Setting = Setting {
        gain_code: 7,
        rate_code: 7,
    };

    /// Step `bridge` once at each of `times` (in milliseconds), expecting every step to succeed.
    fn run(
        bridge: &mut Bridge,
        buses: &mut Buses<Chip, Chip>,
        setting: Setting,
        wanted: [bool; 2],
        times: &[u64],
        raw: &mut [i32; 2],
    ) {
        for &t in times {
            block_on(bridge.step(buses, I2cBus::Bus0, setting, wanted, Instant::from_millis(t), raw)).expect("step");
        }
    }

    /// Five init writes with the ID read after the second, the warm-up (215.6 ms at 320 SPS), a
    /// calibration per channel and the first channel select: converting on the first wanted
    /// channel, from 245 ms on.
    fn bring_up(bridge: &mut Bridge, buses: &mut Buses<Chip, Chip>, wanted: [bool; 2], raw: &mut [i32; 2]) {
        bridge.found();
        run(bridge, buses, DEFAULT, wanted, &[0, 1, 2, 3, 4, 5, 220], raw);
        assert!(!bridge.is_ready(), "still warming up");
        run(bridge, buses, DEFAULT, wanted, &[221, 222, 223, 224, 225], raw);
        assert!(bridge.is_ready());
    }

    #[test]
    fn bring_up_programs_the_ldo_gain_and_rate() {
        let mut buses = buses(Chip::new([0, 0]));
        let (mut bridge, mut raw) = (Bridge::new(), [RAW_BRIDGE_INVALID; 2]);
        bring_up(&mut bridge, &mut buses, [true, true], &mut raw);

        let chip = chip(&buses);
        assert_eq!(chip.regs[CTRL1 as usize], VLDO_4V2 | 7, "4.2 V LDO, gain 128");
        assert_eq!(chip.regs[PU_CTRL as usize] & !PUR, AVDDS | PUA | PUD | CS);
        assert_eq!(chip.regs[ADC_CTRL as usize], ADC_CHPS);
        assert_eq!(chip.regs[CTRL2 as usize], 0b111 << CRS_SHIFT, "320 SPS, converting on channel A");
        assert_eq!(bridge.gain_code(), 7);
    }

    /// Each channel is read only once its own settling time is up, and the mux then moves on.
    #[test]
    fn both_channels_are_sampled_in_turn_after_settling() {
        let mut buses = buses(Chip::new([1_000_000, -5]));
        let (mut bridge, mut raw) = (Bridge::new(), [RAW_BRIDGE_INVALID; 2]);
        bring_up(&mut bridge, &mut buses, [true, true], &mut raw);

        run(&mut bridge, &mut buses, DEFAULT, [true, true], &[230], &mut raw);
        assert_eq!(raw, [RAW_BRIDGE_INVALID; 2], "channel A has not settled yet");

        run(&mut bridge, &mut buses, DEFAULT, [true, true], &[245], &mut raw);
        assert_eq!(raw[0], 1_000_000);
        run(&mut bridge, &mut buses, DEFAULT, [true, true], &[246], &mut raw);
        assert_eq!(chip(&buses).regs[CTRL2 as usize] & CHS, CHS, "switched to B");
        assert!(bridge.is_ready(), "and still ready through the switch");

        run(&mut bridge, &mut buses, DEFAULT, [true, true], &[257, 266], &mut raw);
        assert_eq!(raw, [1_000_000, -5], "a negative 24-bit conversion is sign-extended");
    }

    /// A bus with one channel in use never pays for a switch, and does not keep showing a stale
    /// sample from the other.
    #[test]
    fn a_single_wanted_channel_is_never_switched_away_from() {
        let mut buses = buses(Chip::new([11, 22]));
        let (mut bridge, mut raw) = (Bridge::new(), [33, 44]);
        bring_up(&mut bridge, &mut buses, [false, true], &mut raw);

        run(&mut bridge, &mut buses, DEFAULT, [false, true], &[245, 246, 247], &mut raw);
        assert_eq!(raw, [RAW_BRIDGE_INVALID, 22]);
        assert_eq!(chip(&buses).regs[CTRL2 as usize] & CHS, CHS, "still on B");
    }

    /// A gain written over SDO takes the chip back through calibration, since its offsets were
    /// taken at the old gain.
    #[test]
    fn a_gain_change_recalibrates() {
        let mut buses = buses(Chip::new([0, 0]));
        let (mut bridge, mut raw) = (Bridge::new(), [RAW_BRIDGE_INVALID; 2]);
        bring_up(&mut bridge, &mut buses, [true, true], &mut raw);

        let gain_16 = Setting {
            gain_code: 4,
            ..DEFAULT
        };
        run(&mut bridge, &mut buses, gain_16, [true, true], &[300], &mut raw);
        assert!(!bridge.is_ready());
        assert_eq!(chip(&buses).regs[CTRL1 as usize], VLDO_4V2 | 4);
        run(&mut bridge, &mut buses, gain_16, [true, true], &[301, 302, 518, 519, 520, 521], &mut raw);
        assert!(bridge.is_ready());
        assert_eq!(bridge.gain_code(), 4);
    }

    /// The same for a rate change, which also stretches the warm-up and the settling time: six
    /// conversions at 10 SPS is 600 ms, against 19 at 320.
    #[test]
    fn a_rate_change_recalibrates_at_the_new_rate() {
        let mut buses = buses(Chip::new([0, 0]));
        let (mut bridge, mut raw) = (Bridge::new(), [RAW_BRIDGE_INVALID; 2]);
        bring_up(&mut bridge, &mut buses, [true, false], &mut raw);

        let sps_10 = Setting {
            rate_code: 0,
            ..DEFAULT
        };
        run(&mut bridge, &mut buses, sps_10, [true, false], &[300, 301, 302], &mut raw);
        run(&mut bridge, &mut buses, sps_10, [true, false], &[1001], &mut raw);
        assert!(!bridge.is_ready(), "warm-up at 10 SPS is 700 ms");
        run(&mut bridge, &mut buses, sps_10, [true, false], &[1002, 1003, 1004, 1005, 1006], &mut raw);
        assert!(bridge.is_ready());
        assert_eq!(chip(&buses).regs[CTRL2 as usize], 0, "10 SPS, channel A");

        run(&mut bridge, &mut buses, sps_10, [true, false], &[1500], &mut raw);
        assert_eq!(raw[0], RAW_BRIDGE_INVALID, "still settling");
        run(&mut bridge, &mut buses, sps_10, [true, false], &[1608], &mut raw);
        assert_eq!(raw[0], 0);
    }

    #[test]
    fn a_calibration_error_is_an_init_fault() {
        let mut chip = Chip::new([0, 0]);
        chip.cal_err = true;
        let mut buses = buses(chip);
        let (mut bridge, mut raw) = (Bridge::new(), [RAW_BRIDGE_INVALID; 2]);
        bridge.found();
        run(&mut bridge, &mut buses, DEFAULT, [true, true], &[0, 1, 2, 3, 4, 5, 221], &mut raw);

        let at = Instant::from_millis(222);
        let result = block_on(bridge.step(&mut buses, I2cBus::Bus0, DEFAULT, [true, true], at, &mut raw));
        assert_eq!(result, Err(Fault::Init));
    }

    /// A chip reading `0x00` before power-up is a NAU7802 being itself, so it is found; one that
    /// still reads something else once powered is not, and bring-up stops there.
    #[test]
    fn the_revision_id_is_checked_after_power_up_not_before() {
        let mut chip = Chip::new([0, 0]);
        chip.revision = 0x00;
        let mut buses = buses(chip);
        let (mut bridge, mut raw) = (Bridge::new(), [RAW_BRIDGE_INVALID; 2]);
        assert!(block_on(buses.probe_bridge(I2cBus::Bus0)), "anything answering at 0x2A is worth a look");

        bridge.found();
        run(&mut bridge, &mut buses, DEFAULT, [true, true], &[0, 1], &mut raw);
        let at = Instant::from_millis(2);
        let result = block_on(bridge.step(&mut buses, I2cBus::Bus0, DEFAULT, [true, true], at, &mut raw));
        assert_eq!(result, Err(Fault::Init));
    }

    /// A chip that stops answering is a bus fault — the caller drops it from presence — and an
    /// absent one costs no transfers at all.
    #[test]
    fn a_vanished_chip_is_a_bus_fault() {
        // It counts a NACK, which the error-counter tests must not see.
        let _guard = crate::errors::test_lock();
        let mut buses: Buses<Chip, Chip> = Buses { bus0: None, bus1: None };
        let (mut bridge, mut raw) = (Bridge::new(), [RAW_BRIDGE_INVALID; 2]);
        let at = Instant::from_millis(0);
        assert_eq!(block_on(bridge.step(&mut buses, I2cBus::Bus0, DEFAULT, [true, true], at, &mut raw)), Ok(()));

        bridge.found();
        let result = block_on(bridge.step(&mut buses, I2cBus::Bus0, DEFAULT, [true, true], at, &mut raw));
        assert_eq!(result, Err(Fault::Bus));
    }
}
