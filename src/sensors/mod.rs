//! Reading the I2C devices, calibrating them, and keeping an eye on who is actually there.
//!
//! Three things happen on the same tick, in one task, because they all contend for the same two
//! I2C buses:
//!
//! 1. **Sampling** every device we currently believe is present — the ADC101C027 amplifiers and,
//!    on each bus, the AS5600 magnetic encoder.
//! 2. **Calibration**, turning a raw device number into the value a slot is configured to report.
//!    Both the slot mapping and the calibration coefficients are runtime-writable
//!    (0x3020..0x3027), so a sensor can be recalibrated, moved to a different amplifier, or taken
//!    off the process data plane without a firmware build.
//! 3. **Presence scanning**, one address at a time.
//!
//! Only [`calibrate`] and its supporting arithmetic are unconditional; the sampling loop needs
//! embassy's clock and an I2C bus, so it is gated behind `hardware` (or `test`, against a mock).
//!
//! # One pipeline, every kind
//!
//! What differs between a pressure transducer, a Pt1000, an MCP9700, an NTC and a rotary encoder
//! is confined to two places: [`linearise`], which turns a raw device number into thousandths of
//! whatever the sensor is actually linear in, and `calibrate`'s choice of a wrapping zero for the
//! encoder. Everything downstream — the affine trim, the scale to the slot's unit, the i16
//! saturation, the wire — is shared, which is what makes recalibrating any of them the same three
//! SDO writes.
//!
//! The NTC kinds are the ones that are not on an I2C bus: they sit on a COM5/COM6 pin of the
//! STM32's own ADC, which the control task owns. That sampling reaches this module through the
//! store — see [`AnalogSensing`] — and joins the same pipeline from [`linearise`] onwards.

#[cfg(any(feature = "hardware", test))]
pub mod ext_adc;

#[cfg(any(feature = "hardware", test))]
use embassy_time::{Instant, Timer};
#[cfg(any(feature = "hardware", test))]
use embedded_hal_async::i2c::I2c;

#[cfg(any(feature = "hardware", test))]
use crate::config::AMPLIFIER_ADDRESSES;
#[cfg(any(feature = "hardware", test))]
use crate::config::Config;
#[cfg(any(feature = "hardware", test))]
use crate::config::{ENCODER_ADDRESS, ENCODER_MAGNET_OK_BIT, ENCODER_PRESENT_BIT, SensorSource};
use crate::config::{ENCODER_FULL_SCALE, SensorKind, SensorSlotConfig, Unit};
#[cfg(any(feature = "hardware", test))]
use crate::errors::{ErrorCounter, bump};
#[cfg(any(feature = "hardware", test))]
use crate::index::{AdcSlot, I2cBus, PerAdcSlot, PerI2cBus, PerSensorSlot};
// Ungated: `AnalogSensing` is part of the calibration half of this module, which builds on the
// host with no hardware feature at all.
use crate::index::PerAnalogInput;
use crate::store::SENSOR_INVALID;
#[cfg(any(feature = "hardware", test))]
use crate::store::{RAW_INVALID, STORE};
#[cfg(any(feature = "hardware", test))]
use ext_adc::Buses;

/// Convert a raw Pt1000 bridge reading to milli-degrees Celsius.
///
/// The analogue chain is a Wheatstone bridge into an instrumentation amplifier:
///
/// ```text
///   v_out  = raw * 3.3 / 1024          ADC counts to amplifier output
///   v_diff = (v_out - 1.65) / 10.69    remove amplifier offset and gain
///   x      = v_diff / 3.3              normalise to the bridge excitation
///   R      = 1000 * (x + 0.5) / (0.5 - x)
///   T      = (R - 1000) / 3.85         Pt1000 linear approximation
/// ```
///
/// Substituting and clearing denominators gives an exact integer form with `n = 33*raw - 16896`:
///
/// ```text
///   milli_celsius = 400_000_000 * n / (385 * (361236 - 2n))
/// ```
///
/// which is what is implemented here. Integer rather than float because the STM32F105 is a
/// Cortex-M3 with no FPU; the widest intermediate is about 7e12, hence i64. The denominator
/// cannot reach zero: `raw` is 10-bit, so `n` stays within +-17864 and `361236 - 2n` within
/// [325508, 396964].
///
/// This is the *bridge*, not the *probe*: the constants above are the board's resistors and the
/// amplifier's nominal gain, which is why the result then goes through the slot's own
/// [`crate::config::SensorCalib`] to trim out the tolerance of those parts. Millicelsius rather than the
/// centicelsius this used to return so that the trim's offset lands on a sensible scale — see
/// the table on [`crate::config::SensorCalib`].
pub fn pt1000_milli_celsius(raw: u16) -> i32 {
    let n = 33i64 * raw.min(1023) as i64 - 16_896;
    let denominator = 385 * (361_236 - 2 * n);
    ((400_000_000 * n) / denominator) as i32
}

/// The 10k NTC's curve, sampled every [`NTC_CURVE_STEP_C`] from [`NTC_CURVE_MIN_C`]: the 12-bit
/// ADC reading at each temperature. It falls as the thermistor heats.
///
/// Tabulated for a 10k upper leg to +3.3V and the NTC to ground, with R25 = 10k and B = 3950 K —
/// the divider the heating pads are wired with, and the one an NTC on COM5/COM6 is expected to
/// use. The beta is **unmeasured**; regenerate if it or the fixed leg differs:
///
/// ```text
///   R_ntc(T) = 10k * exp(3950 * (1/T - 1/298.15))
///   counts(T) = 4095 * R_ntc / (10k + R_ntc)
/// ```
///
/// The reading is ratiometric as long as the divider is fed from the same +3.3V as the ADC
/// reference. Fed from 5V, the table is wrong, and the pin can be driven above its rating when
/// the NTC opens.
///
/// Hardcoded rather than configurable on purpose: the curve is a property of the part, the same
/// way a Pt1000's bridge is a property of the board. What varies between installations is the
/// tolerance of the two resistances, and that is what the slot's own
/// [`crate::config::SensorCalib`] trims — see [`crate::config::SensorKind::Ntc`].
const NTC_CURVE: [u16; 34] = [
    3996, // -40 C
    3955, // -35 C
    3900, // -30 C
    3830, // -25 C
    3740, // -20 C
    3629, // -15 C
    3495, // -10 C
    3337, //  -5 C
    3156, //   0 C
    2955, //   5 C
    2738, //  10 C
    2510, //  15 C
    2278, //  20 C
    2048, //  25 C
    1825, //  30 C
    1614, //  35 C
    1419, //  40 C
    1241, //  45 C
    1081, //  50 C
    940,  //  55 C
    815,  //  60 C
    707,  //  65 C
    613,  //  70 C
    532,  //  75 C
    462,  //  80 C
    401,  //  85 C
    350,  //  90 C
    305,  //  95 C
    267,  // 100 C
    234,  // 105 C
    206,  // 110 C
    181,  // 115 C
    160,  // 120 C
    142,  // 125 C
];

/// Full scale of the STM32's 12-bit ADC, which is what [`NTC_CURVE`] is tabulated against.
pub const ADC_FULL_SCALE: u16 = 4095;

/// Temperature of `NTC_CURVE[0]`, in degrees Celsius.
const NTC_CURVE_MIN_C: i32 = -40;
/// Spacing between adjacent `NTC_CURVE` entries, in degrees Celsius.
const NTC_CURVE_STEP_C: i32 = 5;

/// Convert a raw 12-bit reading of an NTC divider to millidegrees Celsius, interpolating between
/// the points of [`NTC_CURVE`].
///
/// `None` for anything off the ends of the curve. That covers an open thermistor (pin pulled to
/// +3.3V) and a short (pin at ground) — the two failures that must not be reported as a
/// plausible temperature: anything regulating on one of them would act on a number that is not a
/// temperature at all.
pub fn ntc_milli_celsius(counts: u16) -> Option<i32> {
    // Descending curve, so the bracket is `curve[i] >= counts >= curve[i + 1]`.
    if counts > NTC_CURVE[0] || counts < NTC_CURVE[NTC_CURVE.len() - 1] {
        return None;
    }

    let mut i = 0;
    while NTC_CURVE[i + 1] > counts {
        i += 1;
    }

    let (high, low) = (NTC_CURVE[i] as i32, NTC_CURVE[i + 1] as i32);
    let base_milli_c = (NTC_CURVE_MIN_C + i as i32 * NTC_CURVE_STEP_C) * 1000;
    Some(base_milli_c + (NTC_CURVE_STEP_C * 1000 * (high - counts as i32)) / (high - low))
}

/// Where the raw counts for a [`crate::config::SensorSource::Analog`] slot come from.
///
/// The COM5/COM6 pins are on the STM32's own ADC, which belongs to the control task — the sensor
/// task owns the two I2C buses and nothing else. So the control task samples them each tick and
/// leaves the counts in [`crate::store::Store::raw_analog`], where the sensor task picks them up
/// and calibrates them like any other slot. This trait is what lets that sampling be mocked on
/// the host, the way [`crate::rail_sense::RailSensing`] is.
#[allow(async_fn_in_trait)]
pub trait AnalogSensing {
    /// Raw 12-bit counts on each COM5/COM6 pin `wanted` asks for, [`crate::store::RAW_INVALID`]
    /// for the rest and for any pin this build did not hand over — rev2, which has no ADC wired
    /// up at all.
    ///
    /// Every pin in one call, rather than one call per pin, so that the conversions and the loop
    /// around them stay inside this implementation instead of unrolling into the control task's
    /// own state machine. On a board with 114 KiB for the whole application, where that future
    /// ends up is worth a line of trait design.
    async fn read_analog(&mut self, wanted: PerAnalogInput<bool>) -> PerAnalogInput<u16>;
}

/// rev2 has no ADC wired up at all, so no analog slot on it ever reads.
impl AnalogSensing for crate::rail_sense::NoRails {
    async fn read_analog(&mut self, _wanted: PerAnalogInput<bool>) -> PerAnalogInput<u16> {
        PerAnalogInput::splat(crate::store::RAW_INVALID)
    }
}

/// A kind's raw device number, linearised into *thousandths* of the quantity its calibration is
/// affine in.
///
/// This is the only place a kind's physics lives; everything downstream — the trim, the unit
/// scaling, the wire — is the same for all of them. Thousandths rather than whole units so that
/// one [`crate::config::SensorCalib`] can carry a sub-count zero offset for an analogue channel and a
/// millidegree one for the Pt1000 in the same field.
pub fn linearise(kind: SensorKind, raw: u16) -> Option<i32> {
    match kind {
        SensorKind::None => None,
        // Already linear in the measured quantity, so the calibration slope carries the whole
        // scale and all the linearisation does is move to milli-counts.
        //
        // Angle counts are linear in angle too; what is different about them is that the zeroing
        // has to wrap, which is `calibrate`'s choice of `apply_wrapped` over `apply`.
        SensorKind::Pressure | SensorKind::Mcp9700 | SensorKind::Angle => Some(raw as i32 * 1000),
        SensorKind::Pt1000 => Some(pt1000_milli_celsius(raw)),
        // The one kind that can refuse a reading here: off either end of the curve is an open or
        // shorted thermistor, not a very hot or very cold one, and the slot reports nothing.
        SensorKind::Ntc => ntc_milli_celsius(raw),
        // The same curve read from the other end: swapping the two legs of a divider turns `x`
        // into `full scale - x`, and with a 10k fixed leg that is exact.
        SensorKind::NtcToSupply => ntc_milli_celsius(ADC_FULL_SCALE.saturating_sub(raw)),
    }
}

/// Clamp an i32 into the wire's i16, keeping [`SENSOR_INVALID`] reserved for "no reading".
fn saturate(v: i32) -> i16 {
    v.clamp(i16::MIN as i32 + 1, i16::MAX as i32) as i16
}

/// Turn a raw device reading into the value a slot reports, in the unit it declares.
///
/// The unit is per slot rather than global because no single scale works for everything here:
/// centibar is the natural resolution for a 40 bar transducer but overflows i16 at 400 bar,
/// which is what decibar is for, and neither is any use for a temperature or a valve angle.
///
/// The kind's only say is how `raw` is linearised ([`linearise`]) and whether its zero wraps.
/// After that every kind goes through the same affine trim to milli-units and the same scale
/// down to the reported unit — so recalibrating a Pt1000 and recalibrating a transducer are the
/// same three SDO writes.
pub fn calibrate(slot: &SensorSlotConfig, raw: Option<u16>) -> i16 {
    let Some(raw) = raw else {
        return SENSOR_INVALID;
    };

    // Raw counts bypass the calibration entirely — it is what you read *while* working one out.
    if slot.unit == Unit::RawCounts {
        return match slot.kind {
            SensorKind::None => SENSOR_INVALID,
            _ => saturate(raw as i32),
        };
    }

    let Some(linear) = linearise(slot.kind, raw) else {
        return SENSOR_INVALID;
    };
    // A slot whose slope is zero has not been calibrated: it would map every reading onto the
    // same constant. Reporting nothing is the honest answer and the safe one — a transducer
    // confidently reading 0.00 bar is far worse than one admitting it has no reading, and this is
    // exactly the state a slot is in between "kind written over SDO" and "coefficients written".
    if !slot.calib.is_calibrated() {
        return SENSOR_INVALID;
    }
    let milli = match slot.kind {
        SensorKind::Angle => slot.calib.apply_wrapped(linear, ENCODER_FULL_SCALE),
        _ => slot.calib.apply(linear),
    };
    saturate(milli / slot.unit.per_milli())
}

/// One address the presence sweep can knock on.
///
/// The two device families are probed by the same cursor rather than by two, so a board with an
/// encoder fitted does not probe it 9 times as often as any one amplifier.
#[cfg(any(feature = "hardware", test))]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum ProbeTarget {
    Amplifier(AdcSlot),
    Encoder(I2cBus),
}

#[cfg(any(feature = "hardware", test))]
impl ProbeTarget {
    /// Amplifiers first, in [`AdcSlot`] order, then one encoder per bus.
    const COUNT: usize = AdcSlot::COUNT + I2cBus::COUNT;

    fn from_index(index: usize) -> Option<Self> {
        match AdcSlot::from_index(index) {
            Some(slot) => Some(Self::Amplifier(slot)),
            None => I2cBus::from_index(index - AdcSlot::COUNT).map(Self::Encoder),
        }
    }

    fn bus(self) -> I2cBus {
        match self {
            Self::Amplifier(slot) => slot.bus(),
            Self::Encoder(bus) => bus,
        }
    }

    /// Which bit of that bus's presence word this target owns.
    fn present_bit(self) -> u16 {
        match self {
            Self::Amplifier(slot) => 1 << slot.amplifier().index(),
            Self::Encoder(_) => ENCODER_PRESENT_BIT,
        }
    }
}

/// Which of the probe-able addresses the incremental scan looks at next.
#[cfg(any(feature = "hardware", test))]
struct ScanCursor {
    next: usize,
    last_probe: Instant,
}

#[cfg(any(feature = "hardware", test))]
impl ScanCursor {
    fn new(now: Instant) -> Self {
        Self {
            next: 0,
            last_probe: now,
        }
    }

    /// Return the next address to probe, if the scan interval has elapsed. A zero interval
    /// disables scanning entirely, freezing the presence bitmap at whatever it last held.
    fn due(&mut self, cfg: &Config, now: Instant) -> Option<ProbeTarget> {
        if cfg.scan_interval_ms == 0 {
            return None;
        }
        if (now - self.last_probe).as_millis() < cfg.scan_interval_ms as u64 {
            return None;
        }
        self.last_probe = now;
        let target = ProbeTarget::from_index(self.next)?;
        self.next = (self.next + 1) % ProbeTarget::COUNT;
        Some(target)
    }

    /// True when the slot just handed out was the last of a sweep.
    fn wrapped(&self) -> bool {
        self.next == 0
    }
}

/// Generic over the I2C transport (see [`ext_adc::Buses`]) so this can be built and tested on
/// the host against a mock bus. [`BoardSensors`] is the concrete alias the firmware spawns.
#[cfg(any(feature = "hardware", test))]
pub struct Sensors<I0: I2c, I1: I2c> {
    buses: Buses<I0, I1>,
    /// Bit `slot.amplifier()` of `present[slot.bus()]` — the same bitmap that goes out at 0x2002
    /// and in TPDO kind 14. Bits 9 and 10 carry the bus's encoder; see [`ENCODER_PRESENT_BIT`].
    present: PerI2cBus<u16>,
    raw: PerAdcSlot<u16>,
    /// Last raw angle from each bus's AS5600, or [`RAW_INVALID`]. Published at 0x2006 so an
    /// encoder can be zeroed against its valve's stops without first guessing a calibration.
    raw_angle: PerI2cBus<u16>,
    /// Last raw counts from each COM5/COM6 pin, copied out of the store where the control task
    /// left them. Not sampled here: the STM32's own ADC belongs to the control task, and this
    /// task owns the two I2C buses and nothing else — see [`AnalogSensing`].
    raw_analog: PerAnalogInput<u16>,
    scan: ScanCursor,
    sweeps: u32,
}

/// The concrete `Sensors` the firmware spawns; mirrors `control::BoardControl`. Same concrete
/// bus type twice, matching [`ext_adc::BoardBuses`].
#[cfg(feature = "hardware")]
pub type BoardSensors = Sensors<
    &'static mut embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>,
    &'static mut embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>,
>;

#[cfg(any(feature = "hardware", test))]
impl<I0: I2c, I1: I2c> Sensors<I0, I1> {
    pub fn new(buses: Buses<I0, I1>) -> Self {
        Self {
            buses,
            present: PerI2cBus::splat(0),
            raw: PerAdcSlot::splat(RAW_INVALID),
            raw_angle: PerI2cBus::splat(RAW_INVALID),
            raw_analog: PerAnalogInput::splat(RAW_INVALID),
            scan: ScanCursor::new(Instant::now()),
            sweeps: 0,
        }
    }

    fn is_present(&self, slot: AdcSlot) -> bool {
        self.present[slot.bus()] & (1 << slot.amplifier().index()) != 0
    }

    fn encoder_present(&self, bus: I2cBus) -> bool {
        self.present[bus] & ENCODER_PRESENT_BIT != 0
    }

    /// Set or clear one bit of a bus's presence word, logging only the transitions.
    ///
    /// A device appearing or vanishing is the thing worth a log line — a loose connector during
    /// assembly shows up here — while the steady state is already on the bus at 0x2002 every
    /// sweep and does not need repeating.
    fn set_present(&mut self, target: ProbeTarget, present: bool) {
        let bit = target.present_bit();
        let bus = target.bus();
        let mask = &mut self.present[bus];
        let was = *mask & bit != 0;
        if present {
            *mask |= bit;
        } else {
            *mask &= !bit;
        }
        if was == present {
            return;
        }
        if !present {
            // Counted on the transition, not per failed read: a connector that opens once counts
            // once, and one that chatters counts every time it goes.
            bump(ErrorCounter::I2cDeviceLost);
        }
        match target {
            ProbeTarget::Amplifier(slot) => {
                let address = AMPLIFIER_ADDRESSES[slot.amplifier()];
                if present {
                    defmt::info!(
                        "amplifier appeared: bus {} addr {=u8:#04x} (index {})",
                        bus.as_u8(),
                        address,
                        slot.amplifier().as_u8()
                    );
                } else {
                    defmt::warn!(
                        "amplifier vanished: bus {} addr {=u8:#04x} (index {})",
                        bus.as_u8(),
                        address,
                        slot.amplifier().as_u8()
                    );
                }
            }
            ProbeTarget::Encoder(_) => {
                if present {
                    defmt::info!("as5600 appeared: bus {} addr {=u8:#04x}", bus.as_u8(), ENCODER_ADDRESS);
                } else {
                    defmt::warn!("as5600 vanished: bus {} addr {=u8:#04x}", bus.as_u8(), ENCODER_ADDRESS);
                }
            }
        }
    }

    /// Record what the encoder thinks of its magnet, logging the transitions the way presence is.
    ///
    /// Worth its own bit because the failure is quiet: a magnet that has drifted too far from the
    /// die still produces an angle, just not the right one.
    fn set_magnet_ok(&mut self, bus: I2cBus, ok: bool) {
        let mask = &mut self.present[bus];
        let was = *mask & ENCODER_MAGNET_OK_BIT != 0;
        if ok {
            *mask |= ENCODER_MAGNET_OK_BIT;
        } else {
            *mask &= !ENCODER_MAGNET_OK_BIT;
        }
        if was != ok && !ok {
            bump(ErrorCounter::EncoderMagnetLost);
            defmt::warn!("as5600 on bus {}: magnet missing or out of range, angle is not usable", bus.as_u8());
        }
    }

    pub async fn run(&mut self) -> ! {
        loop {
            // The analog counts come out of the store in the same lock as the config: the
            // control task put them there on its own tick, and this task only calibrates them.
            let (config, raw_analog) = {
                let store = STORE.lock().await;
                (store.config.clone(), store.raw_analog)
            };
            self.raw_analog = raw_analog;
            let now = Instant::now();

            self.sample(&config).await;
            self.scan_step(&config, now).await;
            self.publish(&config).await;

            Timer::after_millis(config.sensor_interval_ms.max(1) as u64).await;
        }
    }

    /// Read every device currently believed present.
    async fn sample(&mut self, _config: &Config) {
        for slot in AdcSlot::ALL {
            if !self.is_present(slot) {
                self.raw[slot] = RAW_INVALID;
                continue;
            }
            let address = AMPLIFIER_ADDRESSES[slot.amplifier()];
            match self.buses.read(slot.bus(), address).await {
                Some(reading) => {
                    self.raw[slot] = reading.value;
                    if reading.alert {
                        bump(ErrorCounter::AmplifierAlert);
                        defmt::warn!("amplifier ALERT: bus {} addr {=u8:#04x}", slot.bus().as_u8(), address);
                    }
                }
                None => {
                    // A device that stops answering is gone as far as we are concerned; the scan
                    // will find it again if it comes back. This is what makes a cable knocked
                    // loose during assembly visible instead of silently freezing a reading.
                    //
                    // The underlying bus fault is already counted in `ext_adc`; this says which
                    // device it cost us, which is what turns "the bus is unhappy" into "amplifier
                    // 3 on bus 1 is unhappy".
                    bump(ErrorCounter::AmplifierReadFailed);
                    self.raw[slot] = RAW_INVALID;
                    self.set_present(ProbeTarget::Amplifier(slot), false);
                }
            }
        }

        for bus in I2cBus::ALL {
            if !self.encoder_present(bus) {
                self.raw_angle[bus] = RAW_INVALID;
                continue;
            }
            match self.buses.read_angle(bus).await {
                Some(reading) => {
                    self.raw_angle[bus] = reading.angle;
                    self.set_magnet_ok(bus, reading.magnet_detected && !reading.magnet_out_of_range);
                }
                None => {
                    bump(ErrorCounter::EncoderReadFailed);
                    self.raw_angle[bus] = RAW_INVALID;
                    self.set_present(ProbeTarget::Encoder(bus), false);
                    self.set_magnet_ok(bus, false);
                }
            }
        }
    }

    /// Probe at most one address, so scanning never costs more than one NACK per tick.
    async fn scan_step(&mut self, config: &Config, now: Instant) {
        let Some(target) = self.scan.due(config, now) else {
            return;
        };
        let already_here = match target {
            ProbeTarget::Amplifier(slot) => self.is_present(slot),
            ProbeTarget::Encoder(bus) => self.encoder_present(bus),
        };
        // Already sampling it; no need to spend a transfer confirming that.
        if !already_here {
            let answered = match target {
                ProbeTarget::Amplifier(slot) => {
                    self.buses.probe(slot.bus(), AMPLIFIER_ADDRESSES[slot.amplifier()]).await
                }
                ProbeTarget::Encoder(bus) => self.buses.probe_angle(bus).await,
            };
            if answered {
                self.set_present(target, true);
            }
        }
        if self.scan.wrapped() {
            self.sweeps = self.sweeps.wrapping_add(1);
        }
    }

    /// The raw reading feeding a slot, or `None` when its device is absent or it has no source.
    ///
    /// A configured slot whose device is not there is counted on every sample rather than once,
    /// which is what makes the two ways of being absent distinguishable: a device that vanished
    /// also shows up as an [`ErrorCounter::I2cDeviceLost`] event, while one that was never there
    /// at all — a sensor configured onto a bus or address it is not wired to — shows up only
    /// here, climbing steadily from boot.
    fn raw_for(&self, slot: &SensorSlotConfig) -> Option<u16> {
        // An unconfigured slot has no device to be missing, so it is not counted below.
        let raw = match slot.source()? {
            SensorSource::Adc(adc) => self.raw[adc],
            SensorSource::Encoder(bus) => self.raw_angle[bus],
            // Invalid until the control task has sampled the pin, and again if this build never
            // handed that pin over — which counts as a missing source for the same reason an
            // amplifier that is not wired up does.
            SensorSource::Analog(input) => self.raw_analog[input],
        };
        if raw == RAW_INVALID {
            bump(ErrorCounter::SensorSourceMissing);
            return None;
        }
        Some(raw)
    }

    async fn publish(&mut self, config: &Config) {
        let mut values = PerSensorSlot::splat(SENSOR_INVALID);
        for (id, slot) in config.sensors.iter() {
            values[id] = calibrate(slot, self.raw_for(slot));
        }

        let mut store = STORE.lock().await;
        store.raw_adc = self.raw;
        store.raw_angle = self.raw_angle;
        store.i2c_present = self.present;
        store.i2c_sweeps = self.sweeps;
        store.sensor_value = values;
        store.refresh_pdo_sensors();
    }
}

#[cfg(feature = "hardware")]
#[embassy_executor::task]
pub async fn run_sensors(sensors: &'static mut BoardSensors) -> ! {
    sensors.run().await
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use embedded_hal_async::i2c::{Error as I2cError, ErrorKind, ErrorType, Operation};

    use super::*;
    use crate::config::{NUM_ADC_SLOTS, SensorCalib};
    use crate::index::{AmplifierId, I2cBus};

    /// Answers with a fixed 2-byte conversion register for addresses it's been told to have a
    /// device at, and NACKs everything else — the ADC101C027's whole read-only interface.
    struct MockI2c {
        responses: HashMap<u8, [u8; 2]>,
    }

    /// What a real bus reports for an address with nothing on it. The kind matters: it is what
    /// tells [`ext_adc::I2cFault`] a probe found nothing rather than that the bus is broken.
    #[derive(Debug)]
    struct Nack;
    impl I2cError for Nack {
        fn kind(&self) -> ErrorKind {
            ErrorKind::NoAcknowledge(embedded_hal_async::i2c::NoAcknowledgeSource::Address)
        }
    }

    impl MockI2c {
        fn new() -> Self {
            Self {
                responses: HashMap::new(),
            }
        }

        /// Make `address` answer as if it held `value` (10-bit) with the ALERT flag as given.
        fn respond(&mut self, address: u8, value: u16, alert: bool) {
            let register = ((value & 0x3FF) << 2) | if alert { 0x8000 } else { 0 };
            self.responses.insert(address, register.to_be_bytes());
        }
    }

    impl ErrorType for MockI2c {
        type Error = Nack;
    }

    impl embedded_hal_async::i2c::I2c for MockI2c {
        async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Nack> {
            let bytes = self.responses.get(&address).copied().ok_or(Nack)?;
            for op in operations {
                if let Operation::Read(buf) = op {
                    let n = buf.len().min(bytes.len());
                    buf[..n].copy_from_slice(&bytes[..n]);
                }
            }
            Ok(())
        }
    }

    /// No async executor is available on the host build (see the no-dev-dependencies note in
    /// Cargo.toml), so this busy-polls with a no-op waker. Every future here resolves on the
    /// first poll: `MockI2c::transaction` never actually awaits anything, and `with_timeout`
    /// only needs the mock clock to advance if the wrapped future does not resolve immediately.
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

    fn sensors_with(bus0: MockI2c, bus1: MockI2c) -> Sensors<MockI2c, MockI2c> {
        Sensors::new(Buses {
            bus0: Some(bus0),
            bus1: Some(bus1),
        })
    }

    #[test]
    fn sample_reads_a_present_amplifier_on_bus0() {
        let mut bus0 = MockI2c::new();
        bus0.respond(AMPLIFIER_ADDRESSES[AmplifierId::Amp0], 512, false);
        let mut sensors = sensors_with(bus0, MockI2c::new());
        sensors.set_present(ProbeTarget::Amplifier(AdcSlot::new(I2cBus::Bus0, AmplifierId::Amp0)), true);

        block_on(sensors.sample(&Config::new()));

        assert_eq!(sensors.raw[AdcSlot::new(I2cBus::Bus0, AmplifierId::Amp0)], 512);
    }

    #[test]
    fn sample_reads_a_present_amplifier_on_bus1() {
        let mut bus1 = MockI2c::new();
        bus1.respond(AMPLIFIER_ADDRESSES[AmplifierId::Amp2], 300, false);
        let mut sensors = sensors_with(MockI2c::new(), bus1);
        let slot = AdcSlot::new(I2cBus::Bus1, AmplifierId::Amp2);
        sensors.set_present(ProbeTarget::Amplifier(slot), true);

        block_on(sensors.sample(&Config::new()));

        assert_eq!(sensors.raw[slot], 300, "bus 1 reads must land in the second half of the raw array");
    }

    #[test]
    fn a_nacked_read_marks_the_slot_absent() {
        // Present but nobody answers this tick: a cable knocked loose during assembly.
        let mut sensors = sensors_with(MockI2c::new(), MockI2c::new());
        sensors.set_present(ProbeTarget::Amplifier(AdcSlot::new(I2cBus::Bus0, AmplifierId::Amp0)), true);

        block_on(sensors.sample(&Config::new()));

        assert_eq!(sensors.raw[AdcSlot::new(I2cBus::Bus0, AmplifierId::Amp0)], RAW_INVALID);
        assert!(
            !sensors.is_present(AdcSlot::new(I2cBus::Bus0, AmplifierId::Amp0)),
            "a device that stops answering is gone as far as we're concerned"
        );
    }

    #[test]
    fn an_alert_flag_is_read_alongside_the_value() {
        let mut bus0 = MockI2c::new();
        bus0.respond(AMPLIFIER_ADDRESSES[AmplifierId::Amp0], 200, true);
        let mut sensors = sensors_with(bus0, MockI2c::new());
        sensors.set_present(ProbeTarget::Amplifier(AdcSlot::new(I2cBus::Bus0, AmplifierId::Amp0)), true);

        block_on(sensors.sample(&Config::new()));

        assert_eq!(
            sensors.raw[AdcSlot::new(I2cBus::Bus0, AmplifierId::Amp0)],
            200,
            "the alert flag must not corrupt the 10-bit value"
        );
    }

    #[test]
    fn scan_step_marks_a_newly_answering_amplifier_present() {
        let mut bus0 = MockI2c::new();
        bus0.respond(AMPLIFIER_ADDRESSES[AmplifierId::Amp0], 100, false);
        let mut sensors = sensors_with(bus0, MockI2c::new());
        let cfg = Config::new();

        // `Sensors::new` seeds its scan cursor from `Instant::now()`, which under the host-test
        // mock clock is always `Instant::from_millis(0)` (nothing in this suite ever advances
        // it) — so one full `scan_interval_ms` later, slot 0 is due.
        let now = Instant::from_millis(cfg.scan_interval_ms as u64);
        block_on(sensors.scan_step(&cfg, now));

        assert!(sensors.is_present(AdcSlot::new(I2cBus::Bus0, AmplifierId::Amp0)));
    }

    #[test]
    fn scan_step_does_nothing_before_a_slot_is_due() {
        let mut bus0 = MockI2c::new();
        bus0.respond(AMPLIFIER_ADDRESSES[AmplifierId::Amp0], 100, false);
        let mut sensors = sensors_with(bus0, MockI2c::new());
        let cfg = Config::new();

        block_on(sensors.scan_step(&cfg, Instant::from_millis(1)));

        assert!(!sensors.is_present(AdcSlot::new(I2cBus::Bus0, AmplifierId::Amp0)), "the interval has not elapsed yet");
    }

    /// The distinction the whole classification exists for: a presence sweep walks addresses that
    /// are *supposed* to be empty, so if a NACK counted as an error a healthy board would clock
    /// up thousands an hour and the counter would mean nothing.
    #[test]
    fn probing_an_empty_address_is_not_an_error() {
        let _guard = crate::errors::test_lock();
        crate::errors::reset_all();

        let mut sensors = sensors_with(MockI2c::new(), MockI2c::new());
        let cfg = Config::new();
        block_on(sensors.scan_step(&cfg, Instant::from_millis(cfg.scan_interval_ms as u64)));

        assert_eq!(crate::errors::count(ErrorCounter::I2cNack), 0);
        assert_eq!(crate::errors::count(ErrorCounter::I2cBusError), 0);
        assert_eq!(crate::errors::count(ErrorCounter::I2cTimeout), 0);
    }

    /// The same NACK from a device the presence bitmap says is there is a real failure, and has
    /// to be counted at both levels: the bus fault, and which device it cost us.
    #[test]
    fn a_device_that_stops_answering_is_counted() {
        let _guard = crate::errors::test_lock();
        crate::errors::reset_all();

        let mut sensors = sensors_with(MockI2c::new(), MockI2c::new());
        sensors.set_present(ProbeTarget::Amplifier(AdcSlot::new(I2cBus::Bus0, AmplifierId::Amp0)), true);

        block_on(sensors.sample(&Config::new()));

        assert_eq!(crate::errors::count(ErrorCounter::I2cNack), 1);
        assert_eq!(crate::errors::count(ErrorCounter::AmplifierReadFailed), 1);
        assert_eq!(crate::errors::count(ErrorCounter::I2cDeviceLost), 1, "and it left the presence bitmap");

        // Sampling again must not count another loss: it is already gone.
        block_on(sensors.sample(&Config::new()));
        assert_eq!(crate::errors::count(ErrorCounter::I2cDeviceLost), 1);
    }

    /// The handover from the control task: counts arrive in `raw_analog` and are calibrated like
    /// any other slot's. Nothing here touches I2C, which is the point — the pin is on the
    /// STM32's own ADC.
    #[test]
    fn an_ntc_slot_reads_the_counts_the_control_task_left_behind() {
        let _guard = crate::errors::test_lock();
        crate::errors::reset_all();

        use crate::index::AnalogInput;
        let mut sensors = sensors_with(MockI2c::new(), MockI2c::new());
        let slot = SensorSlotConfig::ntc(AnalogInput::Com6Pin1);

        // Before the control task has sampled anything, the slot has no reading — the same
        // answer, and the same counter, as an amplifier that is not wired up.
        assert_eq!(calibrate(&slot, sensors.raw_for(&slot)), SENSOR_INVALID);
        assert_eq!(crate::errors::count(ErrorCounter::SensorSourceMissing), 1);

        sensors.raw_analog[AnalogInput::Com6Pin1] = 2048;
        assert_eq!(calibrate(&slot, sensors.raw_for(&slot)), 2_500, "25.00 C");

        // And only the pin it names: the other three are nothing to do with this slot.
        sensors.raw_analog[AnalogInput::Com5Pin1] = 1825;
        assert_eq!(calibrate(&slot, sensors.raw_for(&slot)), 2_500);
    }

    /// A sensor pointed at hardware that is not there never transitions, so nothing event-based
    /// would ever fire — this is the counter that catches a board wired differently from its
    /// config.
    #[test]
    fn a_sensor_configured_onto_absent_hardware_counts_every_sample() {
        let _guard = crate::errors::test_lock();
        crate::errors::reset_all();

        let mut sensors = sensors_with(MockI2c::new(), MockI2c::new());
        let config = Config::new().with_sensor(
            crate::index::SensorSlot::Slot0,
            SensorSlotConfig::pressure(I2cBus::Bus0, AmplifierId::Amp0, Unit::CentiBar, SensorCalib::UNITY),
        );

        block_on(sensors.publish(&config));
        assert_eq!(crate::errors::count(ErrorCounter::SensorSourceMissing), 1);

        block_on(sensors.publish(&config));
        assert_eq!(crate::errors::count(ErrorCounter::SensorSourceMissing), 2, "a sample counter, not an event one");

        assert_eq!(
            crate::errors::count(ErrorCounter::I2cDeviceLost),
            0,
            "nothing was ever there to be lost, which is what distinguishes this from a cable falling out"
        );
    }

    #[test]
    fn pt1000_midscale_is_zero_celsius() {
        // 512 counts puts the amplifier output exactly at its 1.65 V offset, so the bridge is
        // balanced and the RTD is at its nominal 1000 ohm.
        assert_eq!(pt1000_milli_celsius(512), 0);
    }

    #[test]
    fn pt1000_matches_the_float_derivation() {
        // Worked through the float chain by hand: 600 counts -> 8.489 degrees C.
        assert_eq!(pt1000_milli_celsius(600), 8488);
    }

    /// The lookup brackets on a descending curve, so a table that stopped descending would send
    /// it past the end of its own array.
    #[test]
    fn the_ntc_curve_descends() {
        for pair in NTC_CURVE.windows(2) {
            assert!(pair[0] > pair[1], "NTC_CURVE must fall monotonically: {pair:?}");
        }
    }

    #[test]
    fn every_ntc_curve_point_maps_back_to_its_own_temperature() {
        for (i, &counts) in NTC_CURVE.iter().enumerate() {
            let expected = (NTC_CURVE_MIN_C + i as i32 * NTC_CURVE_STEP_C) * 1000;
            assert_eq!(ntc_milli_celsius(counts), Some(expected), "curve point {i}");
        }
    }

    /// 25 C sits at mid-scale with a 10k/10k divider, and an open or shorted thermistor has no
    /// temperature at all rather than the coldest or hottest one on the curve.
    #[test]
    fn the_ntc_curve_reads_room_temperature_at_mid_scale() {
        assert_eq!(ntc_milli_celsius(2048), Some(25_000));
        assert_eq!(ntc_milli_celsius(4095), None, "open thermistor");
        assert_eq!(ntc_milli_celsius(0), None, "shorted thermistor");
    }

    /// An NTC slot is complete as soon as its kind is written: the curve is the part's, so the
    /// trim starts at unity and the slot reports centicelsius off the COM5/COM6 pin.
    #[test]
    fn an_untrimmed_ntc_slot_reports_the_curve_verbatim() {
        let slot = SensorSlotConfig::ntc(crate::index::AnalogInput::Com5Pin1);
        assert_eq!(calibrate(&slot, Some(2048)), 2_500, "25.00 C in centicelsius");
        // Off the curve is no reading, not a clamp to the end of it: an open thermistor must not
        // look like a very cold one.
        assert_eq!(calibrate(&slot, Some(4095)), SENSOR_INVALID);
        assert_eq!(calibrate(&slot, None), SENSOR_INVALID);
    }

    /// The other way of wiring the divider reads the same curve from the other end. A slot on
    /// the wrong one of the two is the failure this kind exists to prevent: 25 C read as a
    /// plausible, wrong temperature that moves the wrong way.
    #[test]
    fn the_two_ntc_wirings_mirror_each_other() {
        use crate::index::AnalogInput;
        let to_ground = SensorSlotConfig::ntc(AnalogInput::Com5Pin1);
        let to_supply = SensorSlotConfig::ntc_to_supply(AnalogInput::Com5Pin1);

        // Mid-scale is 25 C either way round: that is where the two legs are equal.
        assert_eq!(calibrate(&to_ground, Some(2048)), 2_500);
        assert_eq!(calibrate(&to_supply, Some(ADC_FULL_SCALE - 2048)), 2_500);

        // And they move in opposite directions from there.
        assert!(calibrate(&to_ground, Some(1825)) > 2_500, "falls as it heats");
        assert!(calibrate(&to_supply, Some(1825)) < 2_500, "rises as it heats");
    }

    /// What an operator actually does with an NTC: read it against a thermometer and shift it.
    /// The offset is in millidegrees, like a Pt1000's, because both linearise to millicelsius.
    #[test]
    fn an_ntc_offset_trim_shifts_the_reading() {
        let mut slot = SensorSlotConfig::ntc(crate::index::AnalogInput::Com6Pin1);
        slot.calib.constant_milli = -1_500;
        assert_eq!(calibrate(&slot, Some(2048)), 2_350, "25.00 C trimmed down by 1.5 C");
    }

    /// The raw-counts unit bypasses the curve for every kind, which is how a pin gets read while
    /// its divider is still being worked out.
    #[test]
    fn an_ntc_slot_in_raw_counts_passes_the_reading_through() {
        let mut slot = SensorSlotConfig::ntc(crate::index::AnalogInput::Com5Pin2);
        slot.unit = Unit::RawCounts;
        assert_eq!(calibrate(&slot, Some(4095)), 4095, "off the curve, but still a number");
    }

    /// A slot that has been told what it is but not what its numbers mean must say so, rather
    /// than reporting the constant term a zero slope would otherwise produce. A transducer
    /// confidently reading 0.00 bar is far more dangerous than one admitting it has no reading.
    #[test]
    fn an_uncalibrated_slot_reports_no_reading() {
        let mut slot = SensorSlotConfig::pressure(
            I2cBus::Bus0,
            AmplifierId::Amp0,
            Unit::CentiBar,
            crate::config::SensorCalib::ZERO,
        );
        assert_eq!(calibrate(&slot, Some(600)), SENSOR_INVALID);

        // A constant with no slope is still not a calibration — it is the same number forever.
        slot.calib.constant_milli = 5_000;
        assert_eq!(calibrate(&slot, Some(600)), SENSOR_INVALID);

        // The moment a slope arrives it is a real sensor again.
        slot.calib.slope_nano = 100_000_000;
        assert_eq!(calibrate(&slot, Some(600)), 6500);
    }

    /// Raw counts are what you read *while* working a calibration out, so the bypass has to come
    /// before the uncalibrated check — otherwise there would be no way to see the numbers you
    /// need in order to stop being uncalibrated.
    #[test]
    fn an_uncalibrated_slot_still_shows_raw_counts() {
        let mut slot = SensorSlotConfig::pressure(
            I2cBus::Bus0,
            AmplifierId::Amp0,
            Unit::RawCounts,
            crate::config::SensorCalib::ZERO,
        );
        assert_eq!(calibrate(&slot, Some(600)), 600);

        slot.kind = SensorKind::Angle;
        assert_eq!(calibrate(&slot, Some(2048)), 2048, "the same has to hold for an encoder's stops");
    }

    /// The two kinds whose transfer function is a property of the board or the part number come
    /// up working with no calibration written at all; the two whose curve is per-installation
    /// stay silent until someone supplies one.
    #[test]
    fn only_the_kinds_with_a_board_level_curve_have_a_working_default() {
        for (kind, works) in [
            (SensorKind::Pt1000, true),
            (SensorKind::Mcp9700, true),
            (SensorKind::Pressure, false),
            (SensorKind::Angle, false),
        ] {
            let slot = SensorSlotConfig {
                kind,
                bus: Some(I2cBus::Bus0),
                unit: kind.natural_unit(),
                calib: kind.default_calib(),
                ..SensorSlotConfig::unused()
            };
            assert_eq!(
                calibrate(&slot, Some(600)) != SENSOR_INVALID,
                works,
                "{kind:?} should {} report on its default calibration",
                if works { "" } else { "not" }
            );
        }
    }

    /// The bridge is the board; the slot's calibration is the trim on top of it. A Pt1000 slot
    /// straight out of the factory defaults must therefore report exactly the bridge, or the
    /// trim has quietly become part of the curve.
    #[test]
    fn an_untrimmed_pt1000_slot_reports_the_bridge_verbatim() {
        let slot = SensorSlotConfig::pt1000(I2cBus::Bus0, AmplifierId::Amp0);
        assert_eq!(calibrate(&slot, Some(512)), 0);
        // 8.489 degrees C, in the centicelsius the slot reports.
        assert_eq!(calibrate(&slot, Some(600)), 848);
    }

    /// What an operator actually does with a Pt1000: put it in an ice bath, see it read half a
    /// degree high, and take that back out. The offset is in microcelsius because the bridge
    /// linearises to millicelsius — see the table on `SensorCalib`.
    #[test]
    fn a_pt1000_offset_trim_shifts_the_reading() {
        let mut slot = SensorSlotConfig::pt1000(I2cBus::Bus0, AmplifierId::Amp0);
        slot.calib.offset_milli = 500;
        assert_eq!(calibrate(&slot, Some(512)), -50, "half a degree, in centicelsius");
        assert_eq!(calibrate(&slot, Some(600)), 798);
    }

    /// The MCP9700's datasheet curve against a 3.3 V, 10-bit conversion, with no trim applied:
    /// 500 mV at 0 degC, 10 mV per degree.
    #[test]
    fn an_mcp9700_follows_its_datasheet_curve() {
        let slot = SensorSlotConfig::mcp9700(I2cBus::Bus0, AmplifierId::Amp0);
        // 155.15 counts is 0.5 V, the sensor's zero.
        assert_eq!(calibrate(&slot, Some(155)), -4, "within a rounding step of zero");
        // 500 counts is 1.611 V, i.e. 111.1 degC.
        assert_eq!(calibrate(&slot, Some(500)), 11113);
    }

    /// The two temperature kinds are calibrated through the same three fields, which is the whole
    /// point of one calibration record: a gain trim means the same thing on either.
    #[test]
    fn a_gain_trim_scales_either_temperature_kind() {
        for mut slot in [
            SensorSlotConfig::pt1000(I2cBus::Bus0, AmplifierId::Amp0),
            SensorSlotConfig::mcp9700(I2cBus::Bus0, AmplifierId::Amp0),
        ] {
            let nominal = calibrate(&slot, Some(700)) as i32;
            slot.calib.slope_nano = (slot.calib.slope_nano as i64 * 11 / 10) as i32;
            let trimmed = calibrate(&slot, Some(700)) as i32;
            // Within a count, since the trimmed slope itself is rounded to an integer nano-unit.
            assert!(
                (trimmed - nominal * 11 / 10).abs() <= 1,
                "a 10% gain trim should scale the reading by 10%: {nominal} -> {trimmed}"
            );
        }
    }

    /// An encoder whose valve sweeps a quarter turn: closed at count 100, open at 1124.
    #[test]
    fn an_encoder_maps_its_travel_onto_promille() {
        let slot = SensorSlotConfig::encoder(I2cBus::Bus0, 100, 1024);
        assert_eq!(calibrate(&slot, Some(100)), 0, "the closed stop");
        assert_eq!(calibrate(&slot, Some(612)), 500, "halfway");
        assert_eq!(calibrate(&slot, Some(1124)), 1000, "the open stop");
    }

    /// The reason the encoder's zeroing wraps rather than subtracting: a valve whose closed stop
    /// sits near the top of the turn still reads 0 there and climbs, instead of jumping the
    /// whole scale as it crosses the encoder's own origin.
    #[test]
    fn an_encoder_zero_near_the_wrap_point_still_climbs() {
        // Closed at count 4000, open a quarter turn later at 928 — across the wrap.
        let slot = SensorSlotConfig::encoder(I2cBus::Bus0, 4000, 1024);
        assert_eq!(calibrate(&slot, Some(4000)), 0);
        assert_eq!(calibrate(&slot, Some(4090)), 87, "still just short of the stop");
        assert_eq!(calibrate(&slot, Some(0)), 93, "past the wrap, and still climbing");
        assert_eq!(calibrate(&slot, Some(928)), 1000, "the open stop");
    }

    /// A valve that opens counter-clockwise: the same quarter turn, walked the other way. Without
    /// the signed span its travel would sit at the far end of the turn from its own zero.
    #[test]
    fn an_encoder_on_a_reversed_valve_still_opens_upward() {
        // Closed at count 1124, open a quarter turn *below* it at 100.
        let slot = SensorSlotConfig::encoder(I2cBus::Bus0, 1124, -1024);
        assert_eq!(calibrate(&slot, Some(1124)), 0, "the closed stop");
        assert_eq!(calibrate(&slot, Some(612)), 500, "halfway");
        assert_eq!(calibrate(&slot, Some(100)), 1000, "the open stop");
    }

    /// Reversed travel across the encoder's wrap point, which is the case that has both
    /// complications at once.
    #[test]
    fn a_reversed_encoder_also_crosses_the_wrap_point() {
        // Closed at 500, open a quarter turn below at 3572 — down through zero.
        let slot = SensorSlotConfig::encoder(I2cBus::Bus0, 500, -1024);
        assert_eq!(calibrate(&slot, Some(500)), 0);
        assert_eq!(calibrate(&slot, Some(4090)), 494, "just past the wrap, about halfway open");
        assert_eq!(calibrate(&slot, Some(3572)), 1000, "the open stop");
    }

    /// Reading the stops is the first half of commissioning an encoder, and it has to work
    /// before there is any calibration to read them through.
    #[test]
    fn an_encoder_on_raw_counts_reports_the_angle_itself() {
        let mut slot = SensorSlotConfig::encoder(I2cBus::Bus0, 0, ENCODER_FULL_SCALE as i16);
        slot.unit = Unit::RawCounts;
        assert_eq!(calibrate(&slot, Some(2048)), 2048);
    }

    #[test]
    fn pt1000_is_monotonic_across_the_range() {
        let mut previous = i32::MIN;
        for raw in 0..=1023u16 {
            let t = pt1000_milli_celsius(raw);
            assert!(t > previous, "not monotonic at raw={raw}");
            previous = t;
        }
    }

    /// The calibration the bench data is defined against:
    /// `pressure_bar = (adc_reading - offset) * linear_factor`.
    #[test]
    fn the_plain_linear_form_has_no_constant_term() {
        let calib = SensorCalib::from_per_count(100.0, 0.1);
        let slot = SensorSlotConfig::pressure(I2cBus::Bus0, AmplifierId::Amp0, Unit::CentiBar, calib);

        // At the offset the sensor reads exactly zero, not ambient.
        assert_eq!(calibrate(&slot, Some(100)), 0);
        // (200 - 100) * 0.1 bar = 10 bar = 1000 centibar.
        assert_eq!(calibrate(&slot, Some(200)), 1000);
    }

    #[test]
    fn a_constant_term_shifts_the_whole_curve() {
        let calib = SensorCalib::from_per_count(100.0, 0.1).with_constant(1.013);
        let slot = SensorSlotConfig::pressure(I2cBus::Bus0, AmplifierId::Amp0, Unit::CentiBar, calib);

        assert_eq!(calibrate(&slot, Some(100)), 101, "1.013 bar, in centibar");
        assert_eq!(calibrate(&slot, Some(200)), 1101, "and the slope is unchanged");
    }

    #[test]
    fn a_400_bar_sensor_needs_decibar_to_avoid_clipping() {
        // 0.911 bar per count with no offset: full scale is well past what centibar can hold.
        let calib = SensorCalib::from_per_count(0.0, 0.911_161_7);
        let centibar = SensorSlotConfig::pressure(I2cBus::Bus0, AmplifierId::Amp0, Unit::CentiBar, calib);
        let decibar = SensorSlotConfig::pressure(I2cBus::Bus0, AmplifierId::Amp0, Unit::DeciBar, calib);

        // 1023 counts * 0.9111617 bar = 932.12 bar, i.e. 93212 centibar (which does not fit) or
        // 9321 decibar (which does).
        assert_eq!(calibrate(&centibar, Some(1023)), i16::MAX, "centibar saturates");
        assert_eq!(calibrate(&decibar, Some(1023)), 9321, "decibar has the range");
    }

    #[test]
    fn a_40_bar_sensor_keeps_its_resolution_in_centibar() {
        let calib = SensorCalib::from_per_count(15.0, 0.0855);
        let slot = SensorSlotConfig::pressure(I2cBus::Bus0, AmplifierId::Amp0, Unit::CentiBar, calib);
        // (500 - 15) * 0.0855 bar = 41.4675 bar = 4146 centibar.
        assert_eq!(calibrate(&slot, Some(500)), 4146);
    }

    #[test]
    fn a_negative_offset_is_handled() {
        // D_40BAR has offset -385, so every reading sits above the zero point.
        let calib = SensorCalib::from_per_count(-385.0, 0.0535);
        let slot = SensorSlotConfig::pressure(I2cBus::Bus0, AmplifierId::Amp0, Unit::CentiBar, calib);
        // (0 - -385) * 0.0535 bar = 20.5975 bar.
        assert_eq!(calibrate(&slot, Some(0)), 2059);
    }

    #[test]
    fn a_reading_below_the_offset_goes_negative() {
        // Gauge pressure below the calibration zero is a real reading, not an error, so it must
        // survive as a negative number rather than wrapping.
        let calib = SensorCalib::from_per_count(500.0, 0.1);
        let slot = SensorSlotConfig::pressure(I2cBus::Bus0, AmplifierId::Amp0, Unit::CentiBar, calib);
        assert_eq!(calibrate(&slot, Some(400)), -1000);
    }

    #[test]
    fn a_missing_reading_is_reported_as_invalid() {
        let slot = SensorSlotConfig::pt1000(I2cBus::Bus0, AmplifierId::Amp0);
        assert_eq!(calibrate(&slot, None), SENSOR_INVALID);
    }

    #[test]
    fn raw_counts_bypass_calibration() {
        let mut slot = SensorSlotConfig::pressure(
            I2cBus::Bus0,
            AmplifierId::Amp0,
            Unit::RawCounts,
            SensorCalib::from_per_count(9.0, 9.0),
        );
        slot.unit = Unit::RawCounts;
        assert_eq!(calibrate(&slot, Some(777)), 777);
    }

    #[test]
    fn an_unused_slot_reports_nothing_even_with_a_reading() {
        let slot = SensorSlotConfig::unused();
        assert_eq!(calibrate(&slot, Some(500)), SENSOR_INVALID);
    }

    #[test]
    fn the_scan_visits_every_address_before_repeating() {
        let mut cursor = ScanCursor::new(Instant::from_millis(0));
        let cfg = Config::new();
        let mut seen: Vec<ProbeTarget> = Vec::new();

        for step in 1..=ProbeTarget::COUNT as u64 {
            let now = Instant::from_millis(step * cfg.scan_interval_ms as u64);
            let target = cursor.due(&cfg, now).expect("a probe is due");
            assert!(!seen.contains(&target), "{target:?} probed twice in one sweep");
            seen.push(target);
        }
        assert_eq!(seen.len(), NUM_ADC_SLOTS + 2, "every amplifier plus one encoder per bus");
        // The encoders are the tail of the sweep, so they are probed exactly as often as any one
        // amplifier rather than once per amplifier.
        assert_eq!(seen[NUM_ADC_SLOTS..], [ProbeTarget::Encoder(I2cBus::Bus0), ProbeTarget::Encoder(I2cBus::Bus1)]);
        assert!(cursor.wrapped(), "the sweep should have wrapped");
    }

    #[test]
    fn the_scan_waits_for_its_interval() {
        let mut cursor = ScanCursor::new(Instant::from_millis(0));
        let cfg = Config::new();
        assert!(cursor.due(&cfg, Instant::from_millis(100)).is_none());
        assert!(cursor.due(&cfg, Instant::from_millis(500)).is_some());
    }

    #[test]
    fn a_zero_interval_disables_scanning() {
        let mut cursor = ScanCursor::new(Instant::from_millis(0));
        let cfg = Config {
            scan_interval_ms: 0,
            ..Config::new()
        };
        assert!(cursor.due(&cfg, Instant::from_millis(100_000)).is_none());
    }
}
