//! What is on a sensor slot, how its raw number becomes a physical value, and in what unit.
//!
//! One [`SensorSlotConfig`] per slot, covering three separable questions:
//!
//! 1. **Which device** — answered by [`SensorKind`] plus the slot's bus, via [`SensorSlotConfig::source`].
//! 2. **What the number means** — [`SensorCalib`], one affine record that serves every kind
//!    because the kinds differ in how their raw number is *linearised*, not in what a human then
//!    does to trim it. The linearisation itself lives in `crate::sensors::linearise`, next to the
//!    task that samples the hardware.
//! 3. **What to report it as** — [`Unit`], which is per slot rather than global because no single
//!    scale works for a 400 bar transducer, a Pt1000 and a valve angle at once.
//!
//! All of it is runtime-writable (0x3020..0x3028) and persisted, so a sensor can be recalibrated,
//! moved to a different amplifier, or taken off the process data plane without a firmware build.

use crate::index::{AdcSlot, AmplifierId, AnalogInput, I2cBus, PdoSensorChannel, PerAmplifier, SensorSlot};

/// Sizes of the sensor-side domains. See the note in [`super`] on why these exist as plain
/// numbers at all.
pub const NUM_AMPLIFIERS: usize = AmplifierId::COUNT;
/// Every probe-able amplifier slot on the board: both buses, all nine straps.
pub const NUM_ADC_SLOTS: usize = AdcSlot::COUNT;
pub const NUM_SENSOR_SLOTS: usize = SensorSlot::COUNT;
/// How many of those slots can be on the bus as process data at once. Fewer than
/// [`NUM_SENSOR_SLOTS`] on purpose — see [`PdoSensorChannel`].
pub const NUM_PDO_SENSOR_CHANNELS: usize = PdoSensorChannel::COUNT;
/// The STM32's own ADC pins an external NTC can sit on: both pins of COM5 and both of COM6.
pub const NUM_ANALOG_INPUTS: usize = AnalogInput::COUNT;

/// ADC101C027 amplifier addresses, in scan order. Everything that talks about an "amplifier
/// index" means an [`AmplifierId`], never a raw I2C address — the index is what travels over CAN,
/// so that a 9-entry bitmap fits one u16 per bus.
pub const AMPLIFIER_ADDRESSES: PerAmplifier<u8> = PerAmplifier::new([
    0b101_0000, // floating, floating
    0b101_0001, // floating, gnd
    0b101_0010, // floating, vcc
    0b101_0100, // gnd, floating
    0b101_0101, // gnd, gnd
    0b101_0110, // gnd, vcc
    0b101_1000, // vcc, floating
    0b101_1001, // vcc, gnd
    0b101_1010, // vcc, vcc
]);

/// I2C address of the AS5600 magnetic rotary encoder. Fixed in the part — there are no address
/// straps — so a bus can carry exactly one, and naming the bus names the encoder. That is why an
/// `Angle` slot's `amplifier` field is ignored while every other kind needs it.
pub const ENCODER_ADDRESS: u8 = 0x36;

/// Counts per full turn of the AS5600's 12-bit angle register.
pub const ENCODER_FULL_SCALE: u16 = 4096;

/// Bit of a bus's 0x2002 presence word set when its AS5600 answered the last sweep.
///
/// The low nine bits are the amplifier straps, so the encoder takes the next one up and the
/// bitmap keeps its old meaning for anything already reading it.
pub const ENCODER_PRESENT_BIT: u16 = 1 << 9;

/// Bit of a bus's 0x2002 presence word set when its AS5600 reports a magnet at a usable
/// distance. An encoder that answers but reads `MH`/`ML` still returns an angle, and that angle
/// is wrong — which is exactly the assembly mistake this bitmap exists to make visible.
pub const ENCODER_MAGNET_OK_BIT: u16 = 1 << 10;

/// What is physically on a sensor slot.
///
/// The kind decides two things: which device the slot is read from, and how its raw number is
/// linearised before the slot's calibration is applied. Everything after that — the trim, the
/// unit the result is reported in — is per-slot configuration, not per-kind.
///
/// The discriminants are a wire and flash encoding (0x3022, and the persisted record), so 0..2
/// keep the meaning they have always had. Append, never renumber.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
#[repr(u8)]
pub enum SensorKind {
    None = 0,
    /// Linear pressure transducer through an instrumentation amplifier. The raw ADC count is
    /// already linear in pressure, so the calibration is the whole transfer function.
    Pressure = 1,
    /// Pt1000 RTD in a Wheatstone bridge. The bridge inversion is fixed (it is set by the
    /// board's resistors, not by the probe); the calibration on top of it trims out amplifier
    /// offset and gain error.
    Pt1000 = 2,
    /// MCP9700 analogue temperature sensor: 10 mV/degC, 500 mV at 0 degC, straight into an
    /// amplifier channel. Linear like a transducer, so the calibration carries the whole scale.
    Mcp9700 = 3,
    /// AS5600 magnetic rotary encoder on the slot's I2C bus. Read as a 12-bit angle, zeroed
    /// *modulo a full turn* rather than linearly — see [`SensorCalib::apply_wrapped`] — so a
    /// valve whose closed position sits near the wrap point still reads sensibly.
    Angle = 4,
    /// 10k NTC thermistor in a divider on one of the STM32's own ADC pins ([`AnalogInput`]),
    /// i.e. on COM5 or COM6 rather than on an I2C amplifier. **10k from the pin to +3.3V and the
    /// thermistor from the pin to ground**, so the reading falls as it heats; the other way round
    /// is [`Self::NtcToSupply`].
    ///
    /// The curve is hardcoded — it belongs to the part number, not to the installation — so the
    /// calibration on top of it is a trim, the same way a [`Self::Pt1000`]'s is. See
    /// `crate::sensors::ntc_milli_celsius`.
    ///
    /// This and [`Self::NtcToSupply`] are the kinds whose slot names an
    /// [`SensorSlotConfig::analog`] input instead of a bus and an address strap.
    Ntc = 5,
    /// [`Self::Ntc`] with the two legs of the divider swapped: the thermistor to +3.3V and the
    /// 10k to ground, so the reading *rises* as it heats. The same curve, read from the other
    /// end — with a 10k fixed leg, swapping the legs turns a reading `x` into `full scale - x`
    /// exactly.
    ///
    /// Its own kind rather than a flag because the wiring changes what a raw count *means*, and
    /// a slot on the wrong one of the two reads a mirrored curve: a plausible temperature that
    /// moves the wrong way. That is worth a wire code of its own, so which one a slot is on is
    /// visible in the same object that says it is a thermistor at all.
    NtcToSupply = 6,
}

impl SensorKind {
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::None),
            1 => Some(Self::Pressure),
            2 => Some(Self::Pt1000),
            3 => Some(Self::Mcp9700),
            4 => Some(Self::Angle),
            5 => Some(Self::Ntc),
            6 => Some(Self::NtcToSupply),
            _ => None,
        }
    }

    /// The unit a slot of this kind reports in until told otherwise.
    pub const fn natural_unit(self) -> Unit {
        match self {
            Self::None => Unit::CentiBar,
            Self::Pressure => Unit::CentiBar,
            Self::Pt1000 | Self::Mcp9700 | Self::Ntc | Self::NtcToSupply => Unit::CentiCelsius,
            Self::Angle => Unit::Promille,
        }
    }

    /// Whether this kind is read over I2C, i.e. whether the slot's bus and address strap mean
    /// anything.
    ///
    /// False only for the [`Self::Ntc`] kinds, which are on the board's own ADC pins. Written as
    /// a question about the kind rather than checked at each call site so that a new kind on a
    /// new transport has one place to declare itself.
    pub const fn is_on_i2c(self) -> bool {
        match self {
            Self::Pressure | Self::Pt1000 | Self::Mcp9700 | Self::Angle => true,
            Self::None | Self::Ntc | Self::NtcToSupply => false,
        }
    }

    /// The calibration a freshly configured slot of this kind starts from.
    ///
    /// A default is offered exactly where the transfer function is a property of *the board or
    /// the part number*, so one answer is right for every installation:
    ///
    /// - [`Self::Pt1000`] — the bridge inversion is the board's own resistors, already applied
    ///   before the calibration is reached, so a unity trim is the honest starting point and a
    ///   slot that is never trimmed still reads a real temperature.
    /// - [`Self::Mcp9700`] — 10 mV/degC and 500 mV at zero are in the datasheet.
    /// - [`Self::Ntc`] — the thermistor curve is the part's (10k at 25 degC, B = 3950 K) and the
    ///   divider is the harness's, both already applied before the calibration is reached. What
    ///   is left for the trim is the tolerance of the two resistances and the thermistor's own
    ///   beta spread, which is what the offset is for — measured against a thermometer, the same
    ///   way a Pt1000's is against a bath.
    ///
    /// The other kinds get [`SensorCalib::ZERO`], which reports nothing at all rather than a
    /// confident wrong number, because their curve belongs to the individual sensor rather than
    /// to its type: a transducer's slope comes off the bench, and an encoder's comes from where
    /// its magnet happened to end up relative to the valve's stops. Inventing a plausible default
    /// for either would produce a slot that looks configured and reads wrong, which is worse than
    /// one that says it has no reading.
    pub const fn default_calib(self) -> SensorCalib {
        match self {
            // Both already arrive in millicelsius, from a curve that is a property of the board
            // or the part rather than of the installation, so the trim starts at unity.
            Self::Pt1000 | Self::Ntc | Self::NtcToSupply => SensorCalib::UNITY,
            Self::Mcp9700 => SensorCalib::MCP9700,
            Self::None | Self::Pressure | Self::Angle => SensorCalib::ZERO,
        }
    }
}

/// How to scale a slot's physical value into the signed 16-bit number that goes on the bus.
///
/// A fixed unit cannot serve every sensor: a 400 bar transducer overflows i16 centibar, while
/// centibar is the natural resolution for a 40 bar one. So each slot declares its own, and the
/// codes are mirrored read-only into 0x2005 so a master can decode 0x2004 without reading config.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
#[repr(u8)]
pub enum Unit {
    /// 0.01 bar per count. Range +-327.67 bar.
    CentiBar = 0,
    /// 0.1 bar per count. For transducers above 300 bar.
    DeciBar = 1,
    /// 0.01 degrees Celsius per count.
    CentiCelsius = 2,
    /// Uncalibrated device counts, passed through: ADC counts for an amplifier channel, angle
    /// counts for an encoder. Useful while calibrating, and the only way to see what an encoder
    /// actually reads at the closed and open stops.
    RawCounts = 3,
    /// Promille, 0..1000. What an [`SensorKind::Angle`] slot reports so that it can be fed
    /// straight back in as a valve's measured position, which is in the same unit.
    Promille = 4,
}

impl Unit {
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::CentiBar),
            1 => Some(Self::DeciBar),
            2 => Some(Self::CentiCelsius),
            3 => Some(Self::RawCounts),
            4 => Some(Self::Promille),
            _ => None,
        }
    }

    /// What is being measured, independent of how finely.
    pub const fn quantity(self) -> Quantity {
        match self {
            Self::CentiBar | Self::DeciBar => Quantity::Pressure,
            Self::CentiCelsius => Quantity::Temperature,
            Self::RawCounts => Quantity::Raw,
            Self::Promille => Quantity::Ratio,
        }
    }

    /// How finely, independent of what.
    pub const fn prefix(self) -> UnitPrefix {
        match self {
            Self::DeciBar => UnitPrefix::Deci,
            Self::CentiBar | Self::CentiCelsius => UnitPrefix::Centi,
            Self::Promille => UnitPrefix::Milli,
            Self::RawCounts => UnitPrefix::None,
        }
    }

    /// Divisor from the milli-unit that calibration works in to this unit's counts.
    ///
    /// Calibration is done in thousandths throughout — millibar, millicelsius, milli-promille —
    /// and only the last step scales to whatever the slot reports on the wire. Keeping the
    /// intermediate fixed means a calibration coefficient means the same thing regardless of
    /// which unit a slot happens to be set to.
    pub const fn per_milli(self) -> i32 {
        match self.prefix() {
            UnitPrefix::Deci => 100,
            UnitPrefix::Centi => 10,
            UnitPrefix::Milli => 1,
            // Raw counts never reach this path; see `crate::sensors::calibrate`.
            UnitPrefix::None => 1,
        }
    }
}

/// The physical quantity half of a [`Unit`].
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum Quantity {
    Pressure,
    Temperature,
    /// Dimensionless, e.g. how far open a valve is.
    Ratio,
    /// Device counts, with no physical meaning attached.
    Raw,
}

/// The scale half of a [`Unit`].
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum UnitPrefix {
    None,
    Deci,
    Centi,
    Milli,
}

/// Fixed-point affine calibration for one sensor slot.
///
/// One record serves every [`SensorKind`], because the kinds differ in how their raw number is
/// *linearised*, not in what a human then does to trim it. The pipeline is
///
/// ```text
///   linear = kind.linearise(raw)              // milli-units, see crate::sensors::linearise
///   out    = (linear - offset) * slope / 1e9 + constant
/// ```
///
/// Both `linear` and `out` are in thousandths of their quantity, and `offset` is in the same
/// thousandths as `linear` — so the three fields mean:
///
/// | kind | `linear` is | `offset_milli` | `slope_nano` | `constant_milli` |
/// |---|---|---|---|---|
/// | [`SensorKind::Pressure`] | milli-counts | milli-counts | nanobar per count | millibar |
/// | [`SensorKind::Mcp9700`] | milli-counts | milli-counts | nanocelsius per count | millicelsius |
/// | [`SensorKind::Pt1000`] | millicelsius | millicelsius | nanocelsius per millicelsius, unity `1e9` | millicelsius |
/// | [`SensorKind::Ntc`] | millicelsius | millicelsius | nanocelsius per millicelsius, unity `1e9` | millicelsius |
/// | [`SensorKind::Angle`] | milli-counts | milli-counts, wrapping | see [`Self::angle_over`] | milli-promille |
///
/// Everything a slot reports is then scaled from those thousandths to its own [`Unit`], which is
/// what lets a slot's unit change without any of its coefficients moving.
///
/// Counting the raw side in thousandths is what makes one record serve every kind: it gives the
/// analogue kinds a sub-count offset (which is how the bench calibrations are recorded) and the
/// Pt1000 a millidegree one, out of the same field.
///
/// A transducer's bench calibration is `value = (reading - offset) * factor`, which is what
/// [`Self::from_per_count`] expresses. Some transducers are instead characterised against
/// ambient and want a constant added afterwards (1.013 bar, to report absolute rather than gauge
/// pressure); that is [`Self::with_constant`]. Keeping the constant configurable rather than
/// baking one in means both conventions are expressible, and which one a slot uses is visible in
/// its calibration rather than implied by the firmware version.
///
/// Kept in integers on purpose. The STM32F105 is a Cortex-M3 without an FPU, so a float in the
/// hot sensor path pulls the soft-float runtime into a tight flash budget. The human-readable
/// constants stay floats in [`crate::zenith_mapping::sensors`] and are folded down by the `const
/// fn` constructors at compile time.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub struct SensorCalib {
    /// Zero offset, in thousandths of whatever the kind's linearised value is measured in.
    pub offset_milli: i32,
    /// Slope, in nano-output-units per linearised unit. `1_000_000_000` is unity.
    pub slope_nano: i32,
    /// Constant added after the linear part, in thousandths of the output unit.
    pub constant_milli: i32,
}

impl SensorCalib {
    /// `value = (reading - offset) * per_count`, the form the bench calibrations are recorded in.
    ///
    /// `const`, so the float arithmetic happens in the compiler and never reaches the target.
    pub const fn from_per_count(offset_counts: f32, per_count: f32) -> Self {
        Self {
            offset_milli: (offset_counts * 1000.0) as i32,
            slope_nano: (per_count * 1_000_000_000.0) as i32,
            constant_milli: 1_000,
        }
    }

    /// Add a constant term: `value = (reading - offset) * per_count + constant`, in the slot's
    /// whole unit.
    ///
    /// Use `1.013` for a transducer calibrated against ambient that should report absolute
    /// pressure.
    pub const fn with_constant(self, whole_units: f32) -> Self {
        Self {
            constant_milli: (whole_units * 1000.0) as i32,
            ..self
        }
    }

    /// Nothing configured.
    ///
    /// The zero slope is what marks it as such — see [`Self::is_calibrated`] — so a slot left on
    /// this reports no reading rather than a constant.
    pub const ZERO: Self = Self {
        offset_milli: 0,
        slope_nano: 0,
        constant_milli: 0,
    };

    /// A pass-through: the linearised value, in its own milli-units, is the answer.
    ///
    /// The starting point for [`SensorKind::Pt1000`], whose bridge inversion already produces
    /// millicelsius, and for [`SensorKind::Ntc`], whose curve does — in both cases the trim
    /// exists to correct the parts, not to define the curve.
    pub const UNITY: Self = Self {
        offset_milli: 0,
        slope_nano: 1_000_000_000,
        constant_milli: 0,
    };

    /// The MCP9700's datasheet transfer function against a 3.3 V, 10-bit conversion: 500 mV at
    /// 0 degC and 10 mV per degree, so one count is 3.3/1024 V is 0.3223 degC and the zero sits
    /// at 155.15 counts.
    ///
    /// A part-to-part offset of a degree or two is normal for this sensor; trimming `offset_milli`
    /// against a known bath is what the calibration is for.
    pub const MCP9700: Self = Self {
        constant_milli: 0,
        ..Self::from_per_count(155.151_52, 0.322_265_63)
    };

    /// Calibration for an encoder whose valve travels `counts` of the 4096-count turn between
    /// closed and open, with the closed stop at `zero_counts`.
    ///
    /// Reading a valve's two stops in [`Unit::RawCounts`] and putting the numbers in here is the
    /// whole commissioning procedure.
    ///
    /// `counts` is signed: **negative for a valve whose angle decreases as it opens**. Several of
    /// the vehicle's valves open counter-clockwise (the same fact
    /// [`super::ValveConfig::pulse_width_us`] has to cope with), and a reversed encoder is not
    /// expressible any other way — the sign of the slope is what tells [`Self::apply_wrapped`]
    /// which way round the turn the travel runs.
    pub const fn angle_over(zero_counts: u16, counts: i16) -> Self {
        // The span must map to 1_000_000 milli-promille from `counts * 1000` milli-counts, so
        // the slope is 1e9 / span. Rounded away from zero rather than truncated: at 1024 counts
        // the difference is the valve reading 999 promille at its open stop instead of 1000.
        let span = if counts == 0 { 1 } else { counts as i64 };
        let half = if span > 0 { span / 2 } else { -(span / 2) };
        Self {
            offset_milli: zero_counts as i32 * 1000,
            slope_nano: ((1_000_000_000i64 + half) / span) as i32,
            constant_milli: 0,
        }
    }

    /// Whether this record says anything at all.
    ///
    /// A zero slope maps every possible reading onto the same constant, which no real sensor
    /// does — so it is taken as "not calibrated yet" rather than obeyed. `crate::sensors::calibrate`
    /// reports [`crate::store::SENSOR_INVALID`] for such a slot, on the grounds that a
    /// transducer confidently reading 0.00 bar is far more dangerous than one admitting it has no
    /// reading. It is also the state a slot is in the moment its kind is written over SDO and
    /// nothing else has been, which is precisely when a wrong number would be believed.
    pub const fn is_calibrated(&self) -> bool {
        self.slope_nano != 0
    }

    /// `out = (linear - offset) * slope / 1e9 + constant`, both sides in thousandths.
    ///
    /// Widest intermediate is about 9e17 (a full-scale reading against a gain trim at the top of
    /// the i32 range), which is why this is i64.
    pub fn apply(&self, linear: i32) -> i32 {
        let zeroed = linear as i64 - self.offset_milli as i64;
        let out = (zeroed * self.slope_nano as i64) / 1_000_000_000 + self.constant_milli as i64;
        out.clamp(i32::MIN as i64, i32::MAX as i64) as i32
    }

    /// [`Self::apply`], but the zeroing wraps at `full_scale` counts.
    ///
    /// A rotary sensor has no low end: a reading just below the zero angle is a hair short of a
    /// full turn, not a large negative number. Subtracting modulo the turn is what lets a valve's
    /// closed stop sit anywhere on the circle, including across the encoder's own wrap point,
    /// without the reading jumping the whole scale as it passes.
    ///
    /// The sign of the slope picks which way round the circle the travel runs, so the wrap lands
    /// in `[0, turn)` for a valve that opens with increasing angle and in `(-turn, 0]` for one
    /// that opens against it. Reducing into the wrong half would put a reversed valve's whole
    /// travel at the far end of the turn from its zero.
    pub fn apply_wrapped(&self, linear: i32, full_scale: u16) -> i32 {
        let modulus = (full_scale as i32).max(1) * 1000;
        let zeroed = if self.slope_nano < 0 {
            -((self.offset_milli - linear).rem_euclid(modulus))
        } else {
            (linear - self.offset_milli).rem_euclid(modulus)
        };
        Self {
            offset_milli: 0,
            ..*self
        }
        .apply(zeroed)
    }
}

/// Where a slot's raw number comes from.
///
/// Derived from the slot's kind and its addressing fields rather than configured separately: an
/// encoder's address is fixed in the part, so naming the bus names the device; an NTC is on one
/// of the board's own ADC pins, so naming the pin names it; and every other kind is an amplifier
/// at a strap. That keeps 0x3020/0x3021 meaning exactly what they always meant and leaves one
/// fewer object for an operator to get out of step with the kind.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum SensorSource {
    /// An ADC101C027 amplifier channel.
    Adc(AdcSlot),
    /// The AS5600 on this bus.
    Encoder(I2cBus),
    /// One of the STM32's own ADC pins on COM5 or COM6, sampled by the control task rather than
    /// over I2C — see [`crate::sensors::AnalogSensing`].
    Analog(AnalogInput),
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct SensorSlotConfig {
    pub kind: SensorKind,
    /// Which I2C bus, or `None` for an unused slot. Ignored for the [`SensorKind::Ntc`] kinds,
    /// which are not on a bus at all.
    pub bus: Option<I2cBus>,
    /// Which address strap, i.e. which entry of [`AMPLIFIER_ADDRESSES`]. Ignored for
    /// [`SensorKind::Angle`], whose device has no address straps, and for [`SensorKind::Ntc`].
    pub amplifier: AmplifierId,
    /// Which COM5/COM6 pin, for the two [`SensorKind::Ntc`] wirings. Ignored by every other kind.
    ///
    /// A separate field rather than a reinterpretation of `bus` or `amplifier`: those two name a
    /// device on an I2C bus, an NTC is a voltage on one of the STM32's own pins, and giving one
    /// object two meanings depending on a third is how a slot ends up reading a pin nobody
    /// intended. The default is [`AnalogInput::Com5Pin1`], so writing nothing but the kind over
    /// SDO (0x3022 = 5) lands on COM5 pin 1 and reads a real temperature.
    pub analog: AnalogInput,
    pub unit: Unit,
    pub calib: SensorCalib,
    /// Which TPDO sensor channel broadcasts this slot, if any.
    ///
    /// There are more slots than channels, so this is a real choice rather than an index: a slot
    /// with no channel is still sampled, calibrated and readable at 0x2004, it just never goes
    /// out as process data. Two slots may not claim the same channel —
    /// [`super::Config::sanity_check`] rejects that rather than letting one silently win.
    pub pdo_channel: Option<PdoSensorChannel>,
}

impl SensorSlotConfig {
    pub const fn unused() -> Self {
        Self {
            kind: SensorKind::None,
            bus: None,
            amplifier: AmplifierId::Amp0,
            analog: AnalogInput::Com5Pin1,
            unit: Unit::CentiBar,
            calib: SensorCalib::ZERO,
            pdo_channel: None,
        }
    }

    pub const fn pressure(bus: I2cBus, amplifier: AmplifierId, unit: Unit, calib: SensorCalib) -> Self {
        Self {
            kind: SensorKind::Pressure,
            bus: Some(bus),
            amplifier,
            unit,
            calib,
            ..Self::unused()
        }
    }

    /// A slot on `amplifier` whose kind carries its own transfer function, so nothing beyond the
    /// wiring has to be said. The calibration and unit come from [`SensorKind::default_calib`]
    /// and [`SensorKind::natural_unit`], which is the same pair a bare kind write over SDO
    /// installs — so a slot fitted here and a slot configured over the bus start identical.
    const fn from_kind(kind: SensorKind, bus: I2cBus, amplifier: AmplifierId) -> Self {
        Self {
            kind,
            bus: Some(bus),
            amplifier,
            unit: kind.natural_unit(),
            calib: kind.default_calib(),
            ..Self::unused()
        }
    }

    pub const fn pt1000(bus: I2cBus, amplifier: AmplifierId) -> Self {
        Self::from_kind(SensorKind::Pt1000, bus, amplifier)
    }

    pub const fn mcp9700(bus: I2cBus, amplifier: AmplifierId) -> Self {
        Self::from_kind(SensorKind::Mcp9700, bus, amplifier)
    }

    /// A 10k NTC in a divider on one of the COM5/COM6 pins, thermistor to ground.
    ///
    /// No bus and no strap: the pin is the whole address. Like the other kinds whose curve comes
    /// with the part, this is complete as it stands — the slot reads a real temperature before
    /// anyone trims it, and the trim is [`SensorCalib::constant_milli`] in millidegrees.
    pub const fn ntc(analog: AnalogInput) -> Self {
        Self::ntc_wired(SensorKind::Ntc, analog)
    }

    /// [`Self::ntc`] with the divider the other way round: thermistor to +3.3V, 10k to ground.
    pub const fn ntc_to_supply(analog: AnalogInput) -> Self {
        Self::ntc_wired(SensorKind::NtcToSupply, analog)
    }

    const fn ntc_wired(kind: SensorKind, analog: AnalogInput) -> Self {
        Self {
            kind,
            analog,
            unit: kind.natural_unit(),
            calib: kind.default_calib(),
            ..Self::unused()
        }
    }

    /// The AS5600 on `bus`, reporting promille of a valve travel that spans `counts` of the turn
    /// starting at `zero_counts`. Negative `counts` for a valve that opens counter-clockwise —
    /// see [`SensorCalib::angle_over`].
    pub const fn encoder(bus: I2cBus, zero_counts: u16, counts: i16) -> Self {
        Self {
            kind: SensorKind::Angle,
            bus: Some(bus),
            unit: Unit::Promille,
            calib: SensorCalib::angle_over(zero_counts, counts),
            ..Self::unused()
        }
    }

    /// Broadcast this slot on `channel`. Slots 0..12 of a factory default normally take their own
    /// index; anything past that has to be chosen deliberately, which is the point.
    pub const fn on_channel(mut self, channel: PdoSensorChannel) -> Self {
        self.pdo_channel = Some(channel);
        self
    }

    /// Which device this slot reads, or `None` when it is unused or has no bus.
    ///
    /// Both halves of an [`AdcSlot`] are already known-in-range, so there is no bounds check left
    /// to get wrong — the only remaining questions are whether a bus is set and which kind of
    /// device is on it.
    pub const fn source(&self) -> Option<SensorSource> {
        // Exhaustive rather than a catch-all, so a new kind has to say here which device it is
        // read from instead of silently defaulting to an amplifier strap.
        match self.kind {
            SensorKind::None => None,
            // The kinds that are not on a bus, answered before the bus is looked at.
            SensorKind::Ntc | SensorKind::NtcToSupply => Some(SensorSource::Analog(self.analog)),
            SensorKind::Angle => match self.bus {
                Some(bus) => Some(SensorSource::Encoder(bus)),
                None => None,
            },
            SensorKind::Pressure | SensorKind::Pt1000 | SensorKind::Mcp9700 => match self.bus {
                Some(bus) => Some(SensorSource::Adc(AdcSlot::new(bus, self.amplifier))),
                None => None,
            },
        }
    }

    /// Which COM5/COM6 pin this slot reads, or `None` for anything that is not an NTC.
    pub const fn analog_input(&self) -> Option<AnalogInput> {
        match self.source() {
            Some(SensorSource::Analog(input)) => Some(input),
            _ => None,
        }
    }

    /// Which probe-able amplifier position this slot reads, or `None` for an encoder or an unused
    /// slot.
    pub const fn adc_slot(&self) -> Option<AdcSlot> {
        match self.source() {
            Some(SensorSource::Adc(slot)) => Some(slot),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An `Angle` slot has no address strap, so naming its bus has to be enough to find it —
    /// otherwise the amplifier field would have to be kept meaningful for a device that has none.
    #[test]
    fn a_slot_source_follows_its_kind() {
        let encoder = SensorSlotConfig::encoder(I2cBus::Bus1, 0, 1024);
        assert_eq!(encoder.source(), Some(SensorSource::Encoder(I2cBus::Bus1)));
        assert_eq!(encoder.adc_slot(), None, "an encoder is not on an amplifier strap");

        let probe = SensorSlotConfig::pt1000(I2cBus::Bus1, AmplifierId::Amp3);
        assert_eq!(probe.source(), Some(SensorSource::Adc(AdcSlot::new(I2cBus::Bus1, AmplifierId::Amp3))));
        assert_eq!(SensorSlotConfig::unused().source(), None);
    }

    /// An NTC is on one of the board's own pins, so it needs no bus at all — and must not be
    /// held back by one, since nothing would ever set it.
    #[test]
    fn an_ntc_slot_needs_no_bus() {
        let ntc = SensorSlotConfig::ntc(AnalogInput::Com6Pin2);
        assert_eq!(ntc.bus, None);
        assert_eq!(ntc.source(), Some(SensorSource::Analog(AnalogInput::Com6Pin2)));
        assert_eq!(ntc.analog_input(), Some(AnalogInput::Com6Pin2));
        assert_eq!(ntc.adc_slot(), None, "an NTC is not on an amplifier strap");
        // Nothing else reads the analog field, however it happens to be set.
        let mut probe = SensorSlotConfig::pt1000(I2cBus::Bus0, AmplifierId::Amp0);
        probe.analog = AnalogInput::Com6Pin2;
        assert_eq!(probe.analog_input(), None);
    }

    /// A bare `0x3022 = 5` write has to leave a slot that reads a real temperature off COM5, the
    /// way a bare Pt1000 write does — so the kind's own defaults are what the constructor uses.
    #[test]
    fn a_fresh_ntc_slot_is_already_calibrated() {
        let ntc = SensorSlotConfig::ntc(AnalogInput::Com5Pin1);
        assert_eq!(ntc.unit, Unit::CentiCelsius);
        assert_eq!(ntc.calib, SensorCalib::UNITY);
        assert!(ntc.calib.is_calibrated());
        assert_eq!(SensorSlotConfig::unused().analog, AnalogInput::Com5Pin1);
    }

    /// The decomposition the unit codes are really made of, kept as accessors so the wire code
    /// stays one stable byte.
    #[test]
    fn a_unit_splits_into_a_quantity_and_a_prefix() {
        assert_eq!(Unit::DeciBar.quantity(), Quantity::Pressure);
        assert_eq!(Unit::DeciBar.prefix(), UnitPrefix::Deci);
        assert_eq!(Unit::CentiCelsius.quantity(), Quantity::Temperature);
        assert_eq!(Unit::Promille.quantity(), Quantity::Ratio);
        assert_eq!(Unit::RawCounts.prefix(), UnitPrefix::None);
    }

    /// The wire codes are a persisted encoding: 0..3 must keep the meaning they shipped with.
    #[test]
    fn the_original_unit_and_kind_codes_are_unchanged() {
        assert_eq!(Unit::from_u8(0), Some(Unit::CentiBar));
        assert_eq!(Unit::from_u8(3), Some(Unit::RawCounts));
        assert_eq!(SensorKind::from_u8(1), Some(SensorKind::Pressure));
        assert_eq!(SensorKind::from_u8(2), Some(SensorKind::Pt1000));
        assert_eq!(SensorKind::from_u8(5), Some(SensorKind::Ntc));
        assert_eq!(SensorKind::from_u8(6), Some(SensorKind::NtcToSupply));
        assert_eq!(SensorKind::from_u8(7), None);
    }
}
