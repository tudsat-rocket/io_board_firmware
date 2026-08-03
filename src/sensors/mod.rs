//! Reading the amplifier boards, calibrating them, and keeping an eye on who is actually there.
//!
//! Three things happen on the same tick, in one task, because they all contend for the same two
//! I2C buses:
//!
//! 1. **Sampling** every amplifier we currently believe is present.
//! 2. **Calibration**, turning raw counts into the value a slot is configured to report. Both the
//!    slot mapping and the calibration coefficients are runtime-writable (0x3020..0x3025), so a
//!    sensor can be recalibrated or moved to a different amplifier without a firmware build.
//! 3. **Presence scanning**, one address at a time.

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
use crate::config::{SensorKind, SensorSlotConfig, Unit};
#[cfg(any(feature = "hardware", test))]
use crate::index::{AdcSlot, PerAdcSlot, PerI2cBus, PerSensorSlot};
use crate::store::SENSOR_INVALID;
#[cfg(any(feature = "hardware", test))]
use crate::store::{RAW_INVALID, STORE};
#[cfg(any(feature = "hardware", test))]
use ext_adc::Buses;

/// Convert a raw Pt1000 bridge reading to centi-degrees Celsius.
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
///   centi_celsius = 40_000_000 * n / (385 * (361236 - 2n))
/// ```
///
/// which is what is implemented here. Integer rather than float because the STM32F105 is a
/// Cortex-M3 with no FPU; the widest intermediate is about 7e11, hence i64. The denominator
/// cannot reach zero: `raw` is 10-bit, so `n` stays within +-17864 and `361236 - 2n` within
/// [325508, 396964].
pub fn pt1000_centi_celsius(raw: u16) -> i32 {
    let n = 33i64 * raw.min(1023) as i64 - 16_896;
    let denominator = 385 * (361_236 - 2 * n);
    ((40_000_000 * n) / denominator) as i32
}

/// Clamp an i32 into the wire's i16, keeping [`SENSOR_INVALID`] reserved for "no reading".
fn saturate(v: i32) -> i16 {
    v.clamp(i16::MIN as i32 + 1, i16::MAX as i32) as i16
}

/// Turn a raw count into the value a slot reports, in the unit it declares.
///
/// The unit is per slot rather than global because no single scale works for every transducer
/// here: centibar is the natural resolution for a 40 bar sensor but overflows i16 at 400 bar,
/// which is what decibar is for.
pub fn calibrate(slot: &SensorSlotConfig, raw: Option<u16>) -> i16 {
    let Some(raw) = raw else {
        return SENSOR_INVALID;
    };

    if slot.unit == Unit::RawCounts {
        return saturate(raw as i32);
    }

    match slot.kind {
        SensorKind::None => SENSOR_INVALID,
        SensorKind::Pt1000 => saturate(pt1000_centi_celsius(raw)),
        SensorKind::Pressure => {
            let millibar = slot.calib.to_millibar(raw);
            match slot.unit {
                Unit::DeciBar => saturate(millibar / 100),
                // Centibar is the sane reading of a pressure slot mistakenly set to a
                // temperature unit, rather than refusing to report anything.
                Unit::CentiBar | Unit::CentiCelsius | Unit::RawCounts => saturate(millibar / 10),
            }
        }
    }
}

/// Which of the 18 probe-able slots the incremental scan looks at next.
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

    /// Return the next slot to probe, if the scan interval has elapsed. A zero interval disables
    /// scanning entirely, freezing the presence bitmap at whatever it last held.
    fn due(&mut self, cfg: &Config, now: Instant) -> Option<AdcSlot> {
        if cfg.scan_interval_ms == 0 {
            return None;
        }
        if (now - self.last_probe).as_millis() < cfg.scan_interval_ms as u64 {
            return None;
        }
        self.last_probe = now;
        let slot = AdcSlot::from_index(self.next)?;
        self.next = (self.next + 1) % AdcSlot::COUNT;
        Some(slot)
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
    /// and in TPDO kind 14.
    present: PerI2cBus<u16>,
    raw: PerAdcSlot<u16>,
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
            scan: ScanCursor::new(Instant::now()),
            sweeps: 0,
        }
    }

    fn is_present(&self, slot: AdcSlot) -> bool {
        self.present[slot.bus()] & (1 << slot.amplifier().index()) != 0
    }

    fn set_present(&mut self, slot: AdcSlot, present: bool) {
        let bit = 1u16 << slot.amplifier().index();
        let mask = &mut self.present[slot.bus()];
        let was = *mask & bit != 0;
        if present {
            *mask |= bit;
        } else {
            *mask &= !bit;
        }
        if was != present {
            let address = AMPLIFIER_ADDRESSES[slot.amplifier()];
            if present {
                defmt::info!(
                    "amplifier appeared: bus {} addr {=u8:#04x} (index {})",
                    slot.bus().as_u8(),
                    address,
                    slot.amplifier().as_u8()
                );
            } else {
                defmt::warn!(
                    "amplifier vanished: bus {} addr {=u8:#04x} (index {})",
                    slot.bus().as_u8(),
                    address,
                    slot.amplifier().as_u8()
                );
            }
        }
    }

    pub async fn run(&mut self) -> ! {
        loop {
            let config = { STORE.lock().await.config.clone() };
            let now = Instant::now();

            self.sample(&config).await;
            self.scan_step(&config, now).await;
            self.publish(&config).await;

            Timer::after_millis(config.sensor_interval_ms.max(1) as u64).await;
        }
    }

    /// Read every amplifier currently believed present.
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
                        defmt::warn!("amplifier ALERT: bus {} addr {=u8:#04x}", slot.bus().as_u8(), address);
                    }
                }
                None => {
                    // A device that stops answering is gone as far as we are concerned; the scan
                    // will find it again if it comes back. This is what makes a cable knocked
                    // loose during assembly visible instead of silently freezing a reading.
                    self.raw[slot] = RAW_INVALID;
                    self.set_present(slot, false);
                }
            }
        }
    }

    /// Probe at most one address, so scanning never costs more than one NACK per tick.
    async fn scan_step(&mut self, config: &Config, now: Instant) {
        let Some(slot) = self.scan.due(config, now) else {
            return;
        };
        if self.is_present(slot) {
            // Already sampling it; no need to spend a transfer confirming that.
            if self.scan.wrapped() {
                self.sweeps = self.sweeps.wrapping_add(1);
            }
            return;
        }

        let address = AMPLIFIER_ADDRESSES[slot.amplifier()];
        if self.buses.probe(slot.bus(), address).await {
            self.set_present(slot, true);
        }
        if self.scan.wrapped() {
            self.sweeps = self.sweeps.wrapping_add(1);
        }
    }

    async fn publish(&mut self, config: &Config) {
        let mut values = PerSensorSlot::splat(SENSOR_INVALID);
        for (id, slot) in config.sensors.iter() {
            values[id] = match slot.adc_slot() {
                Some(adc) if self.raw[adc] != RAW_INVALID => calibrate(slot, Some(self.raw[adc])),
                _ => SENSOR_INVALID,
            };
        }

        let mut store = STORE.lock().await;
        store.raw_adc = self.raw;
        store.i2c_present = self.present;
        store.i2c_sweeps = self.sweeps;
        store.sensor_value = values;
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
    use crate::config::{NUM_ADC_SLOTS, NUM_AMPLIFIERS, PressureCalib};
    use crate::index::{AmplifierId, I2cBus};

    /// Answers with a fixed 2-byte conversion register for addresses it's been told to have a
    /// device at, and NACKs everything else — the ADC101C027's whole read-only interface.
    struct MockI2c {
        responses: HashMap<u8, [u8; 2]>,
    }

    #[derive(Debug)]
    struct Nack;
    impl I2cError for Nack {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
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
        sensors.set_present(AdcSlot::new(I2cBus::Bus0, AmplifierId::Amp0), true);

        block_on(sensors.sample(&Config::new()));

        assert_eq!(sensors.raw[AdcSlot::new(I2cBus::Bus0, AmplifierId::Amp0)], 512);
    }

    #[test]
    fn sample_reads_a_present_amplifier_on_bus1() {
        let mut bus1 = MockI2c::new();
        bus1.respond(AMPLIFIER_ADDRESSES[AmplifierId::Amp2], 300, false);
        let mut sensors = sensors_with(MockI2c::new(), bus1);
        let slot = AdcSlot::new(I2cBus::Bus1, AmplifierId::Amp2);
        sensors.set_present(slot, true);

        block_on(sensors.sample(&Config::new()));

        assert_eq!(sensors.raw[slot], 300, "bus 1 reads must land in the second half of the raw array");
    }

    #[test]
    fn a_nacked_read_marks_the_slot_absent() {
        // Present but nobody answers this tick: a cable knocked loose during assembly.
        let mut sensors = sensors_with(MockI2c::new(), MockI2c::new());
        sensors.set_present(AdcSlot::new(I2cBus::Bus0, AmplifierId::Amp0), true);

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
        sensors.set_present(AdcSlot::new(I2cBus::Bus0, AmplifierId::Amp0), true);

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

    #[test]
    fn pt1000_midscale_is_zero_celsius() {
        // 512 counts puts the amplifier output exactly at its 1.65 V offset, so the bridge is
        // balanced and the RTD is at its nominal 1000 ohm.
        assert_eq!(pt1000_centi_celsius(512), 0);
    }

    #[test]
    fn pt1000_matches_the_float_derivation() {
        // Worked through the float chain by hand: 600 counts -> 8.489 degrees C.
        assert_eq!(pt1000_centi_celsius(600), 848);
    }

    #[test]
    fn pt1000_is_monotonic_across_the_range() {
        let mut previous = i32::MIN;
        for raw in 0..=1023u16 {
            let t = pt1000_centi_celsius(raw);
            assert!(t > previous, "not monotonic at raw={raw}");
            previous = t;
        }
    }

    /// The calibration the bench data is defined against:
    /// `pressure_bar = (adc_reading - offset) * linear_factor`.
    #[test]
    fn the_plain_linear_form_has_no_constant_term() {
        let calib = PressureCalib::from_bar_per_count(100.0, 0.1);
        let slot = SensorSlotConfig::pressure(I2cBus::Bus0, AmplifierId::Amp0, Unit::CentiBar, calib);

        // At the offset the sensor reads exactly zero, not ambient.
        assert_eq!(calibrate(&slot, Some(100)), 0);
        // (200 - 100) * 0.1 bar = 10 bar = 1000 centibar.
        assert_eq!(calibrate(&slot, Some(200)), 1000);
    }

    #[test]
    fn a_constant_term_shifts_the_whole_curve() {
        let calib = PressureCalib::from_bar_per_count(100.0, 0.1).with_constant_bar(1.013);
        let slot = SensorSlotConfig::pressure(I2cBus::Bus0, AmplifierId::Amp0, Unit::CentiBar, calib);

        assert_eq!(calibrate(&slot, Some(100)), 101, "1.013 bar, in centibar");
        assert_eq!(calibrate(&slot, Some(200)), 1101, "and the slope is unchanged");
    }

    #[test]
    fn a_400_bar_sensor_needs_decibar_to_avoid_clipping() {
        // 0.911 bar per count with no offset: full scale is well past what centibar can hold.
        let calib = PressureCalib::from_bar_per_count(0.0, 0.911_161_7);
        let centibar = SensorSlotConfig::pressure(I2cBus::Bus0, AmplifierId::Amp0, Unit::CentiBar, calib);
        let decibar = SensorSlotConfig::pressure(I2cBus::Bus0, AmplifierId::Amp0, Unit::DeciBar, calib);

        // 1023 counts * 0.9111617 bar = 932.12 bar, i.e. 93212 centibar (which does not fit) or
        // 9321 decibar (which does).
        assert_eq!(calibrate(&centibar, Some(1023)), i16::MAX, "centibar saturates");
        assert_eq!(calibrate(&decibar, Some(1023)), 9321, "decibar has the range");
    }

    #[test]
    fn a_40_bar_sensor_keeps_its_resolution_in_centibar() {
        let calib = PressureCalib::from_bar_per_count(15.0, 0.0855);
        let slot = SensorSlotConfig::pressure(I2cBus::Bus0, AmplifierId::Amp0, Unit::CentiBar, calib);
        // (500 - 15) * 0.0855 bar = 41.4675 bar = 4146 centibar.
        assert_eq!(calibrate(&slot, Some(500)), 4146);
    }

    #[test]
    fn a_negative_offset_is_handled() {
        // D_40BAR has offset -385, so every reading sits above the zero point.
        let calib = PressureCalib::from_bar_per_count(-385.0, 0.0535);
        let slot = SensorSlotConfig::pressure(I2cBus::Bus0, AmplifierId::Amp0, Unit::CentiBar, calib);
        // (0 - -385) * 0.0535 bar = 20.5975 bar.
        assert_eq!(calibrate(&slot, Some(0)), 2059);
    }

    #[test]
    fn a_reading_below_the_offset_goes_negative() {
        // Gauge pressure below the calibration zero is a real reading, not an error, so it must
        // survive as a negative number rather than wrapping.
        let calib = PressureCalib::from_bar_per_count(500.0, 0.1);
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
            PressureCalib::from_bar_per_count(9.0, 9.0),
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
    fn the_scan_visits_every_slot_before_repeating() {
        let mut cursor = ScanCursor::new(Instant::from_millis(0));
        let cfg = Config::new();
        let mut seen = [false; NUM_ADC_SLOTS];

        for step in 1..=NUM_ADC_SLOTS as u64 {
            let now = Instant::from_millis(step * cfg.scan_interval_ms as u64);
            let slot = cursor.due(&cfg, now).expect("a probe is due");
            assert!(!seen[slot.index()], "slot {slot:?} probed twice in one sweep");
            seen[slot.index()] = true;
        }
        assert!(seen.iter().all(|s| *s));
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
