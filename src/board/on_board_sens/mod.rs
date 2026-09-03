use crate::board::pins_rev3::{HC_SENSE, HC2_SENSE, I_SENSE_1, I_SENSE_2, I_SENSE_3, TH_SENSE, V_MAIN_SENSE};
use embassy_stm32::{
    Peri,
    adc::{Adc, AnyAdcChannel, SampleTime, Temperature},
    peripherals::ADC1,
};

use defmt_rtt as _;

// pub trait OnboardAnalouge {
#[allow(async_fn_in_trait)]
pub trait CurrentSens {
    async fn hco12_current_ma(&mut self) -> u16;
    async fn hco34_current_ma(&mut self) -> u16;
    async fn logic_supply_current_ma(&mut self) -> Option<u16>;
}

#[allow(async_fn_in_trait)]
pub trait VoltageSens {
    async fn logic_supply_voltage_milli_v(&mut self) -> u16;
    async fn hco12_supply_voltage_milli_v(&mut self) -> u16;
    async fn hco34_supply_voltage_milli_v(&mut self) -> u16;
}

pub struct OnboardSensRev3 {
    adc: Adc<'static, ADC1>,
    pins: OnboardSens3Peri,
    /// The die sensor's internal channel. Not a pin, so it lives here rather than in
    /// [`OnboardSens3Peri`] — same as `vref` below, which is consumed during `new`.
    temperature: Temperature,
    sample_time: SampleTime,
    vref_sample: u16,
}

/// The pins behind each rail measurement.
///
/// The two HCO rails are crossed with respect to the schematic label numbering, on both the
/// current and the voltage side, and the field names are what the rest of the firmware trusts.
/// Traced through the rev3 netlist, load side back to the pin:
///
/// | outputs | sheet | shunt | fuse / rail | amp | label | pin |
/// |---------|-------|-------|-------------|-----|-------|-----|
/// | HCO1+2  | `Digital Output`  | `R33` (`SH21`-`SH22`) | `F3` from `high_current2` | `U8` | `I_sense_2` | `PA1` |
/// | HCO3+4  | `Digital Output1` | `R32` (`SH11`-`SH12`) | `F2` from `high_current`  | `U7` | `I_sense_1` | `PA0` |
///
/// The sheet-to-output link is the part worth restating, since it is what the numbering suggests
/// and the board contradicts: the *Digital Output* sheet takes `HC_OUT_1`/`HC_OUT_2` on its D1/D2
/// pins and is supplied from `SH22`, while *Digital Output1* takes `HC_OUT_3`/`HC_OUT_4` and is
/// supplied from `SH12`. The voltage dividers follow their own rails (`R43` on `high_current2` to
/// `hc2_sense`, `R42` on `high_current` to `hc_sense`), so they are crossed the same way.
///
/// Only the logic rail lands where the labels imply: `I_sense_3`/`PC0` and `vmain_sense`/`PC1`
/// both sit on `R2`, in front of the 5 V regulator.
pub struct OnboardSens3Peri {
    pub i_sens_hco12: Peri<'static, I_SENSE_2>,
    pub i_sens_hco34: Peri<'static, I_SENSE_1>,
    pub i_sens_supply_current: Option<Peri<'static, I_SENSE_3>>,
    pub v_logic_supply: Peri<'static, V_MAIN_SENSE>,
    pub v_hco12_supply: Peri<'static, HC2_SENSE>,
    pub v_hco34_supply: Peri<'static, HC_SENSE>,
    pub v_temp: Peri<'static, TH_SENSE>,
    /// The analog input a heating pad's NTC divider is on, chosen by the node settings
    /// ([`crate::heater::HeaterConfig::ntc`]). `None` on a node without a heater.
    pub heater_ntc: Option<AnyAdcChannel<'static, ADC1>>,
}

impl OnboardSensRev3 {
    /// Sample time for the internal reference, independent of the caller's `sample_time`.
    ///
    /// VREFINT is not a pin: it is driven through a high internal impedance, and the F105
    /// datasheet gives it a minimum sampling time of 17.1 us (the same figure as the temperature
    /// sensor). The ADC runs at PCLK2/6 = 12 MHz, so 239.5 cycles is 19.96 us — the only sample
    /// time on this part that clears the requirement. Anything shorter leaves the sample capacitor
    /// short of the reference and reads low, and since [`Self::reading_to_mv`] divides *by* this
    /// number, that error scales every voltage and current the board reports.
    const VREF_SAMPLE_TIME: SampleTime = SampleTime::CYCLES239_5;

    /// Sample time for both temperature channels, for two separate reasons that land on the same
    /// answer.
    ///
    /// The die sensor has the same 17.1 us datasheet minimum as VREFINT, so it needs 239.5 cycles
    /// at the 12 MHz ADC clock for exactly the reason [`Self::VREF_SAMPLE_TIME`] does. The
    /// thermistor is a pin, but a slow one: TH1 in parallel with R44 is ~3.4k at room temperature
    /// and climbs past 5k as the board cools, which is at or beyond what the datasheet allows a
    /// 7.5-cycle sample to settle from. Neither reading is in a hurry — the whole point of the
    /// channel is a value that moves over seconds — so both take the longest sample the part has
    /// rather than trading accuracy for 40 us.
    const TEMPERATURE_SAMPLE_TIME: SampleTime = SampleTime::CYCLES239_5;

    pub async fn new(adc: Peri<'static, ADC1>, pins: OnboardSens3Peri, sample_time: SampleTime) -> Self {
        let mut adc = Adc::new(adc);
        let mut vref = adc.enable_vref();
        // Same TSVREFE bit as `enable_vref`, so this costs nothing beyond the handle; enabled
        // here so the one t_START wait below covers both internal channels.
        let temperature = adc.enable_temperature();

        // t_START for the internal reference; the datasheet allows up to 10 us.
        embassy_time::Timer::after_micros(20).await;

        let vref_sample = adc.read(&mut vref, Self::VREF_SAMPLE_TIME).await;
        // guard against division by 0
        let vref_sample = vref_sample.max(1);

        Self {
            adc,
            pins,
            temperature,
            sample_time,
            vref_sample,
        }
    }
    // TODO: are millivolts good enough?
    fn reading_to_mv(&self, raw: u16) -> u16 {
        const VREFINT_MV: u32 = 1200;
        ((raw as u32 * VREFINT_MV) / (self.vref_sample as u32)) as u16
    }
    /// Same conversion as [`Self::reading_to_mv`], with the resolution the die sensor needs: its
    /// slope is 4.3 mV per degree, so a millivolt of quantisation would be a quarter of a degree.
    ///
    /// `u64` for the multiply rather than `u32`: full scale times 1_200_000 is 4.9e9, which
    /// overflows a `u32` even though the die sensor itself never reads anywhere near that high.
    fn reading_to_uv(&self, raw: u16) -> u32 {
        const VREFINT_UV: u64 = 1_200_000;
        ((raw as u64 * VREFINT_UV) / (self.vref_sample as u64)) as u32
    }
}
impl CurrentSens for OnboardSensRev3 {
    async fn hco12_current_ma(&mut self) -> u16 {
        let reading = self.adc.read(&mut self.pins.i_sens_hco12, self.sample_time).await;
        reading_v_to_current_ma(self.reading_to_mv(reading))
    }
    async fn hco34_current_ma(&mut self) -> u16 {
        let reading = self.adc.read(&mut self.pins.i_sens_hco34, self.sample_time).await;
        reading_v_to_current_ma(self.reading_to_mv(reading))
    }
    async fn logic_supply_current_ma(&mut self) -> Option<u16> {
        if let Some(ref mut i_sens_supply_current) = self.pins.i_sens_supply_current {
            let reading = self.adc.read(i_sens_supply_current, self.sample_time).await;

            return Some(reading_v_to_current_ma(self.reading_to_mv(reading)));
        }
        None
    }
}
impl VoltageSens for OnboardSensRev3 {
    async fn logic_supply_voltage_milli_v(&mut self) -> u16 {
        let reading = self.adc.read(&mut self.pins.v_logic_supply, self.sample_time).await;
        reading_v_to_system_v(self.reading_to_mv(reading))
    }
    async fn hco12_supply_voltage_milli_v(&mut self) -> u16 {
        let reading = self.adc.read(&mut self.pins.v_hco12_supply, self.sample_time).await;
        reading_v_to_system_v(self.reading_to_mv(reading))
    }
    async fn hco34_supply_voltage_milli_v(&mut self) -> u16 {
        let reading = self.adc.read(&mut self.pins.v_hco34_supply, self.sample_time).await;
        reading_v_to_system_v(self.reading_to_mv(reading))
    }
}
impl crate::heater::HeaterSensing for OnboardSensRev3 {
    async fn heater_ntc_counts(&mut self) -> Option<u16> {
        // The divider is ~5k source impedance, more than the fast rail sample time allows for,
        // so this channel gets the longest one. Four samples average out ADC noise.
        const SAMPLES: u32 = 4;
        let pin = self.pins.heater_ntc.as_mut()?;
        let mut sum = 0u32;
        for _ in 0..SAMPLES {
            sum += self.adc.read(pin, SampleTime::CYCLES239_5).await as u32;
        }
        Some((sum / SAMPLES) as u16)
    }
}

impl crate::rail_sense::RailSensing for OnboardSensRev3 {
    async fn read(&mut self) -> Option<crate::rail_sense::Rails> {
        // Both arrays are in `RailId` order: Logic, Hco12, Hco34.
        Some(crate::rail_sense::Rails {
            current_ma: crate::index::PerRail::new([
                self.logic_supply_current_ma().await.unwrap_or(0),
                self.hco12_current_ma().await,
                self.hco34_current_ma().await,
            ]),
            voltage_mv: crate::index::PerRail::new([
                self.logic_supply_voltage_milli_v().await,
                self.hco12_supply_voltage_milli_v().await,
                self.hco34_supply_voltage_milli_v().await,
            ]),
        })
    }
}

impl crate::temp_sense::TemperatureSensing for OnboardSensRev3 {
    async fn read_temperatures(&mut self) -> crate::temp_sense::Temperatures {
        use crate::temp_sense::{Temperatures, mcu_sense_uv_to_milli_c, ntc_counts_to_milli_c};

        // The thermistor is read as raw counts, not millivolts: the divider hangs off +3.3V and
        // the ADC reference is +3.3VA, the filtered version of the same rail, so counts already
        // carry the ratio the beta equation wants and VDDA cancels. Going through
        // `reading_to_mv` would put VREFINT's own error into a reading that does not need it.
        // See `NTC_CURVE`.
        let board_raw = self.adc.read(&mut self.pins.v_temp, Self::TEMPERATURE_SAMPLE_TIME).await;

        // The die sensor is the opposite case: its output is an absolute voltage compared against
        // a fixed datasheet reference, so it does need VREFINT to escape VDDA.
        let mcu_raw = self.adc.read(&mut self.temperature, Self::TEMPERATURE_SAMPLE_TIME).await;

        let milli_c = crate::index::PerTemp::new([
            ntc_counts_to_milli_c(board_raw),
            mcu_sense_uv_to_milli_c(self.reading_to_uv(mcu_raw)),
        ]);

        // Once a second, and the fastest way to tell a frozen conversion from a frozen
        // calculation when a board is on a bench with a probe attached.
        defmt::debug!(
            "temp: ntc {} counts -> {} mC, die {} counts -> {} mC (vref {})",
            board_raw,
            milli_c[crate::index::TempSensorId::Board],
            mcu_raw,
            milli_c[crate::index::TempSensorId::Mcu],
            self.vref_sample,
        );

        // In `TempSensorId` order: Board, Mcu.
        Temperatures {
            milli_c,
            raw: crate::index::PerTemp::new([board_raw, mcu_raw]),
        }
    }
}

/// Convert voltage read by adc to actual voltage on the target circuit.
/// This is just because we use a voltage divider.
fn reading_v_to_system_v(v_mv: u16) -> u16 {
    // io board rev3 has a 15k / 2.2k voltage divider
    // v_actual = v_meas * (15 + 2.2) / 2.2 = v_meas * 7.8181
    ((v_mv as u32 * 86) / 11) as u16
}

/// Convert voltage read by adc to current on the target circuit.
/// By knowing the shunt resistance and the amplification gain.
///
/// These are the *nominal* part values, and the board reports almost exactly half the real
/// current. The cause is almost certainly hardware, not this function — see below — so the math
/// here stays nominal rather than absorbing a fudge factor that a board rework would invalidate.
///
/// The chain is short enough to enumerate completely: a single 15 mR shunt (`R2`/`R32`/`R33`, one
/// per instance of `io_board_current_sensing.kicad_sch`), then `R69`/`R70` — **1k in series with
/// the INA181's IN+ and IN-**, with `C36`/`C37` 10n to ground as an input filter — then `U7`
/// INA181A1 at 20 V/V, then straight to the MCU pin. There is no divider on the amplifier output:
/// it goes to the `I_sense_N` global label and nowhere else.
///
/// That leaves the input filter as the only gain-error element in the path. Series resistance at
/// a current-shunt monitor's inputs divides against the amplifier's internal input resistance,
/// which is why TI's guidance for this family is to keep it near zero; the observed factor of 2
/// implies an internal input resistance of ~1k, the right order for the part.
///
/// One probe settles it: drive a known current and measure `U7` pin 5 (OUT) directly.
/// - ~0.30 V/A -> the amplifier is fine and the error is downstream of it (ADC reference or
///   sample time), in which case fix it here.
/// - ~0.15 V/A -> the input filter, as above. The fix is a rework: `R69`/`R70` to 0R links (or
///   <=10R), keeping `C36`/`C37` for the filter.
///
/// Until that measurement exists, `stall_ma` is calibrated against what the board reports rather
/// than against amps, which is the reason it is runtime-configurable in the first place.
fn reading_v_to_current_ma(v_mv: u16) -> u16 {
    const AMP_GAIN: u32 = 20;
    const RESISTOR_VALUE_MOHM: u32 = 15;
    (v_mv as u32 * 1000 / (RESISTOR_VALUE_MOHM * AMP_GAIN)) as u16
}
