//! Board temperature sensing, abstracted so `crate::control::Control` can be built and tested
//! against mocked hardware — the same split as [`crate::rail_sense`], and for the same reason.
//!
//! Deliberately a second trait rather than another method on [`RailSensing`]: the two happen to
//! be served by the same ADC on rev3, but "what is this rail doing" and "how hot is this board"
//! are not the same question, and a future revision could answer one without the other.
//!
//! Both readings end up in `store.temperature_milli_c` (0x2042) and on the bus as
//! [`iocan_proto::TpdoFrame::Temperature`].

use crate::index::PerTemp;
use crate::rail_sense::NoRails;
use crate::store::TEMPERATURE_INVALID;

#[allow(async_fn_in_trait)]
pub trait TemperatureSensing {
    /// This tick's board and MCU temperatures in millidegrees Celsius, indexed by
    /// [`crate::index::TempSensorId`].
    ///
    /// Per-entry rather than one `Option` for the pair, because the two sensors fail
    /// independently: the MCU die sensor cannot go missing, while the board thermistor can read
    /// open or shorted. An entry with no usable reading is [`TEMPERATURE_INVALID`].
    async fn read_temperatures(&mut self) -> PerTemp<i32>;
}

/// rev2 has no on-board sensing at all, so it reports neither temperature. It still publishes the
/// frame — a node that says "I don't know" is easier to tell from a dead one than silence is.
impl TemperatureSensing for NoRails {
    async fn read_temperatures(&mut self) -> PerTemp<i32> {
        PerTemp::splat(TEMPERATURE_INVALID)
    }
}

// ---------------------------------------------------------------------------
// Conversions
// ---------------------------------------------------------------------------
//
// Both live here, not next to the ADC driver, because they are arithmetic over plain integers
// and this module is not gated behind the `hardware` feature — so they are covered by the host
// test suite rather than only by a board on a bench.

/// TH1's curve, sampled every [`NTC_CURVE_STEP_C`] from [`NTC_CURVE_MIN_C`]: the 12-bit ADC
/// reading the divider produces at each temperature, which falls monotonically as the NTC heats.
///
/// Generated from the beta equation for the fitted part — Murata `NCP18XH103F03RB`, R25 = 10k,
/// B25/50 = 3380 K — against the schematic's 5k1 upper leg (R44) and a full scale of 4095:
///
/// ```text
///   R_ntc(T) = 10k * exp(3380 * (1/T - 1/298.15))
///   counts(T) = 4095 * R_ntc / (5100 + R_ntc)
/// ```
///
/// A single beta fits the real R-T table closely around room temperature and drifts from it
/// towards the ends of the range (the part is really specified with a different beta per span:
/// 3428 for 25/80, 3455 for 25/100). That is well inside what this reading is for — telling a
/// warm bay from an overheating one — and the alternative is transcribing a vendor table nobody
/// can check against the board.
///
/// The reading is ratiometric, so it does not depend on VDDA being exactly 3.3 V: the divider is
/// fed from +3.3V, which is the same net as the ADC reference, and both sides of
/// `counts/4095 = R_ntc/(R44 + R_ntc)` scale together. That is why this path uses raw counts
/// while [`mcu_sense_uv_to_milli_c`] has to go through VREFINT for an absolute voltage.
const NTC_CURVE: [u16; 34] = [
    4008, //  -40 C
    3978, //  -35 C
    3940, //  -30 C
    3893, //  -25 C
    3834, //  -20 C
    3764, //  -15 C
    3680, //  -10 C
    3581, //   -5 C
    3468, //    0 C
    3341, //    5 C
    3200, //   10 C
    3047, //   15 C
    2883, //   20 C
    2712, //   25 C
    2536, //   30 C
    2358, //   35 C
    2181, //   40 C
    2007, //   45 C
    1840, //   50 C
    1680, //   55 C
    1529, //   60 C
    1388, //   65 C
    1258, //   70 C
    1138, //   75 C
    1029, //   80 C
    929,  //   85 C
    839,  //   90 C
    758,  //   95 C
    685,  //  100 C
    619,  //  105 C
    560,  //  110 C
    508,  //  115 C
    460,  //  120 C
    418,  //  125 C
];

/// Temperature of `NTC_CURVE[0]`, in degrees Celsius.
const NTC_CURVE_MIN_C: i32 = -40;
/// Spacing between adjacent `NTC_CURVE` entries, in degrees Celsius.
const NTC_CURVE_STEP_C: i32 = 5;

/// Convert a raw 12-bit reading of the `TH_sense` divider to millidegrees Celsius.
///
/// [`TEMPERATURE_INVALID`] for anything off the ends of [`NTC_CURVE`], which is also what an
/// unpopulated or failed thermistor looks like: open circuit pulls the pin to +3.3V (counts above
/// the cold end), a short pulls it to ground (counts below the hot end). Reporting those as
/// -40 C or 125 C would be worse than reporting nothing, because a master cannot tell a real
/// reading at the rail from a broken sensor.
pub fn ntc_counts_to_milli_c(counts: u16) -> i32 {
    // Descending curve, so the bracket is `curve[i] >= counts >= curve[i + 1]`.
    if counts > NTC_CURVE[0] || counts < NTC_CURVE[NTC_CURVE.len() - 1] {
        return TEMPERATURE_INVALID;
    }

    let mut i = 0;
    while NTC_CURVE[i + 1] > counts {
        i += 1;
    }

    let (high, low) = (NTC_CURVE[i] as i32, NTC_CURVE[i + 1] as i32);
    let base_milli_c = (NTC_CURVE_MIN_C + i as i32 * NTC_CURVE_STEP_C) * 1000;
    // `high > low` for every adjacent pair in the curve, so this never divides by zero.
    base_milli_c + (NTC_CURVE_STEP_C * 1000 * (high - counts as i32)) / (high - low)
}

/// The die sensor's output at 25 C, in microvolts. STM32F1 datasheet typical; the spread is
/// 1.34..1.52 V, which is +-20 C of uncorrected offset on its own.
const MCU_V25_UV: i64 = 1_430_000;
/// The die sensor's slope, in microvolts per degree Celsius. Negative-going: V_sense falls as the
/// die heats, which is why the numerator below is `V25 - V_sense` rather than the other way round.
const MCU_AVG_SLOPE_UV_PER_C: i64 = 4_300;

/// Convert the STM32's internal temperature sensor output to millidegrees Celsius, per the
/// datasheet's `T = (V25 - V_sense) / Avg_Slope + 25`.
///
/// Uncalibrated, and this part has no factory calibration values to apply: the datasheet's own
/// figure for absolute accuracy is tens of degrees, essentially all of it a fixed offset from the
/// V25 spread. Read it as a trend and a differential against
/// [`TempSensorId::Board`](crate::index::TempSensorId::Board), not as a thermometer. It is still
/// the only thing on the board that can see the die rather than the copper next to it.
pub fn mcu_sense_uv_to_milli_c(v_sense_uv: u32) -> i32 {
    (25_000 + ((MCU_V25_UV - v_sense_uv as i64) * 1000) / MCU_AVG_SLOPE_UV_PER_C) as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The divider is sized so that 25 C sits near mid-scale; if this moves, the curve was
    /// regenerated against the wrong part or the wrong upper leg.
    #[test]
    fn the_curve_reads_room_temperature_at_its_anchor_point() {
        assert_eq!(ntc_counts_to_milli_c(2712), 25_000);
    }

    #[test]
    fn the_curve_descends_so_the_bracket_search_terminates() {
        for pair in NTC_CURVE.windows(2) {
            assert!(pair[0] > pair[1], "NTC_CURVE must fall monotonically: {pair:?}");
        }
    }

    #[test]
    fn every_curve_point_maps_back_to_its_own_temperature() {
        for (i, &counts) in NTC_CURVE.iter().enumerate() {
            let expected = (NTC_CURVE_MIN_C + i as i32 * NTC_CURVE_STEP_C) * 1000;
            assert_eq!(ntc_counts_to_milli_c(counts), expected, "curve point {i}");
        }
    }

    /// A count between two curve points has to land between their temperatures, monotonically —
    /// this is the whole job of the interpolation.
    #[test]
    fn interpolation_is_monotonic_across_the_whole_curve() {
        let mut previous = TEMPERATURE_INVALID;
        for counts in (NTC_CURVE[NTC_CURVE.len() - 1]..=NTC_CURVE[0]).rev() {
            let milli_c = ntc_counts_to_milli_c(counts);
            assert!(milli_c > previous, "{counts} counts went backwards to {milli_c}");
            previous = milli_c;
        }
    }

    /// Open thermistor pulls `TH_sense` to the rail, a short pulls it to ground. Neither may be
    /// reported as a temperature.
    #[test]
    fn an_open_or_shorted_thermistor_reads_invalid() {
        assert_eq!(ntc_counts_to_milli_c(4095), TEMPERATURE_INVALID);
        assert_eq!(ntc_counts_to_milli_c(0), TEMPERATURE_INVALID);
    }

    #[test]
    fn the_die_sensor_reads_25_c_at_its_datasheet_reference() {
        assert_eq!(mcu_sense_uv_to_milli_c(1_430_000), 25_000);
    }

    /// The slope is negative-going: less voltage means a hotter die.
    #[test]
    fn a_lower_die_voltage_is_a_higher_temperature() {
        assert_eq!(mcu_sense_uv_to_milli_c(1_430_000 - 43_000), 35_000);
        assert_eq!(mcu_sense_uv_to_milli_c(1_430_000 + 43_000), 15_000);
    }

    #[test]
    fn nothing_the_adc_can_produce_overflows_the_die_conversion() {
        // 3.3 V through VREFINT scaling is the widest the reading can be.
        for uv in [0, 1, 1_200_000, 3_300_000] {
            let _ = mcu_sense_uv_to_milli_c(uv);
        }
    }
}
