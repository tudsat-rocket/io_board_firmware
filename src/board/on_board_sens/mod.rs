use crate::board::pins_rev3::{HC_SENSE, HC2_SENSE, I_SENSE_1, I_SENSE_2, I_SENSE_3, TH_SENSE, V_MAIN_SENSE};
use embassy_stm32::{
    Peri,
    adc::{Adc, SampleTime},
    peripherals::ADC1,
};

use {defmt_rtt as _, panic_probe as _};

// pub trait OnboardAnalouge {
#[allow(async_fn_in_trait)]
pub trait CurrentSens {
    async fn hco12_current_ma(&mut self) -> u16;
    async fn hco34_current_ma(&mut self) -> u16;
    async fn logic_supply_current_ma(&mut self) -> Option<u16>;
}

#[allow(async_fn_in_trait)]
pub trait TemperatureSens {
    async fn temperature_milli_c(&mut self) -> i32;
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
    sample_time: SampleTime,
    vref_sample: u16,
}

pub struct OnboardSens3Peri {
    pub i_sens_hco12: Peri<'static, I_SENSE_1>,
    pub i_sens_hco34: Peri<'static, I_SENSE_2>,
    pub i_sens_supply_current: Option<Peri<'static, I_SENSE_3>>,
    pub v_logic_supply: Peri<'static, V_MAIN_SENSE>,
    pub v_hco12_supply: Peri<'static, HC_SENSE>,
    pub v_hco34_supply: Peri<'static, HC2_SENSE>,
    pub v_temp: Peri<'static, TH_SENSE>,
}

impl OnboardSensRev3 {
    pub async fn new(adc: Peri<'static, ADC1>, pins: OnboardSens3Peri, sample_time: SampleTime) -> Self {
        let mut adc = Adc::new(adc);
        let mut vref = adc.enable_vref();

        // NOTE: this is very guessed
        embassy_time::Timer::after_micros(20).await;

        let vref_sample = adc.read(&mut vref, sample_time).await;
        defmt::error!("vref_sample: {}", vref_sample);
        // guard against division by 0
        let vref_sample = vref_sample.max(1);

        Self {
            adc,
            pins,
            sample_time,
            vref_sample,
        }
    }
    // TODO: are millivolts good enough?
    fn reading_to_mv(&self, raw: u16) -> u16 {
        const VREFINT_MV: u32 = 1200;
        ((raw as u32 * VREFINT_MV) / (self.vref_sample as u32)) as u16
    }
    // fn reading_to_uv(&self, raw: u16) -> u32 {
    //     const VREFINT_MV: u32 = 1200;
    //     (raw as u32 * VREFINT_MV * 1000) / (self.vref_sample as u32)
    // }
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
// TODO:
// impl TemperatureSens for OnboardSensRev3 {
//     async fn temperature_milli_c(&mut self) -> i32 {
//         let reading = self.adc.read(&mut self.pins.v_temp, self.sample_time).await;
//         let v_meas_uv = self.reading_to_mv(reading) as u32 * 1000;
//
//         const V_REF_UV: u32 = 3_300_000;
//         const R_UPPER_U_OHM: u32 = 5_100_000;
//         const BETA: f32 = 3380; // 0 - 50 C
//         const T0: f32 = 298.15;
//
//         // thermistor resistance
//         let th_resistance = (R_UPPER_U_OHM * v_meas_uv) / (V_REF_UV - v_meas_uv);
//
//         let t_kelvin = 1.0 / ((1.0/ T0) + (1.0/BETA) * log(th_resistance /
//
//
//     }
// }

/// Convert voltage read by adc to actual voltage on the target circuit.
/// This is just because we use a voltage divider.
fn reading_v_to_system_v(v_mv: u16) -> u16 {
    // io board rev3 has a 15k / 2.2k voltage divider
    // v_actual = v_meas * (15 + 2.2) / 2.2 = v_meas * 7.8181
    ((v_mv as u32 * 86) / 11) as u16
}

/// Convert voltage read by adc to current on the target circuit.
/// By knowing the shunt resistance and the amplification gain.
fn reading_v_to_current_ma(v_mv: u16) -> u16 {
    // FIXME: tests show that we report almost exactly half of the real current
    // where is this factor coming from?
    const AMP_GAIN: u32 = 20;
    const RESISTOR_VALUE_MOHM: u32 = 15;
    (v_mv as u32 * 1000 / (RESISTOR_VALUE_MOHM * AMP_GAIN)) as u16
}
