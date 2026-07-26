use defmt::warn;
/// For reading values from the external adc. "Amplifier boards"
use embassy_stm32::{
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_time::Ticker;

use crate::ext_adc::{AMPLIFIER_ADDRESSES, ExtAdcs, NUM_ADCS, SensorSettings};
use crate::store::STORE;

pub struct PressureSensorCalib {
    pub offset: f32,
    pub linear_factor: f32,
}
impl PressureSensorCalib {
    pub fn apply(&self, raw: f32) -> f32 {
        (raw - self.offset) * self.linear_factor + 1.013
    }
}

pub struct TempSensorCalib {
    pub gain: f32,
    pub offset: f32,
}
impl TempSensorCalib {
    pub fn apply(&self, diff: f32) -> f32 {
        self.gain * diff + self.offset
    }
}

/// how raw i2c bus values are mapped to sensor pdo message
pub struct SensorMapping(pub [Option<Sensor>; 8]);

impl SensorMapping {
    pub const fn new_empty() -> Self {
        Self([const { None }; 8])
    }
    pub const fn add_consecutive(mut self, kind: SensorKind, bus_idx: usize, adc_idx: usize) -> Option<Self> {
        let mut first_empty = None;
        let mut i = 0;
        // for loops are not const compatiple here
        while i < self.0.len() {
            if matches!(self.0[i], None) {
                first_empty = Some(i);
                break;
            }
            i += 1;
        }

        let Some(first_empty) = first_empty else {
            return None;
        };

        self.0[first_empty] = Some(Sensor { kind, bus_idx, adc_idx });
        Some(self)
    }
}

pub struct Sensor {
    pub kind: SensorKind,
    /// 0 or 1
    pub bus_idx: usize,
    /// 0..9
    pub adc_idx: usize,
}

pub enum SensorKind {
    SimplePressure(PressureSensorCalib),
    SimpleTemp(TempSensorCalib),
}

#[embassy_executor::task]
pub async fn run_sensors(
    mut com1_i2c: Option<&'static mut I2c<'static, Async, Master>>,
    mut com2_i2c: Option<&'static mut I2c<'static, Async, Master>>,
    settings: SensorSettings,
    mapping: SensorMapping,
) -> ! {
    let mut ticker = Ticker::every(settings.measure_interval);
    let enabled: [bool; NUM_ADCS] = [false; NUM_ADCS];
    let mut adcs = ExtAdcs::new(enabled);
    adcs.scan_and_enable(com1_i2c.as_deref_mut(), com2_i2c.as_deref_mut()).await;

    loop {
        let _ = adcs.read_all(com1_i2c.as_deref_mut(), com2_i2c.as_deref_mut()).await;
        {
            let mut store = STORE.lock().await;

            for (bus_idx, chunk) in adcs.measurements.0.chunks_exact(AMPLIFIER_ADDRESSES.len()).enumerate() {
                let bus_store = match bus_idx {
                    0 => &mut store.raw_ext_adc_bus0,
                    1 => &mut store.raw_ext_adc_bus1,
                    _ => {
                        defmt::warn!("unexpected ADC bus index {}", bus_idx);
                        continue;
                    }
                };
                for (slot, reading) in chunk.iter().enumerate() {
                    if let Some(reading) = reading {
                        bus_store[slot] = reading.value;
                        if reading.alert_flag {
                            defmt::warn!("i2c ALERT on bus {} addr {}", bus_idx, slot);
                        }
                    }
                }
            }
        }

        {
            let mut store = STORE.lock().await;

            for (idx, mapping) in mapping.0.iter().enumerate() {
                let Some(mapping) = mapping else {
                    continue;
                };
                match &mapping.kind {
                    SensorKind::SimplePressure(calib) => {
                        let raw = adcs.measurements.0[mapping.bus_idx * AMPLIFIER_ADDRESSES.len() + mapping.adc_idx]
                            .map(|r| r.value);
                        let Some(raw) = raw else {
                            continue;
                        };

                        let pressure = calib.apply(raw as f32);
                        // TODO: check unit conversion
                        let pressure_kilo_pc = (pressure * 100.0) as u16;
                        store.selected_sensors[idx] = pressure_kilo_pc;
                    }
                    SensorKind::SimpleTemp(calib) => {
                        let raw = adcs.measurements.0[mapping.bus_idx * AMPLIFIER_ADDRESSES.len() + mapping.adc_idx]
                            .map(|r| r.value);
                        let Some(raw) = raw else {
                            warn!("amplifier mapped to sensor could not be read");
                            continue;
                        };
                        let temp = calib.apply(raw as f32);
                        let temp_centi_celsius = temp as i16;
                        // FIXME: check conversion here
                        store.selected_sensors[idx] = temp_centi_celsius as u16;
                    }
                }
            }
        }
        ticker.next().await;
    }
}
