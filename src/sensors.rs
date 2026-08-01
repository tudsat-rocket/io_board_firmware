/// For reading values from the external adc. "Amplifier boards"
use embassy_stm32::{
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_time::Ticker;

use crate::ext_adc::{AMPLIFIER_ADDRESSES, ExtAdcs, NUM_ADCS, NUM_I2C_BUSES, SensorSettings};
use crate::store::STORE;

#[derive(Clone, Debug)]
pub struct PressureSensorCalib {
    pub offset: f32,
    pub linear_factor: f32,
}
impl PressureSensorCalib {
    pub fn apply(&self, raw: f32) -> f32 {
        (raw - self.offset) * self.linear_factor + 1.013
    }
}

/// Convert raw ADC value to temperature in °C
pub const fn pt1000_conversion(raw_adc: u16) -> f32 {
    const ADC_REF: f32 = 3.3;
    const ADC_MAX: f32 = 1024.0;
    const OFFSET: f32 = 1.65;
    const GAIN: f32 = 10.69;
    const BRIDGE_VOLTAGE: f32 = 3.3;
    const BRIDGE_RESISTOR: f32 = 1000.0;
    // ADC counts -> amplifier output voltage
    let v_out = (raw_adc as f32) * ADC_REF / ADC_MAX;

    // Remove amplifier offset and gain
    let v_diff = (v_out - OFFSET) / GAIN;

    // Normalize bridge differential voltage
    let x = v_diff / BRIDGE_VOLTAGE;

    // Wheatstone bridge -> Pt1000 resistance
    let resistance = BRIDGE_RESISTOR * (x + 0.5) / (0.5 - x);

    // Pt1000 approximation:
    // T = (R - 1000) / 3.85
    (resistance - 1000.0) / 3.85
    //defmt::info!("temp: {}C", temp_c);
}

/// how raw i2c bus values are mapped to sensor pdo message
pub struct SensorMapping(pub [Option<Sensor>; 8]);

impl SensorMapping {
    pub const fn new_empty() -> Self {
        Self([const { None }; 8])
    }
    pub const fn add_consecutive(mut self, kind: SensorKind, bus_idx: usize, adc_idx: usize) -> Option<Self> {
        if bus_idx >= NUM_I2C_BUSES {
            return None;
        }
        if adc_idx >= AMPLIFIER_ADDRESSES.len() {
            return None;
        }

        let mut first_empty = None;
        let mut i = 0;
        // for loops are not const compatiple here
        while i < self.0.len() {
            if self.0[i].is_none() {
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
#[derive(Clone, Debug)]
pub struct Sensor {
    pub kind: SensorKind,
    /// 0 or 1
    pub bus_idx: usize,
    /// 0..9
    pub adc_idx: usize,
}

#[derive(Clone, Debug)]
pub enum SensorKind {
    SimplePressure(PressureSensorCalib),
    TempPt1000,
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
                    store.selected_sensors[idx] = u16::MAX;
                    continue;
                };
                match &mapping.kind {
                    SensorKind::SimplePressure(calib) => {
                        let raw = adcs.measurements.0[mapping.bus_idx * AMPLIFIER_ADDRESSES.len() + mapping.adc_idx]
                            .map(|r| r.value);
                        let Some(raw) = raw else {
                            store.selected_sensors[idx] = u16::MAX;
                            continue;
                        };

                        let pressure = calib.apply(raw as f32);
                        let pressure_kilo_pc = (pressure * 100.0) as u16; // kilo ps = centi bar
                        store.selected_sensors[idx] = pressure_kilo_pc;
                    }
                    SensorKind::TempPt1000 => {
                        let raw = adcs.measurements.0[mapping.bus_idx * AMPLIFIER_ADDRESSES.len() + mapping.adc_idx]
                            .map(|r| r.value);
                        let Some(raw) = raw else {
                            // warn!("amplifier mapped to sensor could not be read");
                            store.selected_sensors[idx] = u16::MAX;
                            continue;
                        };
                        let temp = pt1000_conversion(raw);
                        let temp_centi_celsius = (temp * 100.0) as i16;
                        // FIXME: check conversion here
                        store.selected_sensors[idx] = temp_centi_celsius as u16;
                    }
                }
            }
        }
        ticker.next().await;
    }
}
