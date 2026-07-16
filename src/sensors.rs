use defmt::{Debug2Format, error, info, warn};
/// For reading values from the external adc. "Amplifier boards"
use embassy_stm32::{
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_time::{Duration, Ticker};

use crate::ext_adc::{AMPLIFIER_ADDRESSES, ExtAdcs, NUM_ADCS, SensorSettings};
use crate::store::STORE;

pub struct PressureSensorCalib {
    pub gain: f32,
    pub offset: f32,
}
impl PressureSensorCalib {
    pub fn apply(&self, diff: f32) -> f32 {
        self.gain * diff + self.offset
    }
}

const SENSOR_MAPPING: SensorMapping = SensorMapping([None, None, None, None, None, None, None, None]);

/// how raw i2c bus values are mapped to sensor pdo message
pub struct SensorMapping(pub [Option<Sensor>; 8]);

pub struct Sensor {
    kind: SensorKind,
    bus_idx: usize,
    sensor_idx: usize,
}

pub enum SensorKind {
    SimplePressure(PressureSensorCalib),
    SimpleTemp(),
}

#[embassy_executor::task]
pub async fn run_sensors(
    mut com1_i2c: Option<&'static mut I2c<'static, Async, Master>>,
    mut com2_i2c: Option<&'static mut I2c<'static, Async, Master>>,
    settings: SensorSettings,
) {
    let mut ticker = Ticker::every(settings.broadcast_interval);
    let enabled: [bool; NUM_ADCS] = [false; NUM_ADCS];
    let mut adcs = ExtAdcs::new(enabled);
    adcs.scan_and_enable(com1_i2c.as_deref_mut(), com2_i2c.as_deref_mut()).await;

    loop {
        let _ = adcs.read_all(com1_i2c.as_deref_mut(), com2_i2c.as_deref_mut()).await;

        let mut store = STORE.lock().await;

        for (idx, mapping) in SENSOR_MAPPING.0.iter().enumerate() {
            let Some(mapping) = mapping else {
                continue;
            };
            match &mapping.kind {
                SensorKind::SimplePressure(calib) => {
                    let raw = adcs.measurements.0[mapping.bus_idx * AMPLIFIER_ADDRESSES.len() + mapping.sensor_idx]
                        .map(|r| r.value);
                    let Some(raw) = raw else {
                        warn!("amplifier mapped to sensor could not be read");
                        continue;
                    };
                    let pressure = calib.apply(raw as f32);
                    // TODO: check unit conversion
                    let pressure_kilo_pc = pressure as u16;
                    store.selected_sensors[idx] = pressure_kilo_pc;
                }
                // TODO: figure out temperature sensor calibs
                SensorKind::SimpleTemp() => todo!(),
            }
        }

        // FIXME:
        // com1_i2c float, float | com2_i2c float, float
        // let reading0: Option<[u8; 2]> = adcs.measurements.0[0].map(|meas| meas.value.to_le_bytes());
        // let reading1: Option<[u8; 2]> = adcs.measurements.0[3].map(|meas| meas.value.to_le_bytes());
        // if let Some(reading) = reading0 {
        //     can_pub.publish_immediate((CAN_ID0, Vec::from_slice(&reading).unwrap()));
        // }
        // if let Some(reading) = reading1 {
        //     // can_pub.publish_immediate((CAN_ID1, Vec::from_slice(&reading).unwrap()));
        // }

        ticker.next().await;
    }
}
