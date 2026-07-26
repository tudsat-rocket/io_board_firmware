use defmt::{Debug2Format, error, info, warn};
/// For reading values from the external adc. "Amplifier boards"
use embassy_stm32::{
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_time::{Duration, with_timeout};
use embedded_hal::i2c::Error;

// bus timout if there is a hardware fault or unexpected fault
const I2C_TIMEOUT: Duration = Duration::from_millis(100);

// ADC101C027 addresses in order: ADR floating, GND, VCC
// const AMPLIFIER_ADDRESSES_OLD: [u8; 3] = [0b1010000, 0b1010001, 0b1010010];
pub const AMPLIFIER_ADDRESSES: [u8; 9] = [
    0b1010000, // floating, floating
    0b1010001, // floating, gnd
    0b1010010, // floating, vcc
    0b1010100, // gnd, floating
    0b1010101, // gnd, gnd
    0b1010110, // gnd, vcc
    0b1011000, // vcc, floating
    0b1011001, // vcc, gnd
    0b1011010, // vcc, vcc
];

pub const NUM_I2C_BUSES: usize = 2;

pub const NUM_ADCS: usize = AMPLIFIER_ADDRESSES.len() * NUM_I2C_BUSES;

#[derive(Copy, Clone, Debug)]
pub struct ExtAdcReading {
    pub value: u16,
    pub alert_flag: bool,
}
pub struct AdcMeasurements(pub [Option<ExtAdcReading>; NUM_ADCS]);
impl AdcMeasurements {
    pub const fn default() -> Self {
        AdcMeasurements([None; NUM_ADCS])
    }
}

pub struct SensorSettings {
    pub measure_interval: Duration,
}

pub struct ExtAdcs {
    pub enabled: [bool; NUM_ADCS],
    pub measurements: AdcMeasurements,
}

impl ExtAdcs {
    pub fn new(enabled: [bool; NUM_ADCS]) -> Self {
        Self {
            enabled,
            measurements: AdcMeasurements::default(),
        }
    }
    pub fn default() -> Self {
        Self::new([false; NUM_ADCS])
    }

    /// scan the i2c bus for devices with known amplifier addresses
    /// and add found devices to enable list
    pub async fn scan_and_enable(
        &mut self,
        com1_i2c: Option<&mut I2c<'static, Async, Master>>,
        com2_i2c: Option<&mut I2c<'static, Async, Master>>,
    ) {
        for (bus_idx, mut i2c) in [com1_i2c, com2_i2c].into_iter().enumerate() {
            for (amp_idx, addr) in AMPLIFIER_ADDRESSES.iter().enumerate() {
                let Some(ref mut i2c) = i2c else {
                    continue;
                };
                let success = read_i2c_adc(i2c, *addr).await.is_ok();
                if success {
                    self.enabled[bus_idx * AMPLIFIER_ADDRESSES.len() + amp_idx] = true;
                    let print_num = bus_idx + 1;
                    info!("discovered external idc on com{}, with addr: {}", print_num, addr);
                }
            }
        }
    }

    pub async fn read_all(
        &mut self,
        com1_i2c: Option<&mut I2c<'static, Async, Master>>,
        com2_i2c: Option<&mut I2c<'static, Async, Master>>,
    ) -> Result<(), ()> {
        let mut success = true;
        for (bus_idx, mut i2c) in [com1_i2c, com2_i2c].into_iter().enumerate() {
            let Some(ref mut i2c) = i2c else {
                error!("bug: tried to use i2c bus that was None, non-fatal, skipping");
                continue;
            };

            for (amp_idx, addr) in AMPLIFIER_ADDRESSES.iter().enumerate() {
                let adc_idx = bus_idx * AMPLIFIER_ADDRESSES.len() + amp_idx;
                if !self.enabled[adc_idx] {
                    continue;
                }
                let res = read_i2c_adc(i2c, *addr).await;
                if let Err(e) = res {
                    warn!("error reading adc value: {:?}", Debug2Format(&e));
                    success = false;
                }
                self.measurements.0[adc_idx] = res.ok();
            }
        }
        match success {
            true => Ok(()),
            false => Err(()),
        }
    }
}
async fn read_i2c_adc<I2C: embedded_hal_async::i2c::I2c>(
    i2c: &mut I2C,
    i2c_address: u8,
) -> Result<ExtAdcReading, embedded_hal_async::i2c::ErrorKind> {
    // https://www.ti.com/lit/ds/symlink/adc101c027.pdf
    let mut buffer: [u8; 2] = [0x00, 0x00];
    // NOTE: this function may never return if there is a power issue or other on the i2c bus
    with_timeout(I2C_TIMEOUT, i2c.read(i2c_address, &mut buffer))
        .await
        .map_err(|_| embedded_hal_async::i2c::ErrorKind::Other)?
        .map_err(|i2c_e| i2c_e.kind())?;

    let conversion_register = u16::from_be_bytes(buffer);
    let alert_flag = (conversion_register >> 15) > 0;
    let conversion_result = (conversion_register >> 2) & 0x3ff;

    Ok(ExtAdcReading {
        value: conversion_result,
        alert_flag,
    })
}
