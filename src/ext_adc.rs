use defmt::{Debug2Format, error, info, warn};
/// For reading values from the external adc. "Amplifier boards"
use embassy_stm32::{
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_sync::pubsub::Subscriber;
use embassy_time::{Duration, Ticker};
use heapless::Vec;

use crate::can::CanTxPub;

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

pub enum AmpBoardAddrSetting {
    Floating,
    GND,
    Vcc,
}

pub const NUM_I2C_BUSES: usize = 2;

pub const NUM_ADCS: usize = AMPLIFIER_ADDRESSES.len() * NUM_I2C_BUSES;

#[derive(Copy, Clone)]
pub struct ExtAdcReading {
    pub value: u16,
    pub alert_flag: bool,
}
pub struct AdcMeasurements(pub [Option<ExtAdcReading>; NUM_ADCS]);
impl AdcMeasurements {
    pub const fn default() -> Self {
        AdcMeasurements([None; NUM_ADCS])
    }
    /// bus ids counted from 0
    pub fn get_measurement_via_addr(&self, bus_id: usize, i2c_address: u8) {
        todo!()
    }
    /// bus ids counted from 0
    pub fn get_measurement(&self, bus_id: usize, addr0: AmpBoardAddrSetting, addr1: AmpBoardAddrSetting) {
        todo!()
    }
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
) -> Result<ExtAdcReading, I2C::Error> {
    // https://www.ti.com/lit/ds/symlink/adc101c027.pdf
    let mut buffer: [u8; 2] = [0x00, 0x00];
    i2c.read(i2c_address, &mut buffer).await?;

    let conversion_register = u16::from_be_bytes(buffer);
    let alert_flag = (conversion_register >> 15) > 0;
    let conversion_result = (conversion_register >> 2) & 0x3ff;
    let milli: f32 = to_millivolts(conversion_result);
    defmt::info!("altert: {}, conversion_res: {} => {} mV", alert_flag, conversion_result, milli);

    Ok(ExtAdcReading {
        value: conversion_result,
        alert_flag,
    })
}

fn to_millivolts(sample: u16) -> f32 {
    let u = 3_300f32 / 1024f32;
    let milli_v = u * sample as f32;
    milli_v
}

pub struct SensorSettings {
    pub broadcast_interval: Duration,
}

#[embassy_executor::task]
pub async fn run_external_adc(
    mut com1_i2c: Option<&'static mut I2c<'static, Async, Master>>,
    mut com2_i2c: Option<&'static mut I2c<'static, Async, Master>>,
    can_pub: CanTxPub,
    settings: SensorSettings,
) {
    let mut ticker = Ticker::every(settings.broadcast_interval);
    let enabled: [bool; NUM_ADCS] = [false; NUM_ADCS];
    let mut adcs = ExtAdcs::new(enabled);
    adcs.scan_and_enable(com1_i2c, com2_i2c);
}

#[embassy_executor::task]
pub async fn run_ext_adc_to_can(
    mut com1_i2c: Option<&'static mut I2c<'static, Async, Master>>,
    mut com2_i2c: Option<&'static mut I2c<'static, Async, Master>>,
    can_pub: CanTxPub,
    settings: SensorSettings,
) {
    const CAN_ID0: u16 = 190;
    const CAN_ID1: u16 = 191;
    let mut ticker = Ticker::every(settings.broadcast_interval);
    let enabled: [bool; NUM_ADCS] = [false; NUM_ADCS];
    let mut adcs = ExtAdcs::new(enabled);
    adcs.scan_and_enable(com1_i2c.as_deref_mut(), com2_i2c.as_deref_mut()).await;

    loop {
        let _ = adcs.read_all(com1_i2c.as_deref_mut(), com2_i2c.as_deref_mut()).await;
        // NOTE: publish only from 2 hardcoded i2c devices
        // com1_i2c float, float | com2_i2c float, float
        let reading0: Option<[u8; 2]> = adcs.measurements.0[0].map(|meas| meas.value.to_le_bytes());
        let reading1: Option<[u8; 2]> = adcs.measurements.0[3].map(|meas| meas.value.to_le_bytes());
        if let Some(reading) = reading0 {
            can_pub.publish_immediate((CAN_ID0, Vec::from_slice(&reading).unwrap()));
        }
        if let Some(reading) = reading1 {
            // can_pub.publish_immediate((CAN_ID1, Vec::from_slice(&reading).unwrap()));
        }

        ticker.next().await;
    }
}
