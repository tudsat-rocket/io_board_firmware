use defmt::{Debug2Format, warn};
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
const AMPLIFIER_ADDRESSES: [u8; 3] = [0b1010000, 0b1010001, 0b1010010];

const NUM_ADCS: usize = 6;

#[derive(Copy, Clone)]
pub struct ExtAdcReading {
    value: u16,
    alert_flag: bool,
}

pub struct ExtAdcs {
    pub enabled: [bool; NUM_ADCS],
    pub measurements: [Option<ExtAdcReading>; NUM_ADCS],
}

impl ExtAdcs {
    fn new(enabled: [bool; NUM_ADCS]) -> Self {
        Self {
            enabled,
            measurements: [None; NUM_ADCS],
        }
    }
    fn default() -> Self {
        Self::new([false; NUM_ADCS])
    }
    async fn read_all(
        &mut self,
        mut com1_i2c: Option<&mut I2c<'static, Async, Master>>,
        mut com2_i2c: Option<&mut I2c<'static, Async, Master>>,
    ) -> Result<(), ()> {
        let mut success = true;
        for adc in 0..NUM_ADCS {
            if !self.enabled[adc] {
                continue;
            }
            let i2c_addr = AMPLIFIER_ADDRESSES[adc % 3];
            let i2c = match adc {
                0..3 => &mut com1_i2c,
                3..NUM_ADCS => &mut com2_i2c,
                _ => unreachable!(),
            };
            if let Some(i2c) = i2c {
                let res = read_i2c_adc(i2c, i2c_addr).await;
                if let Err(e) = res {
                    warn!("error reading adc value: {:?}", Debug2Format(&e));
                    success = false;
                }
                self.measurements[adc] = res.ok();
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

    Ok(ExtAdcReading {
        value: conversion_result,
        alert_flag,
    })
}

pub struct Settings {
    pub broadcast_interval: Duration,
}

#[embassy_executor::task]
pub async fn run_ext_adc_to_can(
    mut com1_i2c: Option<&'static mut I2c<'static, Async, Master>>,
    mut com2_i2c: Option<&'static mut I2c<'static, Async, Master>>,
    can_pub: CanTxPub,
    settings: Settings,
) {
    const CAN_ID: u16 = 190;
    let mut ticker = Ticker::every(settings.broadcast_interval);
    let enabled: [bool; NUM_ADCS] = [false, false, false, false, false, false];
    let mut adcs = ExtAdcs::new(enabled);

    loop {
        let _ = adcs.read_all(com1_i2c.as_deref_mut(), com2_i2c.as_deref_mut()).await;
        let reading0: Option<[u8; 2]> = adcs.measurements[0].map(|meas| meas.value.to_le_bytes());
        if let Some(reading) = reading0 {
            can_pub.publish_immediate((CAN_ID, Vec::from_slice(&reading).unwrap()));
        }

        ticker.next().await;
    }
}
