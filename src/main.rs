#![no_std]
#![no_main]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{
    adc::Adc,
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker};
use heapless::Vec;
use static_cell::StaticCell;

use crate::board::LedsState;
use crate::can::CanTxPub;

use {defmt_rtt as _, panic_probe as _};

mod board;
mod can;
mod command_listener;
mod ext_adc;
mod high_current_out;
mod hw;
mod utils;

// const CANOPEN_NODE_ID: u8 = X;

#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

static COM1_I2C: StaticCell<I2c<'static, Async, Master>> = StaticCell::new();
static COM2_I2C: StaticCell<I2c<'static, Async, Master>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut p = hw::setup();
    // let mut iwdg = IndependentWatchdog::new(p.IWDG, 512_000); // 512ms timeout
    // iwdg.unleash();

    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES239_5);

    //configure CANs
    use can::{CAN_IN, CAN_OUT};
    let can_in = CAN_IN.init(PubSubChannel::new());
    let can_out = CAN_OUT.init(PubSubChannel::new());

    let can1 = embassy_stm32::can::Can::new(p.CAN1, p.PB8, p.PB9, hw::Irqs);
    can::spawn(can1, spawner, can_in.publisher().unwrap(), can_out.subscriber().unwrap()).await;

    // -- ext adcs
    let i2c_config = embassy_stm32::i2c::Config::default();

    let com1_i2c = COM1_I2C.init(I2c::new(p.I2C1, p.PB6, p.PB7, hw::Irqs, p.DMA1_CH6, p.DMA1_CH7, i2c_config));
    let com2_i2c = COM2_I2C.init(I2c::new(p.I2C2, p.PB10, p.PB11, hw::Irqs, p.DMA1_CH4, p.DMA1_CH5, i2c_config));
    spawner.spawn(
        ext_adc::run_ext_adc_to_can(
            Some(com1_i2c),
            Some(com2_i2c),
            can_out.publisher().unwrap(),
            ext_adc::Settings {
                broadcast_interval: Duration::from_millis(100),
            },
        )
        .unwrap(),
    );
    // high current outputs,
    // let hco1 = Hco1::new(p.PC0);
    // let hco2 = Hco2::new(p.PC15);
    let mut out_temp = Output::new(p.PC15, Level::High, Speed::Low);
    out_temp.set_high();
    core::mem::forget(out_temp);

    // let (hco3, hco4) = new_hco3and4(p.TIM3, p.PB0, p.PB1);

    // let cm_listener = command_listener::CommandListener::new(
    //     (can_out.publisher().unwrap(), can_in.subscriber().unwrap()),
    //     hco1,
    //     hco2,
    //     hco3,
    //     hco4,
    // );
    // spawner.spawn(command_listener::run_command_listener(cm_listener).unwrap());

    // spawner.spawn(high_current_out::new_virtual_pwm(p.TIM2, p.PC0).unwrap());

    // status leds
    let led_red = Output::new(p.PC7, Level::Low, Speed::Low);
    let led_yellow = Output::new(p.PC8, Level::Low, Speed::Low);
    let led_white = Output::new(p.PC9, Level::Low, Speed::Low);
    let leds = (led_red, led_yellow, led_white);
    let led_pub_sub = board::STATE_LED_PUB_SUB.init(PubSubChannel::new());
    // spawner.spawn(pdo_watcher(led_pub_sub.publisher().unwrap()).unwrap());
    spawner.spawn(board::run_leds(leds, LedsState::default(), led_pub_sub.subscriber().unwrap()).unwrap());
}

// #[embassy_executor::task]
// pub async fn run_heartbeat(can_tx: CanTxPub) {
//     let mut ticker = Ticker::every(Duration::from_hz(1));
//     let id: u16 = 0x12;
//     let body: Vec<u8, 8> = Vec::from_array([0, 0, 0, 0, 0, 0, 0, 0]);
//     loop {
//         can_tx.publish_immediate(((id, body.clone())));
//         ticker.next().await
//     }
// }

// #[embassy_executor::task]
// pub async fn pdo_watcher(publisher: board::StateLedPub) {
//     // red_led
//     use board::LedsState;
//     let mut leds_state = LedsState::from([false, false, false]);
//     let mut ticker = Ticker::every(Duration::from_hz(10));
//     loop {
//         publisher.publish(leds_state).await;
//         ticker.next().await;
//     }
// }
