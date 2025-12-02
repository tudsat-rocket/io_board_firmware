#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_executor::{InterruptExecutor, main};
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::interrupt::Priority;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker};

use heapless::Vec;
use zencan_common::NodeId;
use zencan_node::Node;

use {defmt_rtt as _, panic_probe as _};

mod board;
mod can;
mod current_sens;
mod hw;
mod zencan;

const CANOPEN_NODE_ID: u8 = 4;

#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut p = hw::setup();
    // let mut iwdg = IndependentWatchdog::new(p.IWDG, 512_000); // 512ms timeout
    // iwdg.unleash();

    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES239_5);

    let led_red = Output::new(p.PC7, Level::Low, Speed::Low);
    let led_yellow = Output::new(p.PC8, Level::Low, Speed::Low);
    let led_green = Output::new(p.PC9, Level::Low, Speed::Low);
    let mut leds = (led_red, led_yellow, led_green);

    // let mut i2c_config = embassy_stm32::i2c::Config::default();
    // i2c_config.timeout = Duration::from_millis(10);
    // let i2c_freq = Hertz::khz(100);
    //
    // let input0 = I2c::new(p.I2C1, p.PB6, p.PB7, io_module_firmware::Irqs, p.DMA1_CH6, p.DMA1_CH7, i2c_config);
    // let input1 = I2c::new(p.I2C2, p.PB10, p.PB11, io_module_firmware::Irqs, p.DMA1_CH4, p.DMA1_CH5, i2c_config);
    //
    //configure CANs
    use can::{CAN_IN, CAN_OUT};
    let can_in = CAN_IN.init(PubSubChannel::new());
    let can_out = CAN_OUT.init(PubSubChannel::new());

    let can1 = embassy_stm32::can::Can::new(p.CAN1, p.PB8, p.PB9, hw::Irqs);
    // let can2 = embassy_stm32::can::Can::new(p.CAN2, p.PB12, p.PB13, hw::Irqs);
    //
    // interrupt::SPI2.set_priority(Priority::P6);
    // let high_priority_spawner = io_module_firmware::EXECUTOR_HIGH.start(interrupt::SPI2);
    //
    // // temporary heartbeat for thermal test
    // high_priority_spawner
    //     .spawn(crate::heartbeat::run(can_out.publisher().unwrap(), can_in.subscriber().unwrap()).unwrap());
    // spawner.spawn(crate::heartbeat::run_leds(leds).unwrap());
    //
    // Run CAN bus, publishing received messages on can_in and transmitting messages
    // published on can_out.
    can::spawn(can1, spawner, can_in.publisher().unwrap(), can_out.subscriber().unwrap()).await;

    // CanOpen
    let serial_num: u32 = 42;
    zencan::OBJECT1018.set_serial(serial_num);

    let node =
        Node::new(NodeId::new(CANOPEN_NODE_ID).unwrap(), &zencan::NODE_MBOX, &zencan::NODE_STATE, &zencan::OD_TABLE);
    spawner.spawn(can::canopen::run_zencan(node, can_in.subscriber().unwrap(), can_out.publisher().unwrap()).unwrap());

    let mut ticker = Ticker::every(Duration::from_secs(1));
    // let can_heartbeat_pub = can_out.publisher().unwrap();
    loop {
        leds.0.toggle();
        // let _ = can_heartbeat_pub.publish((0x704, Vec::from_array([0x7f]))).await;
        ticker.next().await;
    }
}
