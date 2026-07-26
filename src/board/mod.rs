use embassy_executor::Spawner;
use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{
    Peri,
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_sync::pubsub::PubSubChannel;

use cancan::{CanCan, CanCanConfig};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

pub mod high_current_outputs;
pub use high_current_outputs::*;

pub mod leds;
pub use leds::{LedsState, StateLedPub};
mod hw;

#[cfg(feature = "rev3")]
mod on_board_sens;

#[cfg(feature = "rev3")]
use crate::board::on_board_sens::OnboardSens3Peri;
#[cfg(feature = "rev3")]
pub use crate::board::on_board_sens::*;

#[cfg(all(feature = "rev2", feature = "rev3"))]
compile_error!("rev2 and rev3 are mutually exclusive");

#[cfg(not(any(feature = "rev2", feature = "rev3")))]
compile_error!("must enable exactly one of: rev2, rev3");

pub struct Board {
    #[cfg(feature = "rev2")]
    pub hco_controller: HcoControllerRev2,
    #[cfg(feature = "rev3")]
    pub hco_controller: HcoControllerRev3,
    pub leds: StateLedPub,
    pub com1_i2c: &'static mut I2c<'static, Async, Master>,
    pub com2_i2c: &'static mut I2c<'static, Async, Master>,
    pub can1: embassy_stm32::can::Can<'static>,
    #[cfg(feature = "rev3")]
    pub onboard_sens: OnboardSensRev3,
    // can2: embassy_stm32::can::Can<'static>,
    // pub cancan: CanCan<Flash<'static, Blocking>>,
    pub flash_peri: Peri<'static, peripherals::FLASH>,
}

static COM1_I2C: StaticCell<I2c<'static, Async, Master>> = StaticCell::new();
static COM2_I2C: StaticCell<I2c<'static, Async, Master>> = StaticCell::new();

// current sensing
use embassy_stm32::dma;
use embassy_stm32::peripherals::{ADC1, DMA1_CH1};
use embassy_stm32::{adc, peripherals};
embassy_stm32::bind_interrupts!(struct Irqs {
    ADC1_2 => adc::InterruptHandler<ADC1>;
    DMA1_CHANNEL1 => dma::InterruptHandler<DMA1_CH1>;
});

pub async fn init_board(spawner: Spawner) -> Board {
    let p = hw::setup();

    // TODO: watchdog
    // let mut iwdg = IndependentWatchdog::new(p.IWDG, 512_000); // 512ms timeout
    // iwdg.unleash();

    // NOTE: not used
    // let mut adc = Adc::new(p.ADC1);
    // adc.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES239_5);

    let can1 = embassy_stm32::can::Can::new(p.CAN1, p.PB8, p.PB9, hw::Irqs);
    // can::spawn(can1, spawner, can_in.publisher().unwrap(), can_out.subscriber().unwrap()).await;

    // -- ext adcs
    let i2c_config = embassy_stm32::i2c::Config::default();

    let com1_i2c = COM1_I2C.init(I2c::new(p.I2C1, p.PB6, p.PB7, p.DMA1_CH6, p.DMA1_CH7, hw::Irqs, i2c_config));
    let com2_i2c = COM2_I2C.init(I2c::new(p.I2C2, p.PB10, p.PB11, p.DMA1_CH4, p.DMA1_CH5, hw::Irqs, i2c_config));

    // FIXME: temp
    let mut hco_state_temp = HcoState::default();
    // rev3
    hco_state_temp.set_high(HighCurrentOutput::_1);
    hco_state_temp.set_pwm_micros(HighCurrentOutput::_2, 1500);
    hco_state_temp.set_high(HighCurrentOutput::_3);
    hco_state_temp.set_pwm_micros(HighCurrentOutput::_4, 1500);
    //
    // rev2
    // hco_state_temp.set_high(HighCurrentOutput::_1);
    // hco_state_temp.set_pwm_micros(HighCurrentOutput::_2, 1500);
    // hco_state_temp.set_high(HighCurrentOutput::_3);
    // hco_state_temp.set_pwm_micros(HighCurrentOutput::_4, 1500);

    #[cfg(feature = "rev2")]
    let hco_controller = HcoControllerRev2::new(p.PC0, p.PC15, p.PB0, p.PB1, p.TIM2, p.TIM3, hco_state_temp).await;

    #[cfg(feature = "rev3")]
    let hco_controller = HcoControllerRev3::new(p.PA7, p.PA8, p.PB0, p.PB1, p.TIM1, p.TIM3, hco_state_temp).await;

    // let can_open_interface =
    //     CanOpenInterface::new((can_out.publisher().unwrap(), can_in.subscriber().unwrap()), hco_controller);
    // spawner.spawn(run_can_command_listener(can_open_interface).unwrap());

    // status leds
    let led_red = Output::new(p.PC7, Level::Low, Speed::Low);
    let led_yellow = Output::new(p.PC8, Level::Low, Speed::Low);
    let led_white = Output::new(p.PC9, Level::Low, Speed::Low);
    let leds = (led_red, led_yellow, led_white);
    let led_pub_sub = leds::STATE_LED_PUB_SUB.init(PubSubChannel::new());
    // spawner.spawn(pdo_watcher(led_pub_sub.publisher().unwrap()).unwrap());
    spawner.spawn(leds::run_leds(leds, LedsState::default(), led_pub_sub.subscriber().unwrap()).unwrap());

    #[cfg(feature = "rev3")]
    let onboard_sens = OnboardSensRev3::new(
        p.ADC1,
        OnboardSens3Peri {
            i_sens_hco12: p.PA0,
            i_sens_hco34: p.PA1,
            i_sens_supply_current: Some(p.PC0),
            v_logic_supply: p.PC1,
            v_hco12_supply: p.PC3,
            v_hco34_supply: p.PC2,
            v_temp: p.PA4,
        },
        adc::SampleTime::CYCLES7_5,
    )
    .await;

    Board {
        hco_controller,
        leds: led_pub_sub.publisher().unwrap(),
        com1_i2c,
        com2_i2c,
        can1,
        #[cfg(feature = "rev3")]
        onboard_sens,
        // cancan,
        // for cancan
        flash_peri: p.FLASH,
    }
}

#[allow(non_camel_case_types)]
#[allow(clippy::upper_case_acronyms)]
#[allow(unused)]
pub mod pins_rev3 {
    use embassy_stm32::peripherals::*;
    pub type HSE_IN = PD0;
    pub type HSE_OUT = PD1;
    // X PD2;

    pub type I_SENSE_3 = PC0;
    pub type V_MAIN_SENSE = PC1;
    pub type HC2_SENSE = PC2;
    pub type HC_SENSE = PC3;
    pub type A_IN_3 = PC4;
    pub type A_IN_2 = PC5;
    pub type SWITCH = PC6;
    pub type STAT_LED_0 = PC7;
    pub type STAT_LED_1 = PC8;
    pub type STAT_LED_2 = PC9;
    pub type COM3_1 = PC10;
    // X PC11;
    // X PC12;
    // X PC13;
    // X PC14;
    // X PC15;

    pub type I_SENSE_1 = PA0;
    pub type I_SENSE_2 = PA1;
    pub type COM4_1 = PA2;
    pub type COM4_2 = PA3;
    pub type TH_SENSE = PA4;
    pub type A_IN_1 = PA5;
    pub type A_IN_0 = PA6;
    pub type HC_OUT_1 = PA7;
    pub type HC_OUT_2 = PA8;
    pub type USB_FS_VBUS = PA9;
    // X PA10;
    pub type USB_D_NEG = PA11;
    pub type USB_D_POS = PA12;
    // pub type SWDIO = PA13;
    // pub type SWCLK = PA14;
    // pub type SWO = PA15;

    pub type HC_OUT_3 = PB0;
    pub type HC_OUT_4 = PB1;
    pub type SPI_CS_FLASH = PB2;
    pub type SPI1_SCK = PB3;
    pub type SPI1_MISO = PB4;
    pub type SPI1_MOSI = PB5;
    pub type COM1_1 = PB6;
    pub type COM1_2 = PB7;
    pub type CAN1_RX = PB8;
    pub type CAN1_TX = PB9;
    pub type COM2_1 = PB10;
    pub type COM2_2 = PB11;
    pub type CAN2_RX = PB12;
    pub type CAN2_TX = PB13;
    // X PB14;
    // X PB15;
}

#[allow(non_camel_case_types)]
#[allow(clippy::upper_case_acronyms)]
#[allow(unused)]
pub mod pins_rev2 {
    use embassy_stm32::peripherals::*;
    pub type HSE_IN = PD0;
    pub type HSE_OUT = PD1;
    pub type COM_4_2 = PD2;
    pub type HC_OUT_1 = PC0;
    pub type V_MAIN_SENSE = PC1;
    pub type HC2_SENSE = PC2;
    pub type HC_SENSE = PC3;
    pub type A_IN_3 = PC4;
    pub type A_IN_2 = PC5;
    pub type SWITCH = PC6;
    pub type STAT_LED_0 = PC7;
    pub type STAT_LED_1 = PC8;
    pub type STAT_LED_2 = PC9;
    pub type IO_0 = PC10;
    pub type IO_1 = PC11;
    pub type COM_4_1 = PC12;
    pub type IO_6 = PC13;
    pub type IO_7 = PC14;
    pub type HC_OUT_2 = PC15;

    pub type I_SENSE_1 = PA0;
    pub type I_SENSE_2 = PA1;
    pub type COM3_1 = PA2;
    pub type COM3_2 = PA3;
    pub type TH_SENSE = PA4;
    pub type IO_2 = PA5;
    pub type A_IN_1 = PA6;
    pub type A_IN_0 = PA7;
    pub type IO_3 = PA8;
    pub type USB_FS_VBUS = PA9;
    pub type IO_8 = PA10;
    pub type USB_D_NEG = PA11;
    pub type USB_D_POS = PA12;
    // pub type SWDIO = PA13;
    // pub type SWCLK = PA14;
    // pub type SWO = PA15;

    pub type HC_OUT_3 = PB0;
    pub type HC_OUT_4 = PB1;
    pub type SPI_CS_FLASH = PB2;
    pub type SPI1_SCK = PB3;
    pub type SPI1_MISO = PB4;
    pub type SPI1_MOSI = PB5;
    pub type COM1_1 = PB6;
    pub type COM1_2 = PB7;
    pub type CAN1_RX = PB8;
    pub type CAN1_TX = PB9;
    pub type COM2_1 = PB10;
    pub type COM2_2 = PB11;
    pub type CAN2_RX = PB12;
    pub type CAN2_TX = PB13;
    pub type IO_4 = PB14;
    pub type IO_5 = PB15;
}
