use embassy_stm32::Peripherals;
use embassy_stm32::peripherals::{
    CAN1, CAN2, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7, I2C1, I2C2, UART5, USART1, USART2, USART3,
};
use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz;

embassy_stm32::bind_interrupts!(pub struct Irqs {
    CAN1_RX0 => embassy_stm32::can::Rx0InterruptHandler<CAN1>;
    CAN1_TX => embassy_stm32::can::TxInterruptHandler<CAN1>;
    CAN1_RX1 => embassy_stm32::can::Rx1InterruptHandler<CAN1>;
    CAN1_SCE => embassy_stm32::can::SceInterruptHandler<CAN1>;

    CAN2_RX0 => embassy_stm32::can::Rx0InterruptHandler<CAN2>;
    CAN2_TX => embassy_stm32::can::TxInterruptHandler<CAN2>;
    CAN2_RX1 => embassy_stm32::can::Rx1InterruptHandler<CAN2>;
    CAN2_SCE => embassy_stm32::can::SceInterruptHandler<CAN2>;


    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<I2C1>;
    DMA1_CHANNEL6 => embassy_stm32::dma::InterruptHandler<DMA1_CH6>;
    DMA1_CHANNEL7 => embassy_stm32::dma::InterruptHandler<DMA1_CH7>;


    I2C2_EV => embassy_stm32::i2c::EventInterruptHandler<I2C2>;
    I2C2_ER => embassy_stm32::i2c::ErrorInterruptHandler<I2C2>;
    DMA1_CHANNEL4 => embassy_stm32::dma::InterruptHandler<DMA1_CH4>;
    DMA1_CHANNEL5 => embassy_stm32::dma::InterruptHandler<DMA1_CH5>;

    USART1 => embassy_stm32::usart::InterruptHandler<USART1>;
    USART2 => embassy_stm32::usart::InterruptHandler<USART2>;
    USART3 => embassy_stm32::usart::InterruptHandler<USART3>;
    UART5 => embassy_stm32::usart::InterruptHandler<UART5>;
});
pub fn setup() -> Peripherals {
    //configure "p"
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(embassy_stm32::rcc::Hse {
        mode: embassy_stm32::rcc::HseMode::Oscillator,
        freq: Hertz::mhz(8), // our high-speed external oscillator speed
    });

    // 72 MHz
    config.rcc.pll = Some(embassy_stm32::rcc::Pll {
        src: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: embassy_stm32::rcc::PllMul::MUL9,
    });

    config.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_P;

    // advanced high performace bus: 72 MHz
    config.rcc.ahb_pre = AHBPrescaler::DIV1;

    // peripheral bus 1: 36 MHz
    config.rcc.apb1_pre = APBPrescaler::DIV2;

    // peripheral bus 2: 72 MHz
    config.rcc.apb2_pre = APBPrescaler::DIV1;

    // analog digital converter: 12 MHz (max: 14MHz)
    config.rcc.adc_pre = ADCPrescaler::DIV6;
    embassy_stm32::init(config)
}
