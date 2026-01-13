use embassy_stm32::gpio::Output;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use static_cell::StaticCell;

#[allow(non_camel_case_types)]
#[allow(clippy::upper_case_acronyms)]
#[allow(unused)]
pub mod pins {
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
    pub type COM_3_1 = PA2;
    pub type COM_3_2 = PA3;
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
    pub type SPI_1_SCK = PB3;
    pub type SPI_1_MISO = PB4;
    pub type SPI_1_MOSI = PB5;
    pub type COM_1_1 = PB6;
    pub type COM_1_2 = PB7;
    pub type CAN_1_RX = PB8;
    pub type CAN_1_TX = PB9;
    pub type COM_2_1 = PB10;
    pub type COM_2_2 = PB11;
    pub type CAN_2_RX = PB12;
    pub type CAN_2_TX = PB13;
    pub type IO_4 = PB14;
    pub type IO_5 = PB15;
}

pub type HcOutputsSub = Subscriber<'static, CriticalSectionRawMutex, HcOutputsState, 4, 1, 1>;
pub type StateLedSub = Subscriber<'static, CriticalSectionRawMutex, LedsState, 4, 1, 1>;
pub type StateLedPub = Publisher<'static, CriticalSectionRawMutex, LedsState, 4, 1, 1>;

pub static STATE_LED_PUB_SUB: StaticCell<PubSubChannel<CriticalSectionRawMutex, LedsState, 4, 1, 1>> =
    StaticCell::new();

#[derive(Clone)]
pub struct HcOutputsState {
    pub out1: bool,
    pub out2: bool,
    pub out3: bool,
    pub out4: bool,
}
impl From<[bool; 4]> for HcOutputsState {
    fn from(value: [bool; 4]) -> Self {
        HcOutputsState {
            out1: value[0],
            out2: value[1],
            out3: value[2],
            out4: value[3],
        }
    }
}
/// Task controlls high current outputs.
#[embassy_executor::task]
pub async fn run_hc_outputs(
    mut out1: Output<'static>,
    mut out2: Output<'static>,
    mut out3: Output<'static>,
    mut out4: Output<'static>,
    initial: HcOutputsState,
    mut sub: HcOutputsSub,
) {
    // hc outputs io board rev 2:
    // 1: PC0
    // 2: PC15
    // 3: PB0
    // 4: PB1
    out1.set_level(initial.out1.into());
    out2.set_level(initial.out2.into());
    out3.set_level(initial.out3.into());
    out4.set_level(initial.out4.into());

    loop {
        let new_state = sub.next_message_pure().await;
        out1.set_level(new_state.out1.into());
        out2.set_level(new_state.out2.into());
        out3.set_level(new_state.out3.into());
        out4.set_level(new_state.out4.into());
    }
}

#[derive(Default, Clone, Copy)]
pub struct LedsState {
    pub red: bool,
    pub yellow: bool,
    pub white: bool,
}
impl From<[bool; 3]> for LedsState {
    fn from(value: [bool; 3]) -> Self {
        Self {
            red: value[0],
            yellow: value[1],
            white: value[2],
        }
    }
}

#[embassy_executor::task]
pub async fn run_leds(
    mut leds: (Output<'static>, Output<'static>, Output<'static>),
    initial: LedsState,
    mut leds_state_sub: StateLedSub,
) -> ! {
    leds.0.set_level((!initial.red).into());
    leds.1.set_level((!initial.yellow).into());
    leds.2.set_level((!initial.white).into());
    loop {
        let leds_state = leds_state_sub.next_message_pure().await;
        leds.0.set_level((!leds_state.red).into());
        leds.1.set_level((!leds_state.yellow).into());
        leds.2.set_level((!leds_state.white).into());
    }
}
