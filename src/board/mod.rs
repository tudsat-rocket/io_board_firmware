use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level as GpioLevel, Output, Pull, Speed};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{
    Peri,
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::PubSubChannel;

use defmt_rtt as _;
use static_cell::StaticCell;

pub mod high_current_outputs;
pub use high_current_outputs::*;

pub mod ext_flash;
pub mod leds;
pub use leds::{LedsState, StateLedPub};
mod hw;

use crate::config::persist::NorConfigStore;

#[cfg(feature = "rev3")]
mod on_board_sens;

#[cfg(feature = "rev3")]
pub use crate::board::on_board_sens::*;

#[cfg(all(feature = "rev2", feature = "rev3"))]
compile_error!("rev2 and rev3 are mutually exclusive");

#[cfg(not(any(feature = "rev2", feature = "rev3")))]
compile_error!("must enable exactly one of: rev2, rev3");

/// SPI device wrapping the one chip on SPI1. A shared-bus device rather than an exclusive one
/// only because that is what `embassy-embedded-hal` offers; the mutex is uncontended.
pub type ExtFlashBus = Spi<'static, Async, spi::mode::Master>;
pub type ExtFlashSpi =
    embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice<'static, NoopRawMutex, ExtFlashBus, Output<'static>>;
pub type ExtFlash = ext_flash::W25Q<ExtFlashSpi>;
/// The persisted-configuration store, monomorphised so it can cross an `#[embassy_executor::task]`
/// boundary (tasks cannot be generic).
pub type ConfigStore = NorConfigStore<ExtFlash>;

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
    /// `None` when the NOR flash did not identify itself, in which case the node runs on its
    /// compile-time factory defaults and refuses to persist. Both revisions populate the chip.
    pub config_store: Option<ConfigStore>,
    // can2 is populated in hardware but unused; see `can/mod.rs`.
    pub flash_peri: Peri<'static, peripherals::FLASH>,
}

static COM1_I2C: StaticCell<I2c<'static, Async, Master>> = StaticCell::new();
static COM2_I2C: StaticCell<I2c<'static, Async, Master>> = StaticCell::new();
static EXT_FLASH_BUS: StaticCell<Mutex<NoopRawMutex, ExtFlashBus>> = StaticCell::new();

const WATCHDOG_TIMEOUT_US: u32 = 250_000;
const WATCHDOG_PET_INTERVAL: embassy_time::Duration = embassy_time::Duration::from_millis(50);

#[embassy_executor::task]
async fn run_watchdog(mut iwdg: IndependentWatchdog<'static, embassy_stm32::peripherals::IWDG>) -> ! {
    let mut ticker = embassy_time::Ticker::every(WATCHDOG_PET_INTERVAL);
    let mut last = embassy_time::Instant::now();
    loop {
        iwdg.pet();

        // Report near-misses.
        let now = embassy_time::Instant::now();
        let late = (now - last).as_millis().saturating_sub(WATCHDOG_PET_INTERVAL.as_millis());
        if late > 20 {
            defmt::warn!("watchdog: pet {} ms late, executor stalled", late);
        }
        last = now;

        ticker.next().await;
    }
}

pub fn pet_watchdog() {
    use embassy_stm32::pac::iwdg::vals::Key;
    embassy_stm32::pac::IWDG.kr().write(|w| w.set_key(Key::RESET));
}

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

    let mut iwdg = IndependentWatchdog::new(p.IWDG, WATCHDOG_TIMEOUT_US);
    iwdg.unleash();
    spawner.spawn(run_watchdog(iwdg).unwrap());

    // NOTE: not used
    // let mut adc = Adc::new(p.ADC1);
    // adc.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES239_5);

    let can1 = embassy_stm32::can::Can::new(p.CAN1, p.PB8, p.PB9, hw::Irqs);
    // can::spawn(can1, spawner, can_in.publisher().unwrap(), can_out.subscriber().unwrap()).await;

    // -- ext adcs
    let i2c_config = embassy_stm32::i2c::Config::default();

    let com1_i2c = COM1_I2C.init(I2c::new(p.I2C1, p.PB6, p.PB7, p.DMA1_CH6, p.DMA1_CH7, hw::Irqs, i2c_config));
    let com2_i2c = COM2_I2C.init(I2c::new(p.I2C2, p.PB10, p.PB11, p.DMA1_CH4, p.DMA1_CH5, hw::Irqs, i2c_config));

    // DMA1 channels 2 and 3 are SPI1's; channel 1 belongs to ADC1 and 4..7 to the two I2C buses.
    let mut spi_config = spi::Config::default();
    // The W25Q128 is good for 104 MHz; 72/8 = 9 MHz is well inside spec and keeps the config
    // record's few hundred bytes far below a millisecond.
    spi_config.frequency = Hertz::mhz(9);
    let spi = Spi::new(p.SPI1, p.PB3, p.PB5, p.PB4, p.DMA1_CH3, p.DMA1_CH2, hw::Irqs, spi_config);
    let spi_bus = EXT_FLASH_BUS.init(Mutex::new(spi));
    let flash_cs = Output::new(p.PB2, GpioLevel::High, Speed::VeryHigh);
    let mut nor =
        ext_flash::W25Q::new(embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(spi_bus, flash_cs));

    let config_store = match nor.probe().await {
        Ok(()) => Some(NorConfigStore::new(nor)),
        Err(e) => {
            // use compile-time defaults
            defmt::error!("no usable config flash ({}), running on compile-time defaults", e);
            None
        }
    };

    // The on-board button is the physical way into raw debug mode, for a bench where nobody has a
    // CAN adapter to hand. Active low: the schematic pulls it to ground when pressed.
    let debug_button = ExtiInput::new(p.PC6, p.EXTI6, Pull::Up, hw::Irqs);
    spawner.spawn(watch_debug_button(debug_button).unwrap());

    // Every output starts de-energised, which is also what the hardware does on its own: the gate
    // drives have 12k pulldowns, so power-on, reset and a panic all land on "off" before this line
    // runs. The control task takes over from here and is the only thing that moves them after.
    let hco_initial = HcoState::default();

    #[cfg(feature = "rev2")]
    let hco_controller = HcoControllerRev2::new(p.PC0, p.PC15, p.PB0, p.PB1, p.TIM2, p.TIM3, hco_initial).await;

    #[cfg(feature = "rev3")]
    let hco_controller = HcoControllerRev3::new(p.PA7, p.PA8, p.PB0, p.PB1, p.TIM1, p.TIM3, hco_initial).await;

    // let can_open_interface =
    //     CanOpenInterface::new((can_out.publisher().unwrap(), can_in.subscriber().unwrap()), hco_controller);
    // spawner.spawn(run_can_command_listener(can_open_interface).unwrap());

    // status leds
    let led_red = Output::new(p.PC7, GpioLevel::Low, Speed::Low);
    let led_yellow = Output::new(p.PC8, GpioLevel::Low, Speed::Low);
    let led_white = Output::new(p.PC9, GpioLevel::Low, Speed::Low);
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
        config_store,
        // for cancan's A/B image handling
        flash_peri: p.FLASH,
    }
}

/// Toggle raw debug mode on a button press.
#[embassy_executor::task]
async fn watch_debug_button(mut button: ExtiInput<'static, Async>) -> ! {
    loop {
        button.wait_for_falling_edge().await;
        embassy_time::Timer::after_millis(30).await;
        if button.is_high() {
            continue; // bounce or noise, not a press
        }

        {
            let mut store = crate::store::STORE.lock().await;
            store.raw_debug = !store.raw_debug;
            store.pending.outputs = true;
            defmt::warn!("button: raw debug mode {}", if store.raw_debug { "ON" } else { "off" });
        }
        crate::store::CONTROL_WAKE.signal(());

        // debounce
        embassy_time::Timer::after_millis(300).await;
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
