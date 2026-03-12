use core::{
    str::SplitWhitespace,
    sync::atomic::{AtomicU16, Ordering},
};
use defmt::{error, info};

use static_cell::StaticCell;

/// Abstraction for the high current outputs of the IO Board rev2.
/// Numbering is from left to right; two outputs per Molex connector.
use embassy_stm32::{
    Peri,
    gpio::{self, AfioRemap, Output, OutputType, Speed},
    interrupt::{self, InterruptExt},
    // pac::Interrupt::{TIM2, TIM3},
    peripherals as p,
    time::Hertz,
    timer::{
        Ch3, Ch4, Channel, TimerPin,
        low_level::{CountingMode, OutputCompareMode, Timer},
        simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel},
    },
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::Duration;

// pub struct Hco1 {
//     output: Output<'static>,
// }
// impl Hco1 {
//     pub fn new(pin: Peri<'static, p::PC0>) -> Self {
//         Self {
//             output: Output::new(pin, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::Low),
//         }
//     }
// }
// pub struct Hco2 {
//     output: Output<'static>,
// }
// impl Hco2 {
//     pub fn new(pin: Peri<'static, p::PC15>) -> Self {
//         Self {
//             output: Output::new(pin, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::Low),
//         }
//     }
// }
// pub struct Hco3 {
//     pwm: SimplePwmChannel<'static, p::TIM3>,
// }
// pub struct Hco4 {
//     pwm: SimplePwmChannel<'static, p::TIM3>,
// }
//
// pub fn new_hco3and4(
//     tim: Peri<'static, p::TIM3>,
//     pin3: Peri<'static, p::PB0>,
//     pin4: Peri<'static, p::PB1>,
// ) -> (Hco3, Hco4) {
//     <p::PB0 as TimerPin<p::TIM3, Ch3, AfioRemap<0>>>::afio_remap(&pin3);
//     <p::PB1 as TimerPin<p::TIM3, Ch4, AfioRemap<0>>>::afio_remap(&pin4);
//     let pwmpin3: PwmPin<'_, p::TIM3, Ch3, AfioRemap<0>> = PwmPin::new(pin3, OutputType::PushPull);
//     let pwmpin4: PwmPin<'_, p::TIM3, Ch4, AfioRemap<0>> = PwmPin::new(pin4, OutputType::PushPull);
//     let pwm = SimplePwm::new(tim, None, None, Some(pwmpin3), Some(pwmpin4), Hertz::hz(50), CountingMode::EdgeAlignedUp);
//     let mut channels = pwm.split();
//     channels.ch3.enable();
//     channels.ch4.enable();
//     (Hco3 { pwm: channels.ch3 }, Hco4 { pwm: channels.ch4 })
// }
//
// pub trait DigitalOutput {
//     async fn set_level(&mut self, state: bool);
// }
// pub trait ServoOutput {
//     async fn set_duty_micros(&mut self, micros: u16);
// }
// impl DigitalOutput for Hco1 {
//     async fn set_level(&mut self, state: bool) {
//         self.output.set_level(state.into());
//         let print_string = match state {
//             true => "high",
//             false => "low",
//         };
//         defmt::info!("Hco1: set output digitally to: {}", print_string);
//     }
// }
// impl DigitalOutput for Hco2 {
//     async fn set_level(&mut self, state: bool) {
//         self.output.set_level(state.into());
//         let print_string = match state {
//             true => "high",
//             false => "low",
//         };
//         defmt::info!("Hco2: set output digitally to: {}", print_string);
//     }
// }
// impl DigitalOutput for Hco3 {
//     async fn set_level(&mut self, state: bool) {
//         if state {
//             self.pwm.set_duty_cycle_fully_on();
//         } else {
//             self.pwm.set_duty_cycle_fully_off();
//         }
//         let print_string = match state {
//             true => "high",
//             false => "low",
//         };
//         defmt::info!("Hco3: set output digitally to: {}", print_string);
//     }
// }
// impl DigitalOutput for Hco4 {
//     async fn set_level(&mut self, state: bool) {
//         if state {
//             self.pwm.set_duty_cycle_fully_on();
//         } else {
//             self.pwm.set_duty_cycle_fully_off();
//         }
//         let print_string = match state {
//             true => "high",
//             false => "low",
//         };
//         defmt::info!("Hco4: set output digitally to: {}", print_string);
//     }
// }
//
// impl ServoOutput for Hco3 {
//     async fn set_duty_micros(&mut self, micros: u16) {
//         let micros = micros.clamp(500, 2500);
//         // between 500 and 2500us on a 20ms period
//         // 0.5 / 20 = 250 / 10_000
//         // 2.5 / 20 = 1_250 / 10_000
//         let num = (micros * 5) / 10;
//         self.pwm.set_duty_cycle_fraction(num, 10_000);
//         defmt::info!("Hco3: set duty cylce to {} micros", micros);
//     }
// }
// impl ServoOutput for Hco4 {
//     async fn set_duty_micros(&mut self, micros: u16) {
//         let micros = micros.clamp(500, 2500);
//         // between 500 and 2500us on a 20ms period
//         // 0.5 / 20 = 250 / 10_000
//         // 2.5 / 20 = 1_250 / 10_000
//         let num = (micros * 5) / 10;
//         self.pwm.set_duty_cycle_fraction(num, 10_000);
//         defmt::info!("Hco4: set duty cylce to {} micros", micros);
//     }
// }

pub enum HighCurrentOutput {
    _1,
    _2,
    _3,
    _4,
}
#[derive(Clone, Copy, Debug, Default, Eq, Hash, PartialEq)]
pub enum Level {
    High,
    #[default]
    Low,
}
impl From<Level> for gpio::Level {
    fn from(value: Level) -> Self {
        match value {
            Level::High => gpio::Level::High,
            Level::Low => gpio::Level::Low,
        }
    }
}
impl From<gpio::Level> for Level {
    fn from(value: gpio::Level) -> Self {
        match value {
            gpio::Level::High => Level::High,
            gpio::Level::Low => Level::Low,
        }
    }
}

// pub enum HcoCommand {
//     SetDigital((HighCurrentOutput, Level)),
//     /// Set duty cycle for high current output in microseconds
//     SetDuty((HighCurrentOutput, u16)),
// }

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum State {
    Digital(Level),
    /// Set duty cycle for high current output in microseconds
    Pwm(PwmMicros),
}
impl Default for State {
    fn default() -> Self {
        Self::Digital(Level::default())
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct PwmMicros(u16);
impl From<PwmMicros> for u16 {
    fn from(value: PwmMicros) -> Self {
        value.as_u16()
    }
}
impl PwmMicros {
    pub fn as_u16(&self) -> u16 {
        self.0
    }
    pub fn from_u16_clamped(value: u16) -> Self {
        Self(value.clamp(500, 2500))
    }
}
impl TryFrom<u16> for PwmMicros {
    type Error = ();
    fn try_from(value: u16) -> Result<Self, Self::Error> {
        if 500 <= value && value >= 2500 {
            Ok(Self(value))
        } else {
            Err(())
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, Hash, PartialEq)]
pub struct HcoState {
    pub _1: State,
    pub _2: State,
    pub _3: State,
    pub _4: State,
}

impl HcoState {
    fn set_high(&mut self, output: HighCurrentOutput, sig: &Signal<CriticalSectionRawMutex, bool>) {
        match output {
            HighCurrentOutput::_1 => self._1 = State::Digital(Level::High),
            HighCurrentOutput::_2 => self._2 = State::Digital(Level::High),
            HighCurrentOutput::_3 => self._3 = State::Digital(Level::High),
            HighCurrentOutput::_4 => self._4 = State::Digital(Level::High),
        }
        sig.signal(true);
    }
    fn set_low(&mut self, output: HighCurrentOutput, sig: &Signal<CriticalSectionRawMutex, bool>) {
        match output {
            HighCurrentOutput::_1 => self._1 = State::Digital(Level::Low),
            HighCurrentOutput::_2 => self._2 = State::Digital(Level::Low),
            HighCurrentOutput::_3 => self._3 = State::Digital(Level::Low),
            HighCurrentOutput::_4 => self._4 = State::Digital(Level::Low),
        }
        sig.signal(true);
    }
    fn set_pwm_micros(&mut self, output: HighCurrentOutput, micros: u16, sig: &Signal<CriticalSectionRawMutex, bool>) {
        match output {
            HighCurrentOutput::_1 => self._1 = State::Pwm(PwmMicros::from_u16_clamped(micros)),
            HighCurrentOutput::_2 => self._2 = State::Pwm(PwmMicros::from_u16_clamped(micros)),
            HighCurrentOutput::_3 => self._3 = State::Pwm(PwmMicros::from_u16_clamped(micros)),
            HighCurrentOutput::_4 => self._4 = State::Pwm(PwmMicros::from_u16_clamped(micros)),
        }
        sig.signal(true);
    }
}

pub static HCO_STATE_CHANGE_SIG: Signal<CriticalSectionRawMutex, bool> = Signal::new();

pub struct HighCurrentOutputs {
    out1: &'static Hco1OutType,
    out2: &'static Hco2OutType,
    out3: SimplePwmChannel<'static, p::TIM3>,
    out4: SimplePwmChannel<'static, p::TIM3>,
    virtual_timer: Timer<'static, p::TIM2>,
    // out3_4_timer: Timer<'static, p::TIM3>,
}
impl HighCurrentOutputs {
    pub async fn new(
        pin1: Peri<'static, impl gpio::Pin>,
        pin2: Peri<'static, impl gpio::Pin>,
        pin3: Peri<'static, p::PB0>,
        pin4: Peri<'static, p::PB1>,
        virtual_timer: Peri<'static, p::TIM2>,
        out3_4_timer: Peri<'static, p::TIM3>,
    ) -> Self {
        let out1 = Output::new(pin1, gpio::Level::Low, Speed::Low);
        let out2 = Output::new(pin2, gpio::Level::Low, Speed::Low);
        // let out3 = Output::new(pin3, gpio::Level::Low, Speed::Low);
        // let out4 = Output::new(pin4, gpio::Level::Low, Speed::Low);
        // virtual_timing setup:
        const MIN_PULSE_US: u16 = 1000;
        const MAX_PULSE_US: u16 = 2000;
        *HCO1_OUT.lock().await = Some(out1);
        *HCO2_OUT.lock().await = Some(out2);

        let period = Duration::from_hz(50);
        let mut tim2 = Timer::new(virtual_timer);
        // tim.set_frequency(Hertz::mhz(1)); // T = 1us
        tim2.set_tick_freq(Hertz::mhz(1));
        tim2.set_max_compare_value((period.as_micros() - 1) as u32);
        tim2.set_autoreload_preload(true);
        tim2.enable_update_interrupt(true);
        // configure channels
        tim2.set_output_compare_mode(Channel::Ch1, OutputCompareMode::Frozen);
        tim2.set_compare_value(Channel::Ch1, 1500);
        // tim2.enable_input_interrupt(Channel::Ch1, true);
        tim2.set_output_compare_mode(Channel::Ch2, OutputCompareMode::Frozen);
        tim2.set_compare_value(Channel::Ch2, 1500);
        // tim2.enable_input_interrupt(Channel::Ch2, true);

        tim2.start();

        embassy_stm32::interrupt::TIM2.unpend();
        unsafe { embassy_stm32::interrupt::TIM2.enable() };

        <p::PB0 as TimerPin<p::TIM3, Ch3, AfioRemap<0>>>::afio_remap(&pin3);
        <p::PB1 as TimerPin<p::TIM3, Ch4, AfioRemap<0>>>::afio_remap(&pin4);
        let out3: PwmPin<'_, p::TIM3, Ch3, AfioRemap<0>> = PwmPin::new(pin3, OutputType::PushPull);
        let out4: PwmPin<'_, p::TIM3, Ch4, AfioRemap<0>> = PwmPin::new(pin4, OutputType::PushPull);
        let pwm = SimplePwm::new(
            out3_4_timer,
            None,
            None,
            Some(out3),
            Some(out4),
            Hertz::hz(50),
            CountingMode::EdgeAlignedUp,
        );
        let mut channels = pwm.split();
        channels.ch3.enable();
        channels.ch4.enable();

        Self {
            out1: &HCO1_OUT,
            out2: &HCO2_OUT,
            out3: channels.ch3,
            out4: channels.ch4,
            virtual_timer: tim2,
            // out3_4_timer: tim3,
        }
    }
}

#[embassy_executor::task]
pub async fn run_hco(outputs: HighCurrentOutputs, sig: Signal<CriticalSectionRawMutex, bool>, state_mutex: HcoState) {
    let state = state_mutex;
    let mut this = outputs;
    // setup
    loop {
        // update everything
        match state._1 {
            State::Digital(ref level) => {
                this.virtual_timer.enable_input_interrupt(Channel::Ch1, false);
                match state._2 {
                    State::Digital(_) => this.virtual_timer.enable_update_interrupt(false),
                    State::Pwm(_) => (),
                };
                this.out1.lock().await.as_mut().map(|o| o.set_level(gpio::Level::from(*level)));
            }
            State::Pwm(duty) => {
                this.virtual_timer.enable_update_interrupt(true);
                this.virtual_timer.enable_input_interrupt(Channel::Ch1, true);
                PULSE_US_PWM1.store(duty.into(), Ordering::Relaxed);
            }
        };
        match state._2 {
            State::Digital(ref level) => {
                this.virtual_timer.enable_input_interrupt(Channel::Ch2, false);
                match state._1 {
                    State::Digital(_) => this.virtual_timer.enable_update_interrupt(false),
                    State::Pwm(_) => (),
                };
                this.out2.lock().await.as_mut().map(|o| o.set_level(gpio::Level::from(*level)));
            }
            State::Pwm(duty) => {
                this.virtual_timer.enable_update_interrupt(true);
                this.virtual_timer.enable_input_interrupt(Channel::Ch2, true);
                PULSE_US_PWM2.store(duty.into(), Ordering::Relaxed);
            }
        };
        match state._3 {
            State::Digital(level) => match level {
                Level::High => this.out3.set_duty_cycle_fully_on(),
                Level::Low => this.out3.set_duty_cycle_fully_off(),
            },
            State::Pwm(duty) => {
                let micros = duty.as_u16().clamp(500, 2500);
                let num = (micros * 5) / 10;
                this.out3.set_duty_cycle_fraction(num, 10_000);
            }
        }
        match state._4 {
            State::Digital(level) => match level {
                Level::High => this.out4.set_duty_cycle_fully_on(),
                Level::Low => this.out4.set_duty_cycle_fully_off(),
            },
            State::Pwm(duty) => {
                let micros = duty.as_u16().clamp(500, 2500);
                let num = (micros * 5) / 10;
                this.out4.set_duty_cycle_fraction(num, 10_000);
            }
        }
        let _ = sig.wait().await;
    }
}

static HCO_STATE: Mutex<CriticalSectionRawMutex, HcoState> = Mutex::new(HcoState {
    _1: State::Digital(Level::Low),
    _2: State::Digital(Level::Low),
    _3: State::Digital(Level::Low),
    _4: State::Digital(Level::Low),
});

static HCO1_2_TIM: Mutex<CriticalSectionRawMutex, Option<Timer<'static, p::TIM2>>> = Mutex::new(None);
static HCO1_OUT: Mutex<CriticalSectionRawMutex, Option<Output<'static>>> = Mutex::new(None);
type Hco1OutType = Mutex<CriticalSectionRawMutex, Option<Output<'static>>>;
type Hco2OutType = Mutex<CriticalSectionRawMutex, Option<Output<'static>>>;
static HCO2_OUT: Mutex<CriticalSectionRawMutex, Option<Output<'static>>> = Mutex::new(None);
static PULSE_US_PWM1: AtomicU16 = AtomicU16::new(1500);
static PULSE_US_PWM2: AtomicU16 = AtomicU16::new(1500);

// interrupt handler

struct Tim2Handler;

embassy_stm32::bind_interrupts!(struct Irqs {
    TIM2 => Tim2Handler;
});

impl interrupt::typelevel::Handler<interrupt::typelevel::TIM2> for Tim2Handler {
    unsafe fn on_interrupt() {
        let timer = embassy_stm32::pac::TIM2;
        let status_regs = timer.sr().read();
        if status_regs.uif() {
            // timer overflowed, start of new pulse
            // clear the interrupt flag
            timer.sr().modify(|w| w.set_uif(false));
            // reload ccr1 with the latest requested pulse width.
            let pulse_width_ch1 = PULSE_US_PWM1.load(Ordering::Relaxed);
            let pulse_width_ch2 = PULSE_US_PWM2.load(Ordering::Relaxed);
            timer.ccr(0).write(|w| w.set_ccr(pulse_width_ch1));
            timer.ccr(1).write(|w| w.set_ccr(pulse_width_ch2));
            // drive the gpio for HCO1
            if let Ok(mut guard) = HCO1_OUT.try_lock()
                && let Some(output) = guard.as_mut()
            {
                output.set_high();
            } else {
                error!("mutex bug");
            }
            // drive the gpio for HCO2
            if let Ok(mut guard) = HCO2_OUT.try_lock()
                && let Some(output) = guard.as_mut()
            {
                output.set_high();
            } else {
                error!("mutex bug");
            }
        }
        // CH1: compare match (end of pulse)
        if status_regs.ccif(0) {
            timer.sr().modify(|w| w.set_ccif(0, false));
            // drive the gpio pin
            if let Ok(mut guard) = HCO1_OUT.try_lock()
                && let Some(output) = guard.as_mut()
            {
                output.set_low();
            } else {
                error!("mutex bug");
            }
        }
        // CH2: compare match (end of pulse)
        if status_regs.ccif(1) {
            timer.sr().modify(|w| w.set_ccif(1, false));
            // drive the gpio pin
            if let Ok(mut guard) = HCO2_OUT.try_lock()
                && let Some(output) = guard.as_mut()
            {
                output.set_low();
            } else {
                error!("mutex bug");
            }
        }
    }
}
// #[embassy_executor::task]
// pub async fn new_virtual_pwm(tim: Peri<'static, p::TIM2>, pin2: Peri<'static, p::PC0>) {
//     const MIN_PULSE_US: u16 = 1000;
//     const MAX_PULSE_US: u16 = 2000;
//     let pin = Output::new(pin2, gpio::Level::Low, Speed::Low);
//     {
//         *HCO1_OUT.lock().await = Some(pin);
//     }
//
//     let period = Duration::from_hz(50);
//     let mut tim = Timer::new(tim);
//     // tim.set_frequency(Hertz::mhz(1)); // T = 1us
//     tim.set_tick_freq(Hertz::mhz(1));
//     tim.set_max_compare_value((period.as_micros() - 1) as u32);
//     tim.set_autoreload_preload(true);
//     tim.set_output_compare_mode(Channel::Ch1, OutputCompareMode::Frozen);
//     tim.set_compare_value(Channel::Ch1, 1500);
//     tim.enable_input_interrupt(Channel::Ch1, true);
//     tim.enable_update_interrupt(true);
//
//     tim.start();
//
//     embassy_stm32::interrupt::TIM2.unpend();
//     unsafe { embassy_stm32::interrupt::TIM2.enable() };
//     info!("50 Hz software PWM running on PA8.  Sweeping servo…");
//
//     // ── Demo: sweep back and forth ──────────────────────────────────────────
//     // To control the servo from your own code, simply store any value in
//     // [MIN_PULSE_US, MAX_PULSE_US] into `PULSE_US`.  The ISR reads it on the
//     // next 20 ms overflow.
//     loop {
//         // MIN → MAX: one 10 µs step per 20 ms frame ≈ 1 °/frame
//         // PULSE_US_PWM1.store(1500, Ordering::Relaxed);
//         // embassy_time::Timer::after(Duration::from_secs(1000000)).await;
//         let mut pw = MIN_PULSE_US;
//         while pw <= (MAX_PULSE_US - 10) {
//             PULSE_US_PWM1.store(pw, Ordering::Relaxed);
//             // info!("servo pulse = {} µs", pw);
//             embassy_time::Timer::after(Duration::from_millis(20)).await;
//             pw = pw.saturating_add(10);
//         }
//
//         // MAX → MIN
//         let mut pw = MAX_PULSE_US;
//         while pw >= (MIN_PULSE_US + 10) {
//             PULSE_US_PWM1.store(pw, Ordering::Relaxed);
//             // info!("servo pulse = {} µs", pw);
//             embassy_time::Timer::after(Duration::from_millis(20)).await;
//             pw = pw.saturating_sub(10);
//         }
//     }
// }
