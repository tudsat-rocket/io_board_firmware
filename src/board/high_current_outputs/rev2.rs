use core::sync::atomic::{AtomicU16, Ordering};
use defmt::{Debug2Format, error};

use embassy_stm32::{
    Peri,
    gpio::{self, AfioRemap, Output, OutputType, Speed},
    interrupt::{self, InterruptExt},
    peripherals as p,
    time::Hertz,
    timer::{
        Ch3, Ch4, Channel, TimerPin,
        low_level::{CountingMode, OutputCompareMode, Timer},
        simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel},
    },
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::Duration;

use super::HcoControl;
use super::types::*;

type Hco1OutType = Mutex<CriticalSectionRawMutex, Option<Output<'static>>>;
type Hco2OutType = Mutex<CriticalSectionRawMutex, Option<Output<'static>>>;

#[cfg(feature = "rev2")]
static HCO1_OUT: Hco1OutType = Mutex::new(None);

#[cfg(feature = "rev2")]
static HCO2_OUT: Hco2OutType = Mutex::new(None);

#[cfg(feature = "rev2")]
static PULSE_US_PWM1: AtomicU16 = AtomicU16::new(1500);

#[cfg(feature = "rev2")]
static PULSE_US_PWM2: AtomicU16 = AtomicU16::new(1500);

/// Represents the current state of the high current outputs. Use [`HcoController`] to change the
/// state.
// This needs to be implemented via Mutex, because the interrupt service routine needs access to
// the state.
#[cfg(feature = "rev2")]
static HCO_STATE: Mutex<CriticalSectionRawMutex, HcoState> = Mutex::new(HcoState {
    _1: State::Digital(Level::Low),
    _2: State::Digital(Level::Low),
    _3: State::Digital(Level::Low),
    _4: State::Digital(Level::Low),
});

/// High current output controller for IO board rev2.
pub struct HcoControllerRev2 {
    state_mutex: &'static Mutex<CriticalSectionRawMutex, HcoState>,
    out1: &'static Hco1OutType,
    out2: &'static Hco2OutType,
    out3: SimplePwmChannel<'static, p::TIM3>,
    out4: SimplePwmChannel<'static, p::TIM3>,
    virtual_timer: Timer<'static, p::TIM2>,
}

#[cfg(feature = "rev3")]
impl HcoControl for HcoControllerRev2 {
    fn set_level(&mut self, output: HighCurrentOutput, level: Level) {}

    fn set_pwm_micros(&mut self, output: HighCurrentOutput, micros: u16) {}
    fn get_state(&self) -> HcoState {
        HcoState::default()
    }
    fn set_state(&mut self, target_state: HcoState) {}
}

#[cfg(feature = "rev2")]
impl HcoControl for HcoControllerRev2 {
    fn set_level(&mut self, output: HighCurrentOutput, level: Level) {
        let mut new_state = self.get_state();
        new_state.set_level(output, level);
        self.set_state(new_state);
    }

    fn set_pwm_micros(&mut self, output: HighCurrentOutput, micros: u16) {
        defmt::info!("{} set pwm us: {}", Debug2Format(&output), micros);
        let mut new_state = self.get_state();
        new_state.set_pwm_micros(output, micros);
        self.set_state(new_state);
    }
    fn get_state(&self) -> HcoState {
        *self.state_mutex.try_lock().unwrap()
    }
    fn set_state(&mut self, target_state: HcoState) {
        *self.state_mutex.try_lock().unwrap() = target_state;
        match target_state._1 {
            State::Digital(ref level) => {
                self.virtual_timer.enable_input_interrupt(Channel::Ch1, false);
                match target_state._2 {
                    State::Digital(_) => self.virtual_timer.enable_update_interrupt(false),
                    State::Pwm(_) => (),
                };
                self.out1.try_lock().unwrap().as_mut().map(|o| o.set_level(gpio::Level::from(*level)));
            }
            State::Pwm(duty) => {
                self.virtual_timer.enable_update_interrupt(true);
                self.virtual_timer.enable_input_interrupt(Channel::Ch1, true);
                PULSE_US_PWM1.store(duty.into(), Ordering::Relaxed);
            }
        };
        match target_state._2 {
            State::Digital(ref level) => {
                self.virtual_timer.enable_input_interrupt(Channel::Ch2, false);
                match target_state._1 {
                    State::Digital(_) => self.virtual_timer.enable_update_interrupt(false),
                    State::Pwm(_) => (),
                };
                self.out2.try_lock().unwrap().as_mut().map(|o| o.set_level(gpio::Level::from(*level)));
            }
            State::Pwm(duty) => {
                self.virtual_timer.enable_update_interrupt(true);
                self.virtual_timer.enable_input_interrupt(Channel::Ch2, true);
                PULSE_US_PWM2.store(duty.into(), Ordering::Relaxed);
            }
        };
        match target_state._3 {
            State::Digital(level) => match level {
                Level::High => self.out3.set_duty_cycle_fully_on(),
                Level::Low => self.out3.set_duty_cycle_fully_off(),
            },
            State::Pwm(duty) => {
                let micros = duty.as_u16().clamp(500, 2500);
                let num = (micros * 5) / 10;
                self.out3.set_duty_cycle_fraction(num, 10_000);
            }
        }
        match target_state._4 {
            State::Digital(level) => match level {
                Level::High => self.out4.set_duty_cycle_fully_on(),
                Level::Low => self.out4.set_duty_cycle_fully_off(),
            },
            State::Pwm(duty) => {
                let micros = duty.as_u16().clamp(500, 2500);
                let num = (micros * 5) / 10;
                self.out4.set_duty_cycle_fraction(num, 10_000);
            }
        }
    }
}

#[cfg(feature = "rev2")]
impl HcoControllerRev2 {
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

        *HCO1_OUT.lock().await = Some(out1);
        *HCO2_OUT.lock().await = Some(out2);

        let period = Duration::from_hz(50);
        let mut tim2 = Timer::new(virtual_timer);
        tim2.set_tick_freq(Hertz::mhz(1));
        tim2.set_max_compare_value((period.as_micros() - 1) as u32);
        tim2.set_autoreload_preload(true);
        tim2.enable_update_interrupt(true);
        tim2.set_output_compare_mode(Channel::Ch1, OutputCompareMode::Frozen);
        tim2.set_compare_value(Channel::Ch1, 1500);
        tim2.set_output_compare_mode(Channel::Ch2, OutputCompareMode::Frozen);
        tim2.set_compare_value(Channel::Ch2, 1500);

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
            state_mutex: &HCO_STATE,
            out1: &HCO1_OUT,
            out2: &HCO2_OUT,
            out3: channels.ch3,
            out4: channels.ch4,
            virtual_timer: tim2,
        }
    }
}

#[cfg(feature = "rev2")]
embassy_stm32::bind_interrupts!(struct Irqs {
    TIM2 => Tim2Handler;
});

#[cfg(feature = "rev2")]
struct Tim2Handler;

#[cfg(feature = "rev2")]
impl interrupt::typelevel::Handler<interrupt::typelevel::TIM2> for Tim2Handler {
    unsafe fn on_interrupt() {
        let timer = embassy_stm32::pac::TIM2;
        let status_regs = timer.sr().read();
        if status_regs.uif() {
            timer.sr().modify(|w| w.set_uif(false));
            let pulse_width_ch1 = PULSE_US_PWM1.load(Ordering::Relaxed);
            let pulse_width_ch2 = PULSE_US_PWM2.load(Ordering::Relaxed);
            timer.ccr(0).write(|w| w.set_ccr(pulse_width_ch1));
            timer.ccr(1).write(|w| w.set_ccr(pulse_width_ch2));
            if let Ok(mut guard) = HCO1_OUT.try_lock()
                && let Some(output) = guard.as_mut()
            {
                output.set_high();
            } else {
                error!("mutex bug");
            }
            if let Ok(mut guard) = HCO2_OUT.try_lock()
                && let Some(output) = guard.as_mut()
            {
                output.set_high();
            } else {
                error!("mutex bug");
            }
        }
        if status_regs.ccif(0) {
            timer.sr().modify(|w| w.set_ccif(0, false));
            if let Ok(mut guard) = HCO1_OUT.try_lock()
                && let Some(output) = guard.as_mut()
            {
                output.set_low();
            } else {
                error!("mutex bug");
            }
        }
        if status_regs.ccif(1) {
            timer.sr().modify(|w| w.set_ccif(1, false));
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
