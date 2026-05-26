use core::sync::atomic::{AtomicU16, Ordering};
use defmt::error;

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

type Hco1OutType = Mutex<CriticalSectionRawMutex, Option<Output<'static>>>;
type Hco2OutType = Mutex<CriticalSectionRawMutex, Option<Output<'static>>>;
static HCO1_OUT: Hco1OutType = Mutex::new(None);
static HCO2_OUT: Hco2OutType = Mutex::new(None);
static HCO1_2_TIM: Mutex<CriticalSectionRawMutex, Option<Timer<'static, p::TIM2>>> = Mutex::new(None);

static PULSE_US_PWM1: AtomicU16 = AtomicU16::new(1500);
static PULSE_US_PWM2: AtomicU16 = AtomicU16::new(1500);

/// Represents the current state of the high current outputs. Use [`HcoController`] to change the
/// state.
// This needs to be implemented via Mutex, because the interrupt service routine needs access to
// the state.
static HCO_STATE: Mutex<CriticalSectionRawMutex, HcoState> = Mutex::new(HcoState {
    _1: State::Digital(Level::Low),
    _2: State::Digital(Level::Low),
    _3: State::Digital(Level::Low),
    _4: State::Digital(Level::Low),
});

pub struct HcoController {
    state_mutex: &'static Mutex<CriticalSectionRawMutex, HcoState>,
    out1: &'static Hco1OutType,
    out2: &'static Hco2OutType,
    out3: SimplePwmChannel<'static, p::TIM3>,
    out4: SimplePwmChannel<'static, p::TIM3>,
    virtual_timer: Timer<'static, p::TIM2>,
}
impl HcoController {
    pub fn set_level(&mut self, output: HighCurrentOutput, level: Level) {
        let mut new_state = self.get_state();
        new_state.set_level(output, level);
        self.set_state(new_state);
    }

    pub fn set_pwm_micros(&mut self, output: HighCurrentOutput, micros: u16) {
        let mut new_state = self.get_state();
        new_state.set_pwm_micros(output, micros);
        self.set_state(new_state);
    }
    pub fn get_state(&self) -> HcoState {
        self.state_mutex.try_lock().unwrap().clone()
    }
    pub fn set_state(&mut self, target_state: HcoState) {
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

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
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
impl From<bool> for Level {
    fn from(value: bool) -> Self {
        match value {
            false => Level::Low,
            true => Level::High,
        }
    }
}

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
        if 500 <= value && value <= 2500 {
            Ok(Self(value))
        } else {
            Err(())
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, Hash, PartialEq)]
pub struct HcoState {
    _1: State,
    _2: State,
    _3: State,
    _4: State,
}

impl HcoState {
    /// helper method for avoiding match statements
    pub fn set_high(&mut self, output: HighCurrentOutput) {
        match output {
            HighCurrentOutput::_1 => self._1 = State::Digital(Level::High),
            HighCurrentOutput::_2 => self._2 = State::Digital(Level::High),
            HighCurrentOutput::_3 => self._3 = State::Digital(Level::High),
            HighCurrentOutput::_4 => self._4 = State::Digital(Level::High),
        }
    }
    /// helper method for avoiding match statements
    pub fn set_low(&mut self, output: HighCurrentOutput) {
        match output {
            HighCurrentOutput::_1 => self._1 = State::Digital(Level::Low),
            HighCurrentOutput::_2 => self._2 = State::Digital(Level::Low),
            HighCurrentOutput::_3 => self._3 = State::Digital(Level::Low),
            HighCurrentOutput::_4 => self._4 = State::Digital(Level::Low),
        }
    }
    /// helper method for avoiding match statements
    pub fn set_level(&mut self, output: HighCurrentOutput, level: Level) {
        match output {
            HighCurrentOutput::_1 => self._1 = State::Digital(level),
            HighCurrentOutput::_2 => self._2 = State::Digital(level),
            HighCurrentOutput::_3 => self._3 = State::Digital(level),
            HighCurrentOutput::_4 => self._4 = State::Digital(level),
        }
    }

    /// helper method for avoiding match statements
    pub fn set_pwm_micros(&mut self, output: HighCurrentOutput, micros: u16) {
        match output {
            HighCurrentOutput::_1 => self._1 = State::Pwm(PwmMicros::from_u16_clamped(micros)),
            HighCurrentOutput::_2 => self._2 = State::Pwm(PwmMicros::from_u16_clamped(micros)),
            HighCurrentOutput::_3 => self._3 = State::Pwm(PwmMicros::from_u16_clamped(micros)),
            HighCurrentOutput::_4 => self._4 = State::Pwm(PwmMicros::from_u16_clamped(micros)),
        }
    }
}

impl HcoController {
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

embassy_stm32::bind_interrupts!(struct Irqs {
    TIM2 => Tim2Handler;
});

struct Tim2Handler;
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
