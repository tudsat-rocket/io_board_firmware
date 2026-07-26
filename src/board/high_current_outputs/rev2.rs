use core::sync::atomic::{AtomicU16, Ordering};

use embassy_stm32::{
    Peri,
    gpio::{self, AfioRemap, Output, OutputType, Speed},
    interrupt::{self, InterruptExt},
    pac, peripherals as p,
    time::Hertz,
    timer::{
        Ch3, Ch4, Channel, TimerPin,
        low_level::{CountingMode, OutputCompareMode, Timer},
        simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel},
    },
};
use embassy_time::Duration;

use super::HcoControl;
use super::types::*;

/// A duty of u16::MAX means PWM disabled
static PULSE_US_PWM1: AtomicU16 = AtomicU16::new(u16::MAX);

/// A duty of u16::MAX means PWM disabled
static PULSE_US_PWM2: AtomicU16 = AtomicU16::new(u16::MAX);

/// High current output controller for IO board rev2.
pub struct HcoControllerRev2 {
    // state_mutex: &'static Mutex<CriticalSectionRawMutex, HcoState>,
    state: HcoState,
    out1: Output<'static>,
    out2: Output<'static>,
    out3: SimplePwmChannel<'static, p::TIM3>,
    out4: SimplePwmChannel<'static, p::TIM3>,
    virtual_timer: Timer<'static, p::TIM2>,
}

impl HcoControl for HcoControllerRev2 {
    fn set_level(&mut self, output: HighCurrentOutput, level: Level) {
        let mut new_state = self.get_state();
        new_state.set_level(output, level);
        self.set_state(new_state);
    }

    fn set_pwm_micros(&mut self, output: HighCurrentOutput, micros: u16) {
        let mut new_state = self.get_state();
        new_state.set_pwm_micros(output, micros);
        self.set_state(new_state);
    }
    fn get_state(&self) -> HcoState {
        self.state.clone()
    }
    fn set_state(&mut self, target_state: HcoState) {
        match target_state._1 {
            State::Digital(level) => {
                PULSE_US_PWM1.store(u16::MAX, Ordering::Relaxed);
                self.out1.set_level(level.into());
            }
            State::Pwm(duty) => {
                PULSE_US_PWM1.store(u16::from(duty), Ordering::Relaxed);
            }
        }
        match target_state._2 {
            State::Digital(level) => {
                PULSE_US_PWM2.store(u16::MAX, Ordering::Relaxed);
                self.out2.set_level(level.into());
            }
            State::Pwm(duty) => {
                PULSE_US_PWM2.store(u16::from(duty), Ordering::Relaxed);
            }
        }
        match target_state._3 {
            State::Digital(level) => match level {
                Level::High => self.out3.set_duty_cycle_fully_on(),
                Level::Low => self.out3.set_duty_cycle_fully_off(),
            },
            State::Pwm(duty) => {
                let micros = duty.as_u16();
                let num = (u32::from(micros) * 5) / 10;
                self.out3.set_duty_cycle_fraction(num, 10_000);
            }
        }
        match target_state._4 {
            State::Digital(level) => match level {
                Level::High => self.out4.set_duty_cycle_fully_on(),
                Level::Low => self.out4.set_duty_cycle_fully_off(),
            },
            State::Pwm(duty) => {
                let micros = duty.as_u16();
                let num = (u32::from(micros) * 5) / 10;
                self.out4.set_duty_cycle_fraction(num, 10_000);
            }
        }

        let hco1_is_pwm = matches!(target_state._1, State::Pwm(_));
        let hco2_is_pwm = matches!(target_state._2, State::Pwm(_));

        self.virtual_timer.enable_update_interrupt(hco1_is_pwm || hco2_is_pwm);
        self.virtual_timer.enable_input_interrupt(Channel::Ch1, hco1_is_pwm);
        self.virtual_timer.enable_input_interrupt(Channel::Ch2, hco2_is_pwm);

        self.state = target_state;
    }
}

impl HcoControllerRev2 {
    pub async fn new(
        // NOTE: don't change this, since we use raw pac to set this output
        pin1: Peri<'static, p::PC0>,
        // NOTE: don't change this, since we use raw pac to set this output
        pin2: Peri<'static, p::PC15>,
        pin3: Peri<'static, p::PB0>,
        pin4: Peri<'static, p::PB1>,
        virtual_timer: Peri<'static, p::TIM2>,
        out3_4_timer: Peri<'static, p::TIM3>,
        init_state: HcoState,
    ) -> Self {
        let out1 = Output::new(pin1, gpio::Level::Low, Speed::Low);
        let out2 = Output::new(pin2, gpio::Level::Low, Speed::Low);

        // *HCO1_OUT.lock().await = Some(out1);
        // *HCO2_OUT.lock().await = Some(out2);

        let period = Duration::from_hz(50);
        let mut tim2 = Timer::new(virtual_timer);
        tim2.set_tick_freq(Hertz::mhz(1));
        tim2.set_max_compare_value((period.as_micros() - 1) as u16);
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

        let mut hco_ctl = Self {
            state: init_state.clone(),
            out1,
            out2,
            out3: channels.ch3,
            out4: channels.ch4,
            virtual_timer: tim2,
        };
        hco_ctl.set_state(init_state);
        hco_ctl
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
            // update interrupt flag is set, meaning timer event has occured

            use core::u16;
            timer.sr().modify(|w| w.set_uif(false));

            let pulse_width_ch1 = PULSE_US_PWM1.load(Ordering::Relaxed);
            if pulse_width_ch1 != u16::MAX {
                use embassy_stm32::pac;

                timer.ccr(0).write(|w| w.set_ccr(pulse_width_ch1));
                // set PC0 = Level::High
                // access peripheral register and set output for PC0 to 1
                pac::GPIOC.bsrr().write(|w| w.set_bs(0, true))
            }

            let pulse_width_ch2 = PULSE_US_PWM2.load(Ordering::Relaxed);
            if pulse_width_ch2 != u16::MAX {
                timer.ccr(1).write(|w| w.set_ccr(pulse_width_ch2));
                // set PC15 = Level::High
                // access peripheral register and set output for PC15 to 1
                pac::GPIOC.bsrr().write(|w| w.set_bs(15, true))
            }
        }
        if status_regs.ccif(0) {
            // ccif flag is set, meaing capture has occured on channel -> reset ccif flag
            timer.sr().modify(|w| w.set_ccif(0, false));

            if PULSE_US_PWM1.load(Ordering::Relaxed) != u16::MAX {
                // set PC0 = Level::LOW
                // access peripheral register and reset output for PC0 to 0
                pac::GPIOC.bsrr().write(|w| w.set_br(0, true))
            }
        }
        if status_regs.ccif(1) {
            // ccif flag is set, meaing capture has occured on channel -> reset ccif flag
            timer.sr().modify(|w| w.set_ccif(1, false));
            if PULSE_US_PWM2.load(Ordering::Relaxed) != u16::MAX {
                // set PC15 = Level::Low
                // access peripheral register and reset output for PC15 to 0
                pac::GPIOC.bsrr().write(|w| w.set_br(15, true))
            }
        }
    }
}
