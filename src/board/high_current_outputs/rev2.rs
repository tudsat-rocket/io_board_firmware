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
use crate::hco::*;

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
    /// The time base behind the HCO1/HCO2 software PWM. **TIM5, not TIM2** — see
    /// [`HcoControllerRev2::new`].
    virtual_timer: Timer<'static, p::TIM5>,
}

impl HcoControl for HcoControllerRev2 {
    fn get_state(&self) -> HcoState {
        self.state
    }

    fn set_state(&mut self, target_state: HcoState) {
        // Unlike rev3 these four are not interchangeable: HCO1/HCO2 are plain GPIO driven by the
        // TIM5 software-PWM ISR, HCO3/HCO4 are real timer channels. So they stay written out.
        match target_state[HcoId::Hco0] {
            State::Digital(level) => {
                PULSE_US_PWM1.store(u16::MAX, Ordering::Relaxed);
                self.out1.set_level(level.into());
            }
            State::Pwm(duty) => {
                PULSE_US_PWM1.store(u16::from(duty), Ordering::Relaxed);
            }
        }
        match target_state[HcoId::Hco1] {
            State::Digital(level) => {
                PULSE_US_PWM2.store(u16::MAX, Ordering::Relaxed);
                self.out2.set_level(level.into());
            }
            State::Pwm(duty) => {
                PULSE_US_PWM2.store(u16::from(duty), Ordering::Relaxed);
            }
        }
        match target_state[HcoId::Hco2] {
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
        match target_state[HcoId::Hco3] {
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

        let hco1_is_pwm = matches!(target_state[HcoId::Hco0], State::Pwm(_));
        let hco2_is_pwm = matches!(target_state[HcoId::Hco1], State::Pwm(_));

        self.virtual_timer.enable_update_interrupt(hco1_is_pwm || hco2_is_pwm);
        self.virtual_timer.enable_input_interrupt(Channel::Ch1, hco1_is_pwm);
        self.virtual_timer.enable_input_interrupt(Channel::Ch2, hco2_is_pwm);

        self.state = target_state;
    }
}

impl HcoControllerRev2 {
    /// # Why the software PWM runs on TIM5
    ///
    /// HCO1 (PC0) and HCO2 (PC15) are plain GPIO on this revision — no timer channel reaches
    /// them — so their pulse widths are bit-banged from a timer interrupt. Any timer with two
    /// compare channels can be that time base, and it used to be TIM2.
    ///
    /// TIM2 is now the stepper's, because **TIM2 CH3/CH4 are the only timer channels on PA2/PA3**
    /// and PA2/PA3 are the only COM pins that can carry a hardware pulse train (see
    /// `board::stepper`). Nothing else can do that job; this one can be done by anything. So the
    /// software PWM moved and the stepper got the pins it needs.
    ///
    /// TIM5 rather than TIM1 for two reasons: it is a general-purpose timer with the same
    /// register layout as TIM2, so this is a rename rather than a rewrite; and on the STM32F105
    /// **TIM5 has no pin mappings at all**, so a timer used purely as a time base physically
    /// cannot drive an output by mistake. TIM6/TIM7 are basic timers with no compare channels and
    /// could not do it. TIM4 is embassy's time driver.
    pub async fn new(
        // NOTE: don't change this, since we use raw pac to set this output
        pin1: Peri<'static, p::PC0>,
        // NOTE: don't change this, since we use raw pac to set this output
        pin2: Peri<'static, p::PC15>,
        pin3: Peri<'static, p::PB0>,
        pin4: Peri<'static, p::PB1>,
        virtual_timer: Peri<'static, p::TIM5>,
        out3_4_timer: Peri<'static, p::TIM3>,
        init_state: HcoState,
    ) -> Self {
        let out1 = Output::new(pin1, gpio::Level::Low, Speed::Low);
        let out2 = Output::new(pin2, gpio::Level::Low, Speed::Low);

        // *HCO1_OUT.lock().await = Some(out1);
        // *HCO2_OUT.lock().await = Some(out2);

        let period = Duration::from_hz(50);
        let mut soft_pwm = Timer::new(virtual_timer);
        soft_pwm.set_tick_freq(Hertz::mhz(1));
        soft_pwm.set_max_compare_value((period.as_micros() - 1) as u16);
        soft_pwm.set_autoreload_preload(true);
        soft_pwm.enable_update_interrupt(true);
        soft_pwm.set_output_compare_mode(Channel::Ch1, OutputCompareMode::Frozen);
        soft_pwm.set_compare_value(Channel::Ch1, 1500);
        soft_pwm.set_output_compare_mode(Channel::Ch2, OutputCompareMode::Frozen);
        soft_pwm.set_compare_value(Channel::Ch2, 1500);

        soft_pwm.start();

        embassy_stm32::interrupt::TIM5.unpend();
        unsafe { embassy_stm32::interrupt::TIM5.enable() };

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
            state: init_state,
            out1,
            out2,
            out3: channels.ch3,
            out4: channels.ch4,
            virtual_timer: soft_pwm,
        };
        hco_ctl.set_state(init_state);
        hco_ctl
    }
}

embassy_stm32::bind_interrupts!(struct Irqs {
    TIM5 => SoftPwmHandler;
});

struct SoftPwmHandler;

impl interrupt::typelevel::Handler<interrupt::typelevel::TIM5> for SoftPwmHandler {
    unsafe fn on_interrupt() {
        let timer = embassy_stm32::pac::TIM5;
        let status_regs = timer.sr().read();

        if status_regs.uif() {
            // update interrupt flag is set, meaning timer event has occured

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
