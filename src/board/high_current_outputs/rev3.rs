use core::sync::atomic::{AtomicU16, Ordering};
use defmt::{Debug2Format, error};

use embassy_stm32::{
    Peri,
    gpio::{self, AfioRemap, Output, OutputType, Speed},
    interrupt::{self, InterruptExt},
    peripherals as p,
    time::Hertz,
    timer::{
        Ch1, Ch2, Ch3, Ch4, Channel, TimerPin,
        low_level::{CountingMode, OutputCompareMode, Timer},
        simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel},
    },
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use super::HcoControl;
use super::types::*;

/// High current output controller for IO board rev3.
pub struct HcoControllerRev3 {
    state: HcoState,
    // out1: Output<'static, super::super::pins_rev3::HC_OUT_1>,
    out1: SimplePwmChannel<'static, p::TIM3>, //ch2
    out2: SimplePwmChannel<'static, p::TIM1>, //ch1
    out3: SimplePwmChannel<'static, p::TIM3>, // ch3
    out4: SimplePwmChannel<'static, p::TIM3>, // ch4
}

impl HcoControl for HcoControllerRev3 {
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
        self.state.clone()
    }
    fn set_state(&mut self, target_state: HcoState) {
        // if self.state == target_state {
        //     return;
        // }
        self.state = target_state;
        match self.state._1 {
            State::Digital(Level::High) => self.out1.set_duty_cycle_fully_on(),
            State::Digital(Level::Low) => self.out1.set_duty_cycle_fully_off(),
            State::Pwm(duty) => {
                let num = (u32::from(duty.as_u16()) * 5) / 10;
                self.out1.set_duty_cycle_fraction(num, 10_000);
            }
        };
        match self.state._2 {
            State::Digital(Level::High) => self.out2.set_duty_cycle_fully_on(),
            State::Digital(Level::Low) => self.out2.set_duty_cycle_fully_off(),
            State::Pwm(duty) => {
                let num = (u32::from(duty.as_u16()) * 5) / 10;
                self.out2.set_duty_cycle_fraction(num, 10_000);
            }
        };
        match self.state._3 {
            State::Digital(Level::High) => self.out3.set_duty_cycle_fully_on(),
            State::Digital(Level::Low) => self.out3.set_duty_cycle_fully_off(),
            State::Pwm(duty) => {
                let num = (u32::from(duty.as_u16()) * 5) / 10;
                self.out3.set_duty_cycle_fraction(num, 10_000);
            }
        }
        match self.state._4 {
            State::Digital(Level::High) => self.out4.set_duty_cycle_fully_on(),
            State::Digital(Level::Low) => self.out4.set_duty_cycle_fully_off(),
            State::Pwm(duty) => {
                let num = (u32::from(duty.as_u16()) * 5) / 10;
                self.out4.set_duty_cycle_fraction(num, 10_000);
            }
        }
    }
}

impl HcoControllerRev3 {
    pub async fn new(
        pin1: Peri<'static, super::super::pins_rev3::HC_OUT_1>,
        pin2: Peri<'static, super::super::pins_rev3::HC_OUT_2>,
        pin3: Peri<'static, super::super::pins_rev3::HC_OUT_3>,
        pin4: Peri<'static, super::super::pins_rev3::HC_OUT_4>,
        timer_out2: Peri<'static, p::TIM1>,
        timer_out134: Peri<'static, p::TIM3>,
        init_state: HcoState,
    ) -> Self {
        // out 1, 3, 4
        <p::PB0 as TimerPin<p::TIM3, Ch3, AfioRemap<0>>>::afio_remap(&pin3);
        <p::PB1 as TimerPin<p::TIM3, Ch4, AfioRemap<0>>>::afio_remap(&pin4);
        let out1: PwmPin<'_, p::TIM3, Ch2, AfioRemap<0>> = PwmPin::new(pin1, OutputType::PushPull);
        let out3: PwmPin<'_, p::TIM3, Ch3, AfioRemap<0>> = PwmPin::new(pin3, OutputType::PushPull);
        let out4: PwmPin<'_, p::TIM3, Ch4, AfioRemap<0>> = PwmPin::new(pin4, OutputType::PushPull);
        let pwm134 = SimplePwm::new(
            timer_out134,
            None,
            Some(out1),
            Some(out3),
            Some(out4),
            Hertz::hz(50),
            CountingMode::EdgeAlignedUp,
        );
        let mut channels134 = pwm134.split();
        channels134.ch2.enable();
        channels134.ch3.enable();
        channels134.ch4.enable();

        // out2
        let out2: PwmPin<'_, p::TIM1, Ch1, AfioRemap<0>> = PwmPin::new(pin2, OutputType::PushPull);
        let pwm2 = SimplePwm::new(timer_out2, Some(out2), None, None, None, Hertz::hz(50), CountingMode::EdgeAlignedUp);
        let mut channels2 = pwm2.split();
        channels2.ch1.enable();

        let mut ctl = Self {
            state: init_state.clone(),
            out1: channels134.ch2,
            out2: channels2.ch1,
            out3: channels134.ch3,
            out4: channels134.ch4,
        };
        ctl.set_state(init_state);
        ctl
    }
}
