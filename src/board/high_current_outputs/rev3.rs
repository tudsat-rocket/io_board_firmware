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
use embassy_time::Duration;

use super::HcoState;
use super::types::*;

/// Represents the current state of the high current outputs. Use [`HcoController`] to change the
/// state.
// This needs to be implemented via Mutex, because the interrupt service routine needs access to
// the state.

/// High current output controller for IO board rev2.
pub struct HcoControllerRev3 {
    state: HcoState,
    // out1: Output<'static, super::super::pins_rev3::HC_OUT_1>,
    out1: SimplePwmChannel<'static, p::TIM3>, //ch2
    out2: SimplePwmChannel<'static, p::TIM1>, //ch1
    out3: SimplePwmChannel<'static, p::TIM3>, // ch3
    out4: SimplePwmChannel<'static, p::TIM3>, // ch4
}

// impl HcoControl for HcoControllerRev2 {
//     fn set_level(&mut self, output: HighCurrentOutput, level: Level) {
//         let mut new_state = self.get_state();
//         new_state.set_level(output, level);
//         self.set_state(new_state);
//     }
//
//     fn set_pwm_micros(&mut self, output: HighCurrentOutput, micros: u16) {
//         defmt::info!("{} set pwm us: {}", Debug2Format(&output), micros);
//         let mut new_state = self.get_state();
//         new_state.set_pwm_micros(output, micros);
//         self.set_state(new_state);
//     }
//     fn get_state(&self) -> HcoState {
//         *self.state_mutex.try_lock().unwrap()
//     }
//     fn set_state(&mut self, target_state: HcoState) {
//         *self.state_mutex.try_lock().unwrap() = target_state;
//         match target_state._1 {
//             State::Digital(ref level) => {
//                 self.virtual_timer.enable_input_interrupt(Channel::Ch1, false);
//                 match target_state._2 {
//                     State::Digital(_) => self.virtual_timer.enable_update_interrupt(false),
//                     State::Pwm(_) => (),
//                 };
//                 self.out1.try_lock().unwrap().as_mut().map(|o| o.set_level(gpio::Level::from(*level)));
//             }
//             State::Pwm(duty) => {
//                 self.virtual_timer.enable_update_interrupt(true);
//                 self.virtual_timer.enable_input_interrupt(Channel::Ch1, true);
//                 PULSE_US_PWM1.store(duty.into(), Ordering::Relaxed);
//             }
//         };
//         match target_state._2 {
//             State::Digital(ref level) => {
//                 self.virtual_timer.enable_input_interrupt(Channel::Ch2, false);
//                 match target_state._1 {
//                     State::Digital(_) => self.virtual_timer.enable_update_interrupt(false),
//                     State::Pwm(_) => (),
//                 };
//                 self.out2.try_lock().unwrap().as_mut().map(|o| o.set_level(gpio::Level::from(*level)));
//             }
//             State::Pwm(duty) => {
//                 self.virtual_timer.enable_update_interrupt(true);
//                 self.virtual_timer.enable_input_interrupt(Channel::Ch2, true);
//                 PULSE_US_PWM2.store(duty.into(), Ordering::Relaxed);
//             }
//         };
//         match target_state._3 {
//             State::Digital(level) => match level {
//                 Level::High => self.out3.set_duty_cycle_fully_on(),
//                 Level::Low => self.out3.set_duty_cycle_fully_off(),
//             },
//             State::Pwm(duty) => {
//                 let micros = duty.as_u16().clamp(500, 2500);
//                 let num = (micros * 5) / 10;
//                 self.out3.set_duty_cycle_fraction(num, 10_000);
//             }
//         }
//         match target_state._4 {
//             State::Digital(level) => match level {
//                 Level::High => self.out4.set_duty_cycle_fully_on(),
//                 Level::Low => self.out4.set_duty_cycle_fully_off(),
//             },
//             State::Pwm(duty) => {
//                 let micros = duty.as_u16().clamp(500, 2500);
//                 let num = (micros * 5) / 10;
//                 self.out4.set_duty_cycle_fraction(num, 10_000);
//             }
//         }
//     }
// }

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

        Self {
            state: init_state,
            out1: channels134.ch2,
            out2: channels2.ch1,
            out3: channels134.ch3,
            out4: channels134.ch4,
        }
    }
}
