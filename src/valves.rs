use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

use crate::board::high_current_outputs::{GenericHcoController, HcoControl, HighCurrentOutput, Level};

// TODO: gather this somewhere global
pub static VALVES: Mutex<CriticalSectionRawMutex, Valves> = Mutex::new(Valves([
    None,
    None,
    None,
    Some(ValveEntry {
        target_state_promille: 0,
        kind: Valve::Servo(ServoValve {
            power_con: Some(HighCurrentOutput::_3),
            pwm_con: HighCurrentOutput::_4,
            calib: ServoValveCalib {
                open_us: 1000,
                closed_us: 2000,
            },
        }),
    }),
]));

pub const NUM_SUPPORTED_VALVES: usize = 4;

pub struct SolenoidVavle {
    pub con: HighCurrentOutput,
}

pub struct ServoValve {
    pub power_con: Option<HighCurrentOutput>,
    pub pwm_con: HighCurrentOutput,
    pub calib: ServoValveCalib,
}

pub struct ValveEntry {
    kind: Valve,
    target_state_promille: u16,
}

pub struct ServoValveCalib {
    open_us: u16,
    closed_us: u16,
}

impl ServoValveCalib {
    pub fn open_promille_to_pwm_us(&self, open_promille: u16) -> u16 {
        // Clamp input to valid promille range [0, 1000]
        let promille = open_promille.min(1000) as i32;

        let open_us = self.open_us as i32;
        let closed_us = self.closed_us as i32;

        // Linear interpolation: closed_us + (open_us - closed_us) * promille / 1000
        // Works correctly even if open_us < closed_us (negative delta).
        let delta = open_us - closed_us;
        let pwm_us = closed_us + (delta * promille) / 1000;

        pwm_us as u16
    }
}

pub enum Valve {
    Solenoid(SolenoidVavle),
    Servo(ServoValve),
}

pub struct Valves(pub [Option<ValveEntry>; NUM_SUPPORTED_VALVES]);

impl Valves {
    /// Valve_num must be within 0..NUM_SUPPORTED_VALVES
    pub fn set_valve(
        &mut self,
        valve_num: usize,
        open_promille: u16,
        hco_controler: &mut GenericHcoController,
    ) -> Result<(), ()> {
        if valve_num >= self.0.len() {
            defmt::error!("bug: set_valve was called with valve out of range");
            return Err(());
        }
        let Some(ref mut v) = self.0[valve_num] else {
            defmt::warn!("tried to set vavle (num: {}), that has not defined hco output", valve_num);
            return Err(());
        };
        if !(0..=1000).contains(&open_promille) {
            defmt::error!("tried to set valve with out of range target");
            return Err(());
        }
        v.target_state_promille = open_promille;

        match &v.kind {
            Valve::Solenoid(v_kind) => {
                let binary_level = match open_promille {
                    0 => Level::Low,
                    _ => Level::High,
                };
                hco_controler.set_level(v_kind.con, binary_level);
            }
            Valve::Servo(v_kind) => {
                defmt::info!("setting servo valve");
                if let Some(power_con) = v_kind.power_con {
                    hco_controler.set_level(power_con, Level::High);
                }
                let micros = v_kind.calib.open_promille_to_pwm_us(open_promille);

                hco_controler.set_pwm_micros(v_kind.pwm_con, micros);

                if let Some(power) = v_kind.power_con {
                    hco_controler.set_level(power, Level::High);
                }
            }
        };
        Ok(())
    }
}
