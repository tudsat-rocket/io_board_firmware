pub mod types;
pub use types::*;

mod rev2;
mod rev3;
use rev2::HcoControllerRev2;
use rev3::HcoControllerRev3;

pub trait HcoControl {
    fn set_level(&mut self, output: HighCurrentOutput, level: Level);
    fn set_pwm_micros(&mut self, output: HighCurrentOutput, micros: u16);

    fn get_state(&self) -> HcoState;
    fn set_state(&mut self, target_state: HcoState);
}

/// embassy task functions can't be generic
/// so here is a manual vtable impl
pub enum GenericHcoController {
    Rev2(HcoControllerRev2),
    Rev3(HcoControllerRev3),
}

impl HcoControl for GenericHcoController {
    fn set_level(&mut self, output: HighCurrentOutput, level: Level) {
        match self {
            Self::Rev2(c) => c.set_level(output, level),
            Self::Rev3(c) => c.set_level(output, level),
        }
    }
    fn set_pwm_micros(&mut self, output: HighCurrentOutput, micros: u16) {
        match self {
            Self::Rev2(c) => c.set_pwm_micros(output, micros),
            Self::Rev3(c) => c.set_pwm_micros(output, micros),
        }
    }
    fn get_state(&self) -> HcoState {
        match self {
            Self::Rev2(c) => c.get_state(),
            Self::Rev3(c) => c.get_state(),
        }
    }
    fn set_state(&mut self, target_state: HcoState) {
        match self {
            Self::Rev2(c) => c.set_state(),
            Self::Rev3(c) => c.set_state(),
        }
    }
}
