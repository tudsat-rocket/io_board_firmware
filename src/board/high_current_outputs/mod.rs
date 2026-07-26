pub mod types;
pub use types::*;

#[cfg(feature = "rev2")]
mod rev2;
#[cfg(feature = "rev3")]
mod rev3;
#[cfg(feature = "rev2")]
pub use rev2::HcoControllerRev2;
#[cfg(feature = "rev3")]
pub use rev3::HcoControllerRev3;

pub trait HcoControl {
    fn set_level(&mut self, output: HighCurrentOutput, level: Level);
    fn set_pwm_micros(&mut self, output: HighCurrentOutput, micros: u16);

    fn get_state(&self) -> HcoState;
    fn set_state(&mut self, target_state: HcoState);
}
