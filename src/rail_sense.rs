//! Rail current/voltage sensing, abstracted so `crate::control::Control` can be built and tested
//! against mocked hardware.

use crate::index::PerRail;

#[allow(async_fn_in_trait)]
pub trait RailSensing {
    /// A snapshot of rail current/voltage this tick, or `None` if this board has no sensing.
    async fn read(&mut self) -> Option<Rails>;
}

#[derive(Clone, Copy)]
pub struct Rails {
    pub current_ma: PerRail<u16>,
    pub voltage_mv: PerRail<u16>,
}

/// No on-board sensing on rev2
pub struct NoRails;

impl RailSensing for NoRails {
    async fn read(&mut self) -> Option<Rails> {
        None
    }
}
