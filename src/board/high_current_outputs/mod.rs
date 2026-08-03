pub use crate::hco::*;

#[cfg(feature = "rev2")]
mod rev2;
#[cfg(feature = "rev3")]
mod rev3;
#[cfg(feature = "rev2")]
pub use rev2::HcoControllerRev2;
#[cfg(feature = "rev3")]
pub use rev3::HcoControllerRev3;
