//! Wire protocol for the ioCan CAN bus: identifier layout and TPDO frame encode/decode.
//!
//! `no_std`, no heap, and the only dependency (`defmt`) is optional and only affects logging —
//! nothing here needs it to encode or decode a frame. Anything that talks to an ioCan node, or
//! just wants to sniff its bus traffic, can depend on this crate alone rather than pulling in
//! the firmware.
//!
//! The protocol itself — which fields live in which frame, timeouts, node id assignment, etc —
//! is defined by the vehicle's `device-conf/can-io.toml`; this crate is the Rust-typed mirror of
//! the parts of it that actually travel on the bus as fixed process data. Object-dictionary
//! semantics that never leave a node (what `ValveStatus::Stalled` means, how a relief pulse is
//! timed, ...) stay in the firmware; what's here is strictly "given this identifier and these 8
//! bytes, what do they mean."
#![cfg_attr(not(test), no_std)]

pub mod ids;
pub mod tpdo;

pub use ids::{TPDO_KINDS, TpdoKind, decode_pdo};
pub use tpdo::{HcoOutput, TpdoFrame};
