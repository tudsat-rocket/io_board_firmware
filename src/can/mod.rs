//! The CAN hardware layer, shared by the config plane ([`sdo`]) and the data plane ([`tpdo`]).
//!
//! One receive task fans frames out to subscribers and one transmit task serialises publishers
//! onto the peripheral, with the `cancan` firmware updater tapping both so an A/B flash can
//! happen without the application knowing.
//!
//! Only CAN1 is driven. The board populates a second transceiver on CAN2 with its own connector
//! (see `board/hw.rs`, which binds both sets of interrupts), but nothing needs a second bus yet
//! and a second `Can` instance costs RAM and flash for a peripheral we would only idle.

/// CAN identifier layout and the fixed TPDO table — the wire-level part of the protocol, split
/// out into `iocan-proto` so other devices and tooling can depend on it directly. See
/// `iocan-proto/src/lib.rs`.
pub use iocan_proto::ids;

pub mod sdo;
pub mod tpdo;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use heapless::Vec;

const CAN_QUEUE_SIZE: usize = 32;
const NUM_CAN_SUB: usize = 3;
const NUM_CAN_PUBS: usize = 3;

/// A standard-identifier frame, reduced to what this firmware ever cares about.
pub type CanFrame = (u16, Vec<u8, 8>);

pub type CanRxChannel = PubSubChannel<CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUB, NUM_CAN_PUBS>;
pub type CanRxSub = Subscriber<'static, CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUB, NUM_CAN_PUBS>;
pub type CanRxPub = Publisher<'static, CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUB, NUM_CAN_PUBS>;

pub type CanOutChannel = PubSubChannel<CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUB, NUM_CAN_PUBS>;
pub type CanTxPub = Publisher<'static, CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUB, NUM_CAN_PUBS>;
pub type CanTxSub = Subscriber<'static, CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUB, NUM_CAN_PUBS>;

#[cfg(feature = "hardware")]
mod hw;
#[cfg(feature = "hardware")]
pub use hw::{CAN_IN, CAN_OUT, spawn};
