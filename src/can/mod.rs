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
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber, WaitResult};
use heapless::Vec;

use crate::errors::{ErrorCounter, bump_by};

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

/// The next frame for this subscriber, counting anything the fan-out dropped on the way.
///
/// `Subscriber::next_message_pure` does the same thing but throws the lag away, which is how a
/// task that cannot keep up looks identical to one that has nothing to do. The number is worth
/// keeping: it is the difference between "the master went quiet" and "we stopped listening", and
/// those have opposite fixes.
///
/// `lost` is which counter the drop belongs to, because both directions run through the same
/// channel type — [`CanRxSub`] and [`CanTxSub`] are the same subscriber, and only the caller
/// knows whether the frames that went missing were arriving or leaving.
///
/// The lag is counted, not acted on: there is nothing useful to do about frames that are already
/// gone, and stalling here to catch up would only widen the gap.
pub async fn next_frame(sub: &mut CanRxSub, lost: ErrorCounter) -> CanFrame {
    loop {
        match sub.next_message().await {
            WaitResult::Message(frame) => return frame,
            WaitResult::Lagged(n) => {
                bump_by(lost, n.try_into().unwrap_or(u32::MAX));
                defmt::warn!("can: subscriber fell behind, {} frames dropped", n);
            }
        }
    }
}

#[cfg(feature = "hardware")]
mod hw;
#[cfg(feature = "hardware")]
pub use hw::{CAN_IN, CAN_OUT, spawn};
