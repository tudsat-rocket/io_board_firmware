use defmt::{Debug2Format, error, info};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::can::{Can, CanRx, CanTx, Fifo, Frame, Id, StandardId, filter};
use embassy_stm32::flash::{Blocking, Flash};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, with_timeout};

use cancan::{CHANNEL_DEPTH, CanCan, CanCanRx, CanCanTx};
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use heapless::Vec;
use static_cell::StaticCell;

// pub mod canopen;

const CAN_QUEUE_SIZE: usize = 5;
const NUM_CAN_SUB: usize = 2;
const NUM_CAN_PUBS: usize = 2;

pub type CanFrame = (u16, Vec<u8, 8>);

pub type CanRxChannel = PubSubChannel<CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUB, NUM_CAN_PUBS>;
pub type CanRxSub = Subscriber<'static, CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUB, NUM_CAN_PUBS>;
pub type CanRxPub = Publisher<'static, CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUB, NUM_CAN_PUBS>;

pub type CanOutChannel = PubSubChannel<CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUB, NUM_CAN_PUBS>;
pub type CanTxPub = Publisher<'static, CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUB, NUM_CAN_PUBS>;
pub type CanTxSub = Subscriber<'static, CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUB, NUM_CAN_PUBS>;

pub static CAN_IN: StaticCell<CanRxChannel> = StaticCell::new();
pub static CAN_OUT: StaticCell<CanOutChannel> = StaticCell::new();

static CAN: StaticCell<Can<'static>> = StaticCell::new();
static CAN_TX: StaticCell<CanTx<'static>> = StaticCell::new();
static CAN_RX: StaticCell<CanRx<'static>> = StaticCell::new();

pub async fn spawn(
    mut can: Can<'static>,
    cancan: &mut CanCan<Flash<'static, Blocking>>,
    spawner: Spawner,
    publisher: CanRxPub,
    subscriber: CanTxSub,
) {
    info!("Can task spawner activated");
    can.modify_config().set_loopback(false).set_silent(false).set_automatic_retransmit(true);
    can.set_bitrate(125_000);
    let catch_all = filter::Mask32::accept_all();
    can.modify_filters().enable_bank(0, Fifo::Fifo0, catch_all).enable_bank(1, Fifo::Fifo1, catch_all);
    can.enable().await;
    let is_sleeping = can.is_sleeping();
    if is_sleeping {
        info!("Was was set up but is sleeping");
    } else {
        info!("Was was set up and is awake");
    }

    // After a CAN-based flash, we use the polling of the flashing CLI as a health
    // check for the CAN connection, and rollback if that doesn't work.
    if cancan.is_unconfirmed().unwrap_or(false) {
        let mut linked = false;
        for _ in 0..20 {
            // TODO: enable once watchdog is up
            // watchdog.pet();
            if with_timeout(Duration::from_millis(250), can.read()).await.is_ok() {
                linked = true;
                break;
            }
        }

        assert!(linked, "cancan: no CAN traffic in the confirm window");
        // watchdog.pet();

        cancan.confirm().unwrap();
    }

    let can = CAN.init(can);
    let (can_tx, can_rx) = can.split();
    let can_tx = CAN_TX.init(can_tx);
    let can_rx = CAN_RX.init(can_rx);

    let (cancan_rx, cancan_tx) = crate::CANCAN.split();

    info!("spawning...");
    spawner.spawn(run_tx(can_tx, cancan_tx, subscriber).unwrap());
    spawner.spawn(run_rx(can_rx, cancan_rx, publisher).unwrap());
}

#[embassy_executor::task]
async fn run_tx(
    can_tx: &'static mut CanTx<'static>,
    cancan_tx: CanCanTx<'static, CriticalSectionRawMutex, Frame, CHANNEL_DEPTH>,
    mut subscriber: CanTxSub,
) -> ! {
    info!("Can TX task started.");
    loop {
        let frame = match select(cancan_tx.recv(), subscriber.next_message_pure()).await {
            Either::First(frame) => frame,
            Either::Second((address, data)) => {
                let Some(sid) = StandardId::new(address) else {
                    defmt::warn!("Invalid CAN ID: {}", address);
                    continue;
                };

                let Ok(frame) = Frame::new_data(sid, &data) else {
                    defmt::warn!("Invalid frame.");
                    continue;
                };

                frame
            }
        };
        can_tx.write(&frame).await;
    }
}

#[embassy_executor::task]
async fn run_rx(
    can_rx: &'static mut CanRx<'static>,
    cancan_rx: CanCanRx<'static, CriticalSectionRawMutex, Frame, CHANNEL_DEPTH>,
    publisher: CanRxPub,
) -> ! {
    info!("Can rx task started");
    loop {
        match can_rx.read().await {
            Ok(envelope) => {
                let frame = envelope.frame;

                if cancan_rx.claim(&frame).await {
                    continue;
                }

                // let Some(data) = frame.data() else {
                //     continue;
                // };
                // TODO: (embassy upgrade)
                let data = frame.data();

                let Id::Standard(sid) = frame.id() else {
                    // EID package, skipping.
                    continue;
                };
                let id_raw = sid.as_raw();

                let Ok(data_array) = data.as_ref().try_into() else {
                    // frame wasn't 8 bytes long, skip.
                    continue;
                };

                publisher.publish_immediate((id_raw, data_array));
            }
            Err(e) => {
                // error!("can_rx: Failed to read envelope: {:?}", Debug2Format(&e))
            }
        }
    }
}

// pub fn configure(can: &mut Can, role: IoBoardRole) {
//     let command_prefix = CanBusMessageId::IoBoardCommand(role, 0).into();
//     let command_filter =
//         filter::Mask32::frames_with_std_id(StandardId::new(command_prefix).unwrap(), StandardId::new(0x7f0).unwrap());
//
//     let telemetry_filter = filter::Mask32::frames_with_std_id(
//         StandardId::new(CanBusMessageId::TelemetryBroadcast(0).into()).unwrap(),
//         StandardId::new(0x700).unwrap(),
//     );
//
//     can.modify_filters()
//         .enable_bank(0, Fifo::Fifo0, command_filter)
//         .enable_bank(1, Fifo::Fifo1, telemetry_filter);
// }
