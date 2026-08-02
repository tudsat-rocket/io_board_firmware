use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::flash::{Blocking, Flash};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, with_timeout};

use embassy_stm32::can::{BufferedCanRx, Can, CanTx, Fifo, Frame, Id, RxBuf, StandardId, filter};
use embassy_sync::channel::Channel;

use cancan::{CHANNEL_DEPTH, CanCan, CanCanRx, CanCanTx};
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use heapless::Vec;
use static_cell::StaticCell;

const CAN_QUEUE_SIZE: usize = 20;
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

const CAN_RX_BUF_SIZE: usize = 32;
static CAN_RX_BUF: StaticCell<RxBuf<CAN_RX_BUF_SIZE>> = StaticCell::new();

pub async fn spawn(
    mut can: Can<'static>,
    cancan: &mut CanCan<Flash<'static, Blocking>>,
    spawner: Spawner,
    publisher: CanRxPub,
    subscriber: CanTxSub,
) {
    info!("Can task spawner activated");
    can.modify_config().set_loopback(false).set_silent(false);
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

    let can = CAN.init(can);
    let (can_tx, can_rx) = can.split();
    let can_tx = CAN_TX.init(can_tx);
    let mut can_rx = can_rx.buffered(CAN_RX_BUF.init(Channel::new()));

    // After a CAN-based flash, we use traffic on the bus as a health check for the CAN
    // connection, and rollback if that doesn't work.
    //
    // This has to run on the *buffered* receiver, after the split above. In non-buffered
    // mode the RX interrupt disables FMPIE for the FIFO it just filled and relies on a
    // completing `try_read` to switch it back on, and `BufferedCanRx::setup` never touches
    // FMPIE. A `Can::read` dropped by `with_timeout` just after a frame arrived therefore
    // leaves that FIFO's interrupt off permanently - and since the filters spread traffic
    // over FIFO0 and FIFO1, the node goes deaf to part of the bus for the rest of the boot.
    match cancan.is_unconfirmed() {
        Ok(true) => {
            let mut linked = false;
            for _ in 0..20 {
                crate::board::pet_watchdog();
                // A bus error is not a healthy link, so require an actual frame.
                if let Ok(Ok(_)) = with_timeout(Duration::from_millis(250), can_rx.read()).await {
                    linked = true;
                    break;
                }
            }

            if linked {
                // `confirm` erases and writes a flash page without awaiting.
                crate::board::pet_watchdog();
                match cancan.confirm() {
                    Ok(prior) => info!("cancan: image confirmed, prior state {}", defmt::Debug2Format(&prior)),
                    Err(e) => defmt::error!("cancan: confirm failed: {}", defmt::Debug2Format(&e)),
                }
            } else {
                // Deliberately keep running: the image stays unconfirmed, so the bootloader
                // reverts on the next reset, but until then the node is still commandable and
                // this line is visible. Resetting here instead would revert before anyone can
                // see why, and loops if the bus is quiet for an unrelated reason (bench work
                // over SWD, every other node still booting).
                defmt::error!(
                    "cancan: no CAN traffic in the confirm window, leaving image unconfirmed - \
                     the bootloader will revert to the previous image on the next reset"
                );
            }
        }
        Ok(false) => {}
        // Don't silently treat a flash error as "already confirmed".
        Err(e) => defmt::error!("cancan: could not read boot state: {}", defmt::Debug2Format(&e)),
    }

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
        can_tx.flush_any().await;
        if let Some(dropped) = can_tx.write(&frame).await.dequeued_frame()
            && let Id::Standard(sid) = dropped.id()
        {
            defmt::warn!("can_tx: evicted pending frame {=u16:#05x}, this should not happen", sid.as_raw());
        }
    }
}

#[embassy_executor::task]
async fn run_rx(
    mut can_rx: BufferedCanRx<'static, CAN_RX_BUF_SIZE>,
    cancan_rx: CanCanRx<'static, CriticalSectionRawMutex, Frame, CHANNEL_DEPTH>,
    publisher: CanRxPub,
) -> ! {
    info!("Can rx task started");
    loop {
        use embassy_stm32::pac::CAN1;
        if CAN1.rfr(0).read().fovr() {
            defmt::error!("bxCAN RX FIFO0 overrun");
            CAN1.rfr(0).modify(|v| v.set_fovr(true));
        }
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
                //TODO: ratelimiting
                defmt::error!("can_rx: Failed to read envelope: {:?}", defmt::Debug2Format(&e))
            }
        }
    }
}
