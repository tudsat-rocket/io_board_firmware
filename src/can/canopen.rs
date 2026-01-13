use core::error;

use defmt::{Debug2Format, debug, error, warn};
use embassy_stm32::can::frame;
use embassy_time::{Duration, Instant, Timer};
use heapless::{Vec, deque::Deque, vec::VecInner};
use zencan_common::CanMessage;

use crate::{can, zencan};

use super::CanFrame;

#[embassy_executor::task]
pub async fn run_zencan(mut node: zencan_node::Node, mut can_rx_sub: can::CanRxSub, mut can_tx_pub: can::CanTxPub) {
    // This extra buffer is necessary because the zencan callback does not support async
    let mut send_queue: Deque<zencan_common::CanMessage, 8> = Deque::new();

    loop {
        let frame: can::CanFrame = can_rx_sub.next_message_pure().await;
        {
            let disp_id = frame.0;
            let disp_body = frame.1.as_slice();
            debug!("received can frame: id: {:02x}, body: {:02x} ", disp_id, disp_body);
        }
        if let Err(e) = zencan::NODE_MBOX.store_message(to_message(frame.clone())) {
            defmt::debug!("Ignoring CanOpen Message: {}", Debug2Format(&e));
        }

        let mut send_cb = |msg: zencan_common::CanMessage| {
            // NOTE: check verify dropping messages is impossible
            let _ = send_queue.push_front(msg);
        };
        node.process(Instant::now().as_micros(), &mut send_cb);
        if send_queue.is_full() {
            error!("zencan tried to send more messages ({}) than allowed at once", send_queue.len());
        }
        // publish messages in buffer
        while let Some(msg) = send_queue.pop_back() {
            if let Some(frame) = from_message(msg) {
                can_tx_pub.publish_immediate(frame);
            } else {
                error!("CanOpen tried to publish non standard message, dropping");
            }
        }
        // quick and dirty message to led task
    }
}

fn to_message(value: CanFrame) -> zencan_common::CanMessage {
    zencan_common::CanMessage::new(zencan_common::CanId::std(value.0), value.1.as_slice())
}
fn from_message(value: zencan_common::CanMessage) -> Option<CanFrame> {
    let zencan_common::CanId::Std(id) = value.id() else {
        warn!("Tried to convert from 29bit CanMessage id into 11bit");
        return None;
    };
    if value.data().len() > 8 {
        warn!("Error when converting CanMessage data to long");
        return None;
    }

    Some((id, Vec::from_slice(value.data()).unwrap()))
}
