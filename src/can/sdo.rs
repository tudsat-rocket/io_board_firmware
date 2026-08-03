//! The config plane: a deliberately small SDO server, plus heartbeat in and out.
//!
//! We borrow CANopen's *framing* — [`zencan_common::sdo`]'s request and response types and its
//! abort codes — so that ordinary CANopen tooling can read and write this node's dictionary,
//! without borrowing CANopen's *machinery*. There is no NMT, no LSS, no SYNC and no runtime PDO
//! mapping: the master is a single fixed peer that needs none of it, and a full `zencan_node`
//! object dictionary does not fit alongside the cancan A/B updater in 114 KiB.
//!
//! Only **expedited** transfers are supported, which is not a limitation in practice: no object
//! in the dictionary is wider than four bytes, precisely so that every config read and write is
//! one frame each way. Segmented and block transfers are refused with `UnsupportedAccess` rather
//! than silently mishandled.

#[cfg(any(feature = "hardware", test))]
use zencan_common::sdo::{AbortCode, SdoRequest, SdoResponse};

#[cfg(feature = "hardware")]
use crate::can::CanFrame;
#[cfg(any(feature = "hardware", test))]
use crate::can::ids::{HEARTBEAT_BASE, SDO_REQUEST_BASE, SDO_RESPONSE_BASE};
#[cfg(any(feature = "hardware", test))]
use crate::can::{CanRxSub, CanTxPub};
#[cfg(any(feature = "hardware", test))]
use crate::safety;
#[cfg(any(feature = "hardware", test))]
use crate::store::{self, CONTROL_WAKE, PERSIST_WAKE, STORE};

/// The config plane's SDO request/response loop. Dual-gated (not just `hardware`) so a host test
/// can drive it against a plain `PubSubChannel` pair with no real CAN hardware — only the
/// `#[embassy_executor::task]` wrappers below (`run_sdo_server`/`run_heartbeat`) need real
/// hardware.
#[cfg(any(feature = "hardware", test))]
pub struct SdoServer {
    node_id: u8,
    rx: CanRxSub,
    tx: CanTxPub,
}

#[cfg(any(feature = "hardware", test))]
impl SdoServer {
    pub fn new(node_id: u8, rx: CanRxSub, tx: CanTxPub) -> Self {
        Self { node_id, rx, tx }
    }

    pub async fn run(&mut self) -> ! {
        loop {
            let (cob_id, body) = self.rx.next_message_pure().await;

            // The master's node id is runtime-configurable, so which heartbeat matters is read
            // from the store rather than fixed at build time.
            let master_id = { STORE.lock().await.config.master_node_id };
            if cob_id == HEARTBEAT_BASE + master_id as u16 {
                safety::note_master_heartbeat();
                continue;
            }

            if cob_id != SDO_REQUEST_BASE + self.node_id as u16 {
                continue;
            }

            let request = match SdoRequest::try_from(body.as_slice()) {
                Ok(request) => request,
                Err(e) => {
                    // The command byte is enough to tell a malformed request from an unsupported one,
                    // and printing it avoids dragging core::fmt in through Debug2Format.
                    let _ = e;
                    defmt::warn!("sdo: undecodable request, command byte {=u8:#04x}", body[0]);
                    continue;
                }
            };

            let response = self.handle(request).await;
            self.reply(response).await;
        }
    }

    async fn handle(&mut self, request: SdoRequest) -> SdoResponse {
        match request {
            SdoRequest::InitiateUpload { index, sub } => {
                let value = {
                    let store = STORE.lock().await;
                    store::read(&store, index, sub)
                };
                match value {
                    Ok(value) => SdoResponse::expedited_upload(index, sub, value.data()),
                    Err(code) => {
                        defmt::debug!("sdo: read {=u16:#06x}.{} aborted {=u32:#010x}", index, sub, code as u32);
                        SdoResponse::abort(index, sub, code)
                    }
                }
            }

            SdoRequest::InitiateDownload {
                n,
                e,
                s,
                index,
                sub,
                data,
            } => {
                if !e {
                    // Segmented download: the dictionary has nothing that needs it.
                    return SdoResponse::abort(index, sub, AbortCode::UnsupportedAccess);
                }
                // `n` counts the unused bytes only when the size is flagged valid; without the
                // flag we cannot know the width, and guessing would let a 1-byte write land in a
                // 2-byte object.
                if !s {
                    return SdoResponse::abort(index, sub, AbortCode::DataTypeMismatch);
                }
                if n > 3 {
                    return SdoResponse::abort(index, sub, AbortCode::DataTypeMismatchLengthLow);
                }

                let payload = &data[..4 - n as usize];
                let result = {
                    let mut store = STORE.lock().await;
                    let result = store::write(&mut store, index, sub, payload);
                    if result.is_ok() {
                        if store.pending.valves.any() || store.pending.outputs || store.pending.config {
                            CONTROL_WAKE.signal(());
                        }
                        if store.pending.save || store.pending.restore {
                            PERSIST_WAKE.signal(());
                        }
                    }
                    result
                };

                match result {
                    Ok(()) => SdoResponse::download_acknowledge(index, sub),
                    Err(code) => {
                        defmt::warn!("sdo: write {=u16:#06x}.{} rejected, abort {=u32:#010x}", index, sub, code as u32);
                        SdoResponse::abort(index, sub, code)
                    }
                }
            }

            // Block and segmented transfers, and anything else the framing can express.
            other => {
                let _ = other;
                defmt::warn!("sdo: only expedited transfers are supported");
                SdoResponse::abort(0, 0, AbortCode::UnsupportedAccess)
            }
        }
    }

    async fn reply(&mut self, response: SdoResponse) {
        let cob_id = SDO_RESPONSE_BASE + self.node_id as u16;
        let Ok(body) = heapless::Vec::from_slice(&response.to_bytes()) else {
            defmt::error!("sdo: response did not fit a frame, dropping");
            return;
        };
        self.tx.publish((cob_id, body)).await;
    }
}

#[cfg(feature = "hardware")]
#[embassy_executor::task]
pub async fn run_sdo_server(mut server: SdoServer) -> ! {
    server.run().await
}

/// Emit our own heartbeat so the master can tell a silent node from an absent one.
///
/// The payload is the CANopen NMT state byte; we always report 0x05 ("operational") because this
/// node has no other state to be in — it is either running or it is not on the bus.
#[cfg(feature = "hardware")]
#[embassy_executor::task]
pub async fn run_heartbeat(node_id: u8, tx: CanTxPub) -> ! {
    const OPERATIONAL: u8 = 0x05;
    let cob_id = HEARTBEAT_BASE + node_id as u16;
    loop {
        let period = { STORE.lock().await.config.heartbeat_period_ms };
        if period == 0 {
            // Disabled: check back occasionally in case it is re-enabled over SDO.
            embassy_time::Timer::after_millis(1000).await;
            continue;
        }
        let body: CanFrame = (cob_id, heapless::Vec::from_slice(&[OPERATIONAL]).unwrap());
        tx.publish(body).await;
        embassy_time::Timer::after_millis(period as u64).await;
    }
}
