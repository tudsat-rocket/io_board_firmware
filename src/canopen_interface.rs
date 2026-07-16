use embassy_sync::signal::Signal;
use zencan_common::messages::ZencanMessage;
use zencan_common::{CanMessage, messages::CanId, sdo::SdoRequest};

use defmt::{Debug2Format, error, info, warn};

use crate::board::HcoControl;
use crate::can::{CanFrame, CanRxSub, CanTxPub};
use crate::high_current_out::{HcoController, HighCurrentOutput as Hco, Level, PwmMicros, State};
use crate::store::{CanInterfaceStore, NODE_ID, STORE, StoreWriteError, store_idx::*};
use crate::utils::anychannel::{AnyReceiver, AnySender};
use crate::valves::{self, NUM_SUPPORTED_VALVES, VALVES};

// pub type StoreDirtySigType = Signal<CriticalSectionRawMutex, bool>;
// pub static STORE_DIRTY_SIG: StoreDirtySigType = Signal::new();

pub struct CanOpenInterface<SC: AnySender<CanFrame>, RC: AnyReceiver<CanFrame>, HCO: HcoControl> {
    node_id: u8,
    can: (SC, RC),
    // store_dirty_sig: StoreDirtySigType,
    hco_controller: HCO,
}

/// update store hco to reflect new actual state
pub fn update_store_hco_state(store: &mut CanInterfaceStore, hco_controller: &mut HcoController) {
    let state = hco_controller.get_state();
    for (i, hco_binary_state) in store.hco_binary.iter_mut().enumerate() {
        *hco_binary_state = match state.get_state_0_indexed(i) {
            None => 0,
            Some(State::Digital(l)) => l.as_u8(),
            Some(State::Pwm(pwm)) => pwm.as_u16().clamp(0, 1) as u8,
        }
    }
    for (i, hco_pwm_us) in store.hco_pwm_us.iter_mut().enumerate() {
        *hco_pwm_us = match state.get_state_0_indexed(i) {
            None => 0,
            Some(State::Digital(_)) => 0,
            Some(State::Pwm(pwm)) => pwm.as_u16(),
        }
    }
}

async fn try_write_to_store(
    store: &mut CanInterfaceStore,
    index: u16,
    sub: u8,
    data: &[u8],
    hco_controller: &mut HcoController,
) -> Result<(), StoreWriteError> {
    defmt::debug!("fn try_write_to_store: index: {}, sub: {}", index, sub);
    let sub = sub as usize;
    match index {
        STORE_IDX_RAW_EXT_ADC_BUS0 | STORE_IDX_RAW_EXT_ADC_BUS1 | STORE_IDX_TEMP_SENS | STORE_IDX_PRESSURE_SENS => {
            Err(StoreWriteError::ReadOnly)
        }
        STORE_IDX_VALVES => {
            let Some(entry) = store.valves.get_mut(sub) else {
                return Err(StoreWriteError::SubIndexOutOfRange);
            };
            if data.len() != 2 {
                return Err(StoreWriteError::DataWrongSize);
            }
            *entry = u16::from_le_bytes(data.try_into().unwrap());
            // NOTE: a proper CanOpen system would send an abort msg instead
            *entry = *entry.clamp(&mut 0, &mut 1000);

            // apply valve from store
            let mut vavles = VALVES.lock().await;
            vavles.set_valve(sub, *entry, hco_controller).unwrap();

            update_store_hco_state(store, hco_controller);

            Ok(())
        }
        STORE_IDX_HCO_BINARY => {
            let Some(entry) = store.hco_binary.get_mut(sub) else {
                return Err(StoreWriteError::SubIndexOutOfRange);
            };
            if data.len() != 1 {
                return Err(StoreWriteError::DataWrongSize);
            }
            *entry = data[0];

            // apply hco from store
            let this_output = match sub {
                0 => Hco::_1,
                1 => Hco::_2,
                2 => Hco::_3,
                3 => Hco::_4,
                _ => unreachable!(),
            };
            let level = match entry {
                0 => Level::Low,
                _ => Level::High,
            };
            hco_controller.set_level(this_output, level);

            update_store_hco_state(store, hco_controller);

            Ok(())
        }
        STORE_IDX_HCO_PWM_US => {
            let Some(entry) = store.hco_pwm_us.get_mut(sub) else {
                return Err(StoreWriteError::SubIndexOutOfRange);
            };
            if data.len() != 2 {
                return Err(StoreWriteError::DataWrongSize);
            }
            *entry = u16::from_le_bytes(data.try_into().unwrap());

            // apply hco from store
            let this_output = match sub {
                0 => Hco::_1,
                1 => Hco::_2,
                2 => Hco::_3,
                3 => Hco::_4,
                _ => unreachable!(),
            };

            let pwm = PwmMicros::from_u16_clamped(*entry);
            hco_controller.set_pwm_micros(this_output, pwm.into());

            update_store_hco_state(store, hco_controller);

            Ok(())
        }
        _ => Err(StoreWriteError::IndexNotMapped),
    }
}

impl<SC: AnySender<CanFrame>, RC: AnyReceiver<CanFrame>> CanOpenInterface<SC, RC, HCO> {
    pub fn new(can: (SC, RC), hco_controller: HcoController) -> Self {
        Self {
            node_id: NODE_ID,
            //store_dirty_sig,
            can,
            hco_controller,
        }
    }
    async fn listen_loop(&mut self) {
        defmt::info!("started CanOpenInterface listen loop");
        loop {
            let (cob_id, body) = self.can.1.anyreceive().await;
            defmt::info!("cob_id: {}, body: {}", cob_id, Debug2Format(&body));
            let can_msg = CanMessage::new(CanId::Std(cob_id), body.as_slice());

            let zencan_msg = match ZencanMessage::try_from(can_msg) {
                Ok(msg) => msg,
                Err(e) => {
                    warn!("could not parse message on bus as canopen: {}", Debug2Format(&e));
                    continue;
                }
            };

            match zencan_msg {
                ZencanMessage::SdoRequest(sdo_req) => {
                    info!("sdoRequest: {}", Debug2Format(&sdo_req));
                    // unique read or write to an object in the store
                    if (cob_id - NODE_ID as u16) != 0x600 {
                        // NOTE: this might not cover full canopen spec
                        defmt::debug!("node_id does not match for sdo request");
                        continue;
                    }
                    match sdo_req {
                        // at the moment only writing to the nodes store via simple SdoRequest is
                        // supported
                        SdoRequest::InitiateDownload {
                            // Number of unused bytes in data
                            n,
                            // Expedited
                            e,
                            // size valid
                            s,
                            // Object index
                            index,
                            // Object sub-index
                            sub,
                            // data (value on expedited, size when e=1 and s=1)
                            data,
                        } => {
                            // write data to the store
                            if !e || !s || !(0..=4).contains(&n) {
                                warn!("only expedited (single message) sdo download with length specified supported");
                                continue;
                            }
                            let valid_len = 4 - n as usize;
                            let data = &data[..valid_len];

                            let res = {
                                let mut store = STORE.lock().await;
                                try_write_to_store(&mut store, index, sub, data, &mut self.hco_controller).await
                            };
                            if let Err(e) = res {
                                warn!("write to store was invalid: {}", Debug2Format(&e));
                            }
                        }
                        _ => {
                            warn!("advanved sdo request ({}) not implemented yet", &sdo_req)
                        }
                    };
                }
                // TODO: heartbeat monitor
                ZencanMessage::Heartbeat(_heartbeat) => todo!(),
                // sync not used yet
                ZencanMessage::Sync(_) => {
                    defmt::warn!("canopen sync not implemented");
                }
                // Network management is never used, heartbeat suffices
                ZencanMessage::NmtCommand { .. } => {
                    defmt::warn!("canopen nmt will not be implemented soon");
                }
                ZencanMessage::LssRequest { .. } | ZencanMessage::LssResponse { .. } => {
                    defmt::warn!("canopen lss will not be implemented soon");
                }

                // since we ar not trying to be CanOpen compatiple, won't process sdo response
                ZencanMessage::SdoResponse { .. } => {
                    defmt::warn!("canopen sdo_response is not implemented yet");
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn run_can_command_listener(mut can_open_interface: CanOpenInterface<CanTxPub, CanRxSub, HCO>) {
    can_open_interface.listen_loop().await;
}
