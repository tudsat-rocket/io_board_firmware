use defmt::{error, info, warn};
use heapless::Vec;

use crate::can::{CanFrame, CanRxSub, CanTxPub};
// use crate::high_current_out::{DigitalOutput, Hco1, Hco2, Hco3, Hco4, ServoOutput};
use crate::utils::anychannel::{AnyReceiver, AnySender};

const NODE_ID: u8 = 10;

pub struct CommandListener<S: AnySender<CanFrame>, R: AnyReceiver<CanFrame>> {
    node_id: u8,
    can: (S, R),
    HcoState,

}

impl<S: AnySender<CanFrame>, R: AnyReceiver<CanFrame>> CommandListener<S, R> {
    pub fn new(can: (S, R), hco1: Hco1, hco2: Hco2, hco3: Hco3, hco4: Hco4) -> Self {
        Self {
            node_id: NODE_ID,
            can,
            hco1,
            hco2,
            hco3,
            hco4,
        }
    }
    async fn listen_loop(&mut self) {
        loop {
            let (id, body) = self.can.1.anyreceive().await;
            let node_id = id & 0x7F;
            info!("command_listener listen_loop, node_id: {}", &node_id);
            if node_id != self.node_id.into() {
                continue;
            }
            if (id & 0x600) == 0x600 {
                // CanOpen SDO receive
                // body length check allows us to unwrap safely later
                if body.len() != 8 {
                    warn!("invalid sdo message received with non 8 byte length, actual={}", body.len());
                    continue;
                }
                info!("body: {:?}", body.as_slice());

                let command_specifier = body.first().unwrap();
                let ccs: u8 = (command_specifier & 0b1110_0000) >> 5;
                let n_empty_data_bytes: u8 = (command_specifier & 0b0000_1100) >> 2;
                let is_exepdited_tranfer: bool = ((command_specifier & 0b0000_0010) >> 1) == 1;
                let _s = command_specifier & 0b0000_0001;
                let dict_index = u16::from_le_bytes([*body.get(1).unwrap(), *body.get(2).unwrap()]);
                let dict_subindex = body.get(3).unwrap();
                if !is_exepdited_tranfer {
                    warn!("unsupported sdo message received, only expedited transfer supported");
                    continue;
                }
                info!("ccs = {}", &ccs);
                match ccs {
                    0 => {
                        warn!("sdo segment download not supported");
                        continue;
                    }
                    1 => {
                        warn!("sdo initiating download not supported");
                        continue;
                    }
                    2 => {
                        // initiate sdo upload
                        let sdo_data: Vec<u8, 4> =
                            Vec::from_slice(&body.as_slice()[4..(4 + 4 - n_empty_data_bytes as usize)]).unwrap(); // TODO: sketchy
                        info!("sdo upload");
                        match dict_index {
                            0x2020 => {
                                // binary high current outputs
                                let Some(level) = sdo_data.get(0) else {
                                    warn!("sdo upload to binary high current output with insufficient data");
                                    continue;
                                };
                                match dict_subindex {
                                    0 => self.hco1.set_level(*level != 0).await,
                                    1 => self.hco2.set_level(*level != 0).await,
                                    2 => self.hco3.set_level(*level != 0).await,
                                    3 => self.hco4.set_level(*level != 0).await,
                                    _ => warn!(
                                        "sdo upload to binary high current output with unassigned subindex {}",
                                        dict_subindex
                                    ),
                                }
                            }
                            0x2021 => {
                                //
                                //duty cycle high current outputs
                                let Some(input) = sdo_data.get(0..2) else {
                                    warn!("sdo upload to duty high current output with insufficient data");
                                    continue;
                                };
                                if !input.len() == 2 {
                                    error!("bug");
                                    continue;
                                }
                                // TODO: sketchy unwrap
                                let duty_micros = u16::from_le_bytes(input.try_into().unwrap());
                                match dict_subindex {
                                    2 => self.hco3.set_duty_micros(duty_micros).await,
                                    3 => self.hco4.set_duty_micros(duty_micros).await,
                                    _ => warn!("sdo upload to duty high current output with unassigned subindex"),
                                }
                            }
                            _ => warn!("sdo upload on dictionary index {} not defined", dict_index),
                        }
                    }
                    3 => {
                        warn!("sdo segment upload not supported");
                        continue;
                    }
                    4 => {
                        warn!("aborting sdo transfer not supported");
                        continue;
                    }
                    5 => {
                        warn!("sdo block upload not supported");
                        continue;
                    }
                    6 => {
                        warn!("sdo block block download not supported");
                        continue;
                    }
                    7 => {
                        warn!("sdo ccs = 7 not supported");
                        continue;
                    }
                    _ => unreachable!("css consists of only 3 bits"),
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn run_command_listener(mut command_listener: CommandListener<CanTxPub, CanRxSub>) {
    command_listener.listen_loop().await;
}
