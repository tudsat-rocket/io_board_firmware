use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

use crate::valves::{self, NUM_SUPPORTED_VALVES, VALVES};

pub static STORE: Mutex<CriticalSectionRawMutex, CanInterfaceStore> = Mutex::new(CanInterfaceStore::new_empty());

// pub const NODE_ID: u8 = 5;
pub const FC_NODE_ID: u8 = 1;

/// See ./device-conf/can-io.toml , but sadly not autogeneration for now
#[derive(Default)]
pub struct CanInterfaceStore {
    pub dirty: bool,
    pub raw_ext_adc_bus0: [u16; 16], // ro
    pub raw_ext_adc_bus1: [u16; 16], // ro
    // unused, this may not be the right approach
    pub temp_sens: [u16; 32], // ro
    // unused, this may not be the right approach
    pub pressure_sens: [u16; 32], // ro
    /// 8 selected Sensors with on board calibration and conversion to proper units
    pub selected_sensors: [u16; 8], // ro
    pub valves: [u16; NUM_SUPPORTED_VALVES], // rw
    pub hco_binary: [u8; 4],      // rw
    pub hco_pwm_us: [u16; 4],     // rw
}
impl CanInterfaceStore {
    pub const fn new_empty() -> Self {
        Self {
            dirty: true,
            raw_ext_adc_bus0: [0; 16],
            raw_ext_adc_bus1: [0; 16],
            temp_sens: [0; 32],
            pressure_sens: [0; 32],
            selected_sensors: [0; 8],
            valves: [0; NUM_SUPPORTED_VALVES],
            hco_binary: [0; 4],
            hco_pwm_us: [0; 4],
        }
    }
}

pub mod store_idx {
    pub const STORE_IDX_RAW_EXT_ADC_BUS0: u16 = 0x2000;
    pub const STORE_IDX_RAW_EXT_ADC_BUS1: u16 = 0x2001;
    pub const STORE_IDX_TEMP_SENS: u16 = 0x2002;
    pub const STORE_IDX_PRESSURE_SENS: u16 = 0x2003;
    pub const STORE_IDX_SELECTED_SENSORS: u16 = 0x2004;
    pub const STORE_IDX_VALVES: u16 = 0x2005;
    pub const STORE_IDX_HCO_BINARY: u16 = 0x2006;
    pub const STORE_IDX_HCO_PWM_US: u16 = 0x2007;
}

#[derive(Debug, Copy, Clone)]
pub enum StoreWriteError {
    IndexNotMapped,
    SubIndexOutOfRange,
    ReadOnly,
    DataWrongSize,
}
