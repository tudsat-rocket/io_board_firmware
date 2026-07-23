///! Transfer process data object - broadcast data regularly
use embassy_time::{Duration, Ticker};

use embassy_executor::Spawner;
use embassy_futures::select::select_array;
use heapless::Vec;

use crate::{
    can::{CanFrame, CanTxPub},
    store::STORE,
};

pub struct TpdoIntervals {
    valves: Option<Duration>,
    binary_outpus: Option<Duration>,
    pwm_us: Option<Duration>,
    raw_bus0a: Option<Duration>,
    raw_bus0b: Option<Duration>,
    raw_bus1a: Option<Duration>,
    raw_bus1b: Option<Duration>,
    sensor0: Option<Duration>,
    sensor1: Option<Duration>,
}

impl TpdoIntervals {
    pub const fn default() -> Self {
        Self {
            valves: Some(Duration::from_millis(500)),
            binary_outpus: Some(Duration::from_millis(500)),
            pwm_us: Some(Duration::from_millis(500)),
            raw_bus0a: Some(Duration::from_millis(100)),
            raw_bus0b: None,
            raw_bus1a: Some(Duration::from_millis(100)),
            raw_bus1b: None,
            sensor0: Some(Duration::from_millis(50)),
            sensor1: None,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum PdMessageKind {
    // NOTE: maybe have a look at num_enum: TryFromPrimitive
    /// holds 4 ValveStates (promille open) as le u16
    Valves,
    /// holds 4 bools as le u16
    BinaryOutputs,
    /// holds 4 pwm microseconds entries as le u16
    PwmUs,
    /// holds fist 4 raw adc measurements of i2c bus 0
    RawBus0a,
    /// holds second 4 raw adc measurements of i2c bus 0
    RawBus0b,
    /// holds first 4 raw adc measurements of i2c bus 1
    RawBus1a,
    /// holds second 4 raw adc measurements of i2c bus 1
    RawBus1b,
    /// holds first 4 preprocessed sensor values as u16 or i16
    /// temp(i16): centi celcius, pressure(u16): kilo pascal
    Sensor0,
    /// holds second 4 preprocessed sensor values as u16 or i16
    /// temp(i16): centi celcius, pressure(u16): kilo pascal
    Sensor1,
}
impl TryFrom<u8> for PdMessageKind {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, ()> {
        use PdMessageKind as K;
        match value {
            0 => Ok(K::Valves),
            1 => Ok(K::BinaryOutputs),
            2 => Ok(K::PwmUs),
            3 => Ok(K::RawBus0a),
            4 => Ok(K::RawBus0b),
            5 => Ok(K::RawBus1a),
            6 => Ok(K::RawBus1b),
            7 => Ok(K::Sensor0),
            8 => Ok(K::Sensor1),
            _ => Err(()),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub struct ProcessDataCanId {
    pub node_id: u8,
    pub kind: PdMessageKind,
}
impl From<ProcessDataCanId> for u16 {
    fn from(value: ProcessDataCanId) -> Self {
        (value.node_id as u16 & 0b1111) | (((value.kind as u16) << 4) & 0b111_1111_0000)
    }
}

impl TryFrom<u16> for ProcessDataCanId {
    type Error = ();
    fn try_from(value: u16) -> Result<Self, ()> {
        // 0d512 = 2^9
        if !(0x1800..0x1A00).contains(&value) {
            return Err(());
        }
        let identifier: u16 = (value >> 4) & 0b111_1111;
        let kind = PdMessageKind::try_from(identifier as u8);
        let Ok(kind) = kind else {
            return Err(());
        };

        Ok(Self {
            node_id: (value & 0b1111) as u8,
            kind,
        })
    }
}
// /// Fixed ordering - index into this array must match the order the
// /// tickers are placed into `select_array` below.
const PD_MSGS: [PdMessageKind; 9] = [
    PdMessageKind::Valves,
    PdMessageKind::BinaryOutputs,
    PdMessageKind::PwmUs,
    PdMessageKind::RawBus0a,
    PdMessageKind::RawBus0b,
    PdMessageKind::RawBus1a,
    PdMessageKind::RawBus1b,
    PdMessageKind::Sensor0,
    PdMessageKind::Sensor1,
];

fn cob_id_from(pd_msg_kind: PdMessageKind, node_id: u8) -> u16 {
    let id = ProcessDataCanId {
        node_id,
        kind: pd_msg_kind,
    };
    u16::from(id)
}

/// Holds one `Ticker` per channel; `None` means that channel is disabled.
struct Tickers {
    valves: Option<Ticker>,
    binary_outputs: Option<Ticker>,
    pwm_us: Option<Ticker>,
    raw_bus0a: Option<Ticker>,
    raw_bus0b: Option<Ticker>,
    raw_bus1a: Option<Ticker>,
    raw_bus1b: Option<Ticker>,
    sensor0: Option<Ticker>,
    sensor1: Option<Ticker>,
}

impl Tickers {
    fn new(settings: &TpdoIntervals) -> Self {
        Self {
            valves: settings.valves.map(Ticker::every),
            binary_outputs: settings.binary_outpus.map(Ticker::every),
            pwm_us: settings.pwm_us.map(Ticker::every),
            raw_bus0a: settings.raw_bus0a.map(Ticker::every),
            raw_bus0b: settings.raw_bus0b.map(Ticker::every),
            raw_bus1a: settings.raw_bus1a.map(Ticker::every),
            raw_bus1b: settings.raw_bus1b.map(Ticker::every),
            sensor0: settings.sensor0.map(Ticker::every),
            sensor1: settings.sensor1.map(Ticker::every),
        }
    }
}

/// Resolves when `ticker` next fires. If `ticker` is `None` (channel
/// disabled), this future never resolves - it just sits inert in the
/// `select_array` set, contributing nothing.
///
/// Important: every call site here monomorphizes to the SAME anonymous
/// future type (it's one async fn), which is what `select_array` requires.
async fn tick_or_pending(ticker: &mut Option<Ticker>) {
    match ticker {
        Some(t) => t.next().await,
        None => core::future::pending::<()>().await,
    }
}

fn to_u8_vec(data: &[u16]) -> Option<heapless::Vec<u8, 8>> {
    // TODO: this hurts
    if data.len() != 4 {
        return None;
    };
    let mut vec: Vec<u8, 8> = heapless::Vec::new();
    vec.extend_from_slice(&data[0].to_le_bytes()).unwrap();
    vec.extend_from_slice(&data[1].to_le_bytes()).unwrap();
    vec.extend_from_slice(&data[2].to_le_bytes()).unwrap();
    vec.extend_from_slice(&data[3].to_le_bytes()).unwrap();
    Some(vec)
}

#[embassy_executor::task]
async fn tpdo_broadcast_task(settings: TpdoIntervals, can_pub: CanTxPub, node_id: u8) {
    let mut tickers = Tickers::new(&settings);

    loop {
        // Order here MUST match the CHANNELS array order above.
        let (_output, idx) = select_array([
            tick_or_pending(&mut tickers.valves),
            tick_or_pending(&mut tickers.binary_outputs),
            tick_or_pending(&mut tickers.pwm_us),
            tick_or_pending(&mut tickers.raw_bus0a),
            tick_or_pending(&mut tickers.raw_bus0b),
            tick_or_pending(&mut tickers.raw_bus1a),
            tick_or_pending(&mut tickers.raw_bus1b),
            tick_or_pending(&mut tickers.sensor0),
            tick_or_pending(&mut tickers.sensor1),
        ])
        .await;

        let message_kind = PD_MSGS[idx];
        let body: Vec<u8, 8> = {
            let store = STORE.lock().await;
            use PdMessageKind as K;
            match message_kind {
                K::PwmUs => to_u8_vec(&store.hco_pwm_us).unwrap(),
                K::Valves => to_u8_vec(&store.valves).unwrap(),
                K::Sensor0 => to_u8_vec(&store.selected_sensors[0..4]).unwrap(),
                K::Sensor1 => to_u8_vec(&store.selected_sensors[4..8]).unwrap(),
                K::RawBus0a => to_u8_vec(&store.raw_ext_adc_bus0[0..4]).unwrap(),
                K::RawBus0b => to_u8_vec(&store.raw_ext_adc_bus0[4..8]).unwrap(),
                K::RawBus1a => to_u8_vec(&store.raw_ext_adc_bus1[0..4]).unwrap(),
                K::RawBus1b => to_u8_vec(&store.raw_ext_adc_bus1[4..8]).unwrap(),
                K::BinaryOutputs => heapless::Vec::from_slice(&[
                    store.hco_binary[0],
                    0,
                    store.hco_binary[1],
                    0,
                    store.hco_binary[2],
                    0,
                    store.hco_binary[3],
                    0,
                ])
                .unwrap(),
            }
        };

        can_pub.publish((cob_id_from(message_kind, node_id), body)).await;
    }
}
#[embassy_executor::task]
pub async fn spawn_tpdo_task(spawner: Spawner, settings: TpdoIntervals, can_pub: CanTxPub, node_id: u8) {
    spawner.spawn(tpdo_broadcast_task(settings, can_pub, node_id).unwrap());
}
