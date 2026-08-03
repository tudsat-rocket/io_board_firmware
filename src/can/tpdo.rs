//! The data plane: fixed-layout process data broadcast on a per-kind period.
//!
//! Every kind in [`TpdoKind`] has a period at 0x3040 sub `kind + 1`, in milliseconds, 0 to
//! disable. A single [`BASE_TICK`] loop checks deadlines rather than one timer per kind, which
//! means a period changed over SDO takes effect on the next tick with no task restart, and adding
//! a kind costs one match arm instead of another future in a `select_array`.
//!
//! Layouts are fixed and documented in `device-conf/can-io.toml`. Everything is little-endian and
//! every frame is a full 8 bytes, so a master can decode by offset without a length check. The
//! byte-level encoding itself lives in [`iocan_proto::TpdoFrame`] — a separate, dependency-light
//! crate — so anything else that needs to build or parse these frames does not have to pull in
//! this firmware to do it. What stays here is firmware-only: which `Store` fields feed which kind,
//! and the scheduling below.
//!
//! A frame that is byte-identical to the last one actually sent for its kind is skipped at its
//! due time rather than resent — most of the bus load here is slowly-changing state (sensor
//! units, I2C presence, an idle valve), and there is no reason to spend a frame saying the same
//! thing twice. [`UNCHANGED_DATA_BROADCAST_DURATION`] bounds how long a kind can go silent: past
//! that, it resends even if nothing changed, so a master that just came up (or a bus analyser
//! catching the tail of a session) is never staring at a kind that looks dead rather than idle.

#[cfg(any(feature = "hardware", test))]
use embassy_time::{Instant, Timer};

#[cfg(any(feature = "hardware", test))]
use crate::can::CanTxPub;
#[cfg(any(feature = "hardware", test))]
use crate::can::ids::{TPDO_KINDS, TpdoKind};
#[cfg(any(feature = "hardware", test))]
use crate::config::NUM_TPDO_KINDS;
#[cfg(any(feature = "hardware", test))]
use crate::store::STORE;
#[cfg(any(feature = "hardware", test))]
use crate::store::Store;
#[cfg(any(feature = "hardware", test))]
use iocan_proto::TpdoFrame;

/// Deadline resolution. The fastest useful period is the 50 ms sensor broadcast, so 10 ms of
/// jitter is well inside what the master cares about and costs one wakeup per 10 ms.
#[cfg(any(feature = "hardware", test))]
const BASE_TICK: u64 = 10;

/// How long an unchanged kind may go without a frame before it is resent anyway. Comfortably
/// above every configured period so it reads as a keepalive, not a second broadcast rate.
#[cfg(any(feature = "hardware", test))]
const UNCHANGED_DATA_BROADCAST_DURATION: u64 = 5_000;

/// Wire scheduling for the fixed TPDO table. Dual-gated (not just `hardware`) so a host test can
/// construct one against a plain `PubSubChannel` and inspect what it publishes — nothing here
/// beyond `#[embassy_executor::task] run_tpdo` actually needs real hardware.
#[cfg(any(feature = "hardware", test))]
pub struct Tpdo {
    node_id: u8,
    tx: CanTxPub,
    /// Milliseconds since boot at which each kind is next due.
    next_due: [u64; NUM_TPDO_KINDS],
    /// Timestamp and payload of the last frame actually transmitted per kind, so a due-but-
    /// unchanged frame can be skipped. `None` until the first transmission, which is always sent.
    last_sent: [Option<(u64, [u8; 8])>; NUM_TPDO_KINDS],
}

#[cfg(any(feature = "hardware", test))]
impl Tpdo {
    pub fn new(node_id: u8, tx: CanTxPub) -> Self {
        Self {
            node_id,
            tx,
            next_due: [0; NUM_TPDO_KINDS],
            last_sent: [None; NUM_TPDO_KINDS],
        }
    }

    pub async fn run(&mut self) -> ! {
        loop {
            let now = Instant::now().as_millis();
            let intervals = { STORE.lock().await.config.tpdo_interval_ms };

            for kind in TPDO_KINDS {
                let index = kind as usize;
                let interval = intervals[kind] as u64;
                if interval == 0 || now < self.next_due[index] {
                    continue;
                }
                // Schedule from `now` rather than from the old deadline: after a long stall we
                // want the next frame one period from here, not a burst catching up.
                self.next_due[index] = now + interval;

                let payload = {
                    let store = STORE.lock().await;
                    build(&store, kind)
                };
                if !should_send(self.last_sent[index], now, payload) {
                    continue;
                }
                let Ok(body) = heapless::Vec::from_slice(&payload) else {
                    continue;
                };
                self.tx.publish((kind.cob_id(self.node_id), body)).await;
                self.last_sent[index] = Some((now, payload));
            }

            Timer::after_millis(BASE_TICK).await;
        }
    }
}

/// Whether a due frame is worth putting on the bus: yes the first time a kind is ever sent, yes
/// if the payload changed since the last transmission, and yes if it has been at least
/// [`UNCHANGED_DATA_BROADCAST_DURATION`] since that last transmission regardless of content.
#[cfg(any(feature = "hardware", test))]
fn should_send(last_sent: Option<(u64, [u8; 8])>, now: u64, payload: [u8; 8]) -> bool {
    match last_sent {
        None => true,
        Some((last_time, last_payload)) => {
            payload != last_payload || now.saturating_sub(last_time) >= UNCHANGED_DATA_BROADCAST_DURATION
        }
    }
}

/// Take `N` entries starting at `offset`, padding with `fill` if the source runs out. Keeps a
/// frame's layout fixed regardless of how many slots are configured (or, for
/// [`TpdoKind::Sensor3`], regardless of whether this node has that many sensor slots at all).
///
/// Firmware-only: it is about slicing *this* object dictionary's variable-length arrays down to
/// a frame's fixed slots, not about the wire format itself, so it stays out of `iocan-proto`.
#[cfg(any(feature = "hardware", test))]
fn window<T: Copy, const N: usize>(source: &[T], offset: usize, fill: T) -> [T; N] {
    let mut out = [fill; N];
    for (slot, value) in out.iter_mut().zip(source.iter().skip(offset)) {
        *slot = *value;
    }
    out
}

/// Gather this kind's payload out of the store. The byte-level layout of the result is entirely
/// [`TpdoFrame::encode`]'s concern; this function only ever answers "which fields".
#[cfg(any(feature = "hardware", test))]
fn frame_for(store: &Store, kind: TpdoKind) -> TpdoFrame {
    use iocan_proto::HcoOutput;

    use crate::config::NUM_AMPLIFIERS;
    use crate::store::{RAW_INVALID, SENSOR_INVALID};

    match kind {
        TpdoKind::ValveCommanded => TpdoFrame::ValveCommanded(*store.valve_commanded.as_array()),
        TpdoKind::ValveTarget => TpdoFrame::ValveTarget(*store.valve_target.as_array()),
        TpdoKind::ValveMeasured => TpdoFrame::ValveMeasured(*store.valve_measured.as_array()),
        TpdoKind::ValveCurrent => TpdoFrame::ValveCurrent(*store.valve_current_ma.as_array()),

        // Status, ownership and relief travel together: reading "valve 1 is stalled" is much
        // more useful next to "and it owns outputs 1 and 2, and relief is currently venting it".
        TpdoKind::ValveStatus => TpdoFrame::ValveStatus {
            status: *store.valve_status.as_array(),
            hco_owner: *store.hco_owner.as_array(),
            relief_state: store.relief_state,
        },

        // A digital output never reports a nonzero pulse width and a PWM output is always
        // clamped to 500..=2500 us, so "pulse width nonzero" and "digital" can never both be
        // true for the same output — the merged frame only needs to look at one field first.
        TpdoKind::HcoState => TpdoFrame::HcoState(crate::index::HcoId::ALL.map(|hco| {
            if store.hco_pwm_us[hco] != 0 {
                HcoOutput::Pwm(store.hco_pwm_us[hco])
            } else if store.hco_digital[hco] != 0 {
                HcoOutput::DigitalOn
            } else {
                HcoOutput::DigitalOff
            }
        })),

        TpdoKind::RawBus0A => TpdoFrame::RawBus0A(window(&store.raw_adc.as_slice()[..NUM_AMPLIFIERS], 0, RAW_INVALID)),
        TpdoKind::RawBus0B => TpdoFrame::RawBus0B(window(&store.raw_adc.as_slice()[..NUM_AMPLIFIERS], 4, RAW_INVALID)),
        TpdoKind::RawBus1A => TpdoFrame::RawBus1A(window(&store.raw_adc.as_slice()[NUM_AMPLIFIERS..], 0, RAW_INVALID)),
        TpdoKind::RawBus1B => TpdoFrame::RawBus1B(window(&store.raw_adc.as_slice()[NUM_AMPLIFIERS..], 4, RAW_INVALID)),

        TpdoKind::Sensor0 => TpdoFrame::Sensor0(window(store.sensor_value.as_slice(), 0, SENSOR_INVALID)),
        TpdoKind::Sensor1 => TpdoFrame::Sensor1(window(store.sensor_value.as_slice(), 4, SENSOR_INVALID)),
        // This node has NUM_SENSOR_SLOTS (8) slots, so offset 8 always runs straight past the end
        // of `sensor_value` and every entry here is the fill value — see the kind's own doc.
        TpdoKind::Sensor3 => TpdoFrame::Sensor3(window(store.sensor_value.as_slice(), 8, SENSOR_INVALID)),
        TpdoKind::SensorUnits => TpdoFrame::SensorUnits(window(store.sensor_unit.as_slice(), 0, 0)),

        TpdoKind::I2cScan => TpdoFrame::I2cScan {
            present: *store.i2c_present.as_array(),
            sweeps: store.i2c_sweeps as u16,
        },

        TpdoKind::RailVoltage => TpdoFrame::RailVoltage(*store.rail_voltage_mv.as_array()),
        TpdoKind::RailCurrent => TpdoFrame::RailCurrent(*store.rail_current_ma.as_array()),

        TpdoKind::Status => TpdoFrame::Status {
            link_state: store.link_state as u8,
            raw_debug: store.raw_debug,
            // Bit i set = valve i is stalled: the one fault worth a dedicated bit, since it is
            // the only thing that says a valve is not where it claims to be.
            stalled_mask: store
                .valve_status
                .iter()
                .filter(|(_, s)| **s == crate::valves::ValveStatus::Stalled as u8)
                .fold(0u8, |bits, (valve, _)| bits | 1 << valve.index()),
            ms_since_heartbeat: store.ms_since_heartbeat,
        },
    }
}

#[cfg(any(feature = "hardware", test))]
fn build(store: &Store, kind: TpdoKind) -> [u8; 8] {
    frame_for(store, kind).encode()
}

#[cfg(feature = "hardware")]
#[embassy_executor::task]
pub async fn run_tpdo(mut tpdo: Tpdo) -> ! {
    tpdo.run().await
}

#[cfg(test)]
mod tests {
    use iocan_proto::HcoOutput;

    use super::*;
    use crate::index::{HcoId, ValveId};
    use crate::store::LinkState;

    #[test]
    fn every_kind_produces_a_full_frame() {
        let store = Store::new();
        for kind in TPDO_KINDS {
            assert_eq!(build(&store, kind).len(), 8, "{kind:?} must fill the frame");
        }
    }

    #[test]
    fn rails_split_into_separate_voltage_and_current_frames() {
        let mut store = Store::new();
        store.rail_current_ma = crate::index::PerRail::new([111, 222, 333]);
        store.rail_voltage_mv = crate::index::PerRail::new([444, 555, 666]);

        assert_eq!(build(&store, TpdoKind::RailCurrent), TpdoFrame::RailCurrent([111, 222, 333]).encode());
        assert_eq!(build(&store, TpdoKind::RailVoltage), TpdoFrame::RailVoltage([444, 555, 666]).encode());
    }

    #[test]
    fn valve_status_carries_ownership_and_relief_alongside() {
        let mut store = Store::new();
        store.valve_status = crate::index::PerValve::new([1, 2, 3, 4]);
        store.hco_owner = crate::index::PerHco::new([1, 1, 0, 0]);
        store.relief_state = crate::relief::ReliefState::Relieving as u8;

        assert_eq!(
            build(&store, TpdoKind::ValveStatus),
            TpdoFrame::ValveStatus {
                status: [1, 2, 3, 4],
                hco_owner: [1, 1, 0, 0],
                relief_state: crate::relief::ReliefState::Relieving as u8,
            }
            .encode()
        );
    }

    #[test]
    fn hco_state_reports_pwm_when_a_pulse_width_is_set_and_digital_otherwise() {
        let mut store = Store::new();
        // output 0: PWM at 1500us; output 1: digital high; output 2: digital low (default);
        // output 3: also digital high.
        store.hco_pwm_us[HcoId::Hco0] = 1500;
        store.hco_digital[HcoId::Hco0] = 1; // set alongside pwm_us on the real path too; pwm_us wins
        store.hco_digital[HcoId::Hco1] = 1;
        store.hco_digital[HcoId::Hco3] = 1;

        let frame = build(&store, TpdoKind::HcoState);
        assert_eq!(
            frame,
            TpdoFrame::HcoState([
                HcoOutput::Pwm(1500),
                HcoOutput::DigitalOn,
                HcoOutput::DigitalOff,
                HcoOutput::DigitalOn,
            ])
            .encode()
        );
    }

    #[test]
    fn status_reports_stalled_valves_as_a_bitmask_and_no_longer_carries_relief() {
        let mut store = Store::new();
        store.valve_status[ValveId::Valve0] = crate::valves::ValveStatus::Holding as u8;
        store.valve_status[ValveId::Valve2] = crate::valves::ValveStatus::Stalled as u8;
        store.link_state = LinkState::FallbackA;
        store.ms_since_heartbeat = 0x0102_0304;

        let frame = build(&store, TpdoKind::Status);
        assert_eq!(frame[0], LinkState::FallbackA as u8);
        assert_eq!(frame[2], 0b0000_0100, "only valve 2 is stalled");
        assert_eq!(&frame[4..], &0x0102_0304u32.to_le_bytes());
    }

    #[test]
    fn sensor_windows_do_not_overlap_and_slot_3_is_always_invalid_here() {
        let mut store = Store::new();
        store.sensor_value = crate::index::PerSensorSlot::new([10, 11, 12, 13, 14, 15, 16, 17]);
        assert_eq!(build(&store, TpdoKind::Sensor0), TpdoFrame::Sensor0([10, 11, 12, 13]).encode());
        assert_eq!(build(&store, TpdoKind::Sensor1), TpdoFrame::Sensor1([14, 15, 16, 17]).encode());
        assert_eq!(
            build(&store, TpdoKind::Sensor3),
            TpdoFrame::Sensor3([crate::store::SENSOR_INVALID; 4]).encode(),
            "this node only has 8 sensor slots, so slots 8..12 never have real data"
        );
    }

    #[test]
    fn sensor_units_covers_all_twelve_protocol_slots() {
        let mut store = Store::new();
        store.sensor_unit = crate::index::PerSensorSlot::new([0, 1, 2, 3, 1, 1, 1, 1]);

        let frame = build(&store, TpdoKind::SensorUnits);
        assert_eq!(
            frame,
            TpdoFrame::SensorUnits([0, 1, 2, 3, 1, 1, 1, 1, 0, 0, 0, 0]).encode(),
            "slots 8..12 pad with 0 (this node has no sensors there)"
        );
    }

    #[test]
    fn a_kind_is_always_sent_the_first_time() {
        assert!(should_send(None, 0, [0; 8]));
    }

    #[test]
    fn an_unchanged_payload_is_skipped_before_the_keepalive_elapses() {
        let payload = [1, 2, 3, 4, 5, 6, 7, 8];
        let last_sent = Some((1_000, payload));
        assert!(!should_send(last_sent, 1_000 + UNCHANGED_DATA_BROADCAST_DURATION - 1, payload));
    }

    #[test]
    fn an_unchanged_payload_is_resent_once_the_keepalive_elapses() {
        let payload = [1, 2, 3, 4, 5, 6, 7, 8];
        let last_sent = Some((1_000, payload));
        assert!(should_send(last_sent, 1_000 + UNCHANGED_DATA_BROADCAST_DURATION, payload));
    }

    #[test]
    fn a_changed_payload_is_sent_immediately_regardless_of_the_keepalive() {
        let last_sent = Some((1_000, [0; 8]));
        assert!(should_send(last_sent, 1_000, [1; 8]));
    }

    #[test]
    fn the_scan_frame_exposes_both_presence_bitmaps() {
        let mut store = Store::new();
        store.i2c_present = crate::index::PerI2cBus::new([0b1_0000_0001, 0b0_0000_0110]);
        store.i2c_sweeps = 42;

        let frame = build(&store, TpdoKind::I2cScan);
        assert_eq!(u16::from_le_bytes([frame[0], frame[1]]), 0b1_0000_0001);
        assert_eq!(u16::from_le_bytes([frame[2], frame[3]]), 0b0_0000_0110);
        assert_eq!(u16::from_le_bytes([frame[4], frame[5]]), 42);
    }
}
