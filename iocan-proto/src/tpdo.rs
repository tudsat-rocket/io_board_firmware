//! TPDO frame layouts: turn a typed value into the 8 bytes that go on the bus, or turn those
//! bytes back into a typed value.
//!
//! Every frame is a full 8 bytes regardless of how much of it is meaningful, and everything is
//! little-endian, so a decoder can pull any field out by fixed offset without a length check —
//! matching the SDO payloads elsewhere in the protocol. [`TpdoFrame::decode`] and
//! [`TpdoFrame::encode`] are exact inverses of each other; a round trip through both is the
//! contract this module exists to keep.
//!
//! A TPDO frame is not always a straight mirror of one object in the device profile: several
//! pack more than one related object into a single 8-byte broadcast to save bandwidth (e.g.
//! [`TpdoFrame::ValveStatus`] carries valve status, HCO ownership *and* relief state together,
//! because a listener wants all three in the same breath). A couple pack tighter still, trading
//! a fixed byte offset for bit-fields, because the values genuinely don't need a whole byte each
//! ([`TpdoFrame::HcoState`]'s digital/PWM merge, [`TpdoFrame::ValveStatus`]'s nibble-packed
//! status/ownership, [`TpdoFrame::SensorUnits`]'s 2-bit-per-slot unit codes). Each variant's doc
//! comment says which `device-conf/can-io.toml` object(s) it mirrors.

use crate::ids::TpdoKind;

/// Number of sensor slots this protocol can carry over TPDO: [`TpdoFrame::Sensor0`] (slots
/// 0..4), [`TpdoFrame::Sensor1`] (4..8) and [`TpdoFrame::Sensor3`] (8..12). Independent of any
/// one node's actual sensor slot count — see [`TpdoFrame::Sensor3`]'s doc for why.
pub const NUM_PROTOCOL_SENSOR_SLOTS: usize = 12;

/// One high current output's state, as packed into [`TpdoFrame::HcoState`].
///
/// `0x8000` and `0x0000` are sentinels rather than real pulse widths — every legal PWM width is
/// well below `0x8000` (this protocol's boards clamp to 500..=2500 us) — so a decoder can tell
/// "digital, energised", "digital, de-energised" and "PWM at this width" apart with no extra
/// flag byte.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HcoOutput {
    DigitalOn,
    DigitalOff,
    Pwm(u16),
}

impl HcoOutput {
    const DIGITAL_ON: u16 = 0x8000;
    const DIGITAL_OFF: u16 = 0x0000;

    pub const fn to_u16(self) -> u16 {
        match self {
            Self::DigitalOn => Self::DIGITAL_ON,
            Self::DigitalOff => Self::DIGITAL_OFF,
            Self::Pwm(us) => us,
        }
    }

    pub const fn from_u16(v: u16) -> Self {
        match v {
            Self::DIGITAL_ON => Self::DigitalOn,
            Self::DIGITAL_OFF => Self::DigitalOff,
            us => Self::Pwm(us),
        }
    }
}

/// One kind's typed payload, decoded from (or about to be encoded to) its 8-byte frame.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TpdoFrame {
    /// Mirrors 0x2010, one entry per valve.
    ValveCommanded([u16; 4]),
    /// Mirrors 0x2011, one entry per valve.
    ValveTarget([u16; 4]),
    /// Mirrors 0x2012, one entry per valve.
    ValveMeasured([u16; 4]),
    /// `status` mirrors 0x2013, `hco_owner` mirrors 0x2022, `relief_state` mirrors 0x2015 —
    /// packed together so a listener never has to correlate three frames to answer "is this
    /// valve stalled, what does it own, and is relief active". `status` and `hco_owner` are each
    /// nibble-packed (every value fits in 4 bits: 5 `ValveStatus` variants, and `hco_owner`'s
    /// 0..=4 range of "unowned or owned by valve N") to leave room for `relief_state` in the
    /// same 8 bytes; the last 3 bytes are unused padding.
    ValveStatus {
        status: [u8; 4],
        hco_owner: [u8; 4],
        relief_state: u8,
    },
    /// Merges what used to be two separate frames (digital level and PWM width) into one: each
    /// output's [`HcoOutput`] already says whether it's driven digitally or by PWM, so there's
    /// nothing left for a second frame to add. Mirrors 0x2020/0x2021.
    HcoState([HcoOutput; 4]),
    /// A window of 0x2000 (I2C bus 0): amplifier indices 0..4.
    RawBus0A([u16; 4]),
    /// A window of 0x2000 (I2C bus 0): amplifier indices 4..8.
    RawBus0B([u16; 4]),
    /// A window of 0x2001 (I2C bus 1): amplifier indices 0..4.
    RawBus1A([u16; 4]),
    /// A window of 0x2001 (I2C bus 1): amplifier indices 4..8.
    RawBus1B([u16; 4]),
    /// A window of 0x2004 (calibrated sensor values): slots 0..4.
    Sensor0([i16; 4]),
    /// A window of 0x2004 (calibrated sensor values): slots 4..8.
    Sensor1([i16; 4]),
    /// A window of 0x2004 (calibrated sensor values): slots 8..12. No node built against this
    /// protocol has more than 8 sensor slots today, so in practice every slot here reads as
    /// "invalid" — kept as a real kind rather than reserved for later so a board with more
    /// amplifier headroom does not need a protocol version bump to use it.
    Sensor3([i16; 4]),
    /// Mirrors 0x2005: the unit code each sensor slot reports its value in, 2 bits per slot
    /// (4 possible units) so all 12 protocol-wide slots fit in 3 of the 8 bytes.
    SensorUnits([u8; NUM_PROTOCOL_SENSOR_SLOTS]),
    /// `present` mirrors 0x2002, `sweeps` mirrors 0x2003 (truncated to 16 bits). The remaining 2
    /// bytes are unused padding.
    I2cScan { present: [u16; 2], sweeps: u16 },
    /// The logic, HCO1+2 and HCO3+4 rail voltages (0x2041). The remaining 2 bytes are unused
    /// padding.
    RailVoltage([u16; 3]),
    /// The logic, HCO1+2 and HCO3+4 rail currents (0x2040). The remaining 2 bytes are unused
    /// padding.
    RailCurrent([u16; 3]),
    /// `link_state` mirrors 0x2032, `raw_debug` mirrors 0x2031, `ms_since_heartbeat` mirrors
    /// 0x2033. `stalled_mask` (bit i set = valve i stalled) has no single-object mirror; it
    /// exists only here. Relief state moved to [`Self::ValveStatus`], since a listener wants it
    /// next to which valve relief is acting on, not next to the link state.
    Status {
        link_state: u8,
        raw_debug: bool,
        stalled_mask: u8,
        ms_since_heartbeat: u32,
    },
    /// Mirrors 0x2014, one entry per valve.
    ValveCurrent([u16; 4]),
}

impl TpdoFrame {
    /// Which kind this payload belongs on the bus as.
    pub const fn kind(&self) -> TpdoKind {
        match self {
            Self::ValveCommanded(_) => TpdoKind::ValveCommanded,
            Self::ValveTarget(_) => TpdoKind::ValveTarget,
            Self::ValveMeasured(_) => TpdoKind::ValveMeasured,
            Self::ValveStatus { .. } => TpdoKind::ValveStatus,
            Self::HcoState(_) => TpdoKind::HcoState,
            Self::RawBus0A(_) => TpdoKind::RawBus0A,
            Self::RawBus0B(_) => TpdoKind::RawBus0B,
            Self::RawBus1A(_) => TpdoKind::RawBus1A,
            Self::RawBus1B(_) => TpdoKind::RawBus1B,
            Self::Sensor0(_) => TpdoKind::Sensor0,
            Self::Sensor1(_) => TpdoKind::Sensor1,
            Self::Sensor3(_) => TpdoKind::Sensor3,
            Self::SensorUnits(_) => TpdoKind::SensorUnits,
            Self::I2cScan { .. } => TpdoKind::I2cScan,
            Self::RailVoltage(_) => TpdoKind::RailVoltage,
            Self::RailCurrent(_) => TpdoKind::RailCurrent,
            Self::Status { .. } => TpdoKind::Status,
            Self::ValveCurrent(_) => TpdoKind::ValveCurrent,
        }
    }

    /// Serialize to the 8 bytes that go on the bus.
    pub fn encode(&self) -> [u8; 8] {
        match *self {
            Self::ValveCommanded(v) => u16x4_to_bytes(v),
            Self::ValveTarget(v) => u16x4_to_bytes(v),
            Self::ValveMeasured(v) => u16x4_to_bytes(v),
            Self::ValveCurrent(v) => u16x4_to_bytes(v),
            Self::RawBus0A(v) | Self::RawBus0B(v) | Self::RawBus1A(v) | Self::RawBus1B(v) => u16x4_to_bytes(v),
            Self::Sensor0(v) => i16x4_to_bytes(v),
            Self::Sensor1(v) => i16x4_to_bytes(v),
            Self::Sensor3(v) => i16x4_to_bytes(v),

            Self::HcoState(outputs) => u16x4_to_bytes(outputs.map(HcoOutput::to_u16)),

            Self::ValveStatus {
                status,
                hco_owner,
                relief_state,
            } => {
                let mut out = [0u8; 8];
                out[..2].copy_from_slice(&pack_nibbles(status));
                out[2..4].copy_from_slice(&pack_nibbles(hco_owner));
                out[4] = relief_state;
                out
            }
            Self::SensorUnits(units) => {
                let mut out = [0u8; 8];
                out[..3].copy_from_slice(&pack_2bit(units));
                out
            }
            Self::I2cScan { present, sweeps } => u16x4_to_bytes([present[0], present[1], sweeps, 0]),
            Self::RailVoltage(v) => u16x4_to_bytes([v[0], v[1], v[2], 0]),
            Self::RailCurrent(v) => u16x4_to_bytes([v[0], v[1], v[2], 0]),
            Self::Status {
                link_state,
                raw_debug,
                stalled_mask,
                ms_since_heartbeat,
            } => {
                let mut out = [0u8; 8];
                out[0] = link_state;
                out[1] = raw_debug as u8;
                out[2] = stalled_mask;
                out[4..].copy_from_slice(&ms_since_heartbeat.to_le_bytes());
                out
            }
        }
    }

    /// Parse a captured frame's bytes according to `kind`. The exact inverse of `encode`.
    pub fn decode(kind: TpdoKind, bytes: [u8; 8]) -> Self {
        match kind {
            TpdoKind::ValveCommanded => Self::ValveCommanded(u16x4_from_bytes(bytes)),
            TpdoKind::ValveTarget => Self::ValveTarget(u16x4_from_bytes(bytes)),
            TpdoKind::ValveMeasured => Self::ValveMeasured(u16x4_from_bytes(bytes)),
            TpdoKind::ValveCurrent => Self::ValveCurrent(u16x4_from_bytes(bytes)),
            TpdoKind::RawBus0A => Self::RawBus0A(u16x4_from_bytes(bytes)),
            TpdoKind::RawBus0B => Self::RawBus0B(u16x4_from_bytes(bytes)),
            TpdoKind::RawBus1A => Self::RawBus1A(u16x4_from_bytes(bytes)),
            TpdoKind::RawBus1B => Self::RawBus1B(u16x4_from_bytes(bytes)),
            TpdoKind::Sensor0 => Self::Sensor0(i16x4_from_bytes(bytes)),
            TpdoKind::Sensor1 => Self::Sensor1(i16x4_from_bytes(bytes)),
            TpdoKind::Sensor3 => Self::Sensor3(i16x4_from_bytes(bytes)),

            TpdoKind::HcoState => Self::HcoState(u16x4_from_bytes(bytes).map(HcoOutput::from_u16)),

            TpdoKind::ValveStatus => Self::ValveStatus {
                status: unpack_nibbles(bytes[..2].try_into().unwrap()),
                hco_owner: unpack_nibbles(bytes[2..4].try_into().unwrap()),
                relief_state: bytes[4],
            },
            TpdoKind::SensorUnits => Self::SensorUnits(unpack_2bit(bytes[..3].try_into().unwrap())),
            TpdoKind::I2cScan => {
                let words = u16x4_from_bytes(bytes);
                Self::I2cScan {
                    present: [words[0], words[1]],
                    sweeps: words[2],
                }
            }
            TpdoKind::RailVoltage => {
                let words = u16x4_from_bytes(bytes);
                Self::RailVoltage([words[0], words[1], words[2]])
            }
            TpdoKind::RailCurrent => {
                let words = u16x4_from_bytes(bytes);
                Self::RailCurrent([words[0], words[1], words[2]])
            }
            TpdoKind::Status => Self::Status {
                link_state: bytes[0],
                raw_debug: bytes[1] != 0,
                stalled_mask: bytes[2],
                ms_since_heartbeat: u32::from_le_bytes(bytes[4..].try_into().unwrap()),
            },
        }
    }
}

/// Four little-endian `u16`s: the shape most TPDO kinds use. `pub` so a frame builder gathering
/// data into one of the plain-array [`TpdoFrame`] variants can reuse it directly.
pub fn u16x4_to_bytes(values: [u16; 4]) -> [u8; 8] {
    let mut out = [0u8; 8];
    for (chunk, value) in out.chunks_exact_mut(2).zip(values) {
        chunk.copy_from_slice(&value.to_le_bytes());
    }
    out
}

/// The inverse of [`u16x4_to_bytes`].
pub fn u16x4_from_bytes(bytes: [u8; 8]) -> [u16; 4] {
    let mut out = [0u16; 4];
    for (slot, chunk) in out.iter_mut().zip(bytes.chunks_exact(2)) {
        *slot = u16::from_le_bytes(chunk.try_into().unwrap());
    }
    out
}

/// Four little-endian `i16`s, reinterpreting the same bit pattern as [`u16x4_to_bytes`].
pub fn i16x4_to_bytes(values: [i16; 4]) -> [u8; 8] {
    u16x4_to_bytes(values.map(|v| v as u16))
}

/// The inverse of [`i16x4_to_bytes`].
pub fn i16x4_from_bytes(bytes: [u8; 8]) -> [i16; 4] {
    u16x4_from_bytes(bytes).map(|v| v as i16)
}

/// Four 4-bit values, low nibble first, into 2 bytes. Callers are responsible for keeping each
/// value under 16 — every current use (`ValveStatus`, `ValveStatus::Stalled` = 4, `hco_owner`'s
/// 0..=4) has comfortable headroom below that.
fn pack_nibbles(values: [u8; 4]) -> [u8; 2] {
    [values[0] | (values[1] << 4), values[2] | (values[3] << 4)]
}

/// The inverse of [`pack_nibbles`].
fn unpack_nibbles(bytes: [u8; 2]) -> [u8; 4] {
    [bytes[0] & 0xF, bytes[0] >> 4, bytes[1] & 0xF, bytes[1] >> 4]
}

/// [`NUM_PROTOCOL_SENSOR_SLOTS`] 2-bit values, low bits first, into 3 bytes.
fn pack_2bit(values: [u8; NUM_PROTOCOL_SENSOR_SLOTS]) -> [u8; 3] {
    let mut out = [0u8; 3];
    for (i, &v) in values.iter().enumerate() {
        out[i / 4] |= (v & 0b11) << ((i % 4) * 2);
    }
    out
}

/// The inverse of [`pack_2bit`].
fn unpack_2bit(bytes: [u8; 3]) -> [u8; NUM_PROTOCOL_SENSOR_SLOTS] {
    let mut out = [0u8; NUM_PROTOCOL_SENSOR_SLOTS];
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = (bytes[i / 4] >> ((i % 4) * 2)) & 0b11;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every variant has to survive a round trip through its own kind, because a mismatch
    /// between `encode` and `decode` would only show up as a listener silently misreading a
    /// node's state.
    #[test]
    fn every_frame_round_trips_through_its_own_kind() {
        let samples = [
            TpdoFrame::ValveCommanded([1, 2, 3, 4]),
            TpdoFrame::ValveTarget([5, 6, 7, 8]),
            TpdoFrame::ValveMeasured([9, 10, 11, 12]),
            TpdoFrame::ValveStatus {
                status: [1, 2, 3, 4],
                hco_owner: [0, 1, 2, 4],
                relief_state: 2,
            },
            TpdoFrame::HcoState([
                HcoOutput::DigitalOn,
                HcoOutput::DigitalOff,
                HcoOutput::Pwm(1500),
                HcoOutput::Pwm(500),
            ]),
            TpdoFrame::RawBus0A([100, 200, 300, 400]),
            TpdoFrame::RawBus0B([500, 600, 700, 800]),
            TpdoFrame::RawBus1A([900, 1000, 1100, 1200]),
            TpdoFrame::RawBus1B([1300, 1400, 1500, 1600]),
            TpdoFrame::Sensor0([-1, -2, -3, i16::MIN]),
            TpdoFrame::Sensor1([1, 2, 3, i16::MAX]),
            TpdoFrame::Sensor3([0, 0, 0, 0]),
            TpdoFrame::SensorUnits([0, 1, 2, 3, 0, 1, 2, 3, 3, 2, 1, 0]),
            TpdoFrame::I2cScan {
                present: [0b1010, 0b0101],
                sweeps: 42,
            },
            TpdoFrame::RailVoltage([111, 222, 333]),
            TpdoFrame::RailCurrent([444, 555, 666]),
            TpdoFrame::Status {
                link_state: 2,
                raw_debug: true,
                stalled_mask: 0b0100,
                ms_since_heartbeat: 0x0102_0304,
            },
            TpdoFrame::ValveCurrent([50, 60, 70, 80]),
        ];

        for frame in samples {
            let bytes = frame.encode();
            assert_eq!(bytes.len(), 8);
            assert_eq!(TpdoFrame::decode(frame.kind(), bytes), frame, "{frame:?} did not round-trip");
        }
    }

    #[test]
    fn status_packs_the_fields_at_their_fixed_offsets() {
        let frame = TpdoFrame::Status {
            link_state: 2,
            raw_debug: false,
            stalled_mask: 0b0000_0100,
            ms_since_heartbeat: 0x0102_0304,
        };
        let bytes = frame.encode();
        assert_eq!(bytes[0], 2);
        assert_eq!(bytes[1], 0);
        assert_eq!(bytes[2], 0b0000_0100);
        assert_eq!(&bytes[4..], &0x0102_0304u32.to_le_bytes());
    }

    #[test]
    fn valve_status_nibble_packs_and_carries_relief_state() {
        let frame = TpdoFrame::ValveStatus {
            status: [1, 2, 3, 4],
            hco_owner: [1, 1, 0, 0],
            relief_state: 1,
        };
        // status[0]=1, status[1]=2 -> byte0 = 0x21; status[2]=3, status[3]=4 -> byte1 = 0x43
        // hco_owner[0]=1, hco_owner[1]=1 -> byte2 = 0x11; hco_owner[2]=0, hco_owner[3]=0 -> byte3 = 0x00
        assert_eq!(frame.encode(), [0x21, 0x43, 0x11, 0x00, 1, 0, 0, 0]);
    }

    #[test]
    fn hco_state_merges_digital_and_pwm() {
        let frame = TpdoFrame::HcoState([
            HcoOutput::DigitalOn,
            HcoOutput::DigitalOff,
            HcoOutput::Pwm(1500),
            HcoOutput::Pwm(2500),
        ]);
        let bytes = frame.encode();
        assert_eq!(u16::from_le_bytes([bytes[0], bytes[1]]), 0x8000);
        assert_eq!(u16::from_le_bytes([bytes[2], bytes[3]]), 0x0000);
        assert_eq!(u16::from_le_bytes([bytes[4], bytes[5]]), 1500);
        assert_eq!(u16::from_le_bytes([bytes[6], bytes[7]]), 2500);
    }

    #[test]
    fn sensor_units_packs_all_twelve_slots_into_three_bytes() {
        let frame = TpdoFrame::SensorUnits([0, 1, 2, 3, 3, 2, 1, 0, 1, 1, 1, 1]);
        let bytes = frame.encode();
        assert_eq!(bytes[3..], [0, 0, 0, 0, 0], "only the first 3 bytes carry data");
        assert_eq!(TpdoFrame::decode(TpdoKind::SensorUnits, bytes), frame);
    }

    #[test]
    fn rails_are_two_separate_frames() {
        let voltage = TpdoFrame::RailVoltage([12000, 12100, 12200]);
        let current = TpdoFrame::RailCurrent([100, 200, 300]);
        assert_ne!(voltage.encode(), current.encode());
        assert_eq!(voltage.kind(), TpdoKind::RailVoltage);
        assert_eq!(current.kind(), TpdoKind::RailCurrent);
    }

    #[test]
    fn kind_matches_what_was_used_to_decode() {
        for kind in crate::ids::TPDO_KINDS {
            let decoded = TpdoFrame::decode(kind, [0; 8]);
            assert_eq!(decoded.kind(), kind);
        }
    }
}
