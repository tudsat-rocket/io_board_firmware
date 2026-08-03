//! CAN identifier layout.
//!
//! The whole bus fits in the 11-bit identifier space with room to spare, because there are at
//! most 16 nodes and one of them makes every decision. The layout is deliberately readable off a
//! logic analyser without a database:
//!
//! ```text
//!   0x200 | (kind << 4) | node_id     process data out of a node   (5-bit kind, 4-bit node)
//!   0x580 + node_id                   SDO response from a node
//!   0x600 + node_id                   SDO client request to a node
//!   0x700 + node_id                   heartbeat
//! ```
//!
//! The 4-bit node field is what caps the bus at 16 nodes, which matches the vehicle.

/// Base of the process data range.
pub const PDO_BASE: u16 = 0x200;
/// SDO server response, plus node id.
pub const SDO_RESPONSE_BASE: u16 = 0x580;
/// SDO client request, plus node id.
pub const SDO_REQUEST_BASE: u16 = 0x600;
/// Heartbeat, plus node id.
pub const HEARTBEAT_BASE: u16 = 0x700;

pub const NODE_ID_MASK: u16 = 0x000F;

/// Number of fixed TPDO kinds. Must agree with `TpdoKind` and with `array_size` of 0x3040 in
/// `device-conf/can-io.toml`.
pub const NUM_TPDO_KINDS: usize = 18;

/// The fixed TPDO table. The discriminant is the `kind` field of the identifier and the index
/// into a node's `tpdo_interval_ms` (0x3040), so the three never drift apart.
///
/// Kept fixed rather than runtime-mappable: a master that has to read a PDO mapping before it can
/// decode a frame is exactly the complexity this protocol is avoiding.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TpdoKind {
    ValveCommanded = 0,
    ValveTarget = 1,
    ValveMeasured = 2,
    /// Also carries `hco_owner` and `relief_state` — see [`crate::TpdoFrame::ValveStatus`].
    ValveStatus = 3,
    /// Digital and PWM outputs merged into one frame — see [`crate::TpdoFrame::HcoState`].
    HcoState = 4,
    RawBus0A = 5,
    RawBus0B = 6,
    RawBus1A = 7,
    RawBus1B = 8,
    Sensor0 = 9,
    Sensor1 = 10,
    /// Slots 8..12. Unused on any node with `NUM_SENSOR_SLOTS <= 8` (today, every node), kept
    /// for boards with more amplifier slots.
    Sensor3 = 11,
    SensorUnits = 12,
    /// Presence bitmaps and the sweep counter: the assembly-verification channel.
    I2cScan = 13,
    RailVoltage = 14,
    RailCurrent = 15,
    /// No longer carries `relief_state` — see [`crate::TpdoFrame::ValveStatus`].
    Status = 16,
    ValveCurrent = 17,
}

/// All kinds, in discriminant order. The broadcaster walks this.
pub const TPDO_KINDS: [TpdoKind; NUM_TPDO_KINDS] = [
    TpdoKind::ValveCommanded,
    TpdoKind::ValveTarget,
    TpdoKind::ValveMeasured,
    TpdoKind::ValveStatus,
    TpdoKind::HcoState,
    TpdoKind::RawBus0A,
    TpdoKind::RawBus0B,
    TpdoKind::RawBus1A,
    TpdoKind::RawBus1B,
    TpdoKind::Sensor0,
    TpdoKind::Sensor1,
    TpdoKind::Sensor3,
    TpdoKind::SensorUnits,
    TpdoKind::I2cScan,
    TpdoKind::RailVoltage,
    TpdoKind::RailCurrent,
    TpdoKind::Status,
    TpdoKind::ValveCurrent,
];

impl TpdoKind {
    /// The identifier this kind is broadcast on for a given node.
    pub const fn cob_id(self, node_id: u8) -> u16 {
        PDO_BASE | ((self as u16) << 4) | (node_id as u16 & NODE_ID_MASK)
    }

    /// The kind for a given `decode_pdo` index, the other half of decoding a captured frame.
    pub const fn from_index(index: u8) -> Option<Self> {
        match index {
            0 => Some(Self::ValveCommanded),
            1 => Some(Self::ValveTarget),
            2 => Some(Self::ValveMeasured),
            3 => Some(Self::ValveStatus),
            4 => Some(Self::HcoState),
            5 => Some(Self::RawBus0A),
            6 => Some(Self::RawBus0B),
            7 => Some(Self::RawBus1A),
            8 => Some(Self::RawBus1B),
            9 => Some(Self::Sensor0),
            10 => Some(Self::Sensor1),
            11 => Some(Self::Sensor3),
            12 => Some(Self::SensorUnits),
            13 => Some(Self::I2cScan),
            14 => Some(Self::RailVoltage),
            15 => Some(Self::RailCurrent),
            16 => Some(Self::Status),
            17 => Some(Self::ValveCurrent),
            _ => None,
        }
    }
}

/// Decode a process data identifier back into node and kind index. Used by test tooling and by
/// anything on the bus that wants to sniff another node's traffic. Pair with
/// [`TpdoKind::from_index`] to get a typed kind, then [`crate::TpdoFrame::decode`] for the body.
pub fn decode_pdo(cob_id: u16) -> Option<(u8, u8)> {
    if !(PDO_BASE..PDO_BASE + 0x200).contains(&cob_id) {
        return None;
    }
    Some(((cob_id & NODE_ID_MASK) as u8, ((cob_id - PDO_BASE) >> 4) as u8))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identifiers_round_trip() {
        for (index, kind) in TPDO_KINDS.iter().enumerate() {
            assert_eq!(*kind as usize, index, "TPDO_KINDS must stay in discriminant order");
            let cob = kind.cob_id(6);
            assert_eq!(decode_pdo(cob), Some((6, index as u8)));
            assert_eq!(TpdoKind::from_index(index as u8), Some(*kind));
        }
    }

    #[test]
    fn an_out_of_range_index_does_not_decode() {
        assert_eq!(TpdoKind::from_index(18), None);
        assert_eq!(TpdoKind::from_index(255), None);
    }

    #[test]
    fn every_kind_fits_the_five_bit_field() {
        // A 19th kind would collide with the SDO range.
        assert!(TPDO_KINDS.len() <= 32);
        let highest = TPDO_KINDS.last().unwrap().cob_id(15);
        assert!(highest < SDO_RESPONSE_BASE, "process data must not reach the SDO range");
    }

    #[test]
    fn node_ids_stay_inside_their_nibble() {
        assert_eq!(TpdoKind::ValveCommanded.cob_id(0), 0x200);
        assert_eq!(TpdoKind::ValveCommanded.cob_id(15), 0x20F);
        assert_eq!(TpdoKind::ValveTarget.cob_id(2), 0x212);
    }
}
