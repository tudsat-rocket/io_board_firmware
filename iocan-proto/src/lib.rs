//! Wire protocol for the ioCan CAN bus: identifiers, the object dictionary, and TPDO frames.
//!
//! **This crate is the source of truth for the protocol.** `device-conf/can-io.toml` restates the
//! object table in the form `zencan-build` can parse — name, data type, array size, default — and
//! carries one summary line per object; everything about what the numbers *mean* lives here. When
//! the two disagree, this crate is right and the schema is stale.
//!
//! `no_std`, no heap, and the only dependency (`defmt`) is optional and only affects logging —
//! nothing here needs it to encode or decode a frame. Anything that talks to an ioCan node, or
//! just wants to sniff its bus traffic, can depend on this crate alone rather than pulling in the
//! firmware.
//!
//! # The bus
//!
//! At most 16 nodes, one of which (default id 1) is the master and makes every decision. Node ids
//! are 4 bits everywhere, which is what caps the count, and it matches the vehicle. The layout is
//! deliberately readable off a logic analyser without a database — see [`ids`].
//!
//! There are three planes:
//!
//! ## Config plane — expedited SDO, one object per frame, always acknowledged
//!
//! ```text
//!   0x600 + node_id   request   [cmd][idx_lo][idx_hi][sub][data0..3]
//!   0x580 + node_id   response  same layout; cmd 0x80 = abort, data = AbortCode
//! ```
//!
//! Initiate-download (write) and initiate-upload (read) only, expedited only, 1/2/4-byte
//! payloads. Segmented and block transfers are rejected with `UnsupportedAccess`, which costs
//! nothing because no object in [`od`] exceeds 4 bytes.
//!
//! ## Heartbeat
//!
//! ```text
//!   0x700 + master_node_id   IN   — resets this node's fallback timers
//!   0x700 + node_id          OUT  — our own liveness, period at 0x1017
//! ```
//!
//! Losing the incoming one is not a comms error to be logged; it is the trigger for the two-stage
//! fallback at [`od::FALLBACK_A_MS`], which is the whole reason a node can be left alone.
//!
//! ## Data plane — fixed TPDOs, no runtime PDO mapping, no RPDOs
//!
//! ```text
//!   COB-ID = 0x200 | (kind << 4) | node_id      (11-bit, kind is 5 bits)
//! ```
//!
//! Every payload is little-endian and exactly 8 bytes, so a decoder pulls any field out by fixed
//! offset without a length check. The broadcast period per kind is configured at
//! [`od::TPDO_INTERVAL_MS`]; 0 disables that kind. [`TpdoKind`] is the table and [`TpdoFrame`] is
//! the encoding.
//!
//! The mapping is fixed rather than runtime-negotiable on purpose: a master that has to read a
//! PDO mapping before it can decode a frame is exactly the complexity this protocol is avoiding.
//!
//! # Persistence
//!
//! Everything in the 0x3000 block is runtime-writable *and* persisted, on request, to the board's
//! NOR flash; nothing in 0x1000 or 0x2000 is. A node's compile-time per-node constants are
//! factory defaults only — a board with a valid stored config ignores them.
//!
//! ```text
//!   write "save" to 0x1010 sub 1  -> commit the 0x3000 block   (od::SIGNATURE_SAVE)
//!   write "load" to 0x1011 sub 1  -> erase it, revert to the compile-time defaults
//! ```
//!
//! # What is here and what is not
//!
//! Identifier layout, object indices and their meanings, frame encodings, and the handful of
//! constants a decoder cannot work without ([`od::SENSOR_INVALID`], the position word in
//! [`valve`]). Node-internal machinery — how a relief pulse is timed, how a stall is debounced,
//! how the NOR record is framed — stays in the firmware.
#![cfg_attr(not(test), no_std)]

pub mod ids;
pub mod od;
pub mod tpdo;
pub mod valve;

pub use ids::{TPDO_KINDS, TpdoKind, decode_pdo};
pub use tpdo::{HcoOutput, NUM_PROTOCOL_SENSOR_SLOTS, TpdoFrame};
pub use valve::{POSITION_MASK, PROMILLE_MAX, UNPOWERED_FLAG, is_unpowered, position_of, unpowered_at};
