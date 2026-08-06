//! Persisting [`Config`] to the on-board W25Q128 NOR flash.
//!
//! Two 4 KiB sectors hold a config record each and are written alternately, so a power loss
//! during a save can only ever destroy the copy that was already the older of the two. Each
//! record carries a sequence number and a CRC32; the highest sequence that passes its CRC wins.
//! Nothing else on the board uses this chip, so the whole thing lives in the first 8 KiB of a
//! 16 MiB part and the remaining space is free for a future log.
//!
//! There is deliberately no wear levelling beyond the alternation. Config writes happen when a
//! human commits a calibration, not in a loop, and a W25Q sector is good for 100k erases.

use embedded_storage_async::nor_flash::NorFlash;

use super::{
    Config, FallbackAction, PressureCalib, ReliefConfig, SensorKind, SensorSlotConfig, Unit, ValveConfig, ValveKind,
};
use crate::index::{AmplifierId, HcoId, I2cBus, Id, SensorSlot, ValveId};

const MAGIC: u32 = 0x4249_4F43; // "COIB", little-endian "IOCB"

/// Bump whenever the body layout changes. A record written by another version is rejected rather
/// than misparsed, and the board falls back to its compile-time defaults — which is the safe
/// outcome, since a config half-read into valve hardware is worse than no config at all.
///
/// 2: pressure calibration gained a constant term (`PressureCalib::constant_millibar`).
/// 3: added the overpressure relief loop (`ReliefConfig`).
const VERSION: u16 = 3;

const HEADER_LEN: usize = 12;
const BODY_LEN: usize = 291;
#[cfg(test)]
const RECORD_LEN: usize = HEADER_LEN + BODY_LEN + 4;

/// Padded to a comfortable margin over `RECORD_LEN` so a future field does not force a format
/// bump just to fit.
const BUF_LEN: usize = 320;

const SECTOR_LEN: u32 = 4096;
const SLOT_OFFSETS: [u32; 2] = [0, SECTOR_LEN];

/// Sentinel for an `Option<u8>` field in the serialized form.
const NONE_U8: u8 = 0xFF;

const CRC: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_ISO_HDLC);

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum PersistError {
    /// The flash rejected a read, write or erase.
    Flash,
    /// Neither sector holds a record we can use. Not an error at boot — it is what a virgin
    /// board looks like — but the caller has to fall back to the compile-time defaults.
    NoValidRecord,
    /// A record was structurally sound but described a configuration we refuse to run.
    Invalid,
}

// ---------------------------------------------------------------------------
// Fixed-layout serialization
// ---------------------------------------------------------------------------
//
// Hand-rolled rather than serde: the layout is a wire format we have to keep stable across
// firmware versions anyway, and postcard/serde derive costs flash we do not have. Everything is
// little-endian, matching the SDO payloads.

struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> Writer<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    fn u8(&mut self, v: u8) {
        self.buf[self.pos] = v;
        self.pos += 1;
    }

    fn u16(&mut self, v: u16) {
        self.buf[self.pos..self.pos + 2].copy_from_slice(&v.to_le_bytes());
        self.pos += 2;
    }

    fn u32(&mut self, v: u32) {
        self.buf[self.pos..self.pos + 4].copy_from_slice(&v.to_le_bytes());
        self.pos += 4;
    }

    fn i32(&mut self, v: i32) {
        self.u32(v as u32);
    }

    /// An optional id, as its raw index with [`NONE_U8`] for `None`.
    fn opt_id<I: Id>(&mut self, v: Option<I>) {
        self.u8(v.map_or(NONE_U8, |id| id.index() as u8));
    }

    fn id<I: Id>(&mut self, v: I) {
        self.u8(v.index() as u8);
    }
}

struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    fn u8(&mut self) -> u8 {
        let v = self.buf[self.pos];
        self.pos += 1;
        v
    }

    fn u16(&mut self) -> u16 {
        let v = u16::from_le_bytes([self.buf[self.pos], self.buf[self.pos + 1]]);
        self.pos += 2;
        v
    }

    fn u32(&mut self) -> u32 {
        let v = u32::from_le_bytes(self.buf[self.pos..self.pos + 4].try_into().unwrap());
        self.pos += 4;
        v
    }

    fn i32(&mut self) -> i32 {
        self.u32() as i32
    }

    /// An optional id. Returns `Err(())` for a value that is neither [`NONE_U8`] nor a valid index
    /// of the domain, which fails the whole record.
    ///
    /// This is where the typed indices pay for themselves at the flash boundary: the old version
    /// returned `Option<u8>` unchecked, so a corrupted record could put e.g. `signal_hco = 200`
    /// into a `ValveConfig`. Nothing downstream rejected it — `sanity_check` never looked at the
    /// range, and the control task's output write silently dropped an out-of-range index — so the
    /// board came up looking configured with a valve that could not move.
    fn opt_id<I: Id>(&mut self) -> Result<Option<I>, ()> {
        match self.u8() {
            NONE_U8 => Ok(None),
            v => I::from_index(v as usize).map(Some).ok_or(()),
        }
    }

    fn id<I: Id>(&mut self) -> Result<I, ()> {
        I::from_index(self.u8() as usize).ok_or(())
    }
}

/// Serialize the config body. Panics only if `BODY_LEN` and this function disagree, which a
/// debug assertion catches on the first save.
fn write_body(cfg: &Config, out: &mut [u8]) -> usize {
    let mut w = Writer::new(out);

    w.u8(cfg.master_node_id);
    w.u8(cfg.fallback_enabled as u8);
    w.u16(cfg.heartbeat_period_ms);
    w.u32(cfg.fallback_a_ms);
    w.u32(cfg.fallback_b_ms);
    w.u16(cfg.sensor_interval_ms);
    w.u16(cfg.scan_interval_ms);

    for v in cfg.valves.values() {
        w.u8(v.kind as u8);
        w.opt_id(v.power_hco);
        w.opt_id(v.signal_hco);
        w.u16(v.closed_us);
        w.u16(v.open_us);
        w.u16(v.travel_ms);
        w.u16(v.stall_ma);
        w.u16(v.stall_ms);
        w.u16(v.settle_ms);
        w.u16(v.min_promille);
        w.u16(v.max_promille);
        w.u16(v.fallback_a.position);
        w.u8(v.fallback_a.unpower as u8);
        w.u16(v.fallback_b.position);
        w.u8(v.fallback_b.unpower as u8);
    }

    for s in cfg.sensors.values() {
        w.u8(s.kind as u8);
        w.opt_id(s.bus);
        w.id(s.amplifier);
        w.u8(s.unit as u8);
        w.i32(s.calib.offset_milli_counts);
        w.i32(s.calib.slope_nanobar);
        w.i32(s.calib.constant_millibar);
    }

    for period in cfg.tpdo_interval_ms.values() {
        w.u16(*period);
    }

    w.u8(cfg.relief.enabled as u8);
    w.opt_id(cfg.relief.valve);
    w.id(cfg.relief.sensor);
    w.u16(cfg.relief.threshold as u16);
    w.u16(cfg.relief.position);
    w.u16(cfg.relief.pulse_ms);
    w.u16(cfg.relief.cooldown_ms);

    w.pos
}

/// Parse a config body. Any out-of-range enum discriminant fails the whole record rather than
/// being silently coerced — a garbled config is not something to half-apply to valve hardware.
fn read_body(body: &[u8]) -> Option<Config> {
    if body.len() < BODY_LEN {
        return None;
    }
    let mut r = Reader::new(body);
    let mut cfg = Config::new();

    cfg.master_node_id = r.u8();
    cfg.fallback_enabled = r.u8() != 0;
    cfg.heartbeat_period_ms = r.u16();
    cfg.fallback_a_ms = r.u32();
    cfg.fallback_b_ms = r.u32();
    cfg.sensor_interval_ms = r.u16();
    cfg.scan_interval_ms = r.u16();

    for i in ValveId::ALL {
        let kind = ValveKind::from_u8(r.u8())?;
        let power_hco: Option<HcoId> = r.opt_id().ok()?;
        let signal_hco: Option<HcoId> = r.opt_id().ok()?;
        cfg.valves[i] = ValveConfig {
            kind,
            power_hco,
            signal_hco,
            closed_us: r.u16(),
            open_us: r.u16(),
            travel_ms: r.u16(),
            stall_ma: r.u16(),
            stall_ms: r.u16(),
            settle_ms: r.u16(),
            min_promille: r.u16(),
            max_promille: r.u16(),
            fallback_a: FallbackAction {
                position: r.u16(),
                unpower: r.u8() != 0,
            },
            fallback_b: FallbackAction {
                position: r.u16(),
                unpower: r.u8() != 0,
            },
        };
    }

    for i in SensorSlot::ALL {
        let kind = SensorKind::from_u8(r.u8())?;
        let bus: Option<I2cBus> = r.opt_id().ok()?;
        let amplifier: AmplifierId = r.id().ok()?;
        let unit = Unit::from_u8(r.u8())?;
        cfg.sensors[i] = SensorSlotConfig {
            kind,
            bus,
            amplifier,
            unit,
            calib: PressureCalib {
                offset_milli_counts: r.i32(),
                slope_nanobar: r.i32(),
                constant_millibar: r.i32(),
            },
        };
    }

    for kind in iocan_proto::TPDO_KINDS {
        cfg.tpdo_interval_ms[kind] = r.u16();
    }

    cfg.relief = ReliefConfig {
        enabled: r.u8() != 0,
        valve: r.opt_id().ok()?,
        sensor: r.id().ok()?,
        threshold: r.u16() as i16,
        position: r.u16(),
        pulse_ms: r.u16(),
        cooldown_ms: r.u16(),
    };

    Some(cfg)
}

/// Serialize just the body, for comparing a candidate save against what is already on the chip.
fn serialize_body(cfg: &Config) -> [u8; BODY_LEN] {
    let mut buf = [0u8; BODY_LEN];
    let len = write_body(cfg, &mut buf);
    debug_assert_eq!(len, BODY_LEN, "BODY_LEN is out of date with write_body");
    buf
}

fn write_record(cfg: &Config, sequence: u32, buf: &mut [u8; BUF_LEN]) -> usize {
    let body_len = write_body(cfg, &mut buf[HEADER_LEN..]);
    debug_assert_eq!(body_len, BODY_LEN, "BODY_LEN is out of date with write_body");

    let mut header = Writer::new(&mut buf[..HEADER_LEN]);
    header.u32(MAGIC);
    header.u16(VERSION);
    header.u16(body_len as u16);
    header.u32(sequence);

    let end = HEADER_LEN + body_len;
    let checksum = CRC.checksum(&buf[..end]);
    buf[end..end + 4].copy_from_slice(&checksum.to_le_bytes());
    end + 4
}

/// Validate a record and return its sequence number and parsed config.
fn read_record(buf: &[u8]) -> Option<(u32, Config)> {
    if buf.len() < HEADER_LEN + 4 {
        return None;
    }
    let mut header = Reader::new(buf);
    if header.u32() != MAGIC || header.u16() != VERSION {
        return None;
    }
    let body_len = header.u16() as usize;
    let sequence = header.u32();

    let end = HEADER_LEN.checked_add(body_len)?;
    if end + 4 > buf.len() {
        return None;
    }
    let stored = u32::from_le_bytes(buf[end..end + 4].try_into().ok()?);
    if CRC.checksum(&buf[..end]) != stored {
        return None;
    }

    read_body(&buf[HEADER_LEN..end]).map(|cfg| (sequence, cfg))
}

// ---------------------------------------------------------------------------
// The NOR-backed store
// ---------------------------------------------------------------------------

pub struct NorConfigStore<F: NorFlash> {
    flash: F,
    /// Sequence number of the record currently on the chip, so a save can succeed it. `None`
    /// until the first `load`.
    last_sequence: Option<u32>,
    /// Which slot the newest record lives in, so a save goes to the other one.
    last_slot: usize,
    /// Body of the record currently on the chip, so `save` can tell a no-op commit from a real
    /// one. A master that resends its whole config on every heartbeat and re-triggers 0x1010
    /// would otherwise erase and rewrite a sector for values that did not change; the W25Q is
    /// only good for 100k erases per sector and there is no wear levelling beyond the two-slot
    /// alternation, so that cost is worth avoiding. `None` until the first load or save.
    last_body: Option<[u8; BODY_LEN]>,
}

impl<F: NorFlash> NorConfigStore<F> {
    pub fn new(flash: F) -> Self {
        Self {
            flash,
            last_sequence: None,
            last_slot: 1,
            last_body: None,
        }
    }

    /// Read both slots and return the newest valid config.
    ///
    /// A virgin board returns [`PersistError::NoValidRecord`], which is the caller's cue to keep
    /// the compile-time factory defaults.
    pub async fn load(&mut self) -> Result<Config, PersistError> {
        let mut buf = [0u8; BUF_LEN];
        let mut best: Option<(u32, usize, Config)> = None;

        for (slot, offset) in SLOT_OFFSETS.iter().enumerate() {
            if self.flash.read(*offset, &mut buf).await.is_err() {
                // A read failure on one slot should not hide a good record in the other.
                defmt::warn!("config: read of slot {} failed", slot);
                continue;
            }
            let Some((sequence, cfg)) = read_record(&buf) else {
                continue;
            };
            if best.as_ref().is_none_or(|(best_seq, _, _)| sequence > *best_seq) {
                best = Some((sequence, slot, cfg));
            }
        }

        let Some((sequence, slot, cfg)) = best else {
            return Err(PersistError::NoValidRecord);
        };

        if let Err(e) = cfg.sanity_check() {
            defmt::error!("config: stored record is not usable: {}", e);
            return Err(PersistError::Invalid);
        }

        self.last_sequence = Some(sequence);
        self.last_slot = slot;
        self.last_body = Some(serialize_body(&cfg));
        defmt::info!("config: loaded from NOR slot {}, sequence {}", slot, sequence);
        Ok(cfg)
    }

    /// Commit a config to the slot that is *not* currently newest, then adopt it as newest.
    ///
    /// A no-op if `cfg` serializes to exactly what is already stored — see `last_body`.
    pub async fn save(&mut self, cfg: &Config) -> Result<(), PersistError> {
        cfg.sanity_check().map_err(|e| {
            defmt::error!("config: refusing to persist an invalid config: {}", e);
            PersistError::Invalid
        })?;

        let body = serialize_body(cfg);
        if self.last_body == Some(body) {
            defmt::debug!("config: save requested but config is unchanged, not touching flash");
            return Ok(());
        }

        let slot = 1 - self.last_slot;
        let offset = SLOT_OFFSETS[slot];
        let sequence = self.last_sequence.map_or(1, |s| s.wrapping_add(1));

        let mut buf = [0u8; BUF_LEN];
        let len = write_record(cfg, sequence, &mut buf);

        self.flash.erase(offset, offset + SECTOR_LEN).await.map_err(|_| PersistError::Flash)?;
        self.flash.write(offset, &buf[..len]).await.map_err(|_| PersistError::Flash)?;

        self.last_sequence = Some(sequence);
        self.last_slot = slot;
        self.last_body = Some(body);
        defmt::info!("config: saved to NOR slot {}, sequence {}", slot, sequence);
        Ok(())
    }

    /// Erase both slots so the next boot comes up on the compile-time factory defaults.
    pub async fn erase_all(&mut self) -> Result<(), PersistError> {
        for offset in SLOT_OFFSETS {
            self.flash.erase(offset, offset + SECTOR_LEN).await.map_err(|_| PersistError::Flash)?;
        }
        self.last_sequence = None;
        self.last_slot = 1;
        self.last_body = None;
        defmt::info!("config: NOR erased, factory defaults apply on next boot");
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The record has to survive a round trip byte for byte, because a mismatch between the
    /// writer and the reader would only show up as a board that silently forgets its calibration.
    #[test]
    fn record_round_trips() {
        let mut cfg = Config::new();
        cfg.master_node_id = 4;
        cfg.valves[ValveId::Valve1] = ValveConfig::servo_on_pair(crate::index::HcoPair::A, 2470, 500, 1200);
        cfg.sensors[SensorSlot::Slot2] = SensorSlotConfig::pressure(
            I2cBus::Bus1,
            AmplifierId::Amp3,
            Unit::DeciBar,
            PressureCalib::from_bar_per_count(439.0, 0.911_161_7),
        );

        let mut buf = [0u8; BUF_LEN];
        let len = write_record(&cfg, 7, &mut buf);
        assert_eq!(len, RECORD_LEN);

        let (sequence, back) = read_record(&buf).expect("record should validate");
        assert_eq!(sequence, 7);
        assert_eq!(back.master_node_id, 4);
        assert_eq!(back.valves[ValveId::Valve1].closed_us, 2470);
        assert_eq!(back.valves[ValveId::Valve1].power_hco, Some(HcoId::Hco0));
        assert_eq!(back.sensors[SensorSlot::Slot2].unit as u8, Unit::DeciBar as u8);
        assert_eq!(
            back.sensors[SensorSlot::Slot2].calib.slope_nanobar,
            cfg.sensors[SensorSlot::Slot2].calib.slope_nanobar
        );
    }

    /// The constant term is the difference between gauge and absolute pressure for a slot, so
    /// losing it across a save would silently shift every reading by a bar.
    #[test]
    fn the_calibration_constant_survives_a_round_trip() {
        let mut cfg = Config::new();
        cfg.sensors[SensorSlot::Slot0] = SensorSlotConfig::pressure(
            I2cBus::Bus0,
            AmplifierId::Amp0,
            Unit::CentiBar,
            PressureCalib::from_bar_per_count(47.0, 0.106_044_5).with_constant_bar(1.013),
        );
        // ...and a slot on the plain form keeps its zero.
        cfg.sensors[SensorSlot::Slot1] = SensorSlotConfig::pressure(
            I2cBus::Bus0,
            AmplifierId::Amp1,
            Unit::CentiBar,
            PressureCalib::from_bar_per_count(15.0, 0.0855),
        );

        let mut buf = [0u8; BUF_LEN];
        write_record(&cfg, 1, &mut buf);
        let (_, back) = read_record(&buf).expect("record should validate");

        assert_eq!(back.sensors[SensorSlot::Slot0].calib.constant_millibar, 1013);
        assert_eq!(back.sensors[SensorSlot::Slot1].calib.constant_millibar, 0);
    }

    #[test]
    fn corrupted_record_is_rejected() {
        let cfg = Config::new();
        let mut buf = [0u8; BUF_LEN];
        write_record(&cfg, 1, &mut buf);
        buf[HEADER_LEN + 3] ^= 0xFF;
        assert!(read_record(&buf).is_none());
    }

    #[test]
    fn erased_flash_is_not_a_record() {
        assert!(read_record(&[0xFF; BUF_LEN]).is_none());
    }

    /// An index that is not a valid id of its domain fails the whole record.
    ///
    /// This used to get through: `power_hco`/`signal_hco` were read as raw `Option<u8>` with no
    /// range check, `sanity_check` only checked sharing and clamps, and `control::apply_drive`
    /// then dropped the out-of-range output silently — so a board would come up reporting a
    /// configured valve that could never move. Typing the field as `Option<HcoId>` moves the check
    /// to the one place that can still reject, which is here.
    ///
    /// The CRC is recomputed after the corruption so this tests the id parse specifically, not the
    /// checksum that would otherwise catch it first.
    #[test]
    fn an_out_of_range_index_fails_the_record() {
        // Byte offsets into the body: the per-valve block starts after the 16-byte scalar prefix,
        // and within a valve `kind` comes first, then power_hco, then signal_hco.
        const VALVE_BLOCK: usize = 16;
        for (offset, what) in [(VALVE_BLOCK + 1, "power_hco"), (VALVE_BLOCK + 2, "signal_hco")] {
            let mut cfg = Config::new();
            cfg.valves[ValveId::Valve0] = ValveConfig::servo_on_pair(crate::index::HcoPair::A, 2000, 1000, 500);

            let mut buf = [0u8; BUF_LEN];
            let len = write_record(&cfg, 1, &mut buf);
            assert!(read_record(&buf).is_some(), "the record should be good before corrupting it");

            // 4 is one past the last output (HCO4 is index 3), and not the 0xFF "none" sentinel.
            buf[HEADER_LEN + offset] = 4;
            let end = len - 4;
            let checksum = CRC.checksum(&buf[..end]).to_le_bytes();
            buf[end..len].copy_from_slice(&checksum);

            assert!(read_record(&buf).is_none(), "{what} = 4 must fail the record, not be accepted");
        }
    }

    // -----------------------------------------------------------------------
    // NorConfigStore::save, against an in-memory flash. No async executor is
    // available on the host build (see the no-dev-dependencies note in Cargo.toml), so `block_on`
    // busy-polls with a no-op waker; every future here resolves on the first poll.
    // -----------------------------------------------------------------------

    use embedded_storage_async::nor_flash::{ErrorType, ReadNorFlash};

    struct MockFlash {
        data: [u8; (SECTOR_LEN * 2) as usize],
        writes: u32,
    }

    impl MockFlash {
        fn new() -> Self {
            Self {
                data: [0xFF; (SECTOR_LEN * 2) as usize],
                writes: 0,
            }
        }
    }

    impl ErrorType for MockFlash {
        type Error = core::convert::Infallible;
    }

    impl ReadNorFlash for MockFlash {
        const READ_SIZE: usize = 1;

        async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let offset = offset as usize;
            bytes.copy_from_slice(&self.data[offset..offset + bytes.len()]);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.data.len()
        }
    }

    impl NorFlash for MockFlash {
        const WRITE_SIZE: usize = 1;
        const ERASE_SIZE: usize = SECTOR_LEN as usize;

        async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            self.data[from as usize..to as usize].fill(0xFF);
            Ok(())
        }

        async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            self.writes += 1;
            let offset = offset as usize;
            self.data[offset..offset + bytes.len()].copy_from_slice(bytes);
            Ok(())
        }
    }

    fn block_on<F: core::future::Future>(fut: F) -> F::Output {
        use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

        fn noop(_: *const ()) {}
        fn clone(_: *const ()) -> RawWaker {
            RawWaker::new(core::ptr::null(), &VTABLE)
        }
        static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);

        let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
        let mut cx = Context::from_waker(&waker);
        let mut fut = core::pin::pin!(fut);
        loop {
            if let Poll::Ready(val) = fut.as_mut().poll(&mut cx) {
                return val;
            }
        }
    }

    /// A lazy master that keeps resending the same config and re-triggering 0x1010 must not
    /// keep erasing and rewriting the sector — that is exactly the flash-wear scenario
    /// `last_body` exists to short-circuit.
    #[test]
    fn save_skips_flash_write_when_config_is_unchanged() {
        let mut store = NorConfigStore::new(MockFlash::new());
        let cfg = Config::new();

        block_on(store.save(&cfg)).expect("first save should succeed");
        assert_eq!(store.flash.writes, 1);

        block_on(store.save(&cfg)).expect("resaving the same config should succeed");
        assert_eq!(store.flash.writes, 1, "unchanged config must not touch flash again");

        let mut changed = cfg.clone();
        changed.master_node_id = 5;
        block_on(store.save(&changed)).expect("saving a changed config should succeed");
        assert_eq!(store.flash.writes, 2, "a real change must still be persisted");
    }

    /// A config loaded from flash is the baseline for comparison too: resaving exactly what was
    /// just loaded should not write anything.
    #[test]
    fn save_after_load_of_the_same_config_is_a_no_op() {
        let mut store = NorConfigStore::new(MockFlash::new());
        let cfg = Config::new();
        block_on(store.save(&cfg)).unwrap();
        assert_eq!(store.flash.writes, 1);

        let mut reloaded = NorConfigStore::new(MockFlash {
            data: store.flash.data,
            writes: 0,
        });
        let loaded = block_on(reloaded.load()).expect("should load the record just saved");

        block_on(reloaded.save(&loaded)).expect("resaving the loaded config should succeed");
        assert_eq!(reloaded.flash.writes, 0, "resaving what was just loaded must not touch flash");
    }
}
