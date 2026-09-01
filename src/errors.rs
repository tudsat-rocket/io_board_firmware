//! Error counters: what has gone wrong on this board, since boot, and how often.
//!
//! A log line says a thing went wrong once, to whoever happened to be attached with a probe. On a
//! vehicle nobody is attached, and the failures worth chasing are the ones that repeat: a
//! marginal I2C pull-up, a connector that opens over a bump, a servo that stalls on one valve and
//! not the others. A counter turns all of those from "did you see that message?" into a number a
//! master can poll, trend, and compare between boards.
//!
//! What each counter means is documented once, on [`ErrorCounter`] in the wire-protocol crate,
//! because the sub-index at [`crate::store::od::ERROR_COUNTERS`] is its discriminant. This module
//! is the mechanism.
//!
//! # Why atomics rather than the store
//!
//! Errors are detected in places that must not block: an I2C read inside the sensor task, the
//! bxCAN receive loop, the watchdog task, the panic handler. [`crate::store::STORE`] is behind an
//! async mutex, so counting there would mean either awaiting a lock on an error path or
//! restructuring the caller. A `fetch_add` on an [`AtomicU32`] is a handful of instructions,
//! needs no lock, works from an interrupt, and cannot deadlock — so [`bump`] is callable from
//! anywhere, including code that is not `async` at all.
//!
//! The store is then a *mirror*: [`crate::store::Store::refresh_error_counters`] copies the
//! snapshot in on the control tick and the SDO server serves it from there like any other object.
//! Nothing outside this module reads the atomics.
//!
//! # What these are, and are not
//!
//! Free-running, monotonic, and they wrap at `u32`. A master reads them periodically and looks at
//! the *difference*; nothing but a reset of the board clears them. There is deliberately no way
//! to zero them over the bus — a counter someone can reset is a counter you cannot trust to have
//! been counting since boot, and "it went back to zero" is itself the signal that the node
//! rebooted between two reads.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::index::{Id, PerErrorCounter};

pub use iocan_proto::od::ErrorCounter;

/// One free-running counter per [`ErrorCounter`].
///
/// `Relaxed` throughout: these order nothing. Every counter is independent, no other memory is
/// published through them, and a reader that catches one a tick stale learns nothing wrong — it
/// reads the settled value on the next poll. Anything stronger would put a barrier on paths that
/// are already busy handling a failure.
static COUNTS: [AtomicU32; ErrorCounter::ALL.len()] = [const { AtomicU32::new(0) }; ErrorCounter::ALL.len()];

/// Set by every [`bump`], cleared by [`take_if_changed`].
///
/// The mirror runs on the control tick, and on a healthy board nothing has changed since the last
/// one — so this turns the common case from reading every counter and writing the whole array
/// into the store into a single load.
static CHANGED: AtomicBool = AtomicBool::new(false);

/// Record one occurrence. Callable from anywhere, including interrupt context and non-`async`
/// code: no lock, no allocation, no `await`.
#[inline]
pub fn bump(which: ErrorCounter) {
    bump_by(which, 1);
}

/// Record `n` occurrences at once, for the cases that discover a batch at a time — the number of
/// messages a lagging queue dropped, say. Zero is a no-op and does not mark the counters changed.
#[inline]
pub fn bump_by(which: ErrorCounter, n: u32) {
    if n == 0 {
        return;
    }
    COUNTS[which.index()].fetch_add(n, Ordering::Relaxed);
    CHANGED.store(true, Ordering::Relaxed);
}

/// Read one counter.
pub fn count(which: ErrorCounter) -> u32 {
    COUNTS[which.index()].load(Ordering::Relaxed)
}

/// Read all of them.
///
/// Not atomic as a set: counters are read one at a time and another task may bump one in between.
/// That is fine for what these are — no invariant relates two counters, and the next poll picks
/// up whatever this one missed.
pub fn snapshot() -> PerErrorCounter<u32> {
    PerErrorCounter::from_fn(count)
}

/// A snapshot, but only when something has been counted since the last call. `None` is the
/// healthy case, and the reason this exists rather than just [`snapshot`].
pub fn take_if_changed() -> Option<PerErrorCounter<u32>> {
    // Cleared before the read, not after: a bump landing during the snapshot then sets the flag
    // again and is picked up next tick, instead of being swallowed by a clear that happens after
    // it.
    if !CHANGED.swap(false, Ordering::Relaxed) {
        return None;
    }
    Some(snapshot())
}

/// Zero every counter. Test-only: on a board these are meant to survive everything short of a
/// reset, and a master's whole use of them is differencing consecutive reads.
#[cfg(test)]
pub fn reset_all() {
    for counter in &COUNTS {
        counter.store(0, Ordering::Relaxed);
    }
    CHANGED.store(false, Ordering::Relaxed);
}

/// Serialises the tests that touch this module's global state, and any test elsewhere that
/// asserts on a counter. Held for the body of such a test.
///
/// Note `cargo test` runs test threads in parallel by default, so without this two tests that
/// both call [`reset_all`] would clear each other's counts.
#[cfg(test)]
pub fn test_lock() -> std::sync::MutexGuard<'static, ()> {
    static SERIAL: std::sync::Mutex<()> = std::sync::Mutex::new(());
    // A test that panicked while holding it poisoned the lock; the state it left behind is
    // irrelevant because every user calls `reset_all` first.
    SERIAL.lock().unwrap_or_else(|e| e.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counting_is_per_kind() {
        let _guard = test_lock();
        reset_all();

        bump(ErrorCounter::I2cTimeout);
        bump(ErrorCounter::I2cTimeout);
        bump(ErrorCounter::ValveStall);

        assert_eq!(count(ErrorCounter::I2cTimeout), 2);
        assert_eq!(count(ErrorCounter::ValveStall), 1);
        assert_eq!(count(ErrorCounter::CanRxOverrun), 0);
    }

    #[test]
    fn a_batch_counts_as_many_as_it_lost() {
        let _guard = test_lock();
        reset_all();

        bump_by(ErrorCounter::CanRxDropped, 7);
        assert_eq!(count(ErrorCounter::CanRxDropped), 7);
    }

    /// Skipping the mirror on a quiet board is the whole point of the flag.
    #[test]
    fn a_quiet_board_costs_nothing_to_mirror() {
        let _guard = test_lock();
        reset_all();

        assert!(take_if_changed().is_none(), "nothing has happened yet");

        bump(ErrorCounter::CanRxError);
        let snapshot = take_if_changed().expect("a bump must be visible to the mirror");
        assert_eq!(snapshot[ErrorCounter::CanRxError], 1);

        assert!(take_if_changed().is_none(), "and once mirrored it is quiet again");
    }

    #[test]
    fn a_zero_batch_is_not_an_event() {
        let _guard = test_lock();
        reset_all();

        bump_by(ErrorCounter::CanRxDropped, 0);
        assert!(take_if_changed().is_none(), "a lag of zero messages is not a loss");
    }

    /// A snapshot is indexed by the same id `bump` takes, so a mismatch between the `Id` impl and
    /// the wire order would show up here rather than as a master trending the wrong column.
    #[test]
    fn a_snapshot_lines_up_with_the_wire_order() {
        let _guard = test_lock();
        reset_all();

        bump_by(ErrorCounter::ValveStall, 3);
        let snapshot = snapshot();

        assert_eq!(snapshot[ErrorCounter::ValveStall], 3);
        assert_eq!(snapshot.as_slice()[ErrorCounter::ValveStall.sub() as usize - 1], 3);
    }
}
