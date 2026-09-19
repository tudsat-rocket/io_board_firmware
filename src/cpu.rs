//! CPU utilization, measured by accounting for idle time rather than for work.
//!
//! Every task on this board runs on the one thread-mode executor, and thread mode is the only
//! context that ever sleeps: an interrupt handler runs to completion and returns, so all the
//! slack in the system ends up in the `wfe` of [`sleep`]. Timing that `wfe` and subtracting it
//! from wall-clock time therefore yields whole-system utilization — every task, every interrupt
//! handler, the time driver, the flash driver — without instrumenting any of them, and without
//! any of them being able to escape the accounting by being busy somewhere this module has never
//! heard of.
//!
//! That is also why [`crate::node::run`] hand-rolls what `#[embassy_executor::main]` would
//! generate: the idle loop has to be ours for its `wfe` to be timed.
//!
//! ```text
//!   node::run  --poll--> tasks ...
//!        |                                  wall clock
//!        \--sleep()--> wfe --> IDLE_TICKS   ----------  =  load
//!                                            idle time
//! ```
//!
//! # What is published, and when
//!
//! [`CpuMonitor`] is driven by the control task (the one loop on this board with a deadline) and
//! rolls the measurement up in two stages: utilization over a [`WINDOW`], and the worst of
//! [`WINDOWS_PER_REPORT`] such windows published to [`iocan_proto::od::CPU_LOAD`] — peak rather
//! than average, because a board that is comfortable on average and saturated for a tenth of a
//! second at a time has a problem an average would hide.
//! [`crate::store::Store::refresh_cpu`] then mirrors the published values into the object
//! dictionary, the same way the error counters get there.
//!
//! # Why atomics
//!
//! For the same reason [`crate::errors`] uses them: [`sleep`] runs in the idle loop with
//! interrupts masked, where taking [`crate::store::STORE`]'s async mutex is not an option. The
//! monitor's own state is plain fields — it is owned by one task — and only the handoff from the
//! idle loop, and the handoff of the published numbers to whoever reads them, goes through an
//! atomic.

use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

use embassy_time::{Duration, Instant};

use crate::errors::{self, ErrorCounter};

pub use iocan_proto::od::CPU_LOAD_UNKNOWN;

/// One measurement window. Short enough that a burst of work shows up as a burst rather than
/// being averaged into the rest, long enough that the control tick's 20 ms granularity does not
/// dominate where the window boundaries land.
const WINDOW: Duration = Duration::from_millis(100);

/// Windows per published value — the reporting period is this many [`WINDOW`]s.
const WINDOWS_PER_REPORT: u32 = 5;

/// Highest utilization that can be reported, matching the promille everything else on this board
/// is expressed in.
const PERMILLE_FULL: u64 = 1000;

/// Time spent in [`sleep`] since the last window rollover, in embassy ticks.
///
/// Ticks rather than microseconds so no conversion (and no rounding) happens on the idle path;
/// the window arithmetic divides one tick count by another, so the tick rate cancels out and
/// never has to be known here.
///
/// Drained every [`WINDOW`], so the `u32` has to hold a tenth of a second of idle time and wraps
/// only if the control task stops draining it for the better part of an hour — by which point
/// the watchdog has long since reset the board.
static IDLE_TICKS: AtomicU32 = AtomicU32::new(0);

/// Last published utilization, permille, or [`CPU_LOAD_UNKNOWN`] before the first report.
static LOAD_PERMILLE: AtomicU16 = AtomicU16::new(CPU_LOAD_UNKNOWN);

/// Last published worst control-loop iteration, microseconds.
static PEAK_TICK_US: AtomicU16 = AtomicU16::new(0);

/// The most recent utilization estimate, permille, or `None` before the first report completes.
pub fn load_permille() -> Option<u16> {
    match LOAD_PERMILLE.load(Ordering::Relaxed) {
        CPU_LOAD_UNKNOWN => None,
        permille => Some(permille),
    }
}

/// The same value in its wire form, sentinel included — what 0x2034 serves.
pub fn load_word() -> u16 {
    LOAD_PERMILLE.load(Ordering::Relaxed)
}

/// Worst control-loop iteration of the last reporting period, microseconds. What 0x2035 serves.
pub fn peak_tick_us() -> u16 {
    PEAK_TICK_US.load(Ordering::Relaxed)
}

/// Charge `idle` to the idle account.
///
/// Called by [`sleep`] on hardware; exposed unconditionally so the window arithmetic can be
/// driven from a host test without an idle loop to drive it.
pub fn record_idle(idle: Duration) {
    let ticks = u32::try_from(idle.as_ticks()).unwrap_or(u32::MAX);
    IDLE_TICKS.fetch_add(ticks, Ordering::Relaxed);
}

/// `SCB_SCR.SEVONPEND`.
#[cfg(feature = "hardware")]
const SCR_SEVONPEND: u32 = 1 << 4;

/// Set SEVONPEND, without which [`sleep`]'s `wfe` would never return.
///
/// `wfe` treats an interrupt as a wake-up event only if that interrupt could actually preempt,
/// and [`sleep`] runs with PRIMASK set so that nothing can. SEVONPEND widens the condition to
/// *any* interrupt entering the pending state, masked or not, which is exactly the case we sleep
/// in. It leaves the event register semantics intact, so the executor pender's `sev` still closes
/// the race where a task is woken between the last poll and the `wfe`.
///
/// Must run before the first [`sleep`].
#[cfg(feature = "hardware")]
pub fn init() {
    // SAFETY: single core, called once at boot from the idle loop before any task runs; nothing
    // else in this firmware touches the SCB.
    unsafe {
        let scb = &*cortex_m::peripheral::SCB::PTR;
        scb.scr.modify(|scr| scr | SCR_SEVONPEND);
    }
}

/// Sleep until the executor has work again, charging the time to the idle account.
///
/// Interrupts are masked across the whole of it so that the measurement cannot be split by a
/// handler running between the `wfe` returning and the clock being read — the handler's own time
/// would then be counted as idle. They are pending, not lost: PRIMASK drops on the way out and
/// everything that arrived runs immediately, before the next poll.
#[cfg(feature = "hardware")]
pub fn sleep() {
    cortex_m::interrupt::free(|_| {
        let start = Instant::now();
        cortex_m::asm::wfe();
        record_idle(start.elapsed());
    });
}

/// Rolls the measurement windows up and publishes the result. Owned by the control task.
pub struct CpuMonitor {
    /// The deadline one iteration of the owning loop is expected to meet.
    deadline: Duration,
    window_start: Instant,
    /// Completed windows since the last report.
    windows: u32,
    peak_permille: u16,
    peak_tick_us: u16,
}

impl CpuMonitor {
    /// `deadline` is the owning loop's tick period: an iteration longer than this has missed it
    /// and is counted as [`ErrorCounter::ControlTickOverrun`].
    pub fn new(now: Instant, deadline: Duration) -> Self {
        Self {
            deadline,
            window_start: now,
            windows: 0,
            peak_permille: 0,
            peak_tick_us: 0,
        }
    }

    /// Call once per iteration of the owning loop, with how long that iteration's work took.
    ///
    /// `work` is wall-clock and includes whatever the iteration awaited, so it is the loop's
    /// latency rather than its CPU time — the two differ, and it is the latency that decides
    /// whether the outputs are being updated on time.
    pub fn update(&mut self, work: Duration, now: Instant) {
        if work > self.deadline {
            errors::bump(ErrorCounter::ControlTickOverrun);
        }
        self.peak_tick_us = self.peak_tick_us.max(u16::try_from(work.as_micros()).unwrap_or(u16::MAX));

        let elapsed = now.saturating_duration_since(self.window_start);
        if elapsed < WINDOW {
            return;
        }

        // Advance by `elapsed` rather than to `now` so consecutive windows tile exactly, with no
        // sliver of time falling outside any of them.
        self.window_start += elapsed;

        // Measured rather than nominal, which is what lets the window be rolled by a 20 ms tick
        // without the ratio drifting: a window that overran to 120 ms divides by 120 ms.
        let total = elapsed.as_ticks();
        let idle = u64::from(IDLE_TICKS.swap(0, Ordering::Relaxed));
        let busy = total.saturating_sub(idle);
        if total == 0 {
            return;
        }

        // `busy <= total` keeps this at or below PERMILLE_FULL, and a window of ticks times 1000
        // is nowhere near u64.
        let permille = (busy * PERMILLE_FULL / total) as u16;
        self.peak_permille = self.peak_permille.max(permille);
        self.windows = self.windows.saturating_add(1);

        if self.windows >= WINDOWS_PER_REPORT {
            LOAD_PERMILLE.store(self.peak_permille, Ordering::Relaxed);
            PEAK_TICK_US.store(self.peak_tick_us, Ordering::Relaxed);
            self.peak_permille = 0;
            self.peak_tick_us = 0;
            self.windows = 0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// This module's published values are globals too, so every test here holds
    /// [`errors::test_lock`] — which serialises them against each other as well as against
    /// anything asserting on the overrun counter.
    fn clear() {
        LOAD_PERMILLE.store(CPU_LOAD_UNKNOWN, Ordering::Relaxed);
        PEAK_TICK_US.store(0, Ordering::Relaxed);
        IDLE_TICKS.store(0, Ordering::Relaxed);
        errors::reset_all();
    }

    fn monitor_at(start: Instant) -> CpuMonitor {
        CpuMonitor::new(start, Duration::from_millis(20))
    }

    /// Run one reporting period's worth of windows, each `WINDOW` long with `idle` of it spent
    /// asleep, and return what ended up published.
    fn report(idle: Duration, work: Duration) -> (u16, u16) {
        clear();
        let start = Instant::from_millis(0);
        let mut monitor = monitor_at(start);
        for window in 1..=WINDOWS_PER_REPORT as u64 {
            record_idle(idle);
            monitor.update(work, start + WINDOW * window as u32);
        }
        (load_word(), peak_tick_us())
    }

    #[test]
    fn a_board_that_never_sleeps_reads_fully_loaded() {
        let _guard = errors::test_lock();
        let (load, _) = report(Duration::from_millis(0), Duration::from_millis(1));
        assert_eq!(load, 1000);
    }

    #[test]
    fn a_board_that_sleeps_through_the_window_reads_idle() {
        let _guard = errors::test_lock();
        let (load, _) = report(WINDOW, Duration::from_millis(1));
        assert_eq!(load, 0);
    }

    #[test]
    fn a_quarter_of_the_window_awake_reads_a_quarter() {
        let _guard = errors::test_lock();
        let (load, _) = report(WINDOW * 3 / 4, Duration::from_millis(1));
        assert_eq!(load, 250);
    }

    /// More idle than elapsed means the two clocks disagree, not that the board was more than
    /// idle: report "nothing to do" rather than wrapping into a wildly loaded reading.
    #[test]
    fn more_idle_than_elapsed_clamps_to_zero() {
        let _guard = errors::test_lock();
        let (load, _) = report(WINDOW * 2, Duration::from_millis(1));
        assert_eq!(load, 0);
    }

    #[test]
    fn nothing_is_published_before_a_full_reporting_period() {
        let _guard = errors::test_lock();
        clear();

        let start = Instant::from_millis(0);
        let mut monitor = monitor_at(start);
        for window in 1..WINDOWS_PER_REPORT as u64 {
            monitor.update(Duration::from_millis(1), start + WINDOW * window as u32);
        }
        assert_eq!(load_permille(), None);
    }

    /// Both published numbers are the worst of the period, not the last of it.
    #[test]
    fn the_peak_is_what_gets_reported() {
        let _guard = errors::test_lock();
        clear();

        let start = Instant::from_millis(0);
        let mut monitor = monitor_at(start);
        for window in 1..=WINDOWS_PER_REPORT as u64 {
            // One busy window in five: idle for all of the others.
            let (idle, work) = if window == 2 {
                (Duration::from_millis(0), Duration::from_micros(9000))
            } else {
                (WINDOW, Duration::from_micros(100))
            };
            record_idle(idle);
            monitor.update(work, start + WINDOW * window as u32);
        }
        assert_eq!(load_word(), 1000);
        assert_eq!(peak_tick_us(), 9000);
    }

    #[test]
    fn an_iteration_longer_than_the_deadline_is_counted() {
        let _guard = errors::test_lock();
        clear();

        let start = Instant::from_millis(0);
        let mut monitor = monitor_at(start);
        monitor.update(Duration::from_millis(19), start);
        assert_eq!(errors::count(ErrorCounter::ControlTickOverrun), 0);
        monitor.update(Duration::from_millis(21), start);
        assert_eq!(errors::count(ErrorCounter::ControlTickOverrun), 1);
    }
}
