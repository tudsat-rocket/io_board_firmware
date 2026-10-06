//! Turning the CAN controller's error state into error counts.
//!
//! bxCAN reports its fault confinement state as a register, not as events: whether it is
//! error-passive or bus-off now, and the current transmit and receive error counters. Nothing
//! interrupts on the transitions that we do not already hand to the driver, so the control tick
//! polls the register ([`super::error_status`] on hardware) and [`BusHealth`] works out what
//! changed since the previous poll.
//!
//! The awkward part is bus-off. The controller is configured to recover by itself (`ABOM`), which
//! takes 128 idle bus slots — a few milliseconds at 500 kbit/s — so a bus-off can begin and end
//! between two 20 ms polls. What it leaves behind is a transmit error counter reset to zero, and
//! that is the second way a bus-off is recognised here: the counter was at the warning level last
//! time and is zero now. Counting down that far takes one successful frame per step, about a
//! hundred, which this node does not send in one tick.

use crate::errors::{ErrorCounter, bump};

/// Where a transmit error counter at or above this, found at zero on the next poll, means a
/// bus-off came and went in between. The controller's own warning level.
const TEC_WARNING: u8 = 96;

/// One reading of the controller's error status register.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct ErrorStatus {
    pub bus_off: bool,
    pub passive: bool,
    /// Transmit error counter.
    pub tec: u8,
    /// Receive error counter.
    pub rec: u8,
}

/// The previous poll, which is all it takes to see a transition.
pub struct BusHealth {
    last: ErrorStatus,
}

impl BusHealth {
    /// Starts from the reset state the controller is in at boot: error-active, both counters zero.
    pub const fn new() -> Self {
        Self {
            last: ErrorStatus {
                bus_off: false,
                passive: false,
                tec: 0,
                rec: 0,
            },
        }
    }

    /// Count whatever happened since the previous call.
    pub fn observe(&mut self, now: ErrorStatus) {
        let last = core::mem::replace(&mut self.last, now);

        let entered_bus_off = !last.bus_off && now.bus_off;
        let recovered_unseen = !last.bus_off && !now.bus_off && last.tec >= TEC_WARNING && now.tec == 0;
        let bus_off = entered_bus_off || recovered_unseen;
        if bus_off {
            bump(ErrorCounter::CanBusOff);
        }

        // Every bus-off passes through error-passive on the way, so one we did not see the start
        // of still counts as an entry into it.
        if !last.passive && (now.passive || bus_off) {
            bump(ErrorCounter::CanErrorPassive);
        }

        // In bus-off the counters are not meaningful, and coming out of it resets them, which is
        // a fall rather than a rise anyway.
        if !last.bus_off && !now.bus_off && (now.tec > last.tec || now.rec > last.rec) {
            bump(ErrorCounter::CanErrorCountRose);
        }
    }
}

impl Default for BusHealth {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::errors::{count, reset_all, test_lock};

    fn status(tec: u8, rec: u8) -> ErrorStatus {
        ErrorStatus {
            bus_off: false,
            passive: tec > 127 || rec > 127,
            tec,
            rec,
        }
    }

    const BUS_OFF: ErrorStatus = ErrorStatus {
        bus_off: true,
        passive: true,
        tec: 248,
        rec: 0,
    };

    fn counts() -> (u32, u32, u32) {
        (count(ErrorCounter::CanErrorCountRose), count(ErrorCounter::CanErrorPassive), count(ErrorCounter::CanBusOff))
    }

    #[test]
    fn a_healthy_bus_counts_nothing() {
        let _guard = test_lock();
        reset_all();
        let mut health = BusHealth::new();
        for _ in 0..10 {
            health.observe(status(0, 0));
        }
        assert_eq!(counts(), (0, 0, 0));
    }

    #[test]
    fn a_rise_in_either_counter_is_counted_once_per_poll() {
        let _guard = test_lock();
        reset_all();
        let mut health = BusHealth::new();
        health.observe(status(16, 0));
        health.observe(status(16, 1));
        health.observe(status(15, 1)); // a successful frame counts back down
        assert_eq!(counts(), (2, 0, 0));
    }

    #[test]
    fn going_passive_counts_on_entry_only() {
        let _guard = test_lock();
        reset_all();
        let mut health = BusHealth::new();
        health.observe(status(136, 0));
        health.observe(status(136, 0));
        health.observe(status(100, 0)); // back to active
        health.observe(status(140, 0)); // and in again
        assert_eq!(counts().1, 2);
    }

    #[test]
    fn a_bus_off_seen_while_it_lasts_counts_once_and_its_recovery_does_not_count_again() {
        let _guard = test_lock();
        reset_all();
        let mut health = BusHealth::new();
        health.observe(status(136, 0));
        health.observe(BUS_OFF);
        health.observe(BUS_OFF);
        health.observe(status(0, 0));
        assert_eq!(counts(), (1, 1, 1), "the rise to passive, the entry into passive, the bus-off");
    }

    /// The common case with automatic recovery: the whole bus-off happens between two polls.
    #[test]
    fn a_bus_off_that_recovered_between_polls_is_still_counted() {
        let _guard = test_lock();
        reset_all();
        let mut health = BusHealth::new();
        health.observe(status(104, 0));
        health.observe(status(0, 0));
        assert_eq!(counts(), (1, 1, 1), "and it went through passive on the way");
    }

    /// Below the warning level, a fall to zero is just a quiet node's counter draining.
    #[test]
    fn a_low_counter_draining_to_zero_is_not_a_bus_off() {
        let _guard = test_lock();
        reset_all();
        let mut health = BusHealth::new();
        health.observe(status(8, 0));
        health.observe(status(0, 0));
        assert_eq!(counts(), (1, 0, 0));
    }
}
