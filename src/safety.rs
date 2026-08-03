//! Master liveness and the two-stage fallback.
//!
//! Actions if heartbeat from master is not received for a configurable time.
//! - **Stage A**, after 3 s by default: *make the vehicle safe*. Every valve closes.
//! - **Stage B**, after 5 minutes by default: *the master is not coming back*. Every valve opens,
//!   venting the vehicle rather than leaving it pressurised indefinitely.
//!
//! Both timeouts, both sets of positions, and whether each valve is released once it gets there
//! are runtime-configurable per valve (0x3001..0x3006).
//!
//! The timers run from boot, not from the first heartbeat. A node that never hears a master ends
//! up in the same state as one that lost it, which is what you want for a board that came up
//! after the master died.
//!
//! Raw debug mode suspends all of this. That matters during assembly: a board on a bench with no
//! master would otherwise drive its valves open five minutes after power-up.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_time::Instant;

use crate::config::{Config, FallbackAction, ValveConfig};
use crate::index::{PerValve, ValveId};
use crate::store::LinkState;
use crate::valves::unpowered_at;

/// Milliseconds since boot at which the last master heartbeat arrived.
///
/// An atomic rather than a store field because the CAN receive path touches it on every heartbeat
/// frame and must not contend for the store lock to do so. Wraps after ~49 days of uptime, which
/// no flight approaches.
static LAST_HEARTBEAT_MS: AtomicU32 = AtomicU32::new(0);
static HEARTBEAT_SEEN: AtomicBool = AtomicBool::new(false);

/// Called from the CAN receive path when a heartbeat from the configured master arrives.
pub fn note_master_heartbeat() {
    LAST_HEARTBEAT_MS.store(Instant::now().as_millis() as u32, Ordering::Relaxed);
    if !HEARTBEAT_SEEN.swap(true, Ordering::Relaxed) {
        defmt::info!("master heartbeat seen, link is up");
    }
}

/// Milliseconds since the last master heartbeat, or since boot if there has never been one.
pub fn since_last_heartbeat() -> u32 {
    let now = Instant::now().as_millis() as u32;
    now.wrapping_sub(LAST_HEARTBEAT_MS.load(Ordering::Relaxed))
}

pub fn master_ever_seen() -> bool {
    HEARTBEAT_SEEN.load(Ordering::Relaxed)
}

/// Classify the link from the elapsed time. Pure, so the state machine is testable off-target.
pub fn evaluate(cfg: &Config, raw_debug: bool, seen: bool, since_ms: u32) -> LinkState {
    if seen && since_ms < cfg.fallback_a_ms {
        return LinkState::Alive;
    }
    if raw_debug || !cfg.fallback_enabled {
        return LinkState::Suspended;
    }
    if since_ms >= cfg.fallback_b_ms {
        LinkState::FallbackB
    } else if since_ms >= cfg.fallback_a_ms {
        LinkState::FallbackA
    } else {
        // Still inside the stage A grace period, and no heartbeat yet: this is a board that has
        // only just powered up.
        LinkState::NeverSeen
    }
}

/// Which fallback action applies in a given link state, if any.
pub fn action_for(link: LinkState, cfg: &ValveConfig) -> Option<FallbackAction> {
    match link {
        LinkState::FallbackA => Some(cfg.fallback_a),
        LinkState::FallbackB => Some(cfg.fallback_b),
        LinkState::Alive | LinkState::NeverSeen | LinkState::Suspended => None,
    }
}

/// Per-valve latch recording that a fallback stage has already released this valve.
///
/// Without it a valve would oscillate: the moment we set the unpowered flag on its target, the
/// "has settled" condition that justified the release stops holding, and the next tick would
/// drive it back to the fallback position.
#[derive(Clone, Copy, Debug)]
pub struct FallbackLatch {
    stage: LinkState,
    unpowered: PerValve<bool>,
}

impl FallbackLatch {
    pub const fn new() -> Self {
        Self {
            stage: LinkState::NeverSeen,
            unpowered: PerValve::splat(false),
        }
    }

    /// Note the current stage, clearing the per-valve latches whenever it changes so that entering
    /// stage B re-drives valves that stage A had already released.
    pub fn enter(&mut self, stage: LinkState) -> bool {
        if self.stage == stage {
            return false;
        }
        self.stage = stage;
        self.unpowered = PerValve::splat(false);
        true
    }

    /// A fresh command from the master takes a valve out of its released state.
    pub fn rearm(&mut self, valve: ValveId) {
        self.unpowered[valve] = false;
    }

    /// Resolve the target for one valve under an active fallback action.
    ///
    /// `settled` is whether the valve has reached the fallback position and held it for its
    /// settle time — only then is it safe to drop the drive.
    ///
    /// Fallback positions are deliberately *not* run through the input clamp (0x3019/0x301A):
    /// the clamp exists to bound what the master may ask for, while a fallback is a local safety
    /// action that must be able to reach fully open or fully closed regardless.
    ///
    /// The returned word keeps the fallback position in its promille field even once released, so
    /// `target` still says where the valve was put rather than going blank.
    pub fn target(&mut self, valve: ValveId, action: FallbackAction, settled: bool) -> u16 {
        if self.unpowered[valve] {
            return unpowered_at(action.position);
        }
        if action.unpower && settled {
            self.unpowered[valve] = true;
            return unpowered_at(action.position);
        }
        action.position
    }
}

impl Default for FallbackLatch {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> Config {
        Config::new()
    }

    #[test]
    fn a_live_master_keeps_the_link_up() {
        assert_eq!(evaluate(&cfg(), false, true, 100), LinkState::Alive);
    }

    #[test]
    fn stages_fire_at_their_configured_timeouts() {
        let c = cfg();
        assert_eq!(evaluate(&c, false, true, 2_999), LinkState::Alive);
        assert_eq!(evaluate(&c, false, true, 3_000), LinkState::FallbackA);
        assert_eq!(evaluate(&c, false, true, 299_999), LinkState::FallbackA);
        assert_eq!(evaluate(&c, false, true, 300_000), LinkState::FallbackB);
    }

    #[test]
    fn a_node_that_never_heard_a_master_still_falls_back() {
        let c = cfg();
        assert_eq!(evaluate(&c, false, false, 500), LinkState::NeverSeen);
        assert_eq!(evaluate(&c, false, false, 3_000), LinkState::FallbackA);
        assert_eq!(evaluate(&c, false, false, 300_000), LinkState::FallbackB);
    }

    #[test]
    fn raw_debug_mode_suspends_the_fallback() {
        let c = cfg();
        assert_eq!(evaluate(&c, true, false, 600_000), LinkState::Suspended);
        // ...but a live master is still reported as live.
        assert_eq!(evaluate(&c, true, true, 10), LinkState::Alive);
    }

    #[test]
    fn disabling_the_fallback_suspends_it_too() {
        let c = Config {
            fallback_enabled: false,
            ..cfg()
        };
        assert_eq!(evaluate(&c, false, true, 600_000), LinkState::Suspended);
    }

    #[test]
    fn default_stage_a_closes_and_stage_b_opens() {
        let v = ValveConfig::servo_on_pair(crate::index::HcoPair::A, 2000, 1000, 1000);
        assert_eq!(action_for(LinkState::FallbackA, &v).unwrap().position, 0);
        assert_eq!(action_for(LinkState::FallbackB, &v).unwrap().position, 1000);
        assert!(action_for(LinkState::Alive, &v).is_none());
    }

    #[test]
    fn a_released_valve_stays_released_within_a_stage() {
        let mut latch = FallbackLatch::new();
        latch.enter(LinkState::FallbackA);
        let action = FallbackAction {
            position: 0,
            unpower: true,
        };

        // Still travelling: drive to the fallback position.
        assert_eq!(latch.target(ValveId::Valve0, action, false), 0);
        // Arrived and settled: release.
        assert_eq!(latch.target(ValveId::Valve0, action, true), unpowered_at(0));
        // And stay released even though "settled" no longer holds, rather than oscillating.
        assert_eq!(latch.target(ValveId::Valve0, action, false), unpowered_at(0));
    }

    #[test]
    fn entering_stage_b_re_drives_a_valve_stage_a_released() {
        let mut latch = FallbackLatch::new();
        latch.enter(LinkState::FallbackA);
        let close = FallbackAction {
            position: 0,
            unpower: true,
        };
        latch.target(ValveId::Valve0, close, true);
        assert_eq!(latch.target(ValveId::Valve0, close, false), unpowered_at(0));

        assert!(latch.enter(LinkState::FallbackB));
        let vent = FallbackAction {
            position: 1000,
            unpower: true,
        };
        assert_eq!(latch.target(ValveId::Valve0, vent, false), 1000, "stage B has to actually open it");
    }

    #[test]
    fn a_valve_configured_to_hold_is_never_released() {
        let mut latch = FallbackLatch::new();
        latch.enter(LinkState::FallbackA);
        let hold = FallbackAction {
            position: 250,
            unpower: false,
        };
        assert_eq!(latch.target(ValveId::Valve0, hold, true), 250);
        assert_eq!(latch.target(ValveId::Valve0, hold, true), 250);
    }
}
