//! Local overpressure relief, controled by the node itself
//!
//! # Precedence
//! Relief overrides *everything*, except:
//! Raw debug mode, if the valve's pwm output has been overridden

use embassy_time::Instant;

use crate::config::ReliefConfig;
#[cfg(test)]
use crate::index::{SensorSlot, ValveId};
use crate::store::SENSOR_INVALID;

/// 0x2015.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
#[repr(u8)]
pub enum ReliefState {
    /// Watching, not intervening.
    Idle = 0,
    /// Holding the valve open to bleed pressure.
    Relieving = 1,
    /// Pulse finished; letting the reading settle before re-arming.
    Cooldown = 2,
    /// Configured, but the watched slot has no valid reading, so it cannot act. See the note on
    /// [`Relief::update`] about why this does not vent.
    Inhibited = 3,
    /// Not configured on this node.
    Disabled = 4,
}

pub struct Relief {
    state: ReliefState,
    /// When the current `Relieving` or `Cooldown` phase ends.
    until: Instant,
    /// Suppresses repeating the "no valid reading" complaint every tick.
    warned_invalid: bool,
}

impl Relief {
    pub const fn new() -> Self {
        Self {
            state: ReliefState::Disabled,
            until: Instant::from_ticks(0),
            warned_invalid: false,
        }
    }

    pub fn state(&self) -> ReliefState {
        self.state
    }

    /// True while relief is overriding normal control.
    pub fn is_active(&self) -> bool {
        self.state == ReliefState::Relieving
    }

    /// Advance one tick and report the position to force, if any.
    ///
    /// `reading` is the current value of the watched sensor slot, in that slot's own unit — the
    /// same number that goes out at 0x2004 — so the threshold is written in the units the sensor
    /// already reports and no conversion can go wrong between them.
    ///
    /// A slot with no valid reading ([`SENSOR_INVALID`]) inhibits relief rather than triggering
    /// it. Venting a vehicle because a sensor cable fell off would be its own incident, and the
    /// state is reported at 0x2015 and in the status TPDO so the condition is visible rather than
    /// silent.
    pub fn update(&mut self, cfg: &ReliefConfig, reading: i16, now: Instant) -> Option<u16> {
        let Some(_) = cfg.valve.filter(|_| cfg.enabled) else {
            self.state = ReliefState::Disabled;
            return None;
        };

        if reading == SENSOR_INVALID {
            // Do not abandon an in-flight pulse just because one sample was missed; finish it,
            // then park in Inhibited until the sensor comes back.
            if self.state == ReliefState::Relieving && now < self.until {
                return Some(cfg.position);
            }
            if !self.warned_invalid {
                defmt::error!("overpressure relief inhibited: sensor slot {} has no valid reading", cfg.sensor.as_u8());
                self.warned_invalid = true;
            }
            self.state = ReliefState::Inhibited;
            return None;
        }
        self.warned_invalid = false;

        match self.state {
            // Mid-pulse: keep holding until the pulse is up.
            ReliefState::Relieving if now < self.until => return Some(cfg.position),
            ReliefState::Relieving => {
                defmt::info!("overpressure relief: pulse complete, reading {}", reading);
                self.state = ReliefState::Cooldown;
                self.until = now + embassy_time::Duration::from_millis(cfg.cooldown_ms as u64);
                return None;
            }
            // Cooling down: hand the valve back, but do not re-arm yet.
            ReliefState::Cooldown if now < self.until => return None,
            // Idle, Inhibited, Disabled, or a cooldown that has just expired: free to arm.
            _ => {}
        }

        if reading > cfg.threshold {
            defmt::warn!(
                "overpressure relief: {} over threshold {}, opening valve {} to {} promille for {} ms",
                reading,
                cfg.threshold,
                cfg.valve.map_or(0xFF, crate::index::ValveId::as_u8),
                cfg.position,
                cfg.pulse_ms
            );
            self.state = ReliefState::Relieving;
            self.until = now + embassy_time::Duration::from_millis(cfg.pulse_ms as u64);
            Some(cfg.position)
        } else {
            self.state = ReliefState::Idle;
            None
        }
    }
}

impl Default for Relief {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 60 bar, in the centibar the watched slot reports.
    const THRESHOLD: i16 = 6000;

    fn cfg() -> ReliefConfig {
        ReliefConfig {
            enabled: true,
            valve: Some(ValveId::Valve0),
            sensor: SensorSlot::Slot0,
            threshold: THRESHOLD,
            position: 1000,
            pulse_ms: 500,
            cooldown_ms: 500,
        }
    }

    fn at(ms: u64) -> Instant {
        Instant::from_millis(ms)
    }

    #[test]
    fn below_threshold_it_does_not_intervene() {
        let mut r = Relief::new();
        assert_eq!(r.update(&cfg(), 5999, at(0)), None);
        assert_eq!(r.state(), ReliefState::Idle);
    }

    #[test]
    fn crossing_the_threshold_opens_the_valve() {
        let mut r = Relief::new();
        assert_eq!(r.update(&cfg(), 6001, at(0)), Some(1000));
        assert_eq!(r.state(), ReliefState::Relieving);
        assert!(r.is_active());
    }

    #[test]
    fn the_pulse_lasts_the_configured_time_then_closes() {
        let c = cfg();
        let mut r = Relief::new();
        r.update(&c, 6001, at(0));

        // Held open for the whole pulse, even as the reading falls.
        assert_eq!(r.update(&c, 6001, at(250)), Some(1000));
        assert_eq!(r.update(&c, 5000, at(499)), Some(1000));

        // At the end of the pulse the valve goes back to normal control.
        assert_eq!(r.update(&c, 5000, at(500)), None);
        assert_eq!(r.state(), ReliefState::Cooldown);
    }

    #[test]
    fn it_does_not_re_arm_during_the_cooldown() {
        let c = cfg();
        let mut r = Relief::new();
        r.update(&c, 6001, at(0));
        r.update(&c, 6001, at(500)); // -> Cooldown until 1000

        // Still over threshold, but the reading has not had time to settle.
        assert_eq!(r.update(&c, 6001, at(600)), None);
        assert_eq!(r.update(&c, 6001, at(999)), None);
        assert_eq!(r.state(), ReliefState::Cooldown);
    }

    #[test]
    fn it_pulses_again_if_still_over_pressure_after_the_cooldown() {
        let c = cfg();
        let mut r = Relief::new();
        r.update(&c, 6001, at(0));
        r.update(&c, 6001, at(500));

        assert_eq!(r.update(&c, 6001, at(1000)), Some(1000), "a heated vessel keeps rising");
        assert_eq!(r.state(), ReliefState::Relieving);
    }

    #[test]
    fn it_settles_once_the_pressure_is_back_in_range() {
        let c = cfg();
        let mut r = Relief::new();
        r.update(&c, 6001, at(0));
        r.update(&c, 6001, at(500));

        assert_eq!(r.update(&c, 5500, at(1000)), None);
        assert_eq!(r.state(), ReliefState::Idle, "and hands the valve back to the master");
    }

    #[test]
    fn exactly_at_the_threshold_is_not_over_it() {
        let mut r = Relief::new();
        assert_eq!(r.update(&cfg(), THRESHOLD, at(0)), None);
    }

    #[test]
    fn an_invalid_reading_inhibits_rather_than_vents() {
        let mut r = Relief::new();
        assert_eq!(r.update(&cfg(), SENSOR_INVALID, at(0)), None);
        assert_eq!(r.state(), ReliefState::Inhibited);
    }

    #[test]
    fn a_dropped_sample_does_not_cut_a_pulse_short() {
        let c = cfg();
        let mut r = Relief::new();
        r.update(&c, 6001, at(0));
        // One bad read mid-pulse: keep venting, do not slam the valve shut.
        assert_eq!(r.update(&c, SENSOR_INVALID, at(200)), Some(1000));
        assert_eq!(r.state(), ReliefState::Relieving);
        // Once the pulse is over, park inhibited until the sensor returns.
        assert_eq!(r.update(&c, SENSOR_INVALID, at(500)), None);
        assert_eq!(r.state(), ReliefState::Inhibited);
    }

    #[test]
    fn it_recovers_when_the_sensor_comes_back() {
        let c = cfg();
        let mut r = Relief::new();
        r.update(&c, SENSOR_INVALID, at(0));
        assert_eq!(r.state(), ReliefState::Inhibited);

        assert_eq!(r.update(&c, 6001, at(100)), Some(1000));
        assert_eq!(r.state(), ReliefState::Relieving);
    }

    #[test]
    fn disabled_or_unmapped_never_intervenes() {
        let mut r = Relief::new();

        let off = ReliefConfig {
            enabled: false,
            ..cfg()
        };
        assert_eq!(r.update(&off, 30_000, at(0)), None);
        assert_eq!(r.state(), ReliefState::Disabled);

        let unmapped = ReliefConfig { valve: None, ..cfg() };
        assert_eq!(r.update(&unmapped, 30_000, at(0)), None);
        assert_eq!(r.state(), ReliefState::Disabled);
    }

    /// The requirement that relief keeps working through a short loss of the master.
    ///
    /// It holds structurally rather than by a check somewhere: `update` takes no `LinkState`, so
    /// there is no way for a fallback stage to reach it. This pins that down — if someone ever
    /// threads a link state in here, this test is what should stop them.
    #[test]
    fn relief_is_reached_the_same_way_in_every_link_state() {
        let c = cfg();
        for _ in [
            crate::store::LinkState::Alive,
            crate::store::LinkState::NeverSeen,
            crate::store::LinkState::FallbackA,
            crate::store::LinkState::FallbackB,
            crate::store::LinkState::Suspended,
        ] {
            let mut r = Relief::new();
            assert_eq!(r.update(&c, 6001, at(0)), Some(1000));
        }
    }

    /// Composition test for the precedence `crate::control` implements: relief is evaluated
    /// first, and when it is acting the fallback target is not used.
    #[test]
    fn relief_opens_the_valve_even_while_stage_a_wants_it_shut() {
        use crate::config::FallbackAction;
        use crate::safety::FallbackLatch;
        use crate::store::LinkState;

        let mut latch = FallbackLatch::new();
        latch.enter(LinkState::FallbackA);
        let close = FallbackAction {
            position: 0,
            unpower: true,
        };

        // Stage A on its own would shut this valve.
        assert_eq!(latch.target(ValveId::Valve0, close, false), 0);

        // With the vessel over pressure, relief is what the control task uses instead.
        let mut relief = Relief::new();
        let relieving = relief.update(&cfg(), 6001, at(0));
        assert_eq!(relieving, Some(1000));
        assert_eq!(relieving.unwrap_or(0), 1000, "relief outranks the fallback");

        // Once the pulse is done the valve goes back to the fallback's idea of safe.
        relief.update(&cfg(), 5000, at(500));
        assert_eq!(relief.update(&cfg(), 5000, at(1000)), None);
        assert_eq!(latch.target(ValveId::Valve0, close, false), 0);
    }

    #[test]
    fn the_default_threshold_never_fires() {
        // A node that has not been configured for relief must not start venting because some
        // unrelated sensor reads high.
        let mut r = Relief::new();
        let c = ReliefConfig {
            enabled: true,
            valve: Some(ValveId::Valve0),
            ..ReliefConfig::disabled()
        };
        assert_eq!(r.update(&c, i16::MAX - 1, at(0)), None);
    }
}
