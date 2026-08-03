//! The valve model: three layers of state, and the estimator that fills in the third.
//!
//! A valve sits *beside* direct output control rather than on top of it. Both are ways of driving
//! the same four high current outputs, and which one may drive a given output is settled by
//! ownership in [`crate::store`] — a valve owns its outputs outright unless raw debug mode is on.
//!
//! # The three layers
//!
//! - **commanded** (0x2010, read/write) — what the master asked for. Nothing else writes it.
//! - **target** (0x2011, read-only) — what we actually aim for. Differs from commanded when the
//!   input clamp (0x3019/0x301A) bites, or when a heartbeat fallback stage has taken over.
//! - **measured** (0x2012, read-only) — where we believe the valve is. With no position sensor
//!   fitted this is integrated from target, elapsed time and the configured full-travel time; the
//!   moment a potentiometer or hall encoder is wired up, [`PositionFeedback`] replaces the
//!   estimate with a real reading and nothing else in the firmware changes.
//!
//! # Unpowered
//!
//! A servo valve can be *unpowered* (msb of the position word)
//! ```text
//!   bit 15    bits 14..0
//!   +------+--------------+
//!   | !pwr | promille     |   0..=1000
//!   +------+--------------+
//! ```

use embassy_time::Instant;

use crate::config::{PROMILLE_MAX, ValveConfig, ValveKind};
use crate::index::ValveId;

/// Bit 15 of a position word: this valve is not being driven.
///
/// Only meaningful for a servo with a separate power output; a solenoid has nothing to release,
/// so for one this simply reads as de-energized, i.e. closed.
pub const UNPOWERED_FLAG: u16 = 0x8000;

/// The promille field of a position word.
pub const POSITION_MASK: u16 = 0x7FFF;

/// Is this position word asking for (or reporting) a released drive?
pub const fn is_unpowered(word: u16) -> bool {
    word & UNPOWERED_FLAG != 0
}

/// The promille part of a position word, with the flag stripped.
pub const fn position_of(word: u16) -> u16 {
    word & POSITION_MASK
}

/// Build a position word that reports `position` but says the drive is released.
pub const fn unpowered_at(position: u16) -> u16 {
    (position & POSITION_MASK) | UNPOWERED_FLAG
}

/// 0x2013.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
#[repr(u8)]
pub enum ValveStatus {
    /// No valve configured on this slot.
    Unmapped = 0,
    /// Deliberately not being driven. `measured` is the last estimate before release.
    Unpowered = 1,
    /// Travelling toward target.
    Moving = 2,
    /// At target and being held.
    Holding = 3,
    /// Drew more than the configured stall current for longer than the debounce while moving.
    Stalled = 4,
}

/// What the valve wants its outputs to do this tick. The control task maps this onto the
/// configured HCO numbers; the valve itself never touches hardware.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum ValveDrive {
    /// Everything off.
    Released,
    /// Signal output energized or not.
    Solenoid(bool),
    /// Power output on, signal output at this pulse width.
    Servo { pulse_us: u16 },
}

/// A real position sensor, once one is fitted.
pub trait PositionFeedback {
    /// Sensed position in promille for `valve`, or `None` if that valve has no sensor.
    fn position(&mut self, valve: ValveId) -> Option<u16>;
}

pub struct NoFeedback;

impl PositionFeedback for NoFeedback {
    fn position(&mut self, _valve: ValveId) -> Option<u16> {
        None
    }
}

/// Per-valve runtime state. Lives in the control task, never in the store — the store gets the
/// flattened view that goes on the wire.
pub struct Valve {
    status: ValveStatus,
    /// Best estimate of the physical position, promille. Never carries the unpowered flag —
    /// [`Valve::measured_word`] adds it when reporting.
    measured: u16,
    /// The position word we were last handed, flag included.
    target: u16,
    last_tick: Instant,
    /// `measured` at the moment travel toward the current `target` began.
    ///
    /// Progress is computed from `move_elapsed_ms` against this fixed point rather than by
    /// adding a fresh per-tick delta to `measured` each time, because a slow valve's per-tick
    /// step can truncate to zero: at the 20 ms control rate, any `travel_ms` above 20 000 makes
    /// `elapsed_ms * 1000 / travel_ms` round down to 0 on every single tick, so a step that is
    /// only ever computed from one tick's elapsed time never becomes nonzero and the valve never
    /// appears to move. Accumulating the elapsed time instead keeps that fraction from the
    /// previous tick alive until enough of it has built up to round to a whole promille.
    move_origin: u16,
    /// Milliseconds of driven motion accumulated since `move_origin`. Reset whenever the target
    /// changes; frozen while [`ValveStatus::Stalled`], so a stall pauses the clock rather than
    /// losing the time to it.
    move_elapsed_ms: u64,
    /// When `measured` first reached `target`, for the settle deadline.
    arrived_at: Option<Instant>,
    /// When the rail current first went over the stall threshold during this movement.
    over_current_since: Option<Instant>,
}

impl Valve {
    pub fn new(now: Instant) -> Self {
        Self {
            status: ValveStatus::Unmapped,
            // A board comes up with every output low (12k gate pulldowns, see the schematic), so
            // "closed and unpowered" is the truth at reset rather than an assumption.
            measured: 0,
            target: UNPOWERED_FLAG,
            last_tick: now,
            move_origin: 0,
            move_elapsed_ms: 0,
            arrived_at: None,
            over_current_since: None,
        }
    }

    pub fn status(&self) -> ValveStatus {
        self.status
    }

    /// The position estimate on its own, without the unpowered flag.
    pub fn measured(&self) -> u16 {
        self.measured
    }

    /// The position word as it goes out at 0x2012: the estimate, plus bit 15 when the drive is
    /// released. A consumer that only wants the number masks it off; one deciding whether to
    /// trust the number checks the flag.
    pub fn measured_word(&self) -> u16 {
        if self.status == ValveStatus::Unpowered {
            unpowered_at(self.measured)
        } else {
            self.measured
        }
    }

    /// True once the valve has reached `position` under drive and held it for `settle_ms`.
    ///
    /// The fallback logic uses this to decide when it is safe to drop servo power: releasing a
    /// servo the instant the estimate says "arrived" would cut the drive before it has actually
    /// got there.
    ///
    /// `position` is checked against the whole target *word*, which is what makes this mean
    /// "settled at the position you are about to release at" rather than "settled at whatever we
    /// were last asked for". Without that, a valve holding its commanded position when a fallback
    /// stage fires — or holding the stage A position when stage B fires — already looks settled on
    /// the very first tick of the new stage, and gets released before it has travelled anywhere.
    /// Comparing words also rules out a target that already carries the unpowered flag: a valve
    /// that is not being driven has no verified position to settle at.
    pub fn settled_at(&self, cfg: &ValveConfig, now: Instant, position: u16) -> bool {
        if self.target != position {
            return false;
        }
        match self.arrived_at {
            Some(at) => (now - at).as_millis() >= cfg.settle_ms as u64,
            None => false,
        }
    }

    /// Advance the state machine one tick and report what to drive.
    ///
    /// `target` is an already-clamped position word: promille in bits 14..0, and bit 15 set to
    /// release the drive. `current_ma` is the rail current attributed to this valve, or `None` on
    /// a board without current sensing (rev2), which disables stall detection just as a zero
    /// threshold does.
    pub fn tick(
        &mut self,
        cfg: &ValveConfig,
        target: u16,
        now: Instant,
        current_ma: Option<u16>,
        feedback: Option<u16>,
    ) -> ValveDrive {
        let elapsed_ms = (now - self.last_tick).as_millis();
        self.last_tick = now;

        if !cfg.is_mapped() {
            self.status = ValveStatus::Unmapped;
            return ValveDrive::Released;
        }

        let released = is_unpowered(target);
        let position = position_of(target);

        // A solenoid has no separate power output and no travel time worth modelling: it is at
        // its commanded position as far as anything here can tell. Releasing one just means
        // de-energising it, which is the same thing as closing it, so it stays `Holding`.
        if cfg.kind == ValveKind::Solenoid {
            let open = !released && position != 0;
            self.target = target;
            self.measured = if open { PROMILLE_MAX } else { 0 };
            self.status = ValveStatus::Holding;
            return ValveDrive::Solenoid(open);
        }

        if target != self.target {
            self.target = target;
            self.move_origin = self.measured;
            self.move_elapsed_ms = 0;
            self.arrived_at = None;
            self.over_current_since = None;
            // A fresh command is also how an operator retries a valve that stalled: we go back to
            // driving rather than latching the fault until reset.
            if self.status == ValveStatus::Stalled {
                self.status = ValveStatus::Moving;
            }
        }

        if released {
            // Keep `measured` where it was: with the drive released we have no new information,
            // and the last estimate beats nothing. The flag on the reported word is what says to
            // distrust it.
            self.status = ValveStatus::Unpowered;
            self.over_current_since = None;
            return ValveDrive::Released;
        }

        self.update_position(cfg, position, elapsed_ms, feedback);
        self.update_stall(cfg, now, current_ma);

        if self.status != ValveStatus::Stalled {
            self.status = if self.measured == position {
                if self.arrived_at.is_none() {
                    self.arrived_at = Some(now);
                }
                ValveStatus::Holding
            } else {
                self.arrived_at = None;
                ValveStatus::Moving
            };
        }

        ValveDrive::Servo {
            pulse_us: cfg.pulse_width_us(position),
        }
    }

    /// Move `measured` toward `target` at the configured travel rate, or adopt a real reading.
    fn update_position(&mut self, cfg: &ValveConfig, target: u16, elapsed_ms: u64, feedback: Option<u16>) {
        if let Some(sensed) = feedback {
            self.measured = sensed.min(PROMILLE_MAX);
            // Resync the extrapolation to the real reading, so if feedback drops out again it
            // resumes from here instead of from wherever the time model last left off.
            self.move_origin = self.measured;
            self.move_elapsed_ms = 0;
            return;
        }
        // A stalled valve is, by definition, not moving; freezing the estimate keeps `measured`
        // honest instead of letting it drift on to a target the valve never reached. Not adding
        // to `move_elapsed_ms` here is what pauses the clock rather than losing the stalled
        // duration outright.
        if self.status == ValveStatus::Stalled {
            return;
        }

        self.move_elapsed_ms += elapsed_ms;
        let travel_ms = cfg.travel_ms.max(1) as u64;
        // From `move_origin` and accumulated time rather than a fresh per-tick delta: see the
        // field doc on `move_origin` for why a per-tick computation loses slow valves entirely.
        let step = (self.move_elapsed_ms * PROMILLE_MAX as u64 / travel_ms).min(PROMILLE_MAX as u64) as u16;

        self.measured = if target > self.move_origin {
            self.move_origin.saturating_add(step).min(target)
        } else {
            self.move_origin.saturating_sub(step).max(target)
        };
    }

    /// Latch a stall once the rail current has been over threshold for the debounce period.
    ///
    /// Only meaningful while moving: a servo pulls its running current for the whole stroke, so
    /// the threshold is set above running current and below locked-rotor current, per valve, on
    /// the bench. A zero threshold disables detection, which is the only correct setting on rev2
    /// (no on-board current sensing) and on any valve nobody has characterised yet.
    ///
    /// Note the two shunts cover HCO1+2 and HCO3+4 as pairs, so this attribution is only
    /// unambiguous when a valve owns a whole pair — which is how the vehicle harness wires servos.
    fn update_stall(&mut self, cfg: &ValveConfig, now: Instant, current_ma: Option<u16>) {
        let Some(current_ma) = current_ma else {
            self.over_current_since = None;
            return;
        };
        if cfg.stall_ma == 0 || self.status != ValveStatus::Moving || current_ma <= cfg.stall_ma {
            self.over_current_since = None;
            return;
        }

        let since = *self.over_current_since.get_or_insert(now);
        if (now - since).as_millis() >= cfg.stall_ms as u64 {
            // Report and stop updating the estimate, but keep driving. Cutting the drive on a
            // suspected stall would drop a partially open valve in a vehicle; that call belongs to
            // the master, which can set the unpowered flag if it wants the servo released.
            if self.status != ValveStatus::Stalled {
                defmt::error!(
                    "valve stall: {} mA over threshold {} mA for {} ms, holding at {} promille",
                    current_ma,
                    cfg.stall_ma,
                    cfg.stall_ms,
                    self.measured
                );
            }
            self.status = ValveStatus::Stalled;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn servo() -> ValveConfig {
        ValveConfig {
            stall_ma: 800,
            stall_ms: 200,
            settle_ms: 100,
            ..ValveConfig::servo_on_pair(crate::index::HcoPair::A, 2000, 1000, 1000)
        }
    }

    fn at(ms: u64) -> Instant {
        Instant::from_millis(ms)
    }

    #[test]
    fn measured_tracks_target_at_the_configured_rate() {
        let cfg = servo();
        let mut v = Valve::new(at(0));

        // A full sweep takes travel_ms, so half of it gets us halfway.
        v.tick(&cfg, 1000, at(500), None, None);
        assert_eq!(v.measured(), 500);
        assert_eq!(v.status(), ValveStatus::Moving);

        v.tick(&cfg, 1000, at(1000), None, None);
        assert_eq!(v.measured(), 1000);
        assert_eq!(v.status(), ValveStatus::Holding);
    }

    #[test]
    fn measured_never_overshoots() {
        let cfg = servo();
        let mut v = Valve::new(at(0));
        v.tick(&cfg, 300, at(10_000), None, None);
        assert_eq!(v.measured(), 300);
    }

    #[test]
    fn servo_drives_the_interpolated_pulse_width() {
        let cfg = servo();
        let mut v = Valve::new(at(0));
        // closed 2000 us, open 1000 us: half open is halfway between.
        assert_eq!(v.tick(&cfg, 500, at(1), None, None), ValveDrive::Servo { pulse_us: 1500 });
    }

    #[test]
    fn unpowered_releases_and_keeps_the_last_estimate() {
        let cfg = servo();
        let mut v = Valve::new(at(0));
        v.tick(&cfg, 1000, at(1000), None, None);
        assert_eq!(v.measured(), 1000);

        // Setting bit 15 releases the drive whatever the promille field says.
        assert_eq!(v.tick(&cfg, unpowered_at(1000), at(1100), None, None), ValveDrive::Released);
        assert_eq!(v.status(), ValveStatus::Unpowered);
        assert_eq!(v.measured(), 1000, "the last estimate is still the best one we have");
        assert_eq!(
            v.measured_word(),
            unpowered_at(1000),
            "and the reported word carries both the estimate and the fact it is not being held"
        );
    }

    #[test]
    fn the_flag_and_the_position_are_independent() {
        assert!(is_unpowered(unpowered_at(750)));
        assert_eq!(position_of(unpowered_at(750)), 750);
        assert!(!is_unpowered(750));
        assert_eq!(position_of(750), 750);
    }

    #[test]
    fn a_released_valve_still_reports_where_it_thinks_it_is() {
        let cfg = servo();
        let mut v = Valve::new(at(0));
        v.tick(&cfg, 400, at(400), None, None);
        assert_eq!(v.measured(), 400);

        // Release from a partly open position: the estimate survives, flagged as unheld.
        v.tick(&cfg, unpowered_at(400), at(500), None, None);
        assert_eq!(position_of(v.measured_word()), 400);
        assert!(is_unpowered(v.measured_word()));
    }

    #[test]
    fn a_driven_valve_never_sets_the_flag() {
        let cfg = servo();
        let mut v = Valve::new(at(0));
        v.tick(&cfg, 1000, at(500), None, None);
        assert!(!is_unpowered(v.measured_word()), "it is being held, so bit 15 stays clear");
    }

    #[test]
    fn settle_needs_the_configured_hold_after_arriving() {
        let cfg = servo();
        let mut v = Valve::new(at(0));
        v.tick(&cfg, 1000, at(1000), None, None);
        assert!(!v.settled_at(&cfg, at(1000), 1000));
        assert!(!v.settled_at(&cfg, at(1050), 1000));
        assert!(v.settled_at(&cfg, at(1100), 1000));
    }

    /// Holding one position says nothing about another: a valve parked at its commanded position
    /// when a fallback stage fires must not count as settled at that stage's position, or the
    /// stage releases its drive before it has moved at all.
    #[test]
    fn settling_is_per_position() {
        let cfg = servo();
        let mut v = Valve::new(at(0));
        v.tick(&cfg, 1000, at(1000), None, None);
        assert!(v.settled_at(&cfg, at(2000), 1000));
        assert!(!v.settled_at(&cfg, at(2000), 0), "it has never been driven to 0");

        // Drive it to 0 and it settles there, and no longer at 1000.
        v.tick(&cfg, 0, at(3000), None, None);
        assert_eq!(v.measured(), 0);
        assert!(v.settled_at(&cfg, at(3100), 0));
        assert!(!v.settled_at(&cfg, at(3100), 1000));
    }

    #[test]
    fn a_released_valve_is_never_settled() {
        let cfg = servo();
        let mut v = Valve::new(at(0));
        v.tick(&cfg, 1000, at(1000), None, None);
        v.tick(&cfg, unpowered_at(1000), at(1100), None, None);
        assert!(!v.settled_at(&cfg, at(5000), 1000), "the estimate is unverified once nothing is holding the valve");
    }

    #[test]
    fn stall_needs_over_current_for_the_debounce_period() {
        let cfg = servo();
        let mut v = Valve::new(at(0));

        // Moving, drawing normal running current: no stall.
        v.tick(&cfg, 1000, at(100), Some(400), None);
        assert_eq!(v.status(), ValveStatus::Moving);

        // Over threshold, but not for long enough yet.
        v.tick(&cfg, 1000, at(200), Some(900), None);
        assert_eq!(v.status(), ValveStatus::Moving);

        v.tick(&cfg, 1000, at(450), Some(900), None);
        assert_eq!(v.status(), ValveStatus::Stalled);
    }

    #[test]
    fn a_stalled_estimate_stops_advancing() {
        let cfg = servo();
        let mut v = Valve::new(at(0));
        // The first over-threshold tick only starts the debounce; the third is past it.
        v.tick(&cfg, 1000, at(100), Some(900), None);
        v.tick(&cfg, 1000, at(400), Some(900), None);
        v.tick(&cfg, 1000, at(700), Some(900), None);
        assert_eq!(v.status(), ValveStatus::Stalled);
        let frozen = v.measured();

        v.tick(&cfg, 1000, at(2000), Some(900), None);
        assert_eq!(v.measured(), frozen, "a stalled valve is not travelling");
    }

    #[test]
    fn a_new_command_retries_a_stalled_valve() {
        let cfg = servo();
        let mut v = Valve::new(at(0));
        v.tick(&cfg, 1000, at(100), Some(900), None);
        v.tick(&cfg, 1000, at(400), Some(900), None);
        v.tick(&cfg, 1000, at(700), Some(900), None);
        assert_eq!(v.status(), ValveStatus::Stalled);

        v.tick(&cfg, 0, at(800), Some(100), None);
        assert_eq!(v.status(), ValveStatus::Moving);
    }

    #[test]
    fn a_zero_threshold_disables_stall_detection() {
        let cfg = ValveConfig { stall_ma: 0, ..servo() };
        let mut v = Valve::new(at(0));
        v.tick(&cfg, 1000, at(100), Some(60_000), None);
        v.tick(&cfg, 1000, at(900), Some(60_000), None);
        assert_ne!(v.status(), ValveStatus::Stalled);
    }

    #[test]
    fn no_current_sensing_disables_stall_detection() {
        let cfg = servo();
        let mut v = Valve::new(at(0));
        v.tick(&cfg, 1000, at(100), None, None);
        v.tick(&cfg, 1000, at(900), None, None);
        assert_ne!(v.status(), ValveStatus::Stalled);
    }

    #[test]
    fn a_real_sensor_overrides_the_estimate() {
        let cfg = servo();
        let mut v = Valve::new(at(0));
        v.tick(&cfg, 1000, at(1000), None, Some(250));
        assert_eq!(v.measured(), 250, "feedback wins over the time model");
        assert_eq!(v.status(), ValveStatus::Moving, "and the valve is not there yet");
    }

    #[test]
    fn a_solenoid_is_open_or_closed_and_nothing_between() {
        let cfg = ValveConfig::solenoid_on(crate::index::HcoId::Hco0);
        let mut v = Valve::new(at(0));

        assert_eq!(v.tick(&cfg, 1, at(1), None, None), ValveDrive::Solenoid(true));
        assert_eq!(v.measured(), 1000);

        assert_eq!(v.tick(&cfg, 0, at(2), None, None), ValveDrive::Solenoid(false));
        assert_eq!(v.measured(), 0);
    }

    #[test]
    fn an_unmapped_valve_drives_nothing() {
        let cfg = ValveConfig::unmapped();
        let mut v = Valve::new(at(0));
        assert_eq!(v.tick(&cfg, 1000, at(1), None, None), ValveDrive::Released);
        assert_eq!(v.status(), ValveStatus::Unmapped);
    }

    #[test]
    fn a_slow_valve_travels_slowly() {
        let cfg = ValveConfig {
            travel_ms: 4000,
            ..servo()
        };
        let mut v = Valve::new(at(0));
        v.tick(&cfg, 1000, at(1000), None, None);
        assert_eq!(v.measured(), 250, "a quarter of a 4 s sweep in 1 s");
    }

    /// A per-tick step computed only from that tick's elapsed time truncates to zero whenever
    /// `travel_ms` exceeds the control rate (20 ms in `control.rs`) times 1000 — 20 s here — and
    /// with no accumulation the valve then never appears to move at all. Drive it in 20 ms ticks,
    /// the way `control.rs` actually does, well past a 30 s travel time and confirm it still
    /// gets there.
    #[test]
    fn a_valve_slower_than_one_tick_still_reaches_target() {
        let cfg = ValveConfig {
            travel_ms: 30_000,
            ..servo()
        };
        let mut v = Valve::new(at(0));

        let mut now = 0;
        for _ in 0..100 {
            now += 20;
            v.tick(&cfg, 1000, at(now), None, None);
        }
        assert!(v.measured() > 0, "100 ticks (2 s) of a 30 s sweep must have moved off zero");

        while now < 30_000 {
            now += 20;
            v.tick(&cfg, 1000, at(now), None, None);
        }
        assert_eq!(v.measured(), 1000, "the full travel time must still bring it to target");
        assert_eq!(v.status(), ValveStatus::Holding);
    }
}
