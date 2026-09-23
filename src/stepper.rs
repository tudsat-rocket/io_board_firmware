//! Stepper actuator: the motion planner, and the seam the pulse hardware sits behind.
//!
//! One Nanotec PD2-C411L18-E-65-01 (or any clock/direction driver) hangs off a COM port and is
//! commanded exactly like a servo valve: promille in at 0x2010, promille back out at 0x2012. What
//! differs is the layer underneath.
//!
//! # Why this is not just another [`crate::hco::State`]
//!
//! A servo is *told* a position — one pulse width, held forever, and the firmware has no idea
//! whether the horn got there, which is why [`crate::valves::Valve`] has to integrate a travel
//! time to guess. A stepper is *moved* a position: it takes a countable number of pulses to get
//! from A to B, and if we count the pulses we emit we know exactly where the shaft is. So a
//! stepper does not need the travel-time estimator at all — it feeds the real number back in
//! through [`crate::valves::PositionFeedback`], and the estimator steps aside.
//!
//! That is also why the pulses are counted in hardware rather than integrated from elapsed time:
//! a time-based count is off by up to a step per move, and with nothing to home against those
//! errors accumulate straight into the reported position. See [`StepPort`].
//!
//! # The split
//!
//! ```text
//!   control tick (20 ms)                       TIM2 update ISR (per step)
//!   --------------------                       --------------------------
//!   Valve  -> promille target                  count one step
//!   Stepper::plan  -> StepCommand              stop the timer at the last one
//!        { target_steps, step_hz }
//!                       |                                   ^
//!                       +--------- StepPort ----------------+
//! ```
//!
//! [`Stepper`] is the planner: promille to steps, and a trapezoidal speed ramp. It is plain
//! arithmetic over plain data and runs on the host. [`StepPort`] is the two-method seam the
//! hardware implements — see `board::stepper` for the TIM2 side.
//!
//! # Homing
//!
//! There is none, and there cannot be: the board has no home switch and the motor's own absolute
//! encoder is not wired back to us. The step count is relative to whatever the shaft was doing at
//! power-on, and because ENABLE is strapped to 5 V the motor holds through a firmware reset — so
//! that assumption survives a reboot but not a power cycle. 0x2016 is the way to fix it: drive
//! the actuator to a known stop by hand (raw debug mode, or a slow command) and then declare the
//! step count you are at.

use crate::config::PROMILLE_MAX;
use crate::index::{PerStepper, StepperId, ValveId};

/// One clock/direction actuator, and how its shaft maps onto valve travel.
///
/// There are two of these per node ([`StepperId`]), not one per valve: the step clocks are the two
/// timer channels on PA2/PA3 and there are no others left, so the board holds at most two
/// actuators regardless of how many valve slots it has. Which valve slot each one answers to is
/// [`StepperConfig::valve`], the same shape as [`crate::config::ReliefConfig`].
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct StepperConfig {
    /// The valve slot this actuator *is*. `None` leaves the port idle regardless of the rest.
    pub valve: Option<ValveId>,
    /// Step count at 0 promille.
    pub closed_steps: i32,
    /// Step count at 1000 promille.
    ///
    /// May be below `closed_steps`: the sign of `open_steps - closed_steps` is what picks the
    /// direction of travel, so a reversed actuator is a configuration change rather than a
    /// separate "invert" flag that could disagree with the endpoints.
    pub open_steps: i32,
    /// Ceiling on the pulse rate, i.e. the traverse speed.
    pub max_step_hz: u32,
    /// Pull-in rate: the speed the motor can start and stop at without losing steps. The ramp
    /// begins here and the final approach never drops below it, so a move always terminates.
    pub start_step_hz: u32,
    /// Ramp rate, in steps per second per second. 0 disables ramping entirely and the actuator
    /// runs the whole move at `start_step_hz`, which is safe but slow.
    pub accel_hz_per_s: u32,
}

impl StepperConfig {
    /// No actuator fitted. Endpoints are equal, so even a config that sets `valve` by hand and
    /// forgets the travel cannot ask for motion.
    pub const fn disabled() -> Self {
        Self {
            valve: None,
            closed_steps: 0,
            open_steps: 0,
            max_step_hz: 0,
            start_step_hz: 0,
            accel_hz_per_s: 0,
        }
    }

    /// A quarter-step-driving actuator whose valve travel is `closed_steps..open_steps`.
    pub const fn new(valve: ValveId, closed_steps: i32, open_steps: i32) -> Self {
        Self {
            valve: Some(valve),
            closed_steps,
            open_steps,
            // Deliberately unambitious defaults: 800 steps/s is one revolution a second at the
            // PD2-C's factory quarter-stepping, and 400 steps/s is inside any 42 mm stepper's
            // pull-in rate. Raise them once the mechanism has been run.
            max_step_hz: 800,
            start_step_hz: 400,
            accel_hz_per_s: 4_000,
        }
    }

    pub const fn with_speed(mut self, max_step_hz: u32, start_step_hz: u32, accel_hz_per_s: u32) -> Self {
        self.max_step_hz = max_step_hz;
        self.start_step_hz = start_step_hz;
        self.accel_hz_per_s = accel_hz_per_s;
        self
    }

    /// Is an actuator configured on this board at all?
    pub const fn is_mapped(&self) -> bool {
        self.valve.is_some()
    }

    /// Signed travel from closed to open, in steps. Negative for a reversed actuator.
    pub const fn span(&self) -> i32 {
        self.open_steps - self.closed_steps
    }

    /// The step count that corresponds to a commanded position.
    pub fn steps_for(&self, promille: u16) -> i32 {
        let promille = promille.min(PROMILLE_MAX) as i64;
        let steps = self.closed_steps as i64 + (self.span() as i64 * promille) / PROMILLE_MAX as i64;
        steps as i32
    }

    /// The position a step count corresponds to, clamped into 0..=1000.
    ///
    /// Clamping rather than reporting out of range is deliberate: the shaft can legitimately sit
    /// outside the configured travel after a re-zero (0x2016) or a hand-turn, and a position word
    /// has nowhere to say so. The clamp keeps 0x2012 meaning what it means everywhere else.
    pub fn promille_at(&self, steps: i32) -> u16 {
        let span = self.span() as i64;
        if span == 0 {
            return 0;
        }
        let travelled = (steps as i64 - self.closed_steps as i64) * PROMILLE_MAX as i64;
        (travelled / span).clamp(0, PROMILLE_MAX as i64) as u16
    }
}

/// What the planner wants the pulse hardware to do until the next tick.
///
/// Absolute rather than incremental — "be at step N, going no faster than `step_hz`" — so a
/// dropped or repeated tick cannot make the actuator overshoot. The hardware is free to be
/// anywhere along the way when the next command lands.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub struct StepCommand {
    pub target_steps: i32,
    /// 0 means stop: either we are there, or nothing is configured.
    pub step_hz: u32,
}

impl StepCommand {
    pub const HOLD: Self = Self {
        target_steps: 0,
        step_hz: 0,
    };
}

/// The motion planner. Owns nothing but the current speed; the position lives in the hardware
/// step counter, which is the only place that can know it.
#[derive(Clone, Copy, Debug, Default)]
pub struct Stepper {
    step_hz: u32,
}

impl Stepper {
    pub const fn new() -> Self {
        Self { step_hz: 0 }
    }

    /// The speed the last [`Stepper::plan`] settled on. Only interesting for logging.
    pub const fn step_hz(&self) -> u32 {
        self.step_hz
    }

    /// Decide this tick's speed toward `target_promille`, given where the shaft actually is.
    ///
    /// A plain trapezoid: ramp up at `accel_hz_per_s`, cap at `max_step_hz`, and hold the speed
    /// below the one from which the remaining distance is still enough to stop —
    /// `v = sqrt(2 * a * remaining)`. The last constraint is what makes an absolute target safe
    /// to hand to hardware that will stop dead on arrival: by the time `remaining` is small the
    /// planned speed is back down at the pull-in rate, so the hard stop costs nothing.
    ///
    /// Called every control tick with the freshly read hardware position, so an actuator that
    /// slipped, was re-zeroed, or was hand-turned is re-planned from where it really is rather
    /// than from where the planner last thought it was.
    pub fn plan(
        &mut self,
        cfg: &StepperConfig,
        target_promille: u16,
        position_steps: i32,
        elapsed_ms: u64,
    ) -> StepCommand {
        if !cfg.is_mapped() || cfg.span() == 0 {
            self.step_hz = 0;
            return StepCommand::HOLD;
        }

        let target_steps = cfg.steps_for(target_promille);
        let remaining = (target_steps as i64 - position_steps as i64).unsigned_abs();
        if remaining == 0 {
            self.step_hz = 0;
            return StepCommand {
                target_steps,
                step_hz: 0,
            };
        }

        // A pull-in rate of zero would let the ramp latch at 0 Hz and never move at all.
        let start_hz = cfg.start_step_hz.max(1);
        let max_hz = cfg.max_step_hz.max(start_hz);

        // Ramp up from wherever the last tick left us, never below the pull-in rate — that is
        // both the speed we may start from and the speed we may stop at.
        let ramped_hz = if self.step_hz < start_hz {
            start_hz
        } else {
            let gained = (cfg.accel_hz_per_s as u64 * elapsed_ms) / 1000;
            self.step_hz.saturating_add(gained.min(u32::MAX as u64) as u32)
        };

        // Braking distance, the other way round: the fastest we may be going and still stop in
        // `remaining` steps. With no configured acceleration there is no braking curve either,
        // and the whole move runs at the pull-in rate.
        let brake_hz = isqrt(2 * cfg.accel_hz_per_s as u64 * remaining).max(start_hz);

        self.step_hz = ramped_hz.min(brake_hz).min(max_hz);
        StepCommand {
            target_steps,
            step_hz: self.step_hz,
        }
    }

    /// Forget the ramp, e.g. because the port was reconfigured underneath us. The next plan
    /// starts from the pull-in rate again.
    pub fn reset(&mut self) {
        self.step_hz = 0;
    }
}

/// The seam the pulse hardware sits behind: an exact step counter and a "go here, no faster than
/// this" command, for both actuators.
///
/// Deliberately synchronous and dyn-safe, the same shape as [`crate::hco::HcoControl`], so the
/// control task can hold a `&'static mut dyn StepPort` and a host test can hand it a pair of
/// counters in a struct instead of a timer.
///
/// # Why [`StepPort::command`] takes both at once
///
/// The two step clocks are two channels of *one* timer, so they share an autoreload and therefore
/// a step rate. Reconciling two planned rates into the one the hardware can actually run is the
/// port's problem, not the control task's — it is a property of the timer, and a port built on
/// two independent timers would simply ignore it. Handing over both commands together is what
/// gives an implementation the chance.
///
/// # What an implementation owes the caller
///
/// [`StepPort::position_steps`] must be **exact**: the number of pulses that have actually left
/// the pin, direction-signed. Integrating `step_hz * elapsed` instead is off by up to a step
/// every time the pulse train starts or stops, and since nothing ever homes these actuators those
/// errors only ever accumulate. Count the pulses.
pub trait StepPort {
    /// Pulses emitted since boot, positive in the direction of increasing step count.
    fn position_steps(&self, id: StepperId) -> i32;

    /// Declare the shaft to physically be at `steps`, without moving it. The homing mechanism —
    /// see the module docs.
    fn set_position_steps(&mut self, id: StepperId, steps: i32);

    /// Run each actuator toward its `target_steps` at no more than its `step_hz`, stopping exactly
    /// on arrival. Called every control tick with freshly planned commands; an implementation must
    /// treat a repeat of what it is already executing as a no-op rather than restarting the move.
    fn command(&mut self, cmds: PerStepper<StepCommand>);
}

/// Integer square root, for the braking curve. Newton's method, which converges in a handful of
/// iterations and needs no FPU — the STM32F105 is a Cortex-M3 and does not have one.
fn isqrt(n: u64) -> u32 {
    if n == 0 {
        return 0;
    }
    let mut x = n;
    let mut next = x.div_ceil(2);
    while next < x {
        x = next;
        next = (x + n / x) / 2;
    }
    x.min(u32::MAX as u64) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 800 steps of travel — one revolution at the PD2-C's factory quarter-stepping — with a
    /// traverse speed the ramp can actually reach inside it. At 8000 steps/s^2 the braking curve
    /// allows sqrt(2 * 8000 * 800) = 3577 Hz at the far end, so a 2 kHz ceiling is the binding
    /// one in the middle of the stroke and the braking curve is what governs the approach.
    fn cfg() -> StepperConfig {
        StepperConfig::new(ValveId::Valve0, 0, 800).with_speed(2_000, 400, 8_000)
    }

    #[test]
    fn promille_maps_onto_the_configured_travel() {
        let cfg = cfg();
        assert_eq!(cfg.steps_for(0), 0);
        assert_eq!(cfg.steps_for(1000), 800);
        assert_eq!(cfg.steps_for(500), 400);
        assert_eq!(cfg.promille_at(0), 0);
        assert_eq!(cfg.promille_at(800), 1000);
        assert_eq!(cfg.promille_at(400), 500);
    }

    /// The sign of the travel is the direction of travel: there is no separate invert flag that
    /// could contradict the endpoints.
    #[test]
    fn a_reversed_actuator_is_just_a_negative_span() {
        let cfg = StepperConfig::new(ValveId::Valve0, 0, -800);
        assert_eq!(cfg.steps_for(1000), -800);
        assert_eq!(cfg.promille_at(-400), 500);
        assert_eq!(cfg.promille_at(-800), 1000);
    }

    /// The shaft can sit outside the configured travel after a re-zero or a hand-turn, and a
    /// position word has no way to say "off the end" — so it reads as the nearest endpoint.
    #[test]
    fn a_position_outside_the_travel_clamps() {
        let cfg = cfg();
        assert_eq!(cfg.promille_at(-50), 0);
        assert_eq!(cfg.promille_at(5_000), 1000);
    }

    #[test]
    fn an_offset_travel_still_starts_at_zero_promille() {
        let cfg = StepperConfig::new(ValveId::Valve0, 1_000, 1_800);
        assert_eq!(cfg.steps_for(0), 1_000);
        assert_eq!(cfg.steps_for(1000), 1_800);
        assert_eq!(cfg.promille_at(1_400), 500);
    }

    #[test]
    fn a_move_starts_at_the_pull_in_rate_and_ramps_up() {
        let cfg = cfg();
        let mut s = Stepper::new();

        // 20 ms ticks, the control task's rate. First tick can only be the pull-in rate.
        let first = s.plan(&cfg, 1000, 0, 20);
        assert_eq!(first.target_steps, 800);
        assert_eq!(first.step_hz, 400, "the first tick of a move may only start at the pull-in rate");

        // Second tick, still far away: 8000 steps/s^2 for 20 ms is another 160 steps/s.
        let second = s.plan(&cfg, 1000, 10, 20);
        assert_eq!(second.step_hz, 560);
    }

    #[test]
    fn the_ramp_is_capped_at_the_configured_traverse_speed() {
        let cfg = StepperConfig::new(ValveId::Valve0, 0, 1_000_000).with_speed(4_000, 400, 8_000);
        let mut s = Stepper::new();
        for _ in 0..1000 {
            s.plan(&cfg, 1000, 0, 20);
        }
        assert_eq!(s.step_hz(), 4_000, "far from the target, the traverse speed is the only ceiling");
    }

    /// The property the whole absolute-target scheme rests on: by the time the actuator is close,
    /// the planned speed is back at the pull-in rate, so hardware that stops dead on the last
    /// step does not lose any.
    #[test]
    fn the_approach_slows_to_the_pull_in_rate() {
        let cfg = cfg();
        let mut s = Stepper::new();
        // Wound fully up first, so the braking curve is what brings it back down.
        for _ in 0..100 {
            s.plan(&cfg, 1000, 0, 20);
        }
        assert_eq!(s.step_hz(), 2_000);

        // 10 steps from the target: sqrt(2 * 8000 * 10) = 400.
        let near = s.plan(&cfg, 1000, 790, 20);
        assert_eq!(near.step_hz, 400);

        // One step out, the curve is below the pull-in rate and the floor takes over.
        let last = s.plan(&cfg, 1000, 799, 20);
        assert_eq!(last.step_hz, 400);
    }

    #[test]
    fn arriving_stops_the_pulse_train() {
        let cfg = cfg();
        let mut s = Stepper::new();
        s.plan(&cfg, 1000, 0, 20);
        let arrived = s.plan(&cfg, 1000, 800, 20);
        assert_eq!(
            arrived,
            StepCommand {
                target_steps: 800,
                step_hz: 0
            }
        );
        assert_eq!(s.step_hz(), 0, "and the next move ramps from scratch");
    }

    /// Reversing is not a special case: the target is absolute, so the hardware sorts out the
    /// direction and the planner only ever talks about how fast.
    #[test]
    fn reversing_replans_from_the_real_position() {
        let cfg = cfg();
        let mut s = Stepper::new();
        for _ in 0..100 {
            s.plan(&cfg, 1000, 0, 20);
        }
        let back = s.plan(&cfg, 0, 800, 20);
        assert_eq!(back.target_steps, 0);
        assert!(back.step_hz > 0);
    }

    #[test]
    fn zero_acceleration_runs_the_whole_move_at_the_pull_in_rate() {
        let cfg = StepperConfig::new(ValveId::Valve0, 0, 10_000).with_speed(4_000, 300, 0);
        let mut s = Stepper::new();
        for _ in 0..50 {
            s.plan(&cfg, 1000, 0, 20);
        }
        assert_eq!(s.step_hz(), 300);
    }

    #[test]
    fn an_unmapped_or_zero_travel_actuator_never_moves() {
        let mut s = Stepper::new();
        assert_eq!(s.plan(&StepperConfig::disabled(), 1000, 0, 20), StepCommand::HOLD);

        let no_travel = StepperConfig::new(ValveId::Valve0, 500, 500);
        assert_eq!(s.plan(&no_travel, 1000, 0, 20), StepCommand::HOLD);
    }

    #[test]
    fn isqrt_is_the_floor_of_the_real_root() {
        assert_eq!(isqrt(0), 0);
        assert_eq!(isqrt(1), 1);
        assert_eq!(isqrt(15), 3);
        assert_eq!(isqrt(16), 4);
        assert_eq!(isqrt(160_000), 400);
        assert_eq!(isqrt(u32::MAX as u64), 65535);
    }
}
