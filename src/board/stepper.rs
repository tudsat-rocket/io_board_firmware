//! The clock/direction pulse train: one or two actuators on TIM2.
//!
//! Implements [`StepPort`] for Nanotec PD2-C411L18-E-65-01 drivers, or anything else taking a step
//! clock and a direction level. Both board revisions, and one or two actuators depending on the
//! `dual-stepper` feature.
//!
//! # Which pins, and why there is no other choice
//!
//! ```text
//!   COM1  PB6/PB7    I2C1   — amplifier bus 0
//!   COM2  PB10/PB11  I2C2   — amplifier bus 1
//!   COM3  PC10       —        no timer channel behind it
//!   COM4  PA2/PA3    TIM2_CH3 / TIM2_CH4
//! ```
//!
//! **PA2/PA3 carry the only timer channels this board has left on pins it can reach.** PC10, PC11,
//! PC12 and PD2 have none, and every other channel is spoken for: TIM3 runs the high current
//! outputs on both revisions, TIM1 runs HCO2 on rev3, TIM4 is embassy's time driver. rev2 used to
//! spend TIM2 on its output software PWM; that moved to TIM5, which can be any timer with two
//! compare channels, so that this could have the two channels only it can use. See
//! [`super::HcoControllerRev2::new`].
//!
//! The pinout therefore depends on how many actuators there are, which is why `dual-stepper` is a
//! build-time feature rather than a runtime setting — the pin muxing is fixed long before any
//! configuration is read:
//!
//! ```text
//!   one actuator (default)          two actuators (`dual-stepper`)
//!   ----------------------          ------------------------------
//!   PA2  step   TIM2 CH3            PA2  step 0  TIM2 CH3
//!   PA3  dir                        PA3  step 1  TIM2 CH4
//!                                   PC10 dir 0   (COM3_1 / IO_0)
//!                                   PA5  dir 1   (A_IN_1 / IO_2)
//! ```
//!
//! Direction is a slow level, so it only needs a GPIO; PC10 and PA5 are unused by the firmware on
//! both revisions. **Those two are the pins to confirm against the schematic** — everything else
//! here is forced by the silicon, but a direction line can move to any free pin by changing the
//! two arguments in [`super::init_board`].
//!
//! No high current output gives up anything either way: two actuators cost two timer channels and
//! two GPIOs, and no HCO.
//!
//! # Wiring
//!
//! ```text
//!   step ──►│ 5 V buffer │──► RD red    ──► X3 pin 8  +Clock
//!   dir  ──►│            │──► PK pink   ──► X3 pin 6  +Direction
//!                             YE yellow ──► X3 pin 4  +Enable   ── strapped to 5 V
//!                             WH white  ──► X3 pin 1  GND
//! ```
//!
//! The buffer is not optional. The PD2-C guarantees a logic high only above **4.94 V** even with
//! `3240h:06h` left at its "5 V" default, and this board's GPIO is 3.3 V — inside the
//! indeterminate band. Enable is strapped high, so the motor is energised and holding whenever it
//! has power; see [`crate::stepper`] on what that means for homing and for the unpowered flag.
//!
//! # How a step is counted
//!
//! One TIM2 period is one step, and the update interrupt at the end of each period is what counts
//! it. The channels run in **PWM mode 2**, so the pulse sits at the *end* of the period rather
//! than the start:
//!
//! ```text
//!         |<---------------- 1/step_hz ---------------->|
//!   OCREF ______________________________________|‾‾‾‾‾‾|______________ ...
//!                                                       ^
//!                                                    update event: one step completed on every
//!                                                    channel that was owed one
//! ```
//!
//! Two things fall out of that ordering, and both matter:
//!
//! - **Stopping is exact.** The ISR shuts a channel down on the update event, which is the moment
//!   its pulse has just finished. In PWM mode 1 the pulse starts at the update instead, so the ISR
//!   would be cutting a pulse that had already begun — a runt the driver would still count as a
//!   step. Counting the pulses is the whole point of doing this in hardware rather than
//!   integrating `step_hz * elapsed`, which drifts by up to a step per move and never gets
//!   corrected because nothing homes these actuators.
//! - **Direction setup is free.** The manual wants ≥ 35 µs between a direction change and the
//!   next clock edge. Restarting the counter from zero puts the first edge a whole
//!   `period - PULSE_US` away, which clears 35 µs for any rate below about 28 kHz — comfortably
//!   guaranteed by [`MAX_STEP_HZ`].
//!
//! A channel that finishes early is parked in `ForceInactive` rather than having its output
//! disable bit cleared: that keeps the pin **actively driven low** instead of releasing it to
//! float, and a floating step line next to a switching supply is a phantom step waiting to happen.
//!
//! # What the two actuators share, and what they do not
//!
//! They share one autoreload, so while **both are moving they move at the same rate** — the lower
//! of the two planned rates, since running a stepper slower than planned is always safe and
//! running it faster is not. Everything else is independent: separate targets, separate step
//! counters, separate directions, and a channel that arrives stops on its own while the other
//! keeps going. Whenever only one is moving it gets its full planned rate, which is the normal
//! case for valve actuators that are commanded occasionally.
//!
//! Giving them genuinely independent rates would mean a free-running counter with per-channel
//! compare scheduling — two interrupts per step per motor, and every edge reprogrammed in
//! software. That is the design to reach for if these ever become coordinated-motion axes; for
//! valve travel the shared rate costs a fraction of a second and no accuracy at all.
//!
//! # The stop/recompute/restart cycle
//!
//! [`StepperPortTim2::command`] stops the counter before touching the shared step state, so the
//! ISR is quiescent while it runs and no lock is needed between the two. It then recomputes the
//! remaining distances from the *counters*, never from what it believed last tick, which is what
//! makes a truncated period cost a little speed and never a step. A command identical to the one
//! already executing is skipped entirely, so a constant-speed traverse is not restarted 50 times
//! a second for nothing.

use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

use embassy_stm32::{
    Peri,
    gpio::{AfioRemap, Level as GpioLevel, Output, OutputType, Speed},
    interrupt::{self, InterruptExt},
    pac, peripherals as p,
    time::Hertz,
    timer::{
        Ch3, Channel, TimerPin,
        low_level::{OutputCompareMode, Timer},
        simple_pwm::PwmPin,
    },
};

use crate::index::{PerStepper, StepperId};
use crate::stepper::{StepCommand, StepPort};

/// Timer tick rate, so an autoreload value is a period in microseconds.
const TICK_HZ: u32 = 1_000_000;

/// Clock pulse width. The PD2-C wants at least 200 ns; 5 µs is generous enough to survive the
/// rise time of a 5 V buffer and a couple of metres of M8 cable, and still tiny next to the
/// shortest period [`MAX_STEP_HZ`] allows.
const PULSE_US: u16 = 5;

/// Slowest pulse rate the hardware can express: the autoreload register is 16 bits, so at a 1 MHz
/// tick the longest period is 65.5 ms. A planner asking for less is clamped up rather than
/// silently wrapping to something fast.
const MIN_STEP_HZ: u32 = 20;

/// Fastest pulse rate this module will emit, well below the driver's 1 MHz ceiling. Two reasons
/// to keep it here: the update ISR runs once per step, and `period - PULSE_US` has to stay above
/// the 35 µs the driver needs to settle a direction change.
pub const MAX_STEP_HZ: u32 = 20_000;

/// How many actuators this build can actually drive. A configuration mapping more than this gets
/// a warning at boot from [`crate::node::spawn_node`] — it is a build mistake, not a bus one.
#[cfg(feature = "dual-stepper")]
pub const CHANNELS: usize = 2;
#[cfg(not(feature = "dual-stepper"))]
pub const CHANNELS: usize = 1;

// Plain arrays rather than `PerStepper` because an atomic is not `Copy` and so cannot be
// `splat`ed into one. Indexed by `StepperId::index` everywhere, never by a loose integer.

/// Pulses emitted since boot, direction-signed, per actuator.
static POSITION: [AtomicI32; StepperId::COUNT] = [AtomicI32::new(0), AtomicI32::new(0)];

/// Pulses still owed on the current move. Nonzero *is* the "this channel is pulsing" flag: the ISR
/// decrements it and parks the channel at zero.
static REMAINING: [AtomicU32; StepperId::COUNT] = [AtomicU32::new(0), AtomicU32::new(0)];

/// What one pulse does to [`POSITION`]: +1 or -1, matching the level on that actuator's DIR pin.
static DIR_STEP: [AtomicI32; StepperId::COUNT] = [AtomicI32::new(1), AtomicI32::new(1)];

/// Which timer channel carries an actuator's step clock.
const fn channel_of(id: StepperId) -> Channel {
    match id {
        StepperId::Stepper0 => Channel::Ch3,
        StepperId::Stepper1 => Channel::Ch4,
    }
}

/// Park a channel with its output held low, without releasing the pin.
///
/// `ForceInactive` rather than clearing the channel enable bit: with `CCxE` cleared the timer
/// stops driving the pin at all and it floats, and a floating step line is a phantom step. Raw PAC
/// because this also runs in the interrupt, which has no access to the `Timer` handle.
///
/// Harmless on a channel this build does not use: only [`StepperPortTim2::configure_channel`] sets
/// `CCxE`, so on a single-actuator build TIM2 never drives PA3 and writing its compare mode
/// changes nothing on the pin the direction line is actually using.
fn park_channel(id: StepperId) {
    let ch = channel_of(id).index();
    pac::TIM2.ccmr_output(ch / 2).modify(|w| w.set_ocm(ch % 2, pac::timer::vals::Ocm::FORCE_INACTIVE));
}

/// Account for one completed pulse on every channel that was still owed one, parking the channels
/// that have just finished. Returns whether anything is still owed.
///
/// Shared by the interrupt and by [`StepperPortTim2::stop`]'s drain, so that a pulse which
/// completed in the instant before the counter stopped is counted exactly once either way.
fn service_completed_pulse() -> bool {
    let mut still_owed = false;
    for id in StepperId::ALL {
        let i = id.index();
        let remaining = REMAINING[i].load(Ordering::Relaxed);
        if remaining == 0 {
            continue;
        }
        POSITION[i].fetch_add(DIR_STEP[i].load(Ordering::Relaxed), Ordering::Relaxed);
        REMAINING[i].store(remaining - 1, Ordering::Relaxed);
        if remaining == 1 {
            park_channel(id);
        } else {
            still_owed = true;
        }
    }
    still_owed
}

embassy_stm32::bind_interrupts!(struct Irqs {
    TIM2 => StepHandler;
});

struct StepHandler;

impl interrupt::typelevel::Handler<interrupt::typelevel::TIM2> for StepHandler {
    unsafe fn on_interrupt() {
        let timer = pac::TIM2;
        if !timer.sr().read().uif() {
            return;
        }
        timer.sr().modify(|w| w.set_uif(false));

        // We are standing at the end of a completed pulse. Stopping here rather than at the start
        // of the next period is what keeps the counts and the shafts in agreement.
        if !service_completed_pulse() {
            timer.cr1().modify(|w| w.set_cen(false));
        }
    }
}

/// One or two step clocks on TIM2, with their direction levels on plain GPIO.
pub struct StepperPortTim2 {
    timer: Timer<'static, p::TIM2>,
    /// Holds PA2 in its alternate function. Never read; dropping it would hand the pin back to
    /// GPIO underneath a running timer.
    _step0: PwmPin<'static, p::TIM2, Ch3, AfioRemap<0>>,
    #[cfg(feature = "dual-stepper")]
    _step1: PwmPin<'static, p::TIM2, embassy_stm32::timer::Ch4, AfioRemap<0>>,
    /// Direction level per actuator, `None` for a channel this build does not have. Plain outputs,
    /// deliberately *not* timer channels: a direction is a level, and leaving it on the timer
    /// would mean a stray compare event could move it.
    dirs: [Option<Output<'static>>; StepperId::COUNT],
    running: bool,
    /// The commands currently being executed, so an unchanged pair can be skipped.
    last: Option<PerStepper<StepCommand>>,
}

impl StepperPortTim2 {
    /// One actuator: `step` is COM4 pin 1 (PA2) and `dir` is COM4 pin 2 (PA3).
    #[cfg(not(feature = "dual-stepper"))]
    pub fn new(step: Peri<'static, p::PA2>, dir: Peri<'static, p::PA3>, timer: Peri<'static, p::TIM2>) -> Self {
        let step0 = Self::step_pin_0(step);
        let dir0 = Output::new(dir, GpioLevel::Low, Speed::Low);
        Self::assemble(timer, step0, [Some(dir0), None])
    }

    /// Two actuators: `step0`/`step1` are COM4 pins 1 and 2 (PA2/PA3), and the two direction
    /// levels move off COM4 onto PC10 and PA5. See the module docs for the whole pinout.
    #[cfg(feature = "dual-stepper")]
    pub fn new(
        step0: Peri<'static, p::PA2>,
        step1: Peri<'static, p::PA3>,
        dir0: Peri<'static, p::PC10>,
        dir1: Peri<'static, p::PA5>,
        timer: Peri<'static, p::TIM2>,
    ) -> Self {
        use embassy_stm32::timer::Ch4;

        let step0 = Self::step_pin_0(step0);
        <p::PA3 as TimerPin<p::TIM2, Ch4, AfioRemap<0>>>::afio_remap(&step1);
        let step1: PwmPin<'static, p::TIM2, Ch4, AfioRemap<0>> = PwmPin::new(step1, OutputType::PushPull);

        let dir0 = Output::new(dir0, GpioLevel::Low, Speed::Low);
        let dir1 = Output::new(dir1, GpioLevel::Low, Speed::Low);

        Self::assemble(timer, step0, step1, [Some(dir0), Some(dir1)])
    }

    /// TIM2 with no remap is CH1..CH4 on PA0..PA3, which is the mapping the board relies on.
    fn step_pin_0(pin: Peri<'static, p::PA2>) -> PwmPin<'static, p::TIM2, Ch3, AfioRemap<0>> {
        <p::PA2 as TimerPin<p::TIM2, Ch3, AfioRemap<0>>>::afio_remap(&pin);
        PwmPin::new(pin, OutputType::PushPull)
    }

    fn assemble(
        timer: Peri<'static, p::TIM2>,
        step0: PwmPin<'static, p::TIM2, Ch3, AfioRemap<0>>,
        #[cfg(feature = "dual-stepper")] step1: PwmPin<'static, p::TIM2, embassy_stm32::timer::Ch4, AfioRemap<0>>,
        dirs: [Option<Output<'static>>; StepperId::COUNT],
    ) -> Self {
        let mut timer = Timer::new(timer);
        timer.set_tick_freq(Hertz::hz(TICK_HZ));
        // Autoreload preloaded so a speed change mid-move takes effect at a period boundary
        // instead of truncating whatever pulse is in flight.
        timer.set_autoreload_preload(true);
        timer.enable_update_interrupt(true);

        let mut port = Self {
            timer,
            _step0: step0,
            #[cfg(feature = "dual-stepper")]
            _step1: step1,
            dirs,
            running: false,
            last: None,
        };
        port.configure_channel(StepperId::Stepper0);
        #[cfg(feature = "dual-stepper")]
        port.configure_channel(StepperId::Stepper1);
        // `set_tick_freq` raises the update flag on its way out (its `UG` does not set `URS`,
        // unlike the rest of the low-level API), so the counter has to be brought to a known state
        // before the interrupt is unmasked. `stop` does that, and unmasks.
        port.stop();
        port
    }

    /// Put one channel into the shape a step clock needs: compare value preloaded, parked low.
    fn configure_channel(&mut self, id: StepperId) {
        let ch = channel_of(id);
        self.timer.set_output_compare_preload(ch, true);
        self.timer.set_output_compare_mode(ch, OutputCompareMode::ForceInactive);
        // The pin stays driven by the timer at all times; `park_channel` decides high or low.
        self.timer.enable_channel(ch, true);
    }

    /// Halt both pulse trains where they are. The step counters keep their values: they are the
    /// only record of where the shafts are.
    ///
    /// The interrupt is masked across the whole thing, which is what makes this a usable critical
    /// section for the shared step state: clearing `CEN` on its own does not stop an update event
    /// that has *already* fired from being serviced somewhere in the middle of the lines below.
    fn stop(&mut self) {
        interrupt::TIM2.disable();

        self.timer.stop();

        // A pulse that completed in the instant before the counter stopped leaves the flag set
        // with its interrupt unserved. That pulse physically happened on every channel that was
        // owed one, so it is counted here rather than discarded — dropping it would put a counter
        // permanently one step behind its shaft, and with no homing that error never comes back
        // out.
        if self.timer.clear_update_interrupt() {
            service_completed_pulse();
        }
        for id in StepperId::ALL {
            REMAINING[id.index()].store(0, Ordering::Relaxed);
            park_channel(id);
        }
        self.running = false;

        interrupt::TIM2.unpend();
        // SAFETY: the handler only touches the step atomics and TIM2's own registers, and the
        // caller is done with both by the time this returns.
        unsafe { interrupt::TIM2.enable() };
    }

    /// Point an actuator's direction pin at `step`, which is +1 or -1.
    ///
    /// Only ever called with the counter stopped, so the ≥ 35 µs the driver wants before the next
    /// clock edge is provided by the restart: PWM mode 2 puts the first edge a full
    /// `period - PULSE_US` after the counter starts.
    fn set_direction(&mut self, id: StepperId, step: i32) {
        let Some(pin) = self.dirs[id.index()].as_mut() else {
            return;
        };
        if DIR_STEP[id.index()].load(Ordering::Relaxed) == step {
            return;
        }
        pin.set_level(if step >= 0 { GpioLevel::High } else { GpioLevel::Low });
        DIR_STEP[id.index()].store(step, Ordering::Relaxed);
    }

    /// Load the shared period and pulse width for `step_hz`, arm the channels that have work, and
    /// let the counter run.
    fn start(&mut self, step_hz: u32, armed: PerStepper<bool>) {
        let period_us = TICK_HZ / step_hz.clamp(MIN_STEP_HZ, MAX_STEP_HZ);
        let arr = (period_us - 1) as u16;
        // PWM mode 2 is active from the compare value to the top of the period, so the pulse is
        // the tail `PULSE_US` of it. Both channels share the period, so they share the width.
        let ccr = arr.saturating_sub(PULSE_US - 1);

        // Compare values first, autoreload second. Every compare register is preloaded, so its
        // shadow only loads on an update event — and `set_max_compare_value` generates one on its
        // way out. Writing the autoreload first would therefore latch the new period against the
        // *previous* move's compare value, and a compare value above the new autoreload leaves
        // PWM mode 2 never going active: a period that emits no pulse but still raises the update
        // that counts one. A count and its shaft would part company on the spot.
        for (id, armed) in armed.iter() {
            let ch = channel_of(id);
            self.timer.set_compare_value(ch, ccr);
            self.timer.set_output_compare_mode(
                ch,
                if *armed {
                    OutputCompareMode::PwmMode2
                } else {
                    OutputCompareMode::ForceInactive
                },
            );
        }
        self.timer.set_max_compare_value(arr);

        // Belt and braces on the shadow load, and a counter that starts from zero so the first
        // edge is a full `period - PULSE_US` away — which is what gives a direction change the
        // settling time the driver asks for. `generate_update_event` masks the update via `URS`,
        // so none of this can be mistaken for a completed step.
        self.timer.generate_update_event();
        self.timer.clear_update_interrupt();
        interrupt::TIM2.unpend();

        self.timer.start();
        self.running = true;
    }
}

impl StepPort for StepperPortTim2 {
    fn position_steps(&self, id: StepperId) -> i32 {
        POSITION[id.index()].load(Ordering::Relaxed)
    }

    fn set_position_steps(&mut self, id: StepperId, steps: i32) {
        self.stop();
        POSITION[id.index()].store(steps, Ordering::Relaxed);
        self.last = None;
    }

    fn command(&mut self, cmds: PerStepper<StepCommand>) {
        // A repeat of what is already running — the plateau of a ramp — must not restart the move,
        // or every control tick would throw away the period in flight.
        if self.running && self.last == Some(cmds) {
            return;
        }

        // Everything below mutates state the ISR also touches. Stopping first is the whole
        // synchronisation scheme: with the counter halted no update event can fire. It also drains
        // any pulse that completed on the way down, so the distances below are computed against
        // counters that are already up to date.
        self.stop();
        self.last = Some(cmds);

        let mut armed = PerStepper::splat(false);
        let mut distance = PerStepper::splat(0u32);
        let mut rate = None;

        for (id, cmd) in cmds.iter() {
            // A channel this build does not have has no direction pin either, and arming it would
            // pulse a step line that is somebody else's GPIO.
            if self.dirs[id.index()].is_none() {
                continue;
            }
            let delta = cmd.target_steps as i64 - self.position_steps(id) as i64;
            if cmd.step_hz == 0 || delta == 0 {
                continue;
            }
            armed[id] = true;
            distance[id] = delta.unsigned_abs().min(u32::MAX as u64) as u32;
            self.set_direction(id, if delta > 0 { 1 } else { -1 });
            // One autoreload, one rate. The slower of the two planned rates is the only safe
            // reconciliation: under-running a stepper costs time, over-running it costs steps.
            rate = Some(rate.map_or(cmd.step_hz, |r: u32| r.min(cmd.step_hz)));
        }

        let Some(rate) = rate else {
            return;
        };
        for (id, distance) in distance.iter() {
            REMAINING[id.index()].store(*distance, Ordering::Relaxed);
        }
        self.start(rate, armed);
    }
}

/// Halt both pulse trains from panic or fault context, using raw PAC access only.
///
/// The counterpart of [`crate::panic::safe_outputs`] for this port, and just as necessary: TIM2
/// keeps clocking the drivers from hardware after the CPU has given up, so a panic with a move in
/// flight would otherwise drive the actuators to wherever the remaining pulses take them.
///
/// # Safety
/// Takes over TIM2 unconditionally. Only call when the control path is already dead.
pub unsafe fn stop_pulses() {
    pac::TIM2.cr1().modify(|w| w.set_cen(false));
}
