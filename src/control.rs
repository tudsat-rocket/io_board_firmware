//! The control task: the only thing in this firmware that drives hardware outputs.
//!
//! Everything else — the SDO server, the TPDO broadcaster, the sensor loop — reads and writes
//! [`crate::store`]. This task is what turns the store's *intent* (commanded valve positions,
//! direct output writes, the link state) into pulse widths and gate levels, and what writes the
//! resulting *observation* (target, measured, status, rail currents) back.
//!mator would be modelling something it does not control.
//!
//! It ticks at [`TICK`] and wakes early on [`crate::store::CONTROL_WAKE`], so a valve command
//! from the bus is acted on immediately rather than at the next tick boundary.

use embassy_time::Instant;

use crate::config::{Config, ValveConfig};
use crate::cpu::CpuMonitor;
use crate::hco::{HcoState, Level, State};
use crate::heating::{Heating, HeatingState};
use crate::index::{HcoId, PerAnalogInput, PerHco, PerSensorSlot, PerValve, ValveId};
use crate::leds::{LedsState, StateLedPub};
use crate::outputs::{Outputs, digital, pwm};
use crate::rail_sense::{NoRails, RailSensing, Rails};
use crate::relief::Relief;
use crate::safety::{self, FallbackLatch};
use crate::sensors::AnalogSensing;
use crate::store::{CONTROL_WAKE, LinkState, SENSOR_INVALID, STORE};
use crate::valves::{
    NoFeedback, PositionFeedback, Valve, ValveDrive, ValveStatus, is_unpowered, position_of, unpowered_at,
};

/// Tick period. Fast enough that the measured-position estimate is smooth for a servo that takes
/// on the order of a second to travel, slow enough to leave the bus and sensor tasks room.
const TICK: embassy_time::Duration = embassy_time::Duration::from_millis(20);

/// Generic over rail sensing so this whole task can be built and driven by a host test against
/// [`NoRails`] and a mocked [`crate::hco::HcoControl`], not just against real hardware.
pub struct Control<R: RailSensing + AnalogSensing = NoRails> {
    outputs: Outputs,
    valves: PerValve<Valve>,
    /// Position feedback from anything that is not a configured sensor slot.
    ///
    /// Nothing on this board has any: an encoder reaches a valve through the sensor plane
    /// (0x301B), which `decide` consults first. The hook stays because [`PositionFeedback`] is
    /// how a directly-wired potentiometer would arrive, and that is a wiring change rather than
    /// a firmware one.
    feedback: NoFeedback,
    // TODO: document what exactly latch is, or choose a better name
    latch: FallbackLatch,
    relief: Relief,
    heating: PerValve<Heating>,
    /// Desired state of each output from the direct-control path. Owns whichever outputs no valve
    /// claims; in raw debug mode it can also override an owned one.
    direct: HcoState,
    rails: R,
    leds: StateLedPub,
    last_leds: LedsState,
    last_link: LinkState,
    /// Toggled once a second so the white LED shows the executor is still running.
    blink: bool,
    last_blink: Instant,
    /// Whole-board CPU utilization. Lives here because this is the one loop on the board with a
    /// deadline to miss, which makes it the natural thing to measure the system against; the
    /// measurement itself is idle time and belongs to nobody. See [`crate::cpu`].
    cpu: CpuMonitor,
}

/// The concrete `Control` the firmware spawns, monomorphised per revision so it can cross an
/// `#[embassy_executor::task]` boundary (tasks cannot be generic) — mirrors
/// `board::ConfigStore = NorConfigStore<ExtFlash>`.
#[cfg(all(feature = "hardware", feature = "rev3"))]
pub type BoardControl = Control<crate::board::OnboardSensRev3>;
#[cfg(all(feature = "hardware", feature = "rev2"))]
pub type BoardControl = Control<NoRails>;

/// The direct-write fields `decide()` needs, read from the store only when `pending.outputs` is
/// set. Kept as a struct rather than four loose parameters so `TickInputs` reads as one thing.
struct DirectWrites {
    digital: PerHco<u8>,
    pwm_us: PerHco<u16>,
    /// Which outputs were actually written since the last tick; the other three fields hold the
    /// last value written to *every* output, so without this a single write would re-apply all
    /// four and stamp on whatever a valve had done to them in between.
    dirty: PerHco<bool>,
    is_pwm: PerHco<bool>,
}

/// Everything [`Control::decide`] needs to run one tick's arbitration, gathered from the store,
/// the clock and (optionally) hardware rail sensing beforehand. Deliberately plain data: nothing
/// here is `STORE` or a global, which is what makes `decide` callable directly from a host test.
struct TickInputs {
    config: Config,
    commanded: PerValve<u16>,
    sensor_value: PerSensorSlot<i16>,
    raw_debug: bool,
    pending: crate::store::Pending,
    direct_writes: Option<DirectWrites>,
    rails: Option<Rails>,
    now: Instant,
    since_heartbeat: u32,
    seen: bool,
}

/// Where a valve's position sensor says it is, in promille, or `None` to fall back to the
/// travel-time estimate.
///
/// `None` covers three cases that all mean the same thing to the valve model — no sensor
/// configured, the sensor has no reading this tick, or the reading is out of range — because a
/// valve that has lost its encoder should coast on the estimate rather than freeze at whatever
/// the last good reading was. A magnet knocked off its shaft is exactly this case: the AS5600
/// stops answering, the slot reports [`SENSOR_INVALID`], and the valve keeps working open-loop.
///
/// The unit is not re-checked here: [`Config::sanity_check`] refuses a config whose position
/// sensor reports anything but promille, so by the time a config is running this reading is
/// already in the right unit.
fn sensed_position(cfg: &ValveConfig, values: &PerSensorSlot<i16>) -> Option<u16> {
    let reading = values[cfg.position_sensor?];
    if reading == SENSOR_INVALID || reading < 0 {
        return None;
    }
    Some((reading as u16).min(crate::config::PROMILLE_MAX))
}

/// What one tick decided, ready to push into the store.
struct TickOutcome {
    targets: PerValve<u16>,
    measured: PerValve<u16>,
    statuses: PerValve<u8>,
    currents: PerValve<u16>,
    relief_state: u8,
    /// 0x2019, a [`crate::heating::HeatingState`] discriminant per valve.
    heating_states: PerValve<u8>,
    link: LinkState,
    /// `Some` only when the LED state actually changed this tick, which is what keeps the `STORE`
    /// write and the pubsub publish conditional. Packing into 0x2030's byte happens at the store
    /// write, via [`LedsState::as_byte`] — a decision, not a wire value, until then.
    leds: Option<LedsState>,
}

impl<R: RailSensing + AnalogSensing> Control<R> {
    pub fn new(outputs: Outputs, rails: R, leds: StateLedPub) -> Self {
        let now = Instant::now();
        Self {
            outputs,
            valves: PerValve::from_fn(|_| Valve::new(now)),
            feedback: NoFeedback,
            latch: FallbackLatch::new(),
            relief: Relief::new(),
            heating: PerValve::from_fn(|_| Heating::new()),
            direct: HcoState::splat(State::Digital(Level::Low)),
            rails,
            leds,
            last_leds: LedsState::default(),
            last_link: LinkState::NeverSeen,
            blink: false,
            last_blink: now,
            cpu: CpuMonitor::new(now, TICK),
        }
    }

    pub async fn run(&mut self) -> ! {
        loop {
            // wait for explicit wake or next tick
            let _ = embassy_time::with_timeout(TICK, CONTROL_WAKE.wait()).await;
            let started = Instant::now();
            self.tick().await;
            // Timed out here so that the awaits *inside* the tick are counted and the wait for
            // the next one is not: how long an iteration takes, not how often it runs.
            let now = Instant::now();
            self.cpu.update(now.saturating_duration_since(started), now);
        }
    }

    async fn tick(&mut self) {
        let rails = self.read_rails().await;
        let now = Instant::now();
        let since_heartbeat = safety::since_last_heartbeat();
        let seen = safety::master_ever_seen();

        // --- pull intent out of the store ----------------------------------
        let (config, commanded, sensor_value, raw_debug, pending) = {
            let mut store = STORE.lock().await;
            let pending = store.pending.take();
            (store.config.clone(), store.valve_commanded, store.sensor_value, store.raw_debug, pending)
        };

        // The COM5/COM6 pins are on this task's ADC but belong to the sensor plane: sampled here,
        // left in the store, and calibrated by the sensor task into whatever slot named the pin.
        // Only the pins a slot actually names are converted, so a board with no NTC fitted spends
        // nothing on this.
        let raw_analog = self.read_analog(&config).await;

        let direct_writes = if pending.outputs {
            let mut store = STORE.lock().await;
            let dirty = core::mem::take(&mut store.hco_direct_dirty);
            Some(DirectWrites {
                digital: store.hco_digital,
                pwm_us: store.hco_pwm_us,
                dirty,
                is_pwm: store.hco_direct_pwm,
            })
        } else {
            None
        };

        // --- the whole arbitration decision, synchronously ------------------
        let outcome = self.decide(TickInputs {
            config,
            commanded,
            sensor_value,
            raw_debug,
            pending,
            direct_writes,
            rails,
            now,
            since_heartbeat,
            seen,
        });

        // --- write observation back ----------------------------------------
        let hco = self.outputs.current();
        let mut store = STORE.lock().await;
        store.raw_analog = raw_analog;
        store.valve_target = outcome.targets;
        store.valve_measured = outcome.measured;
        store.valve_status = outcome.statuses;
        store.valve_current_ma = outcome.currents;
        store.relief_state = outcome.relief_state;
        store.valve_heating_state = outcome.heating_states;
        store.link_state = outcome.link;
        store.ms_since_heartbeat = since_heartbeat;
        for (id, state) in hco.iter() {
            match state {
                State::Digital(level) => {
                    store.hco_digital[id] = level.as_u8();
                    store.hco_pwm_us[id] = 0;
                }
                State::Pwm(us) => {
                    // TODO: should this be =1?
                    store.hco_digital[id] = 1;
                    store.hco_pwm_us[id] = us.as_u16();
                }
            }
        }
        if let Some(rails) = rails {
            store.rail_current_ma = rails.current_ma;
            store.rail_voltage_mv = rails.voltage_mv;
        }
        if let Some(leds) = outcome.leds {
            store.leds = leds.as_byte();
        }
        // Mirrored here rather than from a task of its own: the counters are bumped from places
        // that cannot take this lock, and this is the one place that holds it periodically
        // anyway. On a healthy board it is a single atomic load.
        store.refresh_error_counters();
        // Published by the previous iteration's `cpu.update`, so what the bus reads is one tick
        // behind a number that only moves every half second anyway.
        store.refresh_cpu();
    }

    /// The whole per-tick arbitration decision: apply pending direct writes, resolve
    /// fallback/relief/clamp per valve, drive the valve model, push to `outputs`, and decide the
    /// LED state.
    fn decide(&mut self, inputs: TickInputs) -> TickOutcome {
        self.apply_pending(&inputs.pending, inputs.direct_writes, inputs.raw_debug, &inputs.config);

        let link = safety::evaluate(&inputs.config, inputs.raw_debug, inputs.seen, inputs.since_heartbeat);
        if link != self.last_link {
            defmt::warn!("master link: {} -> {}", self.last_link, link);
            self.last_link = link;
        }
        self.latch.enter(link);

        // Overpressure relief is evaluated before anything else and outranks everything below —
        // the master's command, the input clamp, and both fallback stages. See `crate::relief`.
        // The watched slot is a `SensorSlot`, so there is no longer an out-of-range case to fall
        // back from: an unconfigured slot simply reads `SENSOR_INVALID`, which inhibits.
        let reading = inputs.sensor_value[inputs.config.relief.sensor];
        let relief_position = self.relief.update(&inputs.config.relief, reading, inputs.now);

        // --- run each valve -------------------------------------------------
        let mut desired = self.direct;
        let mut targets = PerValve::splat(0u16);
        let mut measured = PerValve::splat(0u16);
        let mut statuses = PerValve::splat(0u8);
        let mut currents = PerValve::splat(0u16);

        for (valve, cfg) in inputs.config.valves.iter() {
            let current_ma = valve_current(cfg, inputs.rails);
            currents[valve] = current_ma.unwrap_or(0);

            let relieving = relief_position.filter(|_| inputs.config.relief.valve == Some(valve));
            let target = match relieving {
                Some(position) => {
                    // Take this valve back out of any fallback release, so that when the pulse
                    // ends the fallback re-drives it to its own position and re-runs the settle
                    // before unpowering — rather than leaving it released wherever relief left it.
                    self.latch.rearm(valve);
                    position
                }
                None => self.resolve_target(valve, cfg, inputs.commanded[valve], link, inputs.now),
            };
            targets[valve] = target;

            // A configured position sensor wins over the directly-wired hook: it is the one an
            // operator can point at a different slot without touching the firmware.
            let feedback = sensed_position(cfg, &inputs.sensor_value).or_else(|| self.feedback.position(valve));
            let drive = self.valves[valve].tick(cfg, target, inputs.now, current_ma, feedback);
            apply_drive(&mut desired, cfg, drive);

            measured[valve] = self.valves[valve].measured_word();
            statuses[valve] = self.valves[valve].status() as u8;
        }

        // --- heating pads ---------------------------------------------------
        let mut heating_states = PerValve::splat(HeatingState::Disabled as u8);
        for (valve, cfg) in inputs.config.heating.iter() {
            let reading = cfg.sensor.map_or(SENSOR_INVALID, |slot| inputs.sensor_value[slot]);
            let on = self.heating[valve].update(cfg, reading);
            heating_states[valve] = self.heating[valve].state() as u8;
            if inputs.raw_debug {
                continue;
            }
            for hco in cfg.outputs.iter() {
                desired[hco] = digital(on);
            }
        }

        self.outputs.drive(desired);

        let leds = self.decide_leds(link, inputs.raw_debug, &statuses, inputs.now);

        TickOutcome {
            targets,
            measured,
            statuses,
            currents,
            relief_state: self.relief.state() as u8,
            heating_states,
            link,
            leds,
        }
    }

    /// Act on writes that landed since the last tick.
    fn apply_pending(
        &mut self,
        pending: &crate::store::Pending,
        direct_writes: Option<DirectWrites>,
        raw_debug: bool,
        config: &Config,
    ) {
        if pending.config {
            // A remapped valve can leave its old output owned by nobody and still energized.
            // Dropping everything to a known state first is cheaper than reasoning about which
            // outputs changed hands.
            defmt::info!("control: configuration changed, re-deriving outputs");
            self.direct = HcoState::splat(State::Digital(Level::Low));
            self.outputs.all_off();
        }

        if let Some(writes) = direct_writes {
            for hco in HcoId::ALL {
                if !writes.dirty[hco] {
                    continue;
                }
                let state = if writes.is_pwm[hco] {
                    pwm(writes.pwm_us[hco])
                } else {
                    digital(writes.digital[hco] != 0)
                };
                self.direct[hco] = state;
                // Only in raw debug mode can a direct write reach an output a valve owns, and
                // only then does it need to survive the valve recomputing its own outputs.
                if raw_debug && config.hco_owner(hco).is_some() {
                    self.outputs.install_override(hco, state);
                }
            }
        }

        // Leaving raw debug mode hands every output back to its normal owner immediately.
        if !raw_debug && self.outputs.has_overrides() {
            defmt::info!("control: raw debug mode left, releasing output overrides");
            self.outputs.clear_overrides();
        }

        // A fresh command takes that valve's outputs back from any override.
        if pending.valves.any() {
            for valve in ValveId::ALL {
                if !pending.valves[valve] {
                    continue;
                }
                self.latch.rearm(valve);
                let cfg = &config.valves[valve];
                for hco in [cfg.signal_hco, cfg.power_hco].into_iter().flatten() {
                    self.outputs.release_override(hco);
                }
            }
        }
    }

    /// Resolve commanded -> target: the input clamp in normal operation, the fallback action when
    /// a stage is active.
    fn resolve_target(
        &mut self,
        valve: ValveId,
        cfg: &ValveConfig,
        commanded: u16,
        link: LinkState,
        now: Instant,
    ) -> u16 {
        match safety::action_for(link, cfg) {
            Some(action) => {
                // Settled *at the fallback position*: a valve still sitting where the master (or
                // the previous stage) left it must be driven to this stage's position and held
                // there before its drive may be dropped.
                let settled = self.valves[valve].settled_at(cfg, now, action.position);
                self.latch.target(valve, action, settled)
            }
            // The clamp applies to the position regardless; the release flag rides through it, so
            // "release, and you are at X" survives being clamped to a legal X.
            None => {
                let clamped = cfg.clamp(position_of(commanded));
                if is_unpowered(commanded) {
                    unpowered_at(clamped)
                } else {
                    clamped
                }
            }
        }
    }

    async fn read_rails(&mut self) -> Option<Rails> {
        self.rails.read().await
    }

    /// Sample the COM5/COM6 pins a [`crate::config::SensorKind::Ntc`] slot is configured onto.
    ///
    /// [`crate::store::RAW_INVALID`] for the rest, which covers three things the sensor plane
    /// treats alike: a pin nothing is configured on, a pin this build did not hand over, and a
    /// conversion that never happened.
    async fn read_analog(&mut self, config: &Config) -> PerAnalogInput<u16> {
        self.rails.read_analog(config.analog_inputs_used()).await
    }

    /// Decide the LED state and publish it if it changed, returning the state to mirror into the
    /// store (`None` when nothing changed, so the store write stays conditional).
    fn decide_leds(
        &mut self,
        link: LinkState,
        raw_debug: bool,
        statuses: &PerValve<u8>,
        now: Instant,
    ) -> Option<LedsState> {
        if (now - self.last_blink).as_millis() >= 500 {
            self.blink = !self.blink;
            self.last_blink = now;
        }

        let stalled = statuses.values().any(|s| *s == ValveStatus::Stalled as u8);
        let state = LedsState {
            // Red is "this board is not in its normal flight configuration" — which includes
            // actively venting a vessel on its own initiative.
            red: raw_debug || stalled || self.relief.is_active(),
            // Yellow is "the master is not talking to me".
            yellow: !matches!(link, LinkState::Alive),
            // White is a plain "the executor is running" heartbeat.
            white: self.blink,
        };

        if state != self.last_leds {
            self.leds.publish_immediate(state);
            self.last_leds = state;
            Some(state)
        } else {
            None
        }
    }
}

/// Attribute a rail current to a valve.
///
/// The board has one shunt across HCO1+2 and one across HCO3+4, so a valve that owns a whole pair
/// gets an unambiguous reading and a valve sharing a pair with something else does not. The
/// vehicle harness wires servos as whole pairs precisely so this works; a board wired as four
/// independent solenoids should leave `stall_ma` at 0.
fn valve_current(cfg: &ValveConfig, rails: Option<Rails>) -> Option<u16> {
    let rails = rails?;
    let hco = cfg.signal_hco?;
    Some(rails.current_ma[hco.pair().rail()])
}

/// Fold one valve's demand into the desired output states.
fn apply_drive(desired: &mut HcoState, cfg: &ValveConfig, drive: ValveDrive) {
    let set = |desired: &mut HcoState, hco: Option<HcoId>, state: State| {
        if let Some(hco) = hco {
            desired[hco] = state;
        }
    };

    match drive {
        ValveDrive::Released => {
            set(desired, cfg.signal_hco, digital(false));
            set(desired, cfg.power_hco, digital(false));
        }
        ValveDrive::Solenoid(on) => {
            set(desired, cfg.signal_hco, digital(on));
            set(desired, cfg.power_hco, digital(on));
        }
        ValveDrive::Servo { pulse_us } => {
            set(desired, cfg.power_hco, digital(true));
            set(desired, cfg.signal_hco, pwm(pulse_us));
        }
    }
}

#[cfg(feature = "hardware")]
#[embassy_executor::task]
pub async fn run_control(control: &'static mut BoardControl) -> ! {
    control.run().await
}

#[cfg(test)]
mod tests {
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::pubsub::PubSubChannel;

    use super::*;
    use crate::config::{FallbackAction, ReliefConfig};
    use crate::hco::{HcoControl, PwmMicros};
    use crate::index::{HcoPair, SensorSlot};
    use crate::store::Pending;

    /// Records nothing of its own: every assertion below reads back through
    /// `Outputs::current()`, which already mirrors the last state actually pushed. This just
    /// needs to exist and not panic, so `Outputs::new`'s `&'static mut dyn HcoControl` has
    /// something real to point at.
    ///
    /// `set_level`/`set_pwm_micros` come from `HcoControl`'s defaults now, so a mock only has to
    /// supply the two methods that are genuinely revision-specific.
    #[derive(Default)]
    struct MockHco {
        state: HcoState,
    }

    impl HcoControl for MockHco {
        fn get_state(&self) -> HcoState {
            self.state
        }
        fn set_state(&mut self, target_state: HcoState) {
            self.state = target_state;
        }
    }

    /// A `Control<NoRails>` wired to a fresh `MockHco` and a real (but test-local) LED pubsub
    /// channel. `Box::leak` is `std`'s heap, available under `cfg(test)` (the crate is
    /// `#![cfg_attr(not(test), no_std)]`) and never linked into firmware — it exists purely to
    /// satisfy `Outputs::new`'s and the LED channel's `'static` bounds cheaply in test setup.
    ///
    /// `Instant::now()` inside `Control::new` goes through the host-test mock time driver, which
    /// starts at zero and stays there because nothing anywhere in this test suite ever calls
    /// `.advance()` on it — so `last_blink`/each `Valve`'s `last_tick` are deterministically
    /// seeded at `Instant::from_millis(0)`, and every `now` passed into `decide()` below should
    /// stay at or after that.
    fn test_control() -> Control<NoRails> {
        let hco: &'static mut MockHco = Box::leak(Box::new(MockHco::default()));
        let outputs = Outputs::new(hco);
        let channel: &'static PubSubChannel<CriticalSectionRawMutex, LedsState, 4, 1, 1> =
            Box::leak(Box::new(PubSubChannel::new()));
        Control::new(outputs, NoRails, channel.publisher().unwrap())
    }

    fn inputs(config: Config, commanded: [u16; 4], now: Instant) -> TickInputs {
        TickInputs {
            config,
            commanded: PerValve::new(commanded),
            sensor_value: PerSensorSlot::splat(0),
            raw_debug: false,
            pending: Pending::default(),
            direct_writes: None,
            rails: None,
            now,
            since_heartbeat: 0,
            seen: true,
        }
    }

    /// The whole point of an encoder: `measured` stops being "where the travel time says it
    /// should be by now" and becomes "where it is". Here the valve is commanded fully open but
    /// the encoder says it has barely moved — a jammed valve, which the open-loop estimate would
    /// have reported as wide open.
    #[test]
    fn a_position_sensor_overrides_the_travel_time_estimate() {
        use crate::config::SensorSlotConfig;
        use crate::index::{I2cBus, SensorSlot};

        let mut ctl = test_control();
        let cfg = Config::new()
            .with_valve(
                ValveId::Valve0,
                ValveConfig::servo_on_pair(HcoPair::A, 2000, 1000, 500).with_position_sensor(SensorSlot::Slot0),
            )
            .with_sensor(SensorSlot::Slot0, SensorSlotConfig::encoder(I2cBus::Bus0, 0, 1024));

        let mut tick = inputs(cfg.clone(), [1000, 0, 0, 0], Instant::from_millis(0));
        tick.sensor_value[SensorSlot::Slot0] = 120;
        ctl.decide(tick);

        // A full travel time later the estimate alone would say 1000.
        let mut tick = inputs(cfg, [1000, 0, 0, 0], Instant::from_millis(500));
        tick.sensor_value[SensorSlot::Slot0] = 120;
        let outcome = ctl.decide(tick);

        assert_eq!(outcome.targets[ValveId::Valve0], 1000, "the target is still fully open");
        assert_eq!(position_of(outcome.measured[ValveId::Valve0]), 120, "but the encoder says otherwise");
    }

    /// The heating pad end to end: a cold slot switches a real output on, a warm one switches it
    /// off, and the valve beside it is untouched throughout.
    #[test]
    fn a_heating_pad_switches_its_output_from_the_sensor_slot() {
        use crate::config::{SensorSlotConfig, ValveHeatingConfig};
        use crate::index::{AnalogInput, HcoId, SensorSlot};

        let mut ctl = test_control();
        let cfg = Config::new()
            .with_valve(ValveId::Valve0, ValveConfig::solenoid_on(HcoId::Hco0))
            .with_quiet_sensor(SensorSlot::Slot1, SensorSlotConfig::ntc(AnalogInput::Com5Pin1))
            .with_valve_heating(ValveId::Valve0, ValveHeatingConfig::new(HcoId::Hco2, SensorSlot::Slot1, 3_000));
        assert!(cfg.sanity_check().is_ok());

        // 20.00 C against a 30.00 C setpoint: pad on, and the valve stays where it was commanded.
        let mut tick = inputs(cfg.clone(), [0, 0, 0, 0], Instant::from_millis(0));
        tick.sensor_value[SensorSlot::Slot1] = 2_000;
        let outcome = ctl.decide(tick);
        assert_eq!(outcome.heating_states[ValveId::Valve0], HeatingState::Heating as u8);
        assert_eq!(ctl.outputs.current()[HcoId::Hco2], digital(true));
        assert_eq!(ctl.outputs.current()[HcoId::Hco0], digital(false), "the valve is not commanded open");

        // 35.00 C: past the top of the dead band, pad off.
        let mut tick = inputs(cfg.clone(), [0, 0, 0, 0], Instant::from_millis(20));
        tick.sensor_value[SensorSlot::Slot1] = 3_500;
        let outcome = ctl.decide(tick);
        assert_eq!(outcome.heating_states[ValveId::Valve0], HeatingState::Idle as u8);
        assert_eq!(ctl.outputs.current()[HcoId::Hco2], digital(false));

        // A dead thermistor is not "cold": the pad must not come back on.
        let mut tick = inputs(cfg, [0, 0, 0, 0], Instant::from_millis(40));
        tick.sensor_value[SensorSlot::Slot1] = SENSOR_INVALID;
        let outcome = ctl.decide(tick);
        assert_eq!(outcome.heating_states[ValveId::Valve0], HeatingState::SensorFault as u8);
        assert_eq!(ctl.outputs.current()[HcoId::Hco2], digital(false));
    }

    /// Two pads on one thermostat: one cold reading switches both outputs, one warm reading both
    /// off again.
    #[test]
    fn two_pads_follow_one_sensor_together() {
        use crate::config::{SensorSlotConfig, ValveHeatingConfig};
        use crate::index::{AnalogInput, HcoId, SensorSlot};

        let mut ctl = test_control();
        let cfg = Config::new()
            .with_quiet_sensor(SensorSlot::Slot1, SensorSlotConfig::ntc(AnalogInput::Com5Pin1))
            .with_valve_heating(
                ValveId::Valve0,
                ValveHeatingConfig::new(HcoId::Hco2, SensorSlot::Slot1, 3_000).also_on(HcoId::Hco3),
            );
        assert!(cfg.sanity_check().is_ok());

        let mut tick = inputs(cfg.clone(), [0, 0, 0, 0], Instant::from_millis(0));
        tick.sensor_value[SensorSlot::Slot1] = 2_000;
        ctl.decide(tick);
        assert_eq!(ctl.outputs.current()[HcoId::Hco2], digital(true));
        assert_eq!(ctl.outputs.current()[HcoId::Hco3], digital(true));

        let mut tick = inputs(cfg, [0, 0, 0, 0], Instant::from_millis(20));
        tick.sensor_value[SensorSlot::Slot1] = 3_500;
        ctl.decide(tick);
        assert_eq!(ctl.outputs.current()[HcoId::Hco2], digital(false));
        assert_eq!(ctl.outputs.current()[HcoId::Hco3], digital(false));
    }

    /// A heater regulates on its own, so a master that goes quiet must not stop it: a valve that
    /// freezes shut during an outage is the failure the pad exists to prevent.
    #[test]
    fn a_heating_pad_keeps_regulating_through_a_fallback() {
        use crate::config::{SensorSlotConfig, ValveHeatingConfig};
        use crate::index::{AnalogInput, HcoId, SensorSlot};

        let mut ctl = test_control();
        let mut cfg = Config::new()
            .with_quiet_sensor(SensorSlot::Slot1, SensorSlotConfig::ntc(AnalogInput::Com5Pin1))
            .with_valve_heating(ValveId::Valve0, ValveHeatingConfig::new(HcoId::Hco2, SensorSlot::Slot1, 3_000));
        cfg.fallback_enabled = true;

        let mut tick = inputs(cfg, [0, 0, 0, 0], Instant::from_millis(0));
        tick.sensor_value[SensorSlot::Slot1] = 2_000;
        tick.since_heartbeat = u32::MAX; // well past both fallback stages
        let outcome = ctl.decide(tick);

        assert_eq!(outcome.link, LinkState::FallbackB);
        assert_eq!(outcome.heating_states[ValveId::Valve0], HeatingState::Heating as u8);
        assert_eq!(ctl.outputs.current()[HcoId::Hco2], digital(true), "still holding the valve warm");
    }

    /// Raw debug hands every output back to direct control, pads included — which is also the
    /// only way to force a pad on with a dead thermistor, and it takes a deliberate mode change.
    #[test]
    fn raw_debug_takes_the_pad_back_from_the_thermostat() {
        use crate::config::{SensorSlotConfig, ValveHeatingConfig};
        use crate::index::{AnalogInput, HcoId, SensorSlot};

        let mut ctl = test_control();
        let cfg = Config::new()
            .with_quiet_sensor(SensorSlot::Slot1, SensorSlotConfig::ntc(AnalogInput::Com5Pin1))
            .with_valve_heating(ValveId::Valve0, ValveHeatingConfig::new(HcoId::Hco2, SensorSlot::Slot1, 3_000));

        let mut tick = inputs(cfg, [0, 0, 0, 0], Instant::from_millis(0));
        tick.sensor_value[SensorSlot::Slot1] = 2_000; // cold: the thermostat would switch it on
        tick.raw_debug = true;
        let outcome = ctl.decide(tick);

        // The state is still reported — an operator can see what the thermostat wanted — but the
        // output is left to 0x2020.
        assert_eq!(outcome.heating_states[ValveId::Valve0], HeatingState::Heating as u8);
        assert_eq!(ctl.outputs.current()[HcoId::Hco2], digital(false), "direct control has it");
    }

    /// An encoder that drops off the bus must not freeze the valve at its last reading — the
    /// open-loop estimate has to take over, or a lost magnet would look like a stuck valve.
    #[test]
    fn a_valve_falls_back_to_the_estimate_when_its_sensor_has_no_reading() {
        use crate::config::SensorSlotConfig;
        use crate::index::{I2cBus, SensorSlot};

        let mut ctl = test_control();
        let cfg = Config::new()
            .with_valve(
                ValveId::Valve0,
                ValveConfig::servo_on_pair(HcoPair::A, 2000, 1000, 500).with_position_sensor(SensorSlot::Slot0),
            )
            .with_sensor(SensorSlot::Slot0, SensorSlotConfig::encoder(I2cBus::Bus0, 0, 1024));

        // `inputs` seeds every slot at 0, so make the first tick a real reading and the second
        // one the encoder having vanished.
        let mut tick = inputs(cfg.clone(), [1000, 0, 0, 0], Instant::from_millis(0));
        tick.sensor_value[SensorSlot::Slot0] = 0;
        ctl.decide(tick);

        let mut tick = inputs(cfg, [1000, 0, 0, 0], Instant::from_millis(500));
        tick.sensor_value[SensorSlot::Slot0] = SENSOR_INVALID;
        let outcome = ctl.decide(tick);

        assert_eq!(
            position_of(outcome.measured[ValveId::Valve0]),
            1000,
            "with no reading the travel-time estimate takes over rather than the valve freezing"
        );
    }

    #[test]
    fn commanding_a_valve_moves_it_through_the_mock_hco() {
        let mut ctl = test_control();
        let cfg = Config::new().with_valve(ValveId::Valve0, ValveConfig::servo_on_pair(HcoPair::A, 2000, 1000, 500));

        // Command valve 0 fully open and let a full travel time pass.
        ctl.decide(inputs(cfg.clone(), [1000, 0, 0, 0], Instant::from_millis(0)));
        let outcome = ctl.decide(inputs(cfg.clone(), [1000, 0, 0, 0], Instant::from_millis(500)));

        assert_eq!(outcome.targets[ValveId::Valve0], 1000);
        assert_eq!(outcome.measured[ValveId::Valve0], 1000, "a full travel_ms must bring it to target");
        // pair 0: power on HCO0, signal (the servo pulse) on HCO1. Fully open is the configured
        // open_us pulse width.
        assert_eq!(ctl.outputs.current()[HcoId::Hco0], State::Digital(Level::High), "power output energised");
        assert_eq!(
            ctl.outputs.current()[HcoId::Hco1],
            State::Pwm(PwmMicros::from_u16_clamped(1000)),
            "signal output at the fully-open pulse width"
        );
    }

    #[test]
    fn raw_debug_direct_write_reaches_a_valve_owned_output() {
        let mut ctl = test_control();
        let cfg = Config::new().with_valve(ValveId::Valve0, ValveConfig::servo_on_pair(HcoPair::A, 2000, 1000, 500));

        let mut writes_inputs = inputs(cfg.clone(), [500, 0, 0, 0], Instant::from_millis(0));
        writes_inputs.raw_debug = true;
        writes_inputs.pending = Pending {
            outputs: true,
            ..Pending::default()
        };
        // HCO1 (valve 0's signal output) gets a direct PWM write, bypassing the valve model.
        writes_inputs.direct_writes = Some(DirectWrites {
            digital: PerHco::splat(0),
            pwm_us: PerHco::new([0, 1800, 0, 0]),
            dirty: PerHco::new([false, true, false, false]),
            is_pwm: PerHco::new([false, true, false, false]),
        });

        ctl.decide(writes_inputs);

        assert_eq!(
            ctl.outputs.current()[HcoId::Hco1],
            State::Pwm(PwmMicros::from_u16_clamped(1800)),
            "the override must win even though valve 0 owns this output"
        );
    }

    #[test]
    fn a_heartbeat_timeout_drives_every_valve_to_its_fallback_a_position() {
        let mut ctl = test_control();
        let cfg = Config::new().with_valve(ValveId::Valve0, ValveConfig::servo_on_pair(HcoPair::A, 2000, 1000, 500));
        assert_eq!(cfg.fallback_a_ms, 3_000, "test assumes the factory-default timeout");

        // Alive, commanded open.
        ctl.decide(inputs(cfg.clone(), [1000, 0, 0, 0], Instant::from_millis(0)));

        // The master goes quiet past the stage-A timeout: the commanded position stops mattering.
        let mut timed_out = inputs(cfg.clone(), [1000, 0, 0, 0], Instant::from_millis(3_020));
        timed_out.since_heartbeat = 3_020;
        let outcome = ctl.decide(timed_out);

        assert_eq!(outcome.link, LinkState::FallbackA);
        assert_eq!(outcome.targets[ValveId::Valve0], 0, "default fallback A action is fully closed");
    }

    /// A stage configured to unpower must still *get the valve there* first. The valve was
    /// holding its commanded position when the stage fired, which is not the same thing as having
    /// arrived at the stage's position — reading it as such released the drive on the first tick
    /// of the stage, leaving the valve wherever the master had left it.
    #[test]
    fn a_fallback_stage_drives_to_its_position_before_unpowering() {
        let mut ctl = test_control();
        let cfg = Config::new().with_valve(ValveId::Valve0, ValveConfig::servo_on_pair(HcoPair::A, 2000, 1000, 500));
        assert!(cfg.valves[ValveId::Valve0].fallback_b.unpower, "test assumes the factory default");
        assert_eq!(cfg.valves[ValveId::Valve0].settle_ms, 500);

        // Alive and holding closed, long enough to be settled there. The last live tick is just
        // before the stage B deadline, so the valve model gets a realistic 20 ms step into the
        // stage rather than being handed the whole five minutes of travel time at once.
        ctl.decide(inputs(cfg.clone(), [0, 0, 0, 0], Instant::from_millis(299_000)));
        ctl.decide(inputs(cfg.clone(), [0, 0, 0, 0], Instant::from_millis(300_000)));

        let mut stage_b = |ms: u64| {
            let mut i = inputs(cfg.clone(), [0, 0, 0, 0], Instant::from_millis(ms));
            i.since_heartbeat = ms as u32;
            ctl.decide(i)
        };

        // First tick of stage B: vent, under power.
        let outcome = stage_b(300_020);
        assert_eq!(outcome.link, LinkState::FallbackB);
        assert_eq!(outcome.targets[ValveId::Valve0], 1000, "stage B must drive the valve open");
        assert!(!is_unpowered(outcome.targets[ValveId::Valve0]));

        // Arrived after a full travel time, but the settle time has not elapsed yet.
        let outcome = stage_b(300_520);
        assert_eq!(position_of(outcome.measured[ValveId::Valve0]), 1000);
        assert_eq!(outcome.targets[ValveId::Valve0], 1000, "still holding it there through the settle");
        let outcome = stage_b(300_600);
        assert_eq!(outcome.targets[ValveId::Valve0], 1000, "settle time is not up yet");

        // Arrived and settled: now the drive may be dropped.
        let outcome = stage_b(301_100);
        assert_eq!(outcome.targets[ValveId::Valve0], unpowered_at(1000));
        assert_eq!(
            ctl.outputs.current()[HcoId::Hco0],
            State::Digital(Level::Low),
            "the power output is actually released"
        );
    }

    /// The same thing across a stage change: a valve held at the stage A position has not been to
    /// the stage B position, however long it has been sitting still.
    #[test]
    fn stage_b_re_drives_a_valve_stage_a_was_holding() {
        let mut ctl = test_control();
        let hold_closed = ValveConfig {
            fallback_a: FallbackAction {
                position: 0,
                unpower: false,
            },
            ..ValveConfig::servo_on_pair(HcoPair::A, 2000, 1000, 500)
        };
        let cfg = Config::new().with_valve(ValveId::Valve0, hold_closed);

        let mut tick = |ms: u64| {
            let mut i = inputs(cfg.clone(), [1000, 0, 0, 0], Instant::from_millis(ms));
            i.since_heartbeat = ms as u32;
            ctl.decide(i)
        };

        // Stage A: closed and held, well past the settle time.
        let outcome = tick(3_020);
        assert_eq!(outcome.link, LinkState::FallbackA);
        assert_eq!(outcome.targets[ValveId::Valve0], 0);
        tick(10_000);

        let outcome = tick(300_020);
        assert_eq!(outcome.link, LinkState::FallbackB);
        assert_eq!(outcome.targets[ValveId::Valve0], 1000, "stage B has to vent before it releases");
    }

    #[test]
    fn relief_overrides_a_fallback_driven_target() {
        let mut ctl = test_control();
        let cfg = Config::new()
            .with_valve(ValveId::Valve0, ValveConfig::servo_on_pair(HcoPair::A, 2000, 1000, 500))
            .with_relief(ReliefConfig::new(ValveId::Valve0, SensorSlot::Slot0, 500));

        // Past the fallback timeout *and* over the relief threshold on the watched slot: without
        // relief this tick would target the fallback-A position (closed, 0).
        let mut over_pressure = inputs(cfg, [0, 0, 0, 0], Instant::from_millis(3_020));
        over_pressure.since_heartbeat = 3_020;
        over_pressure.sensor_value[SensorSlot::Slot0] = 600;

        let outcome = ctl.decide(over_pressure);

        assert_eq!(outcome.targets[ValveId::Valve0], 1000, "relief must win over the fallback stage");
    }
}
