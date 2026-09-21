# IO board firmware

Firmware for the STM32F105 IO boards on the CAN bus of the Zenith vehicle. Every board runs the
same firmware. Each node binary (`src/bin/nodeN.rs`) only chooses the factory defaults the board
starts with when its NOR flash holds no saved configuration. Those defaults live in
`src/zenith_mapping/`. A board can be reconfigured in the field over SDO and saved, so what a
board actually runs can differ from this list. The object dictionary is documented in
`iocan-proto/src/od.rs`.

## Naming used below

- **HC1–HC4**: the high current outputs, as printed on the silkscreen (`HcoId::Hco0`–`Hco3` in
  code). A servo takes a pair: pair A is HC1 (power) and HC2 (signal); pair B is HC3 (power) and
  HC4 (signal).
- **COM1 / COM2**: the two I2C buses for the sensor amplifiers (`Bus0` / `Bus1`). The
  amplifier number is its address strap (`Amp0`–`Amp8`).
- **Stepper port**: PA2 is the step clock and PA3 the direction. That is COM4 on rev3 and COM3 on
  rev2. See `docs/stepper.md`.

## Nodes

| Node | Binary | Board | Role |
|---|---|---|---|
| 2 | `node2` | rev3 | Nosecone / recovery |
| 3 | `node3` | rev3 | Payload avionics (nothing wired) |
| 4 | `node4` | rev3 | Upper propulsion: oxidizer vent |
| 5 | `node5` | rev3 | Upper propulsion: pressurization, pressurant vent |
| 6 | `node6` | rev3 | Lower propulsion: main valve, oxidizer fill/dump |
| 7 | `node7` | rev3 | Lower propulsion: igniter control (nothing wired) |
| 8 | `node8reg` | rev3 | Self-regulating relief |
| 9 | `node9` | rev3 | Stepper node |
| 10 | `node10` | rev3 | Dual stepper node (`dual-stepper` feature) |
| 15 | `node15` | **rev2** | N2 stepper, external N2 servo |
| 16 | `node16` | **rev2** | OX stepper, external OX fill |
| 6 | `generic` | any | Bench board, not installed in the vehicle |

### Node 2: nosecone / recovery

| Sensor slot | Bus | Amp | Sensor |
|---|---|---|---|
| 0 | COM1 | 0 | PT1000 temperature |

No valves.

### Node 3: payload avionics

Nothing configured.

### Node 4: upper propulsion

| Valve | Type | Outputs | Function |
|---|---|---|---|
| 0 | Solenoid | HC1 | Oxidizer vent |

No sensors.

### Node 5: upper propulsion

| Valve | Type | Outputs | Function |
|---|---|---|---|
| 0 | Servo | HC1 power, HC2 signal | Pressurization |
| 1 | Servo | HC3 power, HC4 signal | Pressurant vent |

| Sensor slot | Bus | Amp | Sensor |
|---|---|---|---|
| 0 | COM1 | 0 | PT1000, regulator temperature |
| 1 | COM1 | 1 | Regulator 2 pressure (100 bar) |
| 2 | COM1 | 2 | Regulator 1 pressure (100 bar) |
| 3 | COM2 | 0 | Upper oxidizer tank pressure (100 bar) |
| 4 | COM2 | 1 | Pressurant (N2) tank pressure (400 bar) |

### Node 6: lower propulsion, valve control

| Valve | Type | Outputs | Function |
|---|---|---|---|
| 0 | Servo | HC1 power, HC2 signal | Main valve |
| 1 | Servo | HC3 power, HC4 signal | Oxidizer fill and dump |

| Sensor slot | Bus | Amp | Sensor |
|---|---|---|---|
| 0 | COM1 | 0 | Lower oxidizer tank pressure (100 bar) |
| 1 | COM2 | 0 | Combustion chamber 1 pressure (40 bar) |
| 2 | COM1 | 2 | PT1000 temperature |
| 3 | COM2 | 1 | Combustion chamber 2 pressure (40 bar) |

### Node 7: lower propulsion, igniter control

Nothing configured.

### Node 8: self-regulating relief

| Valve | Type | Outputs | Function |
|---|---|---|---|
| 0 | Solenoid | HC1 | Relief valve |

| Sensor slot | Bus | Amp | Sensor |
|---|---|---|---|
| 0 | COM1 | 0 | Upper oxidizer tank pressure (100 bar) |

Opens valve 0 in 500 ms pulses, with a 500 ms cooldown between them, whenever slot 0 reads above
60 bar. This works even without a master on the bus. See `src/relief.rs`.

### Node 9: stepper

| Valve | Type | Outputs | Function |
|---|---|---|---|
| 0 | Stepper | COM4 (PA2 step, PA3 dir) | Throttle; ENABLE strapped to 5 V |

| Sensor slot | Bus | Amp | Sensor |
|---|---|---|---|
| 0 | COM1 | 0 | Combustion chamber 1 pressure (40 bar) |

### Node 10: dual stepper

Build with the `dual-stepper` feature (`just build-dual`).

| Valve | Type | Outputs | Function |
|---|---|---|---|
| 0 | Stepper 0 | PA2 step, PC10 dir | Throttle; ENABLE strapped to 5 V |
| 1 | Stepper 1 | PA3 step, PA5 dir | Throttle; ENABLE strapped to 5 V |

| Sensor slot | Bus | Amp | Sensor |
|---|---|---|---|
| 0 | COM1 | 0 | Combustion chamber 1 pressure (40 bar) |
| 1 | COM2 | 0 | Combustion chamber 2 pressure (40 bar) |

All four HC outputs are free.

### Node 15: N2 stepper (rev2)

| Valve | Type | Outputs | Function |
|---|---|---|---|
| 0 | Stepper | COM3 (PA2 step, PA3 dir) | N2 stepper |
| 1 | Solenoid | HC2 | Stepper ENABLE (1000 = enabled) |
| 2 | Servo | HC3 power, HC4 signal | External N2 |

No sensors. HC1 is unused. The master has to enable valve 1 before valve 0 will move. The enable
stays on in both fallback stages.

### Node 16: OX stepper (rev2)

| Valve | Type | Outputs | Function |
|---|---|---|---|
| 0 | Stepper | COM3 (PA2 step, PA3 dir) | OX stepper |
| 1 | Solenoid | HC2 | Stepper ENABLE (1000 = enabled) |
| 2 | Solenoid | HC1 | External OX fill (closes in both fallback stages) |

No sensors. HC3 and HC4 are unused.

### Generic: bench board

Starts as node 6. Don't put it on the vehicle bus next to the real node 6.

| Valve | Type | Outputs |
|---|---|---|
| 0 | Servo (placeholder) | HC1 power, HC2 signal |
| 1 | Servo (placeholder) | HC3 power, HC4 signal |

| Sensor slot | Bus | Amp | Sensor |
|---|---|---|---|
| 0 | COM1 | 0 | Uncalibrated, raw counts |
| 1 | COM1 | 1 | Uncalibrated, raw counts |
| 2 | COM1 | 2 | Uncalibrated, raw counts |
| 3 | COM2 | 0 | Uncalibrated, raw counts |

## Building and flashing

Run `just` to list all recipes. The ones you'll use most:

```sh
just build                 # every node binary, rev3
just flash-one node5       # build and flash one board over CAN
just flash-one node15      # rev2 nodes get the rev2 build automatically
just rev=rev2 build-one node9
just scan                  # list the boards answering on the bus
just test                  # host-side unit tests
```

A board without the cancan bootloader has to be flashed once over SWD first (`just bootloader`).
After that, every flash goes over CAN.

Calibrations, servo endpoints and stepper travel are placeholders in several places, marked
"unmeasured" or "uncounted" in `src/zenith_mapping/`. Measure them on the bench before relying
on them.
