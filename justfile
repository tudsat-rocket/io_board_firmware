cancan_cli := env_var("HOME") / "rapid/cancan/cancan-cli"
cancan_target := justfile_directory() / "target/cancan-cli"
cancan := cancan_target / "release/cancan"

rev := "rev3"
features := "--no-default-features --features " + rev + ",hardware"

boards := "node4:4 node5:5 node6:6 node7:7"

target_dir := justfile_directory() / "target/thumbv7m-none-eabi/release"

# CAN interface. Unset lets the CLI pick, which works when the host has exactly one.
iface := env_var_or_default("CAN_IFACE", "")
iface_arg := if iface == "" { "" } else { "--iface " + iface }

default:
    @just --list

# Build every node binary.
build:
    cargo build --release {{ features }}

# Build one node binary (node2..node7, node8reg, generic).
build-one board:
    cargo build --release {{ features }} --bin {{ board }}

# Build and flash every vehicle board over CAN. Keeps going if one board is silent.
flash: build _cancan
    #!/usr/bin/env bash
    set -uo pipefail
    failed=()
    for entry in {{ boards }}; do
        bin="${entry%%:*}"; id="${entry##*:}"
        echo
        echo "==> ${bin} (node ${id})"
        if ! {{ cancan }} {{ iface_arg }} flash "${id}" "{{ target_dir }}/${bin}"; then
            failed+=("${bin}")
        fi
    done
    echo
    if [ ${#failed[@]} -eq 0 ]; then
        echo "all boards flashed: {{ boards }}"
    else
        echo "FAILED: ${failed[*]}" >&2
        exit 1
    fi

# Build and flash one board over CAN, by binary name (node2..node7, node8reg, generic).
flash-one board: _cancan
    #!/usr/bin/env bash
    set -euo pipefail
    id=""
    for entry in {{ boards }} generic:6; do
        [ "${entry%%:*}" = "{{ board }}" ] && id="${entry##*:}"
    done
    if [ -z "${id}" ]; then
        echo "unknown board '{{ board }}' — known: {{ boards }} generic:6" >&2
        exit 1
    fi
    cargo build --release {{ features }} --bin {{ board }}
    {{ cancan }} {{ iface_arg }} flash "${id}" "{{ target_dir }}/{{ board }}"

# List the boards answering on the bus (probes all 256 cancan node ids).
scan: _cancan
    {{ cancan }} {{ iface_arg }} scan

# Query one board: firmware name, chip, build id/timestamp, boot state, uptime.
info node: _cancan
    {{ cancan }} {{ iface_arg }} info {{ node }}

# First-time provisioning, both via SWD — a board with no bootloader cannot be flashed over CAN.
# Flash the bootloader once, then one image, and everything after that goes over the bus.

# Flash the cancan bootloader with probe-rs (needs a debugger on the board).
bootloader:
    cd bootloader && cargo run --release

# Flash one node binary with probe-rs instead of over CAN (needs a debugger, gives RTT logs).
probe-flash board:
    cargo run --release {{ features }} --bin {{ board }}

# Host-side unit tests (the pure logic, `hardware` off).
test:
    cargo test --no-default-features --features host-test --target x86_64-unknown-linux-gnu --lib

# Build the cancan CLI if it is missing or out of date.
_cancan:
    @cd {{ cancan_cli }} && cargo build --release --quiet --target-dir {{ cancan_target }}
