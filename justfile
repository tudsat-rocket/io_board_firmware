cancan_cli := env_var("HOME") / "rapid/cancan/cancan-cli"
cancan_target := justfile_directory() / "target/cancan-cli"
cancan := cancan_target / "release/cancan"

features := "--no-default-features --features rev3,hardware"
features_rev2 := "--no-default-features --features rev2,hardware"

# Vehicle boards as bin:node_id, split by hardware revision. The rev2 bins carry
# `required-features = ["rev2"]` in Cargo.toml, so the rev3 build skips them.
boards := "node3:3 node4:4 node5:5 node6:6"
boards_rev2 := "node7:7 node8:8 node9:9"
rev2_bins := "--bin node7 --bin node8 --bin node9"

target_dir := justfile_directory() / "target/thumbv7m-none-eabi/release"

# CAN interface. Unset lets the CLI pick, which works when the host has exactly one.
iface := env_var_or_default("CAN_IFACE", "")
iface_arg := if iface == "" { "" } else { "--iface " + iface }

default:
    @just --list

# Build every node binary (rev3 boards, then the rev2 ones).
build:
    cargo build --release {{ features }}
    cargo build --release {{ features_rev2 }} {{ rev2_bins }}

# Build one node binary (node2..node9, generic), picking its board revision.
build-one board:
    cargo build --release {{ if board =~ '^node[789]$' { features_rev2 } else { features } }} --bin {{ board }}

# Build and flash every vehicle board over CAN. Keeps going if one board is silent.
flash: build _cancan
    #!/usr/bin/env bash
    set -uo pipefail
    failed=()
    for entry in {{ boards }} {{ boards_rev2 }}; do
        bin="${entry%%:*}"; id="${entry##*:}"
        echo
        echo "==> ${bin} (node ${id})"
        if ! {{ cancan }} {{ iface_arg }} flash "${id}" "{{ target_dir }}/${bin}"; then
            failed+=("${bin}")
        fi
    done
    echo
    if [ ${#failed[@]} -eq 0 ]; then
        echo "all boards flashed: {{ boards }} {{ boards_rev2 }}"
    else
        echo "FAILED: ${failed[*]}" >&2
        exit 1
    fi

# Build and flash one board over CAN, by binary name (node3..node9, generic).
flash-one board: _cancan
    #!/usr/bin/env bash
    set -euo pipefail
    id=""
    for entry in {{ boards }} {{ boards_rev2 }} generic:6; do
        [ "${entry%%:*}" = "{{ board }}" ] && id="${entry##*:}"
    done
    if [ -z "${id}" ]; then
        echo "unknown board '{{ board }}' — known: {{ boards }} {{ boards_rev2 }} generic:6" >&2
        exit 1
    fi
    just build-one {{ board }}
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
    cargo run --release {{ if board =~ '^node[789]$' { features_rev2 } else { features } }} --bin {{ board }}

# Host-side unit tests (the pure logic, `hardware` off).
test:
    cargo test --no-default-features --features host-test --target x86_64-unknown-linux-gnu --lib

# Build the cancan CLI if it is missing or out of date.
_cancan:
    @cd {{ cancan_cli }} && cargo build --release --quiet --target-dir {{ cancan_target }}
