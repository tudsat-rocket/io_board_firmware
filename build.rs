use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    if let Err(e) = zencan_build::build_node_from_device_config("IO_BOARD", "device-conf/can-io.toml") {
        eprintln!("Failed to parse toml file: {}", e);
        std::process::exit(-1);
    }

    // We ship our own memory.x (the cancan A/B partition map) instead of the whole-flash one the
    // embassy-stm32 `memory-x` feature would generate.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x")).unwrap().write_all(include_bytes!("memory.x")).unwrap();
    // The partition map shared with the bootloader crate, INCLUDEd by memory.x.
    File::create(out.join("partitions.x")).unwrap().write_all(include_bytes!("partitions.x")).unwrap();

    // Stamp this build's firmware metadata (build id + timestamp) so the cancan CLI can tell one
    // build from another and flag a reverted update. Set CANCAN_BUILD_ID / SOURCE_DATE_EPOCH to
    // pin it for a reproducible build.
    cancan_build::emit(out);

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=partitions.x");
    println!("cargo:rerun-if-changed=device-conf/can-io.toml");

    // The rerun-if-changed lines above disable cargo's rerun-on-any-change default, which would
    // starve the build stamp: re-run on source changes too.
    println!("cargo:rerun-if-changed=src");
}
