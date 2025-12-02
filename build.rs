fn main() {
    if let Err(e) = zencan_build::build_node_from_device_config("IO_BOARD", "device-conf/can-io.toml") {
        eprintln!("Failed to parse toml file: {}", e);
        std::process::exit(-1);
    }
}
