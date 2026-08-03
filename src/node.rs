//! Bringing one node up.
//!
//! Each `src/bin/nodeN.rs` is a three-line shell that hands [`spawn_node`] the compile-time
//! factory defaults for its position in the vehicle. Everything else — which valve, which sensor,
//! which calibration — is [`Config`], and a board that has been configured over the bus and told
//! to save ignores the compile-time constants entirely.

use embassy_executor::Spawner;
use embassy_stm32::flash::Flash;
use embassy_sync::pubsub::PubSubChannel;

use cancan::{CanCan, CanCanConfig};
use static_cell::StaticCell;

use crate::{
    board::{Board, ConfigStore, init_board},
    can::{
        CAN_IN, CAN_OUT,
        sdo::{SdoServer, run_heartbeat, run_sdo_server},
        tpdo::{Tpdo, run_tpdo},
    },
    config::Config,
    control::{BoardControl, run_control},
    outputs::Outputs,
    sensors::{BoardSensors, ext_adc::Buses, run_sensors},
    store::{CONTROL_WAKE, PERSIST_WAKE, STORE},
};

use defmt_rtt as _;

#[cfg(feature = "rev2")]
use crate::board::HcoControllerRev2;
#[cfg(feature = "rev3")]
use crate::board::HcoControllerRev3;

#[cfg(feature = "rev2")]
static HCO_CONTROLLER: StaticCell<HcoControllerRev2> = StaticCell::new();
#[cfg(feature = "rev3")]
static HCO_CONTROLLER: StaticCell<HcoControllerRev3> = StaticCell::new();

static CONTROL: StaticCell<BoardControl> = StaticCell::new();
static SENSORS: StaticCell<BoardSensors> = StaticCell::new();
/// The compile-time defaults, kept so a restore (0x1011) has something to revert to without a
/// reboot.
static FACTORY_DEFAULTS: StaticCell<Config> = StaticCell::new();

#[cfg(feature = "rev2")]
pub const NODE_NAME: &str = "I/O [rev2]";
#[cfg(feature = "rev3")]
pub const NODE_NAME: &str = "I/O [rev3]";

/// What distinguishes one physical node from another at build time.
pub use crate::config::NodeSettings;

pub async fn spawn_node(spawner: Spawner, settings: NodeSettings) {
    let board: Board = init_board(spawner).await;

    let cancan_config = CanCanConfig {
        node_id: settings.node_id,
        name: NODE_NAME,
        // The one `unstable-pac` read in the application: cancan reports the chip identity so the
        // host tool can tell one board from another.
        chip_id: embassy_stm32::pac::DBGMCU.idcode().read().0,
        chip_uid: embassy_stm32::uid::uid(),
        flash_kib: (embassy_stm32::flash::FLASH_SIZE / 1024) as u16,
        build_id: crate::CANCAN_BUILD_ID,
        build_timestamp: crate::CANCAN_BUILD_TIMESTAMP,
        ..Default::default()
    };
    let mut cancan = CanCan::new(cancan_config, Flash::new_blocking(board.flash_peri));

    let can_in = CAN_IN.init(PubSubChannel::new());
    let can_out = CAN_OUT.init(PubSubChannel::new());

    // --- configuration: a stored config wins over the compile-time defaults ---
    let defaults = FACTORY_DEFAULTS.init(settings.config);
    if let Err(e) = defaults.sanity_check() {
        // A broken compile-time mapping is a build mistake, and running a valve board on one is
        // worse than not booting at all.
        defmt::panic!("factory default config is invalid: {}", e);
    }

    let mut config_store = board.config_store;
    let config = match config_store {
        Some(ref mut store) => store.load().await.unwrap_or_else(|e| {
            defmt::info!("no stored configuration ({}), using compile-time defaults", e);
            defaults.clone()
        }),
        None => defaults.clone(),
    };
    // Legal but probably-unintended settings, complained about once rather than rejected.
    config.log_warnings();
    {
        let mut store = STORE.lock().await;
        store.config = config;
        store.refresh_derived();
    }

    crate::can::spawn(board.can1, &mut cancan, spawner, can_in.publisher().unwrap(), can_out.subscriber().unwrap())
        .await;

    // --- the control task owns every output ---------------------------------
    let hco_controller = HCO_CONTROLLER.init(board.hco_controller);
    let outputs = Outputs::new(hco_controller);

    #[cfg(feature = "rev3")]
    let rails = board.onboard_sens;
    #[cfg(feature = "rev2")]
    let rails = crate::rail_sense::NoRails;

    let control = CONTROL.init(BoardControl::new(outputs, rails, board.leds));
    spawner.spawn(run_control(control).unwrap());

    // --- sensors ------------------------------------------------------------
    let sensors = SENSORS.init(BoardSensors::new(Buses {
        bus0: Some(board.com1_i2c),
        bus1: Some(board.com2_i2c),
    }));
    spawner.spawn(run_sensors(sensors).unwrap());

    // --- the bus ------------------------------------------------------------
    spawner.spawn(
        run_sdo_server(SdoServer::new(settings.node_id, can_in.subscriber().unwrap(), can_out.publisher().unwrap()))
            .unwrap(),
    );
    spawner.spawn(run_tpdo(Tpdo::new(settings.node_id, can_out.publisher().unwrap())).unwrap());
    spawner.spawn(run_heartbeat(settings.node_id, can_out.publisher().unwrap()).unwrap());

    if let Some(store) = config_store {
        spawner.spawn(run_persistence(store, defaults).unwrap());
    }

    spawner.spawn(run_cancan(cancan).unwrap());
}

/// Handle save (0x1010) and restore (0x1011) requests.
///
/// Deliberately a separate task from [`crate::control`]: a sector erase can take the better part
/// of a second, and a valve board should not stop updating its outputs because somebody committed
/// a calibration. The flash driver awaits between status polls, so the executor keeps running and
/// the watchdog keeps being petted throughout.
#[embassy_executor::task]
async fn run_persistence(mut store: ConfigStore, defaults: &'static Config) -> ! {
    loop {
        PERSIST_WAKE.wait().await;

        let (save, restore, config) = {
            let mut guard = STORE.lock().await;
            let save = core::mem::replace(&mut guard.pending.save, false);
            let restore = core::mem::replace(&mut guard.pending.restore, false);
            (save, restore, guard.config.clone())
        };

        if save {
            match store.save(&config).await {
                Ok(()) => defmt::info!("configuration committed to flash"),
                Err(e) => defmt::error!("could not commit configuration: {}", e),
            }
        }

        if restore {
            match store.erase_all().await {
                Ok(()) => {
                    {
                        let mut guard = STORE.lock().await;
                        guard.config = defaults.clone();
                        guard.refresh_derived();
                        guard.pending.config = true;
                    }
                    defmt::warn!("configuration reset to compile-time defaults");
                    CONTROL_WAKE.signal(());
                }
                Err(e) => defmt::error!("could not erase stored configuration: {}", e),
            }
        }
    }
}

/// Runs cancan, the firmware updater/bootloader task.
#[embassy_executor::task]
pub async fn run_cancan(cancan: CanCan<Flash<'static, embassy_stm32::flash::Blocking>>) {
    cancan.run(&crate::CANCAN).await
}
