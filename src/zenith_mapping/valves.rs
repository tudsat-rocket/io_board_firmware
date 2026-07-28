use crate::valves::ServoValveCalib;

// pub const PRESSURIZATION_old: ServoValveCalib = ServoValveCalib {
//     open_us: 1500 - 550,
//     closed_us: 1500 + 525,
// };

pub const OX_FILL_AND_DUMP: ServoValveCalib = ServoValveCalib {
    open_us: 850,
    closed_us: 1980,
};

pub const PRESSURANT_VENT: ServoValveCalib = ServoValveCalib {
    open_us: 950,
    closed_us: 2050,
};

pub const MAIN: ServoValveCalib = ServoValveCalib {
    closed_us: 2470,
    open_us: 500,
};

pub const PRESSURIZATION: ServoValveCalib = ServoValveCalib {
    open_us: 1080,
    closed_us: 2200,
};

pub const PLACEHOLDER_S: ServoValveCalib = ServoValveCalib {
    open_us: 1000,
    closed_us: 2000,
};
