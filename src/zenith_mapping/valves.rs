use crate::valves::ServoValveCalib;

pub const PRESSURIZATION: ServoValveCalib = ServoValveCalib {
    open_us: 1500 - 550,
    closed_us: 1500 + 525,
};

pub const FILL_AND_DUMP: ServoValveCalib = ServoValveCalib {
    open_us: 850,
    closed_us: 1980,
};

pub const VENT: ServoValveCalib = ServoValveCalib {
    open_us: 950,
    closed_us: 2050,
};
