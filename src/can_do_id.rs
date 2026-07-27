#[derive(Copy, Clone)]
pub struct DeciVolts(pub u8);

#[derive(Copy, Clone)]
pub struct DeciCelsius(pub i16);

#[derive(Copy, Clone)]
pub struct KiloPascals(pub u16);

#[derive(Copy, Clone)]
pub struct Promille(pub u16);

#[derive(Copy, Clone)]
pub struct SensorReading(pub i16);

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum PdMessageKind {
    // NOTE: maybe have a look at num_enum: TryFromPrimitive
    /// holds 4 ValveStates (promille open) as le u16
    Valves,
    /// holds 4 bools as le u16
    BinaryOutputs,
    /// holds 4 pwm microseconds entries as le u16
    PwmUs,
    /// holds fist 4 raw adc measurements of i2c bus 0
    RawBus0a,
    /// holds second 4 raw adc measurements of i2c bus 0
    RawBus0b,
    /// holds first 4 raw adc measurements of i2c bus 1
    RawBus1a,
    /// holds second 4 raw adc measurements of i2c bus 1
    RawBus1b,
    /// holds first 4 preprocessed sensor values as u16 or i16
    /// temp(i16): centi celcius, pressure(u16): kilo pascal
    Sensor0,
    /// holds second 4 preprocessed sensor values as u16 or i16
    /// temp(i16): centi celcius, pressure(u16): kilo pascal
    Sensor1,
    /// holds third 4 preprocessed sensor values as u16 or i16
    /// temp(i16): centi celcius, pressure(u16): kilo pascal
    Sensor2,
    // /// holds status information:
    // NodeStatus,
}
impl TryFrom<u8> for PdMessageKind {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, ()> {
        use PdMessageKind as K;
        match value {
            0 => Ok(K::Valves),
            1 => Ok(K::BinaryOutputs),
            2 => Ok(K::PwmUs),
            3 => Ok(K::RawBus0a),
            4 => Ok(K::RawBus0b),
            5 => Ok(K::RawBus1a),
            6 => Ok(K::RawBus1b),
            7 => Ok(K::Sensor0),
            8 => Ok(K::Sensor1),
            9 => Ok(K::Sensor2),
            _ => Err(()),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub struct ProcessDataCanId {
    pub node_id: u8,
    pub kind: PdMessageKind,
}
impl From<ProcessDataCanId> for u16 {
    fn from(value: ProcessDataCanId) -> Self {
        ((value.node_id as u16 & 0b1111) | (((value.kind as u16) << 4) & 0b1_1111_0000)) | 0x200
    }
}

impl TryFrom<u16> for ProcessDataCanId {
    type Error = ();
    fn try_from(value: u16) -> Result<Self, ()> {
        // 0d512 = 2^9
        if !(0x200..(0x200 + 512)).contains(&value) {
            return Err(());
        }
        let identifier: u16 = (value >> 4) & 0b1_1111;
        let kind = PdMessageKind::try_from(identifier as u8);
        let Ok(kind) = kind else {
            return Err(());
        };

        Ok(Self {
            node_id: (value & 0b1111) as u8,
            kind,
        })
    }
}
