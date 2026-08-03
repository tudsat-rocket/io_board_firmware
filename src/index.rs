//! Typed indices for the board's fixed-size domains, and the arrays they index.

use core::marker::PhantomData;
use core::ops::{Index, IndexMut};

/// A fixed domain of indices, dense from 0.
///
/// Implemented by the id enums below via [`id_domain!`], plus one foreign type
/// ([`iocan_proto::TpdoKind`], whose discriminant is already a dense index into the TPDO table).
pub trait Id: Copy + 'static {
    const COUNT: usize;
    /// Every id in the domain, in index order. Iteration goes through this rather than through
    /// `0..COUNT` plus a conversion, so no fallible step is needed to walk a `Per`.
    const ALL: &'static [Self];

    fn index(self) -> usize;

    /// The boundary check. `None` for anything past the end of the domain.
    fn from_index(index: usize) -> Option<Self>;
}

/// Define an id domain: the enum, its inherent helpers, its [`Id`] impl, and the `Per` alias.
///
/// The variant count is written out explicitly and cross-checked by the type of `ALL`, so
/// forgetting a variant is a compile error rather than a silently short domain.
macro_rules! id_domain {
    (
        $(#[$attr:meta])*
        $name:ident, $per:ident, $count:literal, [$($variant:ident),+ $(,)?]
    ) => {
        $(#[$attr])*
        #[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, defmt::Format)]
        #[repr(u8)]
        pub enum $name {
            $($variant),+
        }

        impl $name {
            /// Every id of this domain, in index order.
            pub const ALL: [Self; $count] = [$(Self::$variant),+];
            pub const COUNT: usize = $count;

            /// Position in a [`Per`] over this domain.
            /// Needed because Id trait is not const
            pub const fn index(self) -> usize {
                self as usize
            }

            pub const fn from_index(index: usize) -> Option<Self> {
                if index < $count {
                    Some(Self::ALL[index])
                } else {
                    None
                }
            }

            /// Convert from a wire or flash byte. `None` is a rejected message or record.
            pub const fn from_u8(value: u8) -> Option<Self> {
                Self::from_index(value as usize)
            }

            pub const fn as_u8(self) -> u8 {
                self as u8
            }
        }

        impl Id for $name {
            const COUNT: usize = $count;
            const ALL: &'static [Self] = &$name::ALL;

            #[inline]
            fn index(self) -> usize {
                $name::index(self)
            }

            #[inline]
            fn from_index(index: usize) -> Option<Self> {
                $name::from_index(index)
            }
        }

        #[doc = concat!("An array with one entry per [`", stringify!($name), "`].")]
        pub type $per<T> = Per<$name, T, $count>;
    };
}

id_domain!(
    /// One of the four high current outputs, 0-indexed.
    ///
    /// The board silkscreen and the wire encoding are both 1-indexed — see
    /// [`crate::store`]'s `hco_to_wire` — but everything inside the firmware counts from zero,
    /// and this type is what keeps the two conventions from meeting anywhere else.
    HcoId, PerHco, 4, [Hco0, Hco1, Hco2, Hco3]
);

id_domain!(
    /// One of the four valve slots.
    ValveId, PerValve, 4, [Valve0, Valve1, Valve2, Valve3]
);

id_domain!(
    /// One of the eight configurable sensor slots (0x2004 / 0x3020..).
    SensorSlot, PerSensorSlot, 8, [Slot0, Slot1, Slot2, Slot3, Slot4, Slot5, Slot6, Slot7]
);

id_domain!(
    /// One of the two I2C buses. Two is a hardware fact: only COM1 and COM2 are I2C-capable.
    I2cBus, PerI2cBus, 2, [Bus0, Bus1]
);

id_domain!(
    /// An index into [`crate::config::AMPLIFIER_ADDRESSES`] — an address strap combination, never
    /// a raw I2C address. The index is what travels over CAN, so a nine-entry bitmap fits a `u16`.
    AmplifierId, PerAmplifier, 9, [Amp0, Amp1, Amp2, Amp3, Amp4, Amp5, Amp6, Amp7, Amp8]
);

id_domain!(
    /// One of the three on-board shunt/divider rails.
    ///
    /// The pairing is not arbitrary: one shunt covers HCO1+2 and one covers HCO3+4, which is why
    /// stall detection is only unambiguous for a valve that owns a whole [`HcoPair`].
    RailId, PerRail, 3, [Logic, Hco12, Hco34]
);

/// A pair of high current outputs sharing one current shunt.
///
/// The vehicle harness wires each servo across a whole pair — the lower output carries power, the
/// upper one the signal — precisely so that the shunt reading can be attributed to one valve.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum HcoPair {
    /// HCO1 and HCO2.
    A,
    /// HCO3 and HCO4.
    B,
}

impl HcoPair {
    pub const ALL: [Self; 2] = [Self::A, Self::B];

    /// The lower output of the pair, which the harness uses for servo power.
    pub const fn power(self) -> HcoId {
        match self {
            Self::A => HcoId::Hco0,
            Self::B => HcoId::Hco2,
        }
    }

    /// The upper output of the pair, which carries the servo signal.
    pub const fn signal(self) -> HcoId {
        match self {
            Self::A => HcoId::Hco1,
            Self::B => HcoId::Hco3,
        }
    }

    /// The shunt covering this pair.
    pub const fn rail(self) -> RailId {
        match self {
            Self::A => RailId::Hco12,
            Self::B => RailId::Hco34,
        }
    }
}

impl HcoId {
    /// Which shunt pair this output sits on.
    pub const fn pair(self) -> HcoPair {
        match self {
            Self::Hco0 | Self::Hco1 => HcoPair::A,
            Self::Hco2 | Self::Hco3 => HcoPair::B,
        }
    }

    /// The 1-indexed number silkscreened on the board, for log messages.
    pub const fn silkscreen(self) -> u8 {
        self as u8 + 1
    }
}

/// One probe-able amplifier position: a bus and an address strap.
///
/// Flattened as `bus * AmplifierId::COUNT + amplifier`, which is the layout of the raw ADC array
/// at 0x2000/0x2001. Carrying the two halves in the type rather than recovering them with `/` and
/// `%` is what stops an amplifier index being used where a slot index belongs — the two differ by
/// a factor of nine and both are plausible subscripts into `AMPLIFIER_ADDRESSES`.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub struct AdcSlot {
    bus: I2cBus,
    amplifier: AmplifierId,
}

impl AdcSlot {
    pub const COUNT: usize = I2cBus::COUNT * AmplifierId::COUNT;

    pub const ALL: [Self; Self::COUNT] = {
        let mut all = [Self {
            bus: I2cBus::Bus0,
            amplifier: AmplifierId::Amp0,
        }; Self::COUNT];
        let mut i = 0;
        while i < Self::COUNT {
            all[i] = Self {
                bus: I2cBus::ALL[i / AmplifierId::COUNT],
                amplifier: AmplifierId::ALL[i % AmplifierId::COUNT],
            };
            i += 1;
        }
        all
    };

    pub const fn new(bus: I2cBus, amplifier: AmplifierId) -> Self {
        Self { bus, amplifier }
    }

    pub const fn bus(self) -> I2cBus {
        self.bus
    }

    pub const fn amplifier(self) -> AmplifierId {
        self.amplifier
    }

    pub const fn index(self) -> usize {
        self.bus.index() * AmplifierId::COUNT + self.amplifier.index()
    }

    pub const fn from_index(index: usize) -> Option<Self> {
        if index < Self::COUNT {
            Some(Self::ALL[index])
        } else {
            None
        }
    }
}

impl Id for AdcSlot {
    const COUNT: usize = Self::COUNT;
    const ALL: &'static [Self] = &AdcSlot::ALL;

    #[inline]
    fn index(self) -> usize {
        AdcSlot::index(self)
    }

    #[inline]
    fn from_index(index: usize) -> Option<Self> {
        AdcSlot::from_index(index)
    }
}

/// An array with one entry per [`AdcSlot`]: both buses, all nine straps.
pub type PerAdcSlot<T> = Per<AdcSlot, T, { AdcSlot::COUNT }>;

// The TPDO table is already a dense enum in the wire-protocol crate, and its discriminant is
// defined to be the index into `tpdo_interval_ms` (0x3040). Giving it an `Id` impl here lets that
// object use `Per` like everything else, without `iocan-proto` having to know this module exists.
impl Id for iocan_proto::TpdoKind {
    const COUNT: usize = iocan_proto::ids::NUM_TPDO_KINDS;
    const ALL: &'static [Self] = &iocan_proto::TPDO_KINDS;

    #[inline]
    fn index(self) -> usize {
        self as usize
    }

    #[inline]
    fn from_index(index: usize) -> Option<Self> {
        if index < <Self as Id>::COUNT {
            Some(iocan_proto::TPDO_KINDS[index])
        } else {
            None
        }
    }
}

/// An array with one entry per TPDO kind, i.e. object 0x3040.
pub type PerTpdoKind<T> = Per<iocan_proto::TpdoKind, T, { iocan_proto::ids::NUM_TPDO_KINDS }>;

/// A fixed array holding one `T` per `I`, indexable only by `I`.
///
/// `N` is always `I::COUNT`; it is a separate parameter only because an associated const cannot
/// be an array length on stable. Use the `PerX<T>` aliases rather than naming `Per` directly and
/// the two can never disagree.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Per<I, T, const N: usize> {
    items: [T; N],
    /// `fn() -> I` rather than `I` so the domain marker never affects auto traits or drop.
    domain: PhantomData<fn() -> I>,
}

impl<I, T, const N: usize> Per<I, T, N> {
    pub const fn new(items: [T; N]) -> Self {
        Self {
            items,
            domain: PhantomData,
        }
    }

    /// The same value in every slot.
    pub const fn splat(value: T) -> Self
    where
        T: Copy,
    {
        Self::new([value; N])
    }

    /// The flat array, for the wire and flash boundaries that legitimately want one — TPDO
    /// payloads, SDO array reads, the persisted record. Everywhere else, index by id.
    pub const fn as_array(&self) -> &[T; N] {
        &self.items
    }

    pub fn into_array(self) -> [T; N] {
        self.items
    }

    pub const fn as_slice(&self) -> &[T] {
        &self.items
    }

    /// Write one slot in a `const fn`.
    ///
    /// The `Index`/`IndexMut` impls are the normal way in, but trait methods cannot be called
    /// from a `const fn`, and the factory-default configs are all built in const context. Callers
    /// pass `id.index()`, so this is still reached through an id — it just cannot say so in the
    /// signature.
    pub const fn with_at(mut self, index: usize, value: T) -> Self
    where
        T: Copy,
    {
        self.items[index] = value;
        self
    }
}

impl<I: Id, T, const N: usize> Per<I, T, N> {
    /// Build from a function of the id.
    pub fn from_fn(mut f: impl FnMut(I) -> T) -> Self {
        let mut ids = I::ALL.iter();
        // `I::ALL` has exactly `N` entries, so the fallback is unreachable; it exists only to
        // keep this panic-free, since a panicking path here would pull `core::fmt` into a crate
        // that is deliberately without it.
        Self::new(core::array::from_fn(|_| match ids.next() {
            Some(id) => f(*id),
            None => f(I::ALL[0]),
        }))
    }

    /// Ids paired with their values.
    pub fn iter(&self) -> impl Iterator<Item = (I, &T)> {
        I::ALL.iter().copied().zip(self.items.iter())
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (I, &mut T)> {
        I::ALL.iter().copied().zip(self.items.iter_mut())
    }

    pub fn values(&self) -> core::slice::Iter<'_, T> {
        self.items.iter()
    }

    pub fn values_mut(&mut self) -> core::slice::IterMut<'_, T> {
        self.items.iter_mut()
    }
}

impl<I: Id, T, const N: usize> Index<I> for Per<I, T, N> {
    type Output = T;

    #[inline]
    fn index(&self, id: I) -> &T {
        &self.items[id.index()]
    }
}

impl<I: Id, T, const N: usize> IndexMut<I> for Per<I, T, N> {
    #[inline]
    fn index_mut(&mut self, id: I) -> &mut T {
        &mut self.items[id.index()]
    }
}

impl<I, const N: usize> Per<I, bool, N> {
    /// True when any slot is set — the "is there anything to do?" question a flag set gets asked
    /// before it is walked.
    pub fn any(&self) -> bool {
        self.items.iter().any(|&flag| flag)
    }
}

impl<I, T: Copy + Default, const N: usize> Default for Per<I, T, N> {
    fn default() -> Self {
        Self::splat(T::default())
    }
}

impl<I, T: defmt::Format, const N: usize> defmt::Format for Per<I, T, N> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "{}", self.items.as_slice())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_are_dense_and_in_order() {
        for (i, id) in HcoId::ALL.iter().enumerate() {
            assert_eq!(id.index(), i);
            assert_eq!(HcoId::from_index(i), Some(*id));
        }
        assert_eq!(HcoId::from_index(HcoId::COUNT), None);
        assert_eq!(ValveId::from_u8(4), None);
        assert_eq!(SensorSlot::from_u8(8), None);
    }

    #[test]
    fn a_per_is_indexed_by_its_own_id() {
        let mut outputs: PerHco<u16> = PerHco::splat(0);
        outputs[HcoId::Hco2] = 7;
        assert_eq!(outputs.as_array(), &[0, 0, 7, 0]);
        assert_eq!(outputs[HcoId::Hco2], 7);
    }

    #[test]
    fn iteration_pairs_each_id_with_its_slot() {
        let per = PerValve::new([10u16, 20, 30, 40]);
        let collected: Vec<_> = per.iter().map(|(id, v)| (id, *v)).collect();
        assert_eq!(
            collected,
            vec![
                (ValveId::Valve0, 10),
                (ValveId::Valve1, 20),
                (ValveId::Valve2, 30),
                (ValveId::Valve3, 40)
            ]
        );
    }

    #[test]
    fn from_fn_sees_every_id_exactly_once() {
        let per = PerSensorSlot::from_fn(|slot| slot.index() as u8);
        assert_eq!(per.as_array(), &[0, 1, 2, 3, 4, 5, 6, 7]);
    }

    /// The flattening the raw ADC array at 0x2000/0x2001 is defined by: bus 0's nine straps, then
    /// bus 1's. Splitting a slot back into its halves has to be exact, since it picks the I2C
    /// address to talk to.
    #[test]
    fn adc_slots_flatten_bus_major_and_split_back() {
        assert_eq!(AdcSlot::new(I2cBus::Bus0, AmplifierId::Amp0).index(), 0);
        assert_eq!(AdcSlot::new(I2cBus::Bus1, AmplifierId::Amp0).index(), 9);
        assert_eq!(AdcSlot::new(I2cBus::Bus1, AmplifierId::Amp8).index(), 17);
        assert_eq!(AdcSlot::from_index(18), None);

        for (i, slot) in AdcSlot::ALL.iter().enumerate() {
            assert_eq!(slot.index(), i);
            assert_eq!(slot.bus().index(), i / AmplifierId::COUNT);
            assert_eq!(slot.amplifier().index(), i % AmplifierId::COUNT);
        }
    }

    /// The harness convention the stall attribution depends on: a servo's power and signal are
    /// the two outputs of one pair, and that pair is the one the shunt covers.
    #[test]
    fn an_hco_pair_owns_two_outputs_and_one_shunt() {
        assert_eq!(HcoPair::A.power(), HcoId::Hco0);
        assert_eq!(HcoPair::A.signal(), HcoId::Hco1);
        assert_eq!(HcoPair::B.power(), HcoId::Hco2);
        assert_eq!(HcoPair::B.signal(), HcoId::Hco3);

        for pair in HcoPair::ALL {
            assert_eq!(pair.power().pair(), pair);
            assert_eq!(pair.signal().pair(), pair);
            assert_eq!(pair.power().pair().rail(), pair.rail());
        }
        assert_eq!(HcoPair::A.rail(), RailId::Hco12);
        assert_eq!(HcoPair::B.rail(), RailId::Hco34);
    }

    #[test]
    fn tpdo_kinds_index_their_own_interval_table() {
        use iocan_proto::TpdoKind;
        let mut intervals: PerTpdoKind<u16> = PerTpdoKind::splat(0);
        intervals[TpdoKind::ValveCurrent] = 250;
        assert_eq!(intervals.as_array()[TpdoKind::ValveCurrent as usize], 250);
        assert_eq!(<TpdoKind as Id>::from_index(TpdoKind::COUNT), None);
    }
}
