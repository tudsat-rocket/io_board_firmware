//! The valve position word: the 16-bit value all three valve state layers use.
//!
//! [`crate::od::VALVE_COMMANDED`], [`crate::od::VALVE_TARGET`] and [`crate::od::VALVE_MEASURED`]
//! — and the TPDO frames that mirror them — all carry the same layout:
//!
//! ```text
//!     bit 15    bits 14..0
//!     +------+--------------+
//!     | !pwr | promille     |   0..=1000
//!     +------+--------------+
//! ```
//!
//! Bit 15 set means the drive is *released*: for a servo with a power output, the signal is no
//! longer being maintained and the valve holds (or does not hold) mechanically. A solenoid has
//! nothing to release, so for one the flag simply reads as de-energised, i.e. closed.
//!
//! # Why the flag and the position are independent
//!
//! That independence is what makes `measured` useful while unpowered: it reports the last
//! position estimate *and* the fact that nothing is holding the valve there. Mask with
//! [`POSITION_MASK`] for the number; test [`is_unpowered`] to decide whether to trust it.
//!
//! # Why 0xFFFF is not a valid word
//!
//! The promille field is range-checked on every write, flag or no flag, so `0xFFFF` is rejected
//! with `ValueTooHigh` rather than silently clamped. That is deliberate: `0xFFFF` means "invalid"
//! or "not connected" elsewhere on this vehicle, and it must never be mistaken for a valve
//! command here.
//!
//! The configuration objects that hold positions (the fallback positions, the input clamp) take a
//! plain promille and reject the flag entirely — whether a fallback releases its valve is the
//! separate [`crate::od::FALLBACK_A_UNPOWER`] field.

/// Fully open. Positions on this bus are promille, so the range is 0..=1000 rather than a
/// fraction of some per-valve full scale.
pub const PROMILLE_MAX: u16 = 1000;

/// Bit 15 of a position word: this valve is not being driven.
///
/// Only meaningful for a servo with a separate power output; a solenoid has nothing to release,
/// so for one this simply reads as de-energised, i.e. closed.
pub const UNPOWERED_FLAG: u16 = 0x8000;

/// The promille field of a position word.
pub const POSITION_MASK: u16 = 0x7FFF;

/// Is this position word asking for (or reporting) a released drive?
pub const fn is_unpowered(word: u16) -> bool {
    word & UNPOWERED_FLAG != 0
}

/// The promille part of a position word, with the flag stripped.
pub const fn position_of(word: u16) -> u16 {
    word & POSITION_MASK
}

/// Build a position word that reports `position` but says the drive is released.
pub const fn unpowered_at(position: u16) -> u16 {
    (position & POSITION_MASK) | UNPOWERED_FLAG
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_flag_and_the_position_do_not_interfere() {
        let word = unpowered_at(750);
        assert!(is_unpowered(word));
        assert_eq!(position_of(word), 750, "the reading survives being released");
        assert!(!is_unpowered(750));
        assert_eq!(position_of(750), 750);
    }

    #[test]
    fn the_full_range_fits_beside_the_flag() {
        for position in [0, 1, PROMILLE_MAX] {
            assert_eq!(position_of(unpowered_at(position)), position);
        }
        // 0xFFFF is not reachable from any legal position, which is what keeps the vehicle's
        // "not connected" sentinel from ever looking like a command.
        assert_ne!(unpowered_at(PROMILLE_MAX), u16::MAX);
    }
}
