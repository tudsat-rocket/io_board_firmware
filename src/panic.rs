//! Flight panic handling: de-energize the high current outputs, then reset.
//!
//! Replaces `panic-probe`, which is a bench tool: it ends in `udf` -> HardFault ->
//! `cortex-m-rt`'s infinite loop, so the CPU stops while TIM1/TIM3 keep driving the
//! outputs from hardware. A panic then means a valve latched in its last position
//! until someone cuts power.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_stm32::pac;

/// Drive all high current outputs low, using raw PAC access only.
///
/// Called from panic/fault context: no `STORE` lock, no `HcoControl` borrow (either may
/// be mid-mutation), no allocation, no `.await`.
///
/// # Safety
/// Takes over the HCO timers and pins unconditionally. Only call when the normal
/// control path is already dead and a reset is about to happen.
pub unsafe fn safe_outputs() {
    // HCO 3 and 4 are TIM3 CH3/CH4 on both revisions.
    pac::TIM3.ccr(2).write(|w| w.set_ccr(0));
    pac::TIM3.ccr(3).write(|w| w.set_ccr(0));

    #[cfg(feature = "rev3")]
    {
        // HCO1 = TIM3 CH2, HCO2 = TIM1 CH1.
        pac::TIM3.ccr(1).write(|w| w.set_ccr(0));
        pac::TIM1.ccr(0).write(|w| w.set_ccr(0));
        // Advanced timer: cut the outputs at the source as well.
        pac::TIM1.bdtr().modify(|w| w.set_moe(false));
        pac::TIM1.egr().write(|w| w.set_ug(true));
    }

    #[cfg(feature = "rev2")]
    {
        // HCO1 = PC0, HCO2 = PC15, driven directly by the TIM2 software PWM ISR.
        // Interrupts are already off, so nothing can set them high again.
        pac::GPIOC.bsrr().write(|w| {
            w.set_br(0, true);
            w.set_br(15, true);
        });
    }

    // CCR has preload enabled, so force the shadow registers to take the new value now
    // instead of at the next update event (up to 20ms at 50Hz).
    pac::TIM3.egr().write(|w| w.set_ug(true));
}

/// De-energize outputs and reset. Never returns.
fn safe_and_reset() -> ! {
    cortex_m::interrupt::disable();
    unsafe { safe_outputs() };
    cortex_m::peripheral::SCB::sys_reset()
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    static PANICKED: AtomicBool = AtomicBool::new(false);

    cortex_m::interrupt::disable();
    unsafe { safe_outputs() };

    // Guard against recursing if the panic came out of defmt itself.
    if !PANICKED.swap(true, Ordering::Relaxed) {
        // Safe to log: defmt-rtt is built with `disable-blocking-mode`, so this cannot
        // hang waiting for a host that is no longer attached.
        defmt::error!("PANIC: {}", defmt::Display2Format(info));
    }

    cortex_m::peripheral::SCB::sys_reset()
}

/// Used by `defmt::panic!` / `defmt::unwrap!` so they don't print twice.
#[defmt::panic_handler]
fn defmt_panic() -> ! {
    safe_and_reset()
}

#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    safe_and_reset()
}
