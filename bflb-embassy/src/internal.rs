//! Stand-ins for the parts of `embassy-hal-internal` that only support Cortex-M, with the
//! same API: the [`interrupt_mod!`] macro generating the `interrupt` module of the chip,
//! and the [`InterruptExt`] trait.
//!
//! The main difference is the binding of handlers: the runtime dispatches interrupts
//! through a table of handlers instead of linking them by symbol name, so a
//! [`Binding`](crate::interrupt::typelevel::Binding) also gives the function to register,
//! and drivers register it with `register` before enabling their interrupt.
#![macro_use]

use bflb_hal::interrupt::Interrupt;
use critical_section::CriticalSection;

/// Priority of an interrupt, which is its CLIC level: when several interrupts are
/// pending, the one with the highest level is taken first.
///
/// The runtime doesn't nest interrupts, a handler is never preempted.
pub type Priority = u8;

/// Generate the standard `mod interrupt` of the chip, with one type-level interrupt per
/// given interrupt. The interrupts owned by this crate (such as the GPIO one) must not be
/// given, so that they cannot be bound by the user.
macro_rules! interrupt_mod {
    ($($irqs:ident),* $(,)?) => {
        /// Interrupt definitions.
        pub mod interrupt {
            pub use $crate::internal::{InterruptExt, Priority};
            pub use bflb_hal::interrupt::{Interrupt, InterruptHandler};
            pub use bflb_hal::interrupt::{$($irqs),*};
            pub use critical_section::CriticalSection;

            /// Type-level interrupt infrastructure.
            ///
            /// This module contains one *type* per interrupt. This is used for checking at compile time that
            /// the interrupts are correctly bound to HAL drivers.
            ///
            /// As an end user, you shouldn't need to use this module directly. Use the [`crate::bind_interrupts!`] macro
            /// to bind interrupts, and the [`crate::interrupt`] module to manipulate interrupts directly
            /// (pending/unpending, enabling/disabling, setting the priority, etc...)
            pub mod typelevel {
                use super::InterruptExt;

                trait SealedInterrupt {}

                /// Type-level interrupt.
                ///
                /// This trait is implemented for all typelevel interrupt types in this module.
                #[allow(private_bounds)]
                pub trait Interrupt: SealedInterrupt {
                    /// Interrupt handle.
                    ///
                    /// This allows going from typelevel interrupts (one type per interrupt) to
                    /// non-typelevel interrupts (a single `Interrupt` type).
                    const IRQ: super::Interrupt;

                    /// Enable the interrupt.
                    ///
                    /// # Safety
                    ///
                    /// The handler of this interrupt must be registered and ready to run.
                    #[inline]
                    unsafe fn enable() {
                        unsafe { Self::IRQ.enable() }
                    }

                    /// Disable the interrupt.
                    #[inline]
                    fn disable() {
                        Self::IRQ.disable()
                    }

                    /// Check if interrupt is enabled.
                    #[inline]
                    fn is_enabled() -> bool {
                        Self::IRQ.is_enabled()
                    }

                    /// Check if interrupt is pending.
                    #[inline]
                    fn is_pending() -> bool {
                        Self::IRQ.is_pending()
                    }

                    /// Set interrupt pending.
                    #[inline]
                    fn pend() {
                        Self::IRQ.pend()
                    }

                    /// Unset interrupt pending.
                    #[inline]
                    fn unpend() {
                        Self::IRQ.unpend()
                    }

                    /// Get the priority of the interrupt.
                    #[inline]
                    fn get_priority() -> super::Priority {
                        Self::IRQ.get_priority()
                    }

                    /// Set the interrupt priority.
                    #[inline]
                    fn set_priority(prio: super::Priority) {
                        Self::IRQ.set_priority(prio)
                    }

                    /// Set the interrupt priority with an already-acquired critical section.
                    #[inline]
                    fn set_priority_with_cs(cs: super::CriticalSection, prio: super::Priority) {
                        Self::IRQ.set_priority_with_cs(cs, prio)
                    }

                    /// Register the handler bound by the given binding for this interrupt. This is
                    /// called by drivers when they are created, before enabling the interrupt.
                    #[inline]
                    fn register<H: Handler<Self>, B: Binding<Self, H>>(_binding: &B)
                    where
                        Self: Sized,
                    {
                        critical_section::with(|cs| {
                            // SAFETY: Interrupts bound with `bind_interrupts!` are not used by the
                            // lower-level HAL, which only owns the machine timer and DMA ones, nor
                            // by this crate internally (they are not declared in this module).
                            unsafe { bflb_hal::interrupt::set_handler(Self::IRQ, B::DISPATCH, cs) };
                        });
                    }
                }

                $(
                    #[allow(non_camel_case_types)]
                    #[doc=stringify!($irqs)]
                    #[doc=" typelevel interrupt."]
                    pub enum $irqs {}
                    impl SealedInterrupt for $irqs {}
                    impl Interrupt for $irqs {
                        const IRQ: super::Interrupt = super::$irqs;
                    }
                )*

                /// Interrupt handler trait.
                ///
                /// Drivers that need to handle interrupts implement this trait.
                /// The user must ensure `on_interrupt()` is called every time the interrupt fires.
                /// Drivers must use use [`Binding`] to assert at compile time that the user has done so.
                pub trait Handler<I: Interrupt> {
                    /// Interrupt handler function.
                    ///
                    /// Must be called every time the `I` interrupt fires, synchronously from
                    /// the interrupt handler context.
                    ///
                    /// # Safety
                    ///
                    /// This function must ONLY be called from the interrupt handler for `I`.
                    unsafe fn on_interrupt();
                }

                /// Compile-time assertion that an interrupt has been bound to a handler.
                ///
                /// For the vast majority of cases, you should use the `bind_interrupts!`
                /// macro instead of writing `unsafe impl`s of this trait.
                ///
                /// # Safety
                ///
                /// By implementing this trait, you are asserting that [`Self::DISPATCH`] calls
                /// `H::on_interrupt()`. Drivers register it as the handler of `I`.
                pub unsafe trait Binding<I: Interrupt, H: Handler<I>> {
                    /// The function to register in the interrupt table of the runtime.
                    const DISPATCH: super::InterruptHandler;
                }
            }
        }
    };
}

/// Represents an interrupt type that can be configured by embassy to handle
/// interrupts.
pub trait InterruptExt: Copy {
    /// Enable the interrupt.
    ///
    /// # Safety
    ///
    /// The handler of this interrupt must be registered and ready to run.
    unsafe fn enable(self);

    /// Disable the interrupt.
    fn disable(self);

    /// Check if interrupt is enabled.
    fn is_enabled(self) -> bool;

    /// Check if interrupt is pending.
    fn is_pending(self) -> bool;

    /// Set interrupt pending.
    fn pend(self);

    /// Unset interrupt pending.
    fn unpend(self);

    /// Get the priority of the interrupt.
    fn get_priority(self) -> Priority;

    /// Set the interrupt priority.
    fn set_priority(self, prio: Priority);

    /// Set the interrupt priority with an already-acquired critical section
    ///
    /// Equivalent to `set_priority`, except you pass a `CriticalSection` to prove
    /// you've already acquired a critical section. This prevents acquiring another
    /// one, which saves code size.
    fn set_priority_with_cs(self, cs: CriticalSection, prio: Priority);
}

impl InterruptExt for Interrupt {
    #[inline]
    unsafe fn enable(self) {
        // SAFETY: Guaranteed by the caller.
        unsafe { self.set_enabled(true) }
    }

    #[inline]
    fn disable(self) {
        // SAFETY: Disabling an interrupt cannot break anything.
        unsafe { self.set_enabled(false) }
    }

    #[inline]
    fn is_enabled(self) -> bool {
        self.enabled()
    }

    #[inline]
    fn is_pending(self) -> bool {
        self.pending()
    }

    #[inline]
    fn pend(self) {
        // SAFETY: Handlers must handle spurious interrupts.
        unsafe { self.set_pending(true) }
    }

    #[inline]
    fn unpend(self) {
        // SAFETY: A not handled interrupt is not unsafe.
        unsafe { self.set_pending(false) }
    }

    #[inline]
    fn get_priority(self) -> Priority {
        self.level()
    }

    #[inline]
    fn set_priority(self, prio: Priority) {
        // SAFETY: All levels are valid because the CLIC is configured with 8 level bits.
        unsafe { self.set_level(prio) }
    }

    #[inline]
    fn set_priority_with_cs(self, _cs: CriticalSection, prio: Priority) {
        self.set_priority(prio)
    }
}
