#![no_std]
#![allow(async_fn_in_trait)]
#![deny(unsafe_op_in_unsafe_fn)]
#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

//! ## Feature flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]

#[cfg(not(any(feature = "bl808-m0")))]
compile_error!(
    "No chip feature activated. You must activate exactly one of the following features:
    bl808-m0,
    "
);

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;
pub(crate) mod internal;

#[cfg(feature = "time-driver")]
mod time_driver;

pub mod gpio;
pub mod uart;

// Everything specific to the selected chip: register blocks, peripheral singletons,
// interrupts and driver instances. It MUST go after the drivers, it uses their macros.
#[cfg_attr(feature = "bl808-m0", path = "chips/bl808_m0.rs")]
mod chip;

/// Macro to bind interrupts to handlers.
///
/// This defines the right interrupt handlers, and creates a unit struct (like
/// `struct Irqs;`) and implements the right [`Binding`](crate::interrupt::typelevel::Binding)s
/// for it. You can pass this struct to drivers to prove at compile-time that the right
/// interrupts have been bound. The handlers are registered in the interrupt table when
/// the driver is created.
///
/// Example of how to bind one interrupt:
///
/// ```rust,ignore
/// use bflb_embassy::{bind_interrupts, uart, peripherals};
///
/// bind_interrupts!(
///     /// Binds the UART0 interrupt.
///     struct Irqs {
///         UART0 => uart::InterruptHandler<peripherals::UART0>;
///     }
/// );
/// ```
///
/// Example of how to bind multiple interrupts in a single macro invocation:
///
/// ```rust,ignore
/// use bflb_embassy::{bind_interrupts, uart, peripherals};
///
/// bind_interrupts!(struct Irqs {
///     UART0 => uart::InterruptHandler<peripherals::UART0>;
///     UART1 => uart::InterruptHandler<peripherals::UART1>;
/// });
/// ```
#[macro_export]
macro_rules! bind_interrupts {
    ($(#[$attr:meta])* $vis:vis struct $name:ident {
        $(
            $(#[cfg($cond_irq:meta)])?
            $irq:ident => $(
                $(#[cfg($cond_handler:meta)])?
                $handler:ty
            ),*;
        )*
    }) => {
        #[derive(Copy, Clone)]
        $(#[$attr])*
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            $(#[cfg($cond_irq)])?
            const _: () = {
                fn $irq(_code: usize, _cs: $crate::interrupt::CriticalSection) {
                    $(
                        $(#[cfg($cond_handler)])?
                        // SAFETY: Called from the interrupt bound to this handler.
                        unsafe {
                            <$handler as $crate::interrupt::typelevel::Handler<
                                $crate::interrupt::typelevel::$irq
                            >>::on_interrupt();
                        }
                    )*
                }

                $(
                    $(#[cfg($cond_handler)])?
                    unsafe impl $crate::interrupt::typelevel::Binding<
                        $crate::interrupt::typelevel::$irq, $handler
                    > for $name {
                        const DISPATCH: $crate::interrupt::InterruptHandler = $irq;
                    }
                )*
            };
        )*
    };
}

// Reexports

/// The lower-level HAL this crate is built on.
pub use bflb_hal as hal;
#[cfg(feature = "unstable-pac")]
pub use chip::pac;
#[cfg(not(feature = "unstable-pac"))]
pub(crate) use chip::pac;
pub use chip::{Peripherals, peripherals};
pub use embassy_hal_internal::{Peri, PeripheralType};

pub use crate::chip::interrupt;

pub mod config {
    //! Configuration options used when initializing the HAL.

    /// Configuration used to initialize the HAL.
    #[derive(Debug, Clone, Default)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[non_exhaustive]
    pub struct Config {}
}

/// Initialize the `bflb-embassy` HAL with the provided configuration.
///
/// This returns the peripheral singletons that can be used for creating drivers.
///
/// The runtime (such as `bflb-rt`) must have initialized [`bflb_hal`] before, which
/// `bflb-rt` does before calling `main`. This takes ownership of the
/// [`bflb_hal::Peripherals`], so that they cannot be used concurrently through the
/// lower-level HAL.
///
/// This should only be called once at startup, otherwise it panics. It also panics if
/// the [`bflb_hal::Peripherals`] were already taken.
pub fn init(config: config::Config) -> Peripherals {
    let _ = config;

    // Do this first, so that it panics if user is calling `init` a second time
    // before doing anything important.
    let peripherals = Peripherals::take();

    // Claim the lower-level HAL peripherals forever, they are superseded by ours.
    let _ = unwrap!(hal::Peripherals::try_take(), "bflb_hal peripherals were already taken");

    critical_section::with(|cs| {
        #[cfg(feature = "time-driver")]
        time_driver::init(cs);

        gpio::init(cs);
    });

    peripherals
}
