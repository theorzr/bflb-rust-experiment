//! General Purpose Input/Output (GPIO) driver.
//!
//! Pins are configured through the GLB registers, whose layout depends on the chip family
//! and is handled by a register backend (`glb_v2` for BL808 and BL616/BL618). Async
//! waiting on pin levels and edges uses the GPIO interrupt, which is handled internally by
//! this module (no binding is required).
#![macro_use]

use core::cell::Cell;
use core::convert::Infallible;
use core::future::Future;
use core::pin::Pin as FuturePin;
use core::task::{Context, Poll};

use critical_section::{CriticalSection, Mutex};
use embassy_hal_internal::{Peri, PeripheralType, impl_peripheral};
use embassy_sync::waitqueue::AtomicWaker;

use crate::chip::{GPIO_IRQ, PIN_COUNT};
use crate::interrupt::InterruptExt;

// Register backend, selected by the GPIO register layout of the chip.
#[cfg_attr(feature = "_gpio-glb-v2", path = "glb_v2.rs")]
mod _version;

use _version as regs;
use regs::InterruptMode;
pub(crate) use regs::{reset_function, set_function};

// The armed pins are tracked in a 64-bit mask.
const _: () = core::assert!(PIN_COUNT <= 64);

static PIN_WAKERS: [AtomicWaker; PIN_COUNT] = [const { AtomicWaker::new() }; PIN_COUNT];

/// Bitmask of the pins with an armed [`InputFuture`], only these are serviced by the
/// interrupt handler, pins used by other cores or drivers are left untouched.
static ARMED_PINS: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));

/// Represents a digital input or output level.
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Level {
    /// Logical low.
    Low,
    /// Logical high.
    High,
}

impl From<bool> for Level {
    fn from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }
}

impl From<Level> for bool {
    fn from(level: Level) -> bool {
        match level {
            Level::Low => false,
            Level::High => true,
        }
    }
}

/// Pull setting for an input.
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pull {
    /// No pull.
    None,
    /// Internal pull-up resistor.
    Up,
    /// Internal pull-down resistor.
    Down,
}

impl Pull {
    /// Return the values of the pull-up and pull-down bits.
    #[inline]
    fn bits(self) -> (u32, u32) {
        match self {
            Pull::None => (0, 0),
            Pull::Up => (1, 0),
            Pull::Down => (0, 1),
        }
    }
}

/// Drive strength of an output.
#[derive(Debug, Eq, PartialEq, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Drive {
    /// Drive strength 0 (weakest).
    #[default]
    Drive0 = 0,
    /// Drive strength 1.
    Drive1 = 1,
    /// Drive strength 2.
    Drive2 = 2,
    /// Drive strength 3 (strongest).
    Drive3 = 3,
}

/// Internal function called by [`crate::init`].
///
/// The interrupt of the pins is not touched here, because the GPIO configuration is
/// shared with the other cores, which may use them. Pins get their interrupt masked when
/// they are taken by a driver of this crate.
pub(crate) fn init(cs: CriticalSection) {
    // SAFETY: The handler only touches the pins armed by this module.
    unsafe {
        bflb_hal::interrupt::set_handler(GPIO_IRQ, gpio_handler, cs);
        GPIO_IRQ.enable();
    }
}

/// Handler of the GPIO interrupt. It masks the interrupt of pins that triggered and wake
/// their task, which will check that the interrupt is masked to complete its future.
fn gpio_handler(_code: usize, cs: CriticalSection) {
    let armed = ARMED_PINS.borrow(cs);
    let mut pending = armed.get();
    while pending != 0 {
        let pin = pending.trailing_zeros() as u8;
        pending &= pending - 1;
        if regs::take_interrupt(pin, cs) {
            armed.set(armed.get() & !(1 << pin));
            PIN_WAKERS[pin as usize].wake();
        }
    }
}

/// Future waiting for a pin interrupt.
#[must_use = "futures do nothing unless you `.await` or poll them"]
struct InputFuture<'a> {
    pin: Peri<'a, AnyPin>,
}

impl<'a> InputFuture<'a> {
    fn new(pin: Peri<'a, AnyPin>, mode: InterruptMode) -> Self {
        let num = pin.pin();
        critical_section::with(|cs| {
            let armed = ARMED_PINS.borrow(cs);
            armed.set(armed.get() | 1 << num);
            regs::arm_interrupt(num, mode, cs);
        });
        Self { pin }
    }
}

impl Drop for InputFuture<'_> {
    fn drop(&mut self) {
        let num = self.pin.pin();
        critical_section::with(|cs| {
            regs::mask_interrupt(num, cs);
            let armed = ARMED_PINS.borrow(cs);
            armed.set(armed.get() & !(1 << num));
        });
    }
}

impl Future for InputFuture<'_> {
    type Output = ();

    fn poll(self: FuturePin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let num = self.pin.pin();
        PIN_WAKERS[num as usize].register(cx.waker());
        // The handler masks the interrupt when it triggered.
        if regs::is_interrupt_masked(num) {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

/// GPIO flexible pin.
///
/// This pin can be either an input or output pin. The output level register bit will
/// remain set while not in output mode, so the pin's level will be 'remembered' when it
/// is not in output mode.
pub struct Flex<'d> {
    pin: Peri<'d, AnyPin>,
}

impl<'d> Flex<'d> {
    /// Wrap the pin in a `Flex`.
    ///
    /// The pin remains disconnected, with its Schmitt trigger enabled. The initial output
    /// level is unspecified, but can be changed before the pin is put into output mode.
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>) -> Self {
        let pin = pin.into();
        regs::init_gpio(pin.pin());
        Self { pin }
    }

    /// Put the pin into input mode.
    ///
    /// The pull and Schmitt trigger settings are left unchanged.
    #[inline]
    pub fn set_as_input(&mut self) {
        regs::set_direction(self.pin.pin(), true, false);
    }

    /// Put the pin into output mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it
    /// to begin at a specific level, call `set_high`/`set_low` on the pin first. The input
    /// stays enabled so the pin level can still be read.
    #[inline]
    pub fn set_as_output(&mut self) {
        regs::set_direction(self.pin.pin(), true, true);
    }

    /// Disconnect the pin, both input and output are disabled.
    #[inline]
    pub fn set_as_disconnected(&mut self) {
        regs::set_direction(self.pin.pin(), false, false);
    }

    /// Set the pin's pull.
    #[inline]
    pub fn set_pull(&mut self, pull: Pull) {
        regs::set_pull(self.pin.pin(), pull);
    }

    /// Set the pin's drive strength.
    #[inline]
    pub fn set_drive(&mut self, drive: Drive) {
        regs::set_drive(self.pin.pin(), drive);
    }

    /// Enable or disable the Schmitt trigger of the input.
    #[inline]
    pub fn set_schmitt(&mut self, enable: bool) {
        regs::set_schmitt(self.pin.pin(), enable);
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        regs::is_input_high(self.pin.pin())
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Get the current pin input level.
    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        regs::set_output(self.pin.pin(), true);
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        regs::set_output(self.pin.pin(), false);
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::Low => self.set_low(),
            Level::High => self.set_high(),
        }
    }

    /// Is the output level high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        regs::is_output_high(self.pin.pin())
    }

    /// Is the output level low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.is_set_high()
    }

    /// What level is the output set to?
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.is_set_high().into()
    }

    /// Toggle pin output.
    #[inline]
    pub fn toggle(&mut self) {
        regs::toggle_output(self.pin.pin())
    }

    /// Wait until the pin is high. If it is already high, return immediately.
    #[inline]
    pub async fn wait_for_high(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptMode::HighLevel).await;
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[inline]
    pub async fn wait_for_low(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptMode::LowLevel).await;
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptMode::RisingEdge).await;
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptMode::FallingEdge).await;
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
    #[inline]
    pub async fn wait_for_any_edge(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptMode::AnyEdge).await;
    }
}

impl Drop for Flex<'_> {
    fn drop(&mut self) {
        regs::deinit_gpio(self.pin.pin());
    }
}

/// GPIO input driver.
pub struct Input<'d> {
    pin: Flex<'d>,
}

impl<'d> Input<'d> {
    /// Create GPIO input driver for a [Pin] with the provided [Pull] configuration.
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>, pull: Pull) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_pull(pull);
        pin.set_as_input();
        Self { pin }
    }

    /// Enable or disable the Schmitt trigger.
    #[inline]
    pub fn set_schmitt(&mut self, enable: bool) {
        self.pin.set_schmitt(enable)
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    /// Returns current pin level.
    #[inline]
    pub fn get_level(&self) -> Level {
        self.pin.get_level()
    }

    /// Wait until the pin is high. If it is already high, return immediately.
    #[inline]
    pub async fn wait_for_high(&mut self) {
        self.pin.wait_for_high().await;
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[inline]
    pub async fn wait_for_low(&mut self) {
        self.pin.wait_for_low().await;
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        self.pin.wait_for_rising_edge().await;
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        self.pin.wait_for_falling_edge().await;
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
    #[inline]
    pub async fn wait_for_any_edge(&mut self) {
        self.pin.wait_for_any_edge().await;
    }
}

/// GPIO output driver.
pub struct Output<'d> {
    pin: Flex<'d>,
}

impl<'d> Output<'d> {
    /// Create GPIO output driver for a [Pin] with the provided [Level].
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>, initial_output: Level) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_level(initial_output);
        pin.set_as_output();
        Self { pin }
    }

    /// Set the pin's drive strength.
    #[inline]
    pub fn set_drive(&mut self, drive: Drive) {
        self.pin.set_drive(drive)
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_high()
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_low()
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level)
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.is_set_low()
    }

    /// What level output is set to.
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.get_output_level()
    }

    /// Toggle pin output.
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle()
    }
}

pub(crate) trait SealedPin {
    fn _pin(&self) -> u8;
}

/// Interface for a pin that can be configured by GPIO drivers, or converted to an
/// [`AnyPin`].
#[allow(private_bounds)]
pub trait Pin: PeripheralType + Into<AnyPin> + SealedPin + Sized + 'static {
    /// Returns the pin number.
    #[inline]
    fn pin(&self) -> u8 {
        self._pin()
    }
}

/// Type-erased GPIO pin.
pub struct AnyPin {
    pub(crate) pin: u8,
}

impl AnyPin {
    /// Unsafely create an `AnyPin` from a pin number.
    ///
    /// # Safety
    ///
    /// You must ensure that you're only using one instance of this type at a time.
    #[inline]
    pub unsafe fn steal(pin: u8) -> Peri<'static, Self> {
        assert!((pin as usize) < PIN_COUNT, "invalid pin number");
        // SAFETY: Guaranteed by the caller.
        unsafe { Peri::new_unchecked(Self { pin }) }
    }
}

impl_peripheral!(AnyPin);

impl SealedPin for AnyPin {
    #[inline]
    fn _pin(&self) -> u8 {
        self.pin
    }
}

impl Pin for AnyPin {}

macro_rules! impl_pin {
    ($type:ident, $pin_num:expr) => {
        impl crate::gpio::Pin for peripherals::$type {}
        impl crate::gpio::SealedPin for peripherals::$type {
            #[inline]
            fn _pin(&self) -> u8 {
                $pin_num
            }
        }

        impl From<peripherals::$type> for crate::gpio::AnyPin {
            fn from(_val: peripherals::$type) -> Self {
                Self { pin: $pin_num }
            }
        }
    };
}

// ====================== embedded-hal 1.0 ====================== //

impl embedded_hal::digital::ErrorType for Input<'_> {
    type Error = Infallible;
}

impl embedded_hal::digital::InputPin for Input<'_> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl embedded_hal_async::digital::Wait for Input<'_> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.pin.wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.pin.wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.pin.wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.pin.wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.pin.wait_for_any_edge().await;
        Ok(())
    }
}

impl embedded_hal::digital::ErrorType for Output<'_> {
    type Error = Infallible;
}

impl embedded_hal::digital::OutputPin for Output<'_> {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        (*self).set_high();
        Ok(())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        (*self).set_low();
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for Output<'_> {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

impl embedded_hal::digital::ErrorType for Flex<'_> {
    type Error = Infallible;
}

impl embedded_hal::digital::InputPin for Flex<'_> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl embedded_hal::digital::OutputPin for Flex<'_> {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        (*self).set_high();
        Ok(())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        (*self).set_low();
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for Flex<'_> {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

impl embedded_hal_async::digital::Wait for Flex<'_> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        (*self).wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        (*self).wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        (*self).wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        (*self).wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        (*self).wait_for_any_edge().await;
        Ok(())
    }
}
