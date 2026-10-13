//! Pin registers of the GLB v2 layout (BL808, BL616/BL618): one configuration register
//! per pin, holding everything including the output level and the interrupt state.
//!
//! All the functions taking a pin number expect it to be valid for the chip.

use critical_section::CriticalSection;
use embedded_util::PtrRw;

use super::{Drive, Pull};
use crate::pac::GLB;
use crate::pac::glb::GlbGpioCfg0;

/// GLB pin function selecting the software controlled GPIO.
const FUNC_GPIO: u32 = 11;

/// Output mode where the output is driven by the `gpio_X_o` bit of the pin, which can be
/// read back. The set/clear mode (1) is not used because its output latch can't be read
/// back, neither from the `gpio_X_o` bits nor from the output bank registers.
const MODE_OUTPUT_VALUE: u32 = 0;

/// Pin interrupt trigger mode, values of the `gpio_X_int_mode_set` field.
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub(super) enum InterruptMode {
    FallingEdge = 0,
    RisingEdge = 1,
    LowLevel = 2,
    HighLevel = 3,
    AnyEdge = 4,
}

/// Internal function to get the configuration register of the given pin.
#[inline]
fn cfg_reg(pin: u8) -> PtrRw<GlbGpioCfg0> {
    let mut cfg = GLB.gpio_cfg0();
    // SAFETY: Pin configuration registers are contiguous, one per pin.
    cfg.0 = unsafe { cfg.0.add(pin as usize) };
    cfg
}

/// Modify the configuration register of a pin. The interrupt handler also modifies these
/// registers, so this is done in a critical section to not lose any modification.
#[inline]
fn modify_cfg(pin: u8, func: impl FnOnce(&mut GlbGpioCfg0)) {
    critical_section::with(|_| cfg_reg(pin).modify(func));
}

/// Pulse the interrupt clear bit of a pin, this must be called in a critical section.
#[inline]
fn clear_interrupt(pin: u8, _cs: CriticalSection) {
    cfg_reg(pin).modify(|reg| reg.gpio_0_int_clr().fill());
    cfg_reg(pin).modify(|reg| reg.gpio_0_int_clr().clear());
}

/// Configure the pin as a disconnected software GPIO, with its Schmitt trigger enabled
/// and its interrupt masked.
#[inline]
pub(super) fn init_gpio(pin: u8) {
    modify_cfg(pin, |reg| {
        reg.gpio_0_func_sel().set(FUNC_GPIO);
        reg.gpio_0_mode().set(MODE_OUTPUT_VALUE);
        reg.gpio_0_ie().clear();
        reg.gpio_0_oe().clear();
        reg.gpio_0_smt().fill();
        reg.gpio_0_int_mask().fill();
    });
}

/// Disconnect the pin, mask its interrupt and remove its pull.
#[inline]
pub(super) fn deinit_gpio(pin: u8) {
    modify_cfg(pin, |reg| {
        reg.gpio_0_int_mask().fill();
        reg.gpio_0_ie().clear();
        reg.gpio_0_oe().clear();
        reg.gpio_0_pu().clear();
        reg.gpio_0_pd().clear();
    });
}

/// Enable or disable the input and output buffers of the pin.
#[inline]
pub(super) fn set_direction(pin: u8, input: bool, output: bool) {
    modify_cfg(pin, |reg| {
        reg.gpio_0_ie().set(input as _);
        reg.gpio_0_oe().set(output as _);
    });
}

#[inline]
pub(super) fn set_pull(pin: u8, pull: Pull) {
    modify_cfg(pin, |reg| {
        let (pu, pd) = pull.bits();
        reg.gpio_0_pu().set(pu);
        reg.gpio_0_pd().set(pd);
    });
}

#[inline]
pub(super) fn set_drive(pin: u8, drive: Drive) {
    modify_cfg(pin, |reg| reg.gpio_0_drv().set(drive as _));
}

#[inline]
pub(super) fn set_schmitt(pin: u8, enable: bool) {
    modify_cfg(pin, |reg| reg.gpio_0_smt().set(enable as _));
}

/// Return true if the input level of the pin is high.
#[inline]
pub(super) fn is_input_high(pin: u8) -> bool {
    cfg_reg(pin).get().gpio_0_i().get() != 0
}

/// Set the output level of the pin.
#[inline]
pub(super) fn set_output(pin: u8, high: bool) {
    modify_cfg(pin, |reg| reg.gpio_0_o().set(high as _));
}

/// Invert the output level of the pin.
#[inline]
pub(super) fn toggle_output(pin: u8) {
    modify_cfg(pin, |reg| {
        let high = reg.gpio_0_o().get();
        reg.gpio_0_o().set(high ^ 1);
    });
}

/// Return true if the output level of the pin is set high.
#[inline]
pub(super) fn is_output_high(pin: u8) -> bool {
    cfg_reg(pin).get().gpio_0_o().get() != 0
}

/// Configure the trigger mode of the pin, then clear and unmask its interrupt. The input
/// is enabled because the interrupt logic samples it.
#[inline]
pub(super) fn arm_interrupt(pin: u8, mode: InterruptMode, cs: CriticalSection) {
    cfg_reg(pin).modify(|reg| {
        reg.gpio_0_int_mask().fill();
        reg.gpio_0_int_mode_set().set(mode as _);
        reg.gpio_0_ie().fill();
    });
    clear_interrupt(pin, cs);
    cfg_reg(pin).modify(|reg| reg.gpio_0_int_mask().clear());
}

/// Mask the interrupt of the pin.
#[inline]
pub(super) fn mask_interrupt(pin: u8, _cs: CriticalSection) {
    cfg_reg(pin).modify(|reg| reg.gpio_0_int_mask().fill());
}

/// Return true if the interrupt of the pin is masked.
#[inline]
pub(super) fn is_interrupt_masked(pin: u8) -> bool {
    cfg_reg(pin).get().gpio_0_int_mask().get() != 0
}

/// If the interrupt of the pin triggered, mask and clear it, then return true.
#[inline]
pub(super) fn take_interrupt(pin: u8, cs: CriticalSection) -> bool {
    if cfg_reg(pin).get().gpio_0_int_stat().get() == 0 {
        return false;
    }
    mask_interrupt(pin, cs);
    clear_interrupt(pin, cs);
    true
}

/// Configure a pin for an alternate function, used by other drivers.
pub(crate) fn set_function(pin: u8, function: u32, pull: Pull, drive: Drive) {
    modify_cfg(pin, |reg| {
        reg.gpio_0_func_sel().set(function);
        let (pu, pd) = pull.bits();
        reg.gpio_0_pu().set(pu);
        reg.gpio_0_pd().set(pd);
        reg.gpio_0_drv().set(drive as _);
        reg.gpio_0_ie().fill();
        reg.gpio_0_smt().fill();
        reg.gpio_0_oe().clear();
        reg.gpio_0_int_mask().fill();
    });
}

/// Reset a pin configured by [`set_function`] to a disconnected GPIO.
pub(crate) fn reset_function(pin: u8) {
    modify_cfg(pin, |reg| {
        reg.gpio_0_func_sel().set(FUNC_GPIO);
        reg.gpio_0_ie().clear();
        reg.gpio_0_oe().clear();
        reg.gpio_0_pu().clear();
        reg.gpio_0_pd().clear();
    });
}
