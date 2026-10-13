# Embassy BouffaloLab HAL

HALs implement safe, idiomatic Rust APIs to use the hardware capabilities, so raw register manipulation is not needed.

The Embassy BouffaloLab HAL targets the BouffaloLab family of RISC-V chips. It is built on top of the lower-level
`bflb-hal` crate, and implements both blocking and async APIs for its peripherals. The benefit of using the async APIs
is that the HAL takes care of waiting for peripherals to complete operations in low power mode and handling interrupts,
so that applications can focus on more important matters.

NOTE: The Embassy HALs can be used both for non-async and async operations. For async, you can choose which runtime you
want to use.

## Hardware support

The `bflb-embassy` HAL currently supports:

* BL808, M0 core (feature `bl808-m0`, example: `examples/ox64-m0-embassy`)

Supported peripherals: GPIO (with async waiting on pin levels and edges) and UART.

The runtime (such as `bflb-rt`) must initialize `bflb-hal` before `bflb_embassy::init` is called, which is what `bflb-rt`
does before calling `main`. Use it with `embassy-executor` and its `platform-riscv32` feature.

## Time driver

If the `time-driver` feature is enabled (it is by default), the HAL uses the RISC-V machine timer as a global time
driver for [embassy-time](https://crates.io/crates/embassy-time), with a tick rate of 1 MHz.

## Interrupts

Interrupts are bound to the drivers with `bind_interrupts!`, like in the other Embassy HALs. As the runtime dispatches
interrupts through a table instead of linking handlers by symbol name, the drivers register the bound handler in that
table when they are created. The GPIO interrupt is owned by the GPIO driver and doesn't need to be bound.

## Embedded-hal

The `bflb-embassy` HAL implements the traits from [embedded-hal](https://crates.io/crates/embedded-hal) (1.0),
[embedded-hal-async](https://crates.io/crates/embedded-hal-async), [embedded-io](https://crates.io/crates/embedded-io)
and [embedded-io-async](https://crates.io/crates/embedded-io-async).

## Adding a chip

The drivers are chip-independent, everything specific to a chip lives in one module, `src/chips/<chip>.rs`, selected
by its chip feature:

1. Support the chip in `bflb-hal`: register blocks in `arch`, interrupts, clocks, time and UART pin routing.
2. Write `src/chips/<chip>.rs` (see `src/chips/bl808_m0.rs`): the `pac` register module, `PIN_COUNT`, `GPIO_IRQ`, the
   `embassy_hal_internal::peripherals!` singletons, and the `impl_pin!`, `impl_uart!` and `interrupt_mod!` invocations.
3. Add the chip feature in `Cargo.toml`, enabling the internal features of the register layouts of the chip (such as
   `_gpio-glb-v2`, used by BL808 and BL616/BL618), and list it in the `compile_error!` check and the
   `#[cfg_attr(..., path = ...)]` of `mod chip` in `src/lib.rs`.
4. If the chip uses another register layout (such as the GLB v1 GPIO registers of BL602/BL702), add a backend next
   to `src/gpio/glb_v2.rs` with the same functions, selected by a new internal feature.
