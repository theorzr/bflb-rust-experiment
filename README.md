# BouffaloLab Rust Experiment
This repository is a Cargo workspace containing five lib crates and 
example binary crates.

## BouffaloLab HAL
This is an HAL implementation for BouffaloLab chips. It currently only 
support BL808 as it's my main goal for now. It can be considered as a 
Rust port of the official Bouffalo SDK.

## BouffaloLab Runtime
This crate provides small runtime for building and linking binaries 
for targeting BouffaloLab chips. It also provides a bootstrap assembly
and interrupt handling.

## BouffaloLab Embassy HAL
The crate `bflb-embassy` is an [Embassy](https://embassy.dev) HAL built on top of the
BouffaloLab HAL, structured like the official Embassy HALs (`embassy-nrf` in particular),
with one module per chip in `bflb-embassy/src/chips`. It currently supports the BL808 M0
core (feature `bl808-m0`), see the `test-ox64-m0-embassy` example, and
[its README](bflb-embassy/README.md) for the details and how to add a chip.

## Embedded HAL utilities and runtime (PoC)
The crate `embedded-util` provides an alternative way of abstracting 
the embedded development in Rust. It provides utilities to generate 
peripherals and exclusive access to them. It aims to complete the 
official `embedded-hal` crate by providing a new way of defining 
Peripheral Access Crate (PAC), MMIO structures and bitfield registers.

## How to compile
Two targets must be installed to support both RV32 and RV64:
- `riscv32imac-unknown-none-elf`
- `riscv64gc-unknown-none-elf`

The following commands can be used to compile the examples:
- `cargo build -p test-ox64-m0 --target riscv32imac-unknown-none-elf --release`
- `cargo build -p test-ox64-m0-embassy --target riscv32imac-unknown-none-elf --release`
- `cargo build -p test-ox64-d0 --target riscv64gc-unknown-none-elf --release`

Get the binary file to flash:
- `cargo objcopy -p test-ox64-m0 --target riscv32imac-unknown-none-elf --release -- -O binary test-m0.bin`
- `cargo objcopy -p test-ox64-m0-embassy --bin test-ox64-m0-embassy --target riscv32imac-unknown-none-elf --release -- -O binary test-m0-embassy.bin`
- `cargo objcopy -p test-ox64-m0-embassy --bin gpio_loopback --target riscv32imac-unknown-none-elf --release -- -O binary gpio-loopback.bin` (GPIO16 and GPIO17 connected, send `g` on UART0)
- `cargo objcopy -p test-ox64-d0 --target riscv64gc-unknown-none-elf --release -- -O binary test-d0.bin`

## Under linux
Be sure to be in the right user group for opening the device file, then connect to it 
using: `picocom --echo --imap lfcrlf -b 115200 /dev/ttyUSB0`.

## Documents

Chip documents:
- [Bouffalo SDK](https://github.com/bouffalolab/bouffalo_sdk)
- [Sipeed SDK](https://github.com/sipeed/M1s_BL808_SDK)
- [BL808 RM](https://raw.githubusercontent.com/bouffalolab/bl_docs/main/BL808_RM/en/BL808_RM_en_1.3.pdf)
- [BL808 DS](https://raw.githubusercontent.com/bouffalolab/bl_docs/main/BL808_DS/en/BL808_DS_1.2_en.pdf)
- [T-head C906](https://github.com/T-head-Semi/openc906)

RISC-V and extensions:
- [RISC-V Specifications](https://riscv.org/technical/specifications/)
- [RISC-V Asm Manual](https://github.com/riscv-non-isa/riscv-asm-manual/blob/master/riscv-asm.md)
- [RISC-V CLIC Spec](https://raw.githubusercontent.com/riscv/riscv-fast-interrupt/master/clic.pdf)
- [RISC-V PLIC Spec](https://raw.githubusercontent.com/riscv/riscv-plic-spec/master/riscv-plic-1.0.0_rc6.pdf)
- [T-head RISC-V Extension](https://github.com/T-head-Semi/thead-extension-spec)
- [SiFive EmbedDev QuickRef](https://five-embeddev.com/quickref/)
- [SiFive Interrupt Cookbook](https://starfivetech.com/uploads/sifive-interrupt-cookbook-v1p2.pdf)

Additional documents:
- [BLDevCube Tutorial](https://bl-mcu-sdk.readthedocs.io/zh_CN/latest/get_started/devcube.html)
- [MIPI D-PHY](http://www.jmrcubed.com/vr/ref_tech/mipi_d_phy_specification_v01-00-00.pdf)
- [MIPI CSI-2](https://caxapa.ru/thumbs/799244/MIPI_Alliance_Specification_for_Camera_S.pdf)
- [Linker Script](https://users.informatik.haw-hamburg.de/~krabat/FH-Labor/gnupro/5_GNUPro_Utilities/c_Using_LD/ldLinker_scripts.html)
- [QSPI Flash W25Q128JW](https://www.winbond.com/hq/product/code-storage-flash-memory/serial-nor-flash/?__locale=en&partNo=W25Q128JW)

## License

This project is licensed under the [Apache License, Version 2.0](LICENSE).
