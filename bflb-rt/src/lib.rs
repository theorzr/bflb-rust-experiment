//! Embedded runtime for BouffaloLab chips.

#![no_std]
#![no_main]
#![crate_type = "rlib"]
#![deny(unsafe_op_in_unsafe_fn)]

// Our crate provides a global allocator usable by alloc.
extern crate alloc;

#[cfg(not(rt_chip_ok))]
compile_error!("no runtime chip selected, use the crate features to select one chip");

#[cfg(rt_chip = "bl808_d0")]
mod bl808_d0;
#[cfg(rt_chip = "bl808_m0")]
mod bl808_m0;

// Re-export HAL.
pub use bflb_hal as hal;

// These modules are intentionally internal.
mod allocator;

use allocator::RuntimeAllocator;
use critical_section::CriticalSection;
use hal::interrupt::{noop_handler, COUNT as INT_COUNT};

/// Module providing externally linked symbols, defined either by
/// assembly or link script.
pub mod sym {

    // Here we define all linker script symbols.
    // Note that we can define all symbols as "u32", because the linker
    // script forces alignments to 4 bytes.
    extern "C" {

        /// First word of the text section.
        pub static mut _ld_text_start: u32;
        /// First word **after** the text section.
        pub static mut _ld_text_end: u32;

        /// First word of the read-only data section.
        pub static mut _ld_rodata_start: u32;
        /// First word **after** the read-only section.
        pub static mut _ld_rodata_end: u32;

        /// First word of the data section in Flash that
        /// should be copied to RAM.
        pub static mut _ld_data_load_start: u32;
        /// First word of the read/write data section.
        pub static mut _ld_data_start: u32;
        /// First word **after** the read/write data section.
        pub static mut _ld_data_end: u32;

        /// First word of the read/write uninit data section.
        pub static mut _ld_bss_start: u32;
        /// First word **after** the read/write uninit data section.
        pub static mut _ld_bss_end: u32;

        /// First word of the stack.
        pub static mut _ld_stack_start: u8;
        /// First word **after** the stack.
        pub static mut _ld_stack_end: u8;

        /// First word of the heap.
        pub static mut _ld_heap_start: u8;
        /// First word **after** the heap.
        pub static mut _ld_heap_end: u8;

        /// The default Machine Trap Generic Handler that is implemented
        /// in assembly and handles context saving and handling via
        /// exception/interrupt handlers that can be registered using
        /// [`register_exception_handler`] and
        /// [`register_interrupt_handler`].
        ///
        /// ***It should only be called by hardware on interrupt, only
        /// use this as the symbol.***
        pub fn _mtrap_generic_handler() -> !;

    }
}

/// The global bump allocator.
#[global_allocator]
static ALLOCATOR: RuntimeAllocator = RuntimeAllocator::empty();

/// This function is responsible for loading mutable static variables
/// and zero-out uninitialized variables, all in RAM.
///
/// **This function is really important but will only trigger on
/// hart 0, because memory should be initialized once, and most
/// importantly we have kernel stack only on hart 0.**
///
/// *This function should not access global variables or allocate
/// memory.*
///
/// *It is called just before entry by the assembly and should not be
/// called from elsewhere because **it will break the program**.*
#[no_mangle]
extern "C" fn _rust_mem_init() {
    // Only hart zero initialize the memory.
    if hal::hart::hart_zero() {
        unsafe {
            // Copy mutable global variables to RAM.
            let src: *mut u32 = &mut sym::_ld_data_load_start;
            let dst: *mut u32 = &mut sym::_ld_data_start;
            let dst_end: *mut u32 = &mut sym::_ld_data_end;
            core::ptr::copy(src, dst, dst_end.offset_from(dst) as _);

            // Zero BSS uninit variables.
            let dst: *mut u32 = &mut sym::_ld_bss_start;
            let dst_end: *mut u32 = &mut sym::_ld_bss_end;
            core::ptr::write_bytes(dst, 0, dst_end.offset_from(dst) as _);

            // Init heap allocator.
            let start: *mut u8 = &mut sym::_ld_heap_start;
            let end: *mut u8 = &mut sym::_ld_heap_end;
            ALLOCATOR.with(|heap| heap.init(start, end.offset_from(start) as _));
        }
    } else {
        // FIXME: Other harts needs to wait until hart zero finished synchronization.
    }
}

/// This function is responsible for initializing the system before
/// calling the entry point. This function internally initialize the
/// hart (with its unique ID, used for HartLocal variables) and the
/// chip **only on hart 0**.
#[no_mangle]
extern "C" fn _rust_init() {
    // SAFETY: We execute it once per hart.
    unsafe {
        hal::init();
    }
}

/// Entry point function, called from assembly.
/// It never returns, so the assembly just don't have to do anything after it.
#[no_mangle]
extern "C" fn _rust_entry() -> ! {
    extern "Rust" {
        /// Externally-defined main function, this should be implemented by the binary.
        fn main();
    }

    unsafe {
        main();
    }

    // This function should no return: spin loop.
    loop {
        hal::hart::wait_for_interrupt();
    }
}

/// It is called on any trap after the context has been saved in the
/// most efficient way (when possible, only dirty registers should be
/// saved).
#[no_mangle]
extern "C" fn _rust_mtrap_handler(cause: usize) {
    // Interrupt bit is at XLEN - 1
    const INTERRUPT_MASK: usize = 1 << (usize::BITS - 1);
    // Only use the lower 12 bits in order to be compliant with
    // CLIC mode where only those are used for the code.
    const CODE_MASK: usize = 0xFFF;

    let code = cause & CODE_MASK;
    let interrupt = cause & INTERRUPT_MASK != 0;

    if !interrupt {
        panic!("hardware exception: {code}");
    }

    // SAFETY: Interrupts are disabled by the hardware when going to interrupt handlers.
    // We give this token to the interruption handler so it can use it and assert to
    // mutexes that it's legal to access interrupt-free memory locations.
    let cs = unsafe { CriticalSection::new() };

    (hal::interrupt::handler(code, cs))(code, cs);
}

/// This implementation of the panic handler will simply abort without
/// any message.
#[panic_handler]
#[cfg(feature = "panic-abort")]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // TODO: Instead of spin looping indefinitely, it might be possible
    // to close clock gates and stop the core.
    loop {
        hal::hart::wait_for_interrupt()
    }
}

/// This implementation of the panic handler will quickly initialize the UART TX on pin 14
/// before communicating the panic message and informations.
#[panic_handler]
#[cfg(feature = "panic-uart-14")]
fn panic(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write;
    use hal::interrupt::Interrupt;
    use hal::uart::UartConfig;
    use hal::Peripherals;

    // We force create a peripheral because the situation is desperate, so we
    // try to communicate the panic information through UART.
    let peripherals = unsafe { Peripherals::new() };
    let mut uart = peripherals
        .uart
        .p0
        .init_simplex_transmit(peripherals.gpio.p14, &UartConfig::new(115200));

    // Retrieve allocator information, may be useful in case of allocation crash.
    let (alloc_cap, alloc_used, alloc_free) =
        ALLOCATOR.with(|heap| (heap.size(), heap.used(), heap.free()));

    let _ = writeln!(uart);
    let _ = writeln!(uart, "============== Panic");
    let _ = writeln!(uart, "Hart {} {info}", hal::hart::hart());

    let _ = writeln!(uart, "============== Allocator");
    let _ = writeln!(uart, "     capacity: {alloc_cap}");
    let _ = writeln!(uart, "         used: {alloc_used}");
    let _ = writeln!(uart, "         free: {alloc_free}");

    let _ = writeln!(uart, "============== Interrupts");
    // let _ = writeln!(uart, "    threshold: {}", get_threshold());  // FIXME: Invalid opcode???
    for code in 0..INT_COUNT {
        let int = Interrupt::new(code);
        let enabled = int.enabled();
        let pending = int.pending();
        let handler = critical_section::with(|cs| hal::interrupt::handler(code, cs));

        // Do not show uninteresting interrupts.
        if !enabled && !pending && handler == noop_handler {
            continue;
        }

        let _ = write!(uart, "        {code:>5}: ");
        if int.enabled() {
            let _ = write!(uart, "enabled ");
        }
        if int.pending() {
            let _ = write!(uart, "pending ");
        }
        if handler != noop_handler {
            let _ = write!(uart, "-> {:?}", handler);
        }
        let _ = writeln!(uart);
    }

    loop {
        hal::hart::wait_for_interrupt()
    }
}
