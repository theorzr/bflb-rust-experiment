//! All supported IRQ types depending on the selected chip.
//!
//! You need a runtime crate such as `bflb-rt` in order to configure
//! interrupts given these numbers.

use core::cell::Cell;
use core::fmt;

use crate::arch::bl808::CLIC;
use crate::arch::riscv::clic::Mintthresh;

use critical_section::{CriticalSection, Mutex};

/// An access to configure an interrupt number.
#[derive(Clone, Copy)]
pub struct Interrupt {
    /// The interrupt code number.
    pub code: usize,
}

impl Interrupt {
    /// Create a new handle to the given interrupt. You should prefer using the predefined
    /// interrupts defined in this module.
    pub const fn new(code: usize) -> Self {
        Self { code }
    }

    /// Return true if this interrupt is enabled.
    #[inline]
    pub fn enabled(self) -> bool {
        CLIC.int(self.code).enable().get() != 0
    }

    /// Enable or not the interrupt. This function is unsafe because caller must guarantee
    /// that this will not cause issues with the rest of the program, and this function
    /// also doesn't guarantee any synchronization.
    #[inline]
    pub unsafe fn set_enabled(self, enabled: bool) {
        CLIC.int(self.code).enable().set(enabled as _);
    }

    /// Return true if this interrupt is pending to be handled.
    #[inline]
    pub fn pending(self) -> bool {
        CLIC.int(self.code).pending().get() != 0
    }

    /// Set this interrupt's pending state. Note that this is only possible to do when
    /// this interrupt is configured in edge-sensitive mode. This function is unsafe
    /// because caller must guarantee that this will not cause issues in the program.
    #[inline]
    pub unsafe fn set_pending(self, pending: bool) {
        // NB: Look at Read-only or Read/Write in "pending" doc.
        CLIC.int(self.code).pending().set(pending as _);
    }

    /// Return the current configured level for this interrupt.
    #[inline]
    pub fn level(self) -> u8 {
        CLIC.int(self.code).control().get()
    }

    /// Set the current level of this interrupt, used when scheduling the interrupt.
    #[inline]
    pub unsafe fn set_level(self, level: u8) {
        // NB: Read doc of "control" to understand that no all level are valid bit patterns.
        CLIC.int(self.code).control().set(level);
    }

    /// Get the current trigger mode for this interrupt.
    #[inline]
    pub fn trigger(self) -> InterruptTrigger {
        let mut tmp = CLIC.int(self.code).attr().get();
        match (tmp.edge_triggered().get(), tmp.negative_edge().get()) {
            (0, 0) => InterruptTrigger::PositiveLevel,
            (0, 1) => InterruptTrigger::NegativeLevel,
            (1, 0) => InterruptTrigger::PositiveEdge,
            (1, 1) => InterruptTrigger::NegativeEdge,
            // Unreachable and should be optimized-out because only these patterns
            // are valid in the fields' range.
            _ => unreachable!(),
        }
    }

    #[inline]
    pub unsafe fn set_trigger(self, trigger: InterruptTrigger) {
        CLIC.int(self.code).attr().modify(|reg| {
            let (edge, neg) = match trigger {
                InterruptTrigger::PositiveLevel => (0, 0),
                InterruptTrigger::NegativeLevel => (0, 1),
                InterruptTrigger::PositiveEdge => (1, 0),
                InterruptTrigger::NegativeEdge => (1, 1),
            };

            reg.edge_triggered().set(edge);
            reg.negative_edge().set(neg);
        });
    }
}

impl fmt::Debug for Interrupt {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Interrupt")
            .field("code", &self.code)
            .field("enabled", &self.enabled())
            .field("pending", &self.pending())
            .field("level", &self.level())
            .field("trigger", &self.trigger())
            .finish()
    }
}

#[inline]
pub fn get_threshold() -> u8 {
    unsafe { Mintthresh::read_csr().0 }
}

#[inline]
pub unsafe fn set_threshold(level: u8) {
    unsafe { Mintthresh(level).write_csr() }
}

/// Trigger mode that can be configured for a particular interrupt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptTrigger {
    /// The interrupt request is considered when its level is 1.
    PositiveLevel,
    /// The interrupt request is considered when its level is 0.
    NegativeLevel,
    /// The interrupt request is considered when its level goes from 0 to 1.
    PositiveEdge,
    /// The interrupt request is considered when its level goes from 1 to 0.
    NegativeEdge,
}

/// Type alias for interrupt handler function pointer.
pub type InterruptHandler = fn(usize, CriticalSection);

/// Interrupt handler that doesn't do anything with the interrupt.
pub fn noop_handler(_code: usize, _cs: CriticalSection) {}

/// The live interrupt handlers table, dispatched by the runtime. It is initialized with
/// the handlers of this HAL, and handlers can be replaced at runtime using
/// [`set_handler`]. Each handler takes a critical section token for proving that the
/// handler is effectively running with interrupts disabled.
static HANDLERS: Mutex<[Cell<InterruptHandler>; COUNT]> = Mutex::new({
    let mut handlers = [const { Cell::new(noop_handler as InterruptHandler) }; COUNT];

    handlers[MACHINE_TIMER.code] = Cell::new(crate::time::mtimer_handler);

    #[cfg(any(feature = "bl808-m0", feature = "bl808-lp"))]
    {
        handlers[DMA0_ALL.code] = Cell::new(super::dma::dma0_handler);
        handlers[DMA1_ALL.code] = Cell::new(super::dma::dma1_handler);
    }

    #[cfg(feature = "bl808-d0")]
    {
        handlers[DMA2_INT0.code] = Cell::new(super::dma::dma2_handler);
        handlers[DMA2_INT1.code] = Cell::new(super::dma::dma2_handler);
        handlers[DMA2_INT2.code] = Cell::new(super::dma::dma2_handler);
        handlers[DMA2_INT3.code] = Cell::new(super::dma::dma2_handler);
        handlers[DMA2_INT4.code] = Cell::new(super::dma::dma2_handler);
        handlers[DMA2_INT5.code] = Cell::new(super::dma::dma2_handler);
        handlers[DMA2_INT6.code] = Cell::new(super::dma::dma2_handler);
        handlers[DMA2_INT7.code] = Cell::new(super::dma::dma2_handler);
    }

    handlers
});

/// Get the handler currently registered for the given interrupt code. This is used by
/// runtimes to dispatch interrupts.
#[inline]
pub fn handler(code: usize, cs: CriticalSection) -> InterruptHandler {
    HANDLERS.borrow(cs)[code].get()
}

/// Replace the handler of the given interrupt, returning the previous one.
///
/// This function is unsafe because replacing a handler installed by this HAL (such as
/// the machine timer or DMA handlers) will break the drivers relying on it.
#[inline]
pub unsafe fn set_handler(
    int: Interrupt,
    handler: InterruptHandler,
    cs: CriticalSection,
) -> InterruptHandler {
    HANDLERS.borrow(cs)[int.code].replace(handler)
}

/// Internal macro for easier definition.
macro_rules! def_irq {
    (
        $(
            $(#[$meta:meta])*
            $name:ident = $value:expr ;
        )*
    ) => {
        $(
            $(#[$meta])*
            pub const $name: Interrupt = Interrupt::new($value);
        )*
    };
}

def_irq! {
    /// Software interrupt for supervisor privilege.
    SUPERVISOR_SOFTWARE = 1;
    /// Software interrupt for machine privilege.
    MACHINE_SOFTWARE = 3;
    /// Timer interrupt for supervisor privilege.
    SUPERVISOR_TIMER = 5;
    /// Timer interrupt for machine privilege.
    MACHINE_TIMER = 7;
    /// External interrupt for supervisor privilege.
    SUPERVISOR_EXTERNAL = 9;
    /// External interrupt for machine privilege.
    MACHINE_EXTERNAL = 11;
}

#[cfg(any(feature = "bl808-m0", feature = "bl808-lp"))]
def_irq! {
    /// BMX bus error.
    BMX_MCU_BUS_ERR     = 16 + 0;
    /// BMX timeout.
    BMX_MCU_TO          = 16 + 1;
    /// IPC M0 interrupt.
    IPC_M0              = 16 + 3;
    /// Audio interrupt.
    AUDIO               = 16 + 4;
    RF_TOP_INT0         = 16 + 5;
    RF_TOP_INT1         = 16 + 6;
    /// LZ4 decompressor interrupt.
    LZ4D                = 16 + 7;
    GAUGE_ITF           = 16 + 8;
    SEC_ENG_ID1         = 16 + 9;
    SEC_ENG_ID0         = 16 + 10;
    SEC_ENG_ID1_CDET    = 16 + 11;
    SEC_ENG_ID0_CDET    = 16 + 12;
    SF_CTRL_ID1         = 16 + 13;
    SF_CTRL_ID0         = 16 + 14;
    /// Interrupt for all DMA 0 channels.
    DMA0_ALL            = 16 + 15;
    /// Interrupt for all DMA 1 channels.
    DMA1_ALL            = 16 + 16;
    SDH                 = 16 + 17;
    MM_ALL              = 16 + 18;
    IR_TX               = 16 + 19;
    IR_RX               = 16 + 20;
    USB                 = 16 + 21;
    AUPDM               = 16 + 22;
    EMAC                = 16 + 24;
    GPADC_DMA           = 16 + 25;
    EFUSE               = 16 + 26;
    SPI0                = 16 + 27;
    UART0               = 16 + 28;
    UART1               = 16 + 29;
    UART2               = 16 + 30;
    GPIO_DMA            = 16 + 31;
    I2C0                = 16 + 32;
    PWN                 = 16 + 33;
    IPC_RSVD            = 16 + 34;
    IPC_LP              = 16 + 35;
    /// Timer0 channel 0 interrupt.
    TIMER0_CH0          = 16 + 36;
    /// Timer0 channel 1 interrupt.
    TIMER0_CH1          = 16 + 37;
    /// Timer0 watch dog interrupt.
    TIMER0_WDT          = 16 + 38;
    I2C1                = 16 + 39;
    I2S                 = 16 + 40;
    ANA_OCP_OUT_TO_CPU0 = 16 + 41;
    ANA_OCP_OUT_TO_CPU1 = 16 + 42;
    ANA_OCP_OUT_TO_CPU2 = 16 + 43;
    GPIO_INT0           = 16 + 44;
    DM                  = 16 + 45;
    /// Bluetooh interrupt.
    BL                  = 16 + 46;
    M154_REQ_ACK        = 16 + 47;
    M154_INT            = 16 + 48;
    M154_AES            = 16 + 49;
    PDS_WAKE_UP         = 16 + 50;
    HBN_OUT0            = 16 + 51;
    HBN_OUT1            = 16 + 52;
    BOR                 = 16 + 53;
    WIFI                = 16 + 54;
    BZ_PHY_INT          = 16 + 55;
    /// Bluetooh low energy interrupt.
    BLE                 = 16 + 56;
    MAC_TX_RX_TIMER     = 16 + 57;
    MAC_TX_RX_MISC      = 16 + 58;
    MAC_RX_TRIGGER      = 16 + 59;
    MAC_TX_TRIGGER      = 16 + 60;
    MAC_GEN             = 16 + 61;
    MAC_PORT_TRIGGER    = 16 + 62;
    WIFI_IPC            = 16 + 63;
}

#[cfg(any(feature = "bl808-m0", feature = "bl808-lp"))]
pub const COUNT: usize = 16 + 64;

#[cfg(feature = "bl808-d0")]
def_irq! {
    /// BMX bus error.
    BMX_DSP_BUS_ERR     = 16 + 0;
    UART3               = 16 + 4;
    I2C2                = 16 + 5;
    I2C3                = 16 + 6;
    SPI1                = 16 + 7;
    SEOF_INT0           = 16 + 10;
    SEOF_INT1           = 16 + 11;
    SEOF_INT2           = 16 + 12;
    DVP2BUS_INT0        = 16 + 13;
    DVP2BUS_INT1        = 16 + 14;
    DVP2BUS_INT2        = 16 + 15;
    DVP2BUS_INT3        = 16 + 16;
    H264_BS             = 16 + 17;
    H264_FRAME          = 16 + 18;
    H264_SEQ_DONE       = 16 + 19;
    MJPEG               = 16 + 20;
    H264_S_BS           = 16 + 21;
    H264_S_FRAME        = 16 + 22;
    H264_S_SEQ_DONE     = 16 + 23;
    DMA2_INT0           = 16 + 24;
    DMA2_INT1           = 16 + 25;
    DMA2_INT2           = 16 + 26;
    DMA2_INT3           = 16 + 27;
    DMA2_INT4           = 16 + 28;
    DMA2_INT5           = 16 + 29;
    DMA2_INT6           = 16 + 30;
    DMA2_INT7           = 16 + 31;
    SDH_MMC1            = 16 + 32;
    SDH_MMC3            = 16 + 33;
    SDH2PMU_WAKEUP1     = 16 + 34;
    SDH2PMU_WAKEUP3     = 16 + 35;
    EMAC2               = 16 + 36;
    MIPI_CSI            = 16 + 37;
    IPC_D0              = 16 + 38;
    APU                 = 16 + 39;
    /// MJPEG decoder interrupt.
    MJDEC               = 16 + 40;
    DVP2BUS_INT4        = 16 + 41;
    DVP2BUS_INT5        = 16 + 42;
    DVP2BUS_INT6        = 16 + 43;
    DVP2BUS_INT7        = 16 + 44;
    DMA2D_INT0          = 16 + 45;
    DMA2D_INT1          = 16 + 46;
    DISPLAY             = 16 + 47;
    PWM                 = 16 + 48;
    SEOF_INT3           = 16 + 49;
    OSD                 = 16 + 52;
    DBI                 = 16 + 53;
    OSDA_BUS_DRAIN      = 16 + 55;
    OSDB_BUS_DRAIN      = 16 + 56;
    OSD_PB              = 16 + 57;
    MIPI_DSI            = 16 + 59;
    TIMER1_CH0          = 16 + 61;
    TIMER1_CH1          = 16 + 62;
    TIMER1_WDT          = 16 + 63;
    AUDIO               = 16 + 64;
    WL_ALL              = 16 + 65;
    PDS                 = 16 + 66;
}

#[cfg(feature = "bl808-d0")]
pub const COUNT: usize = 16 + 67;
