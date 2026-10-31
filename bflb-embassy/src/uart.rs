//! Universal Asynchronous Receiver/Transmitter (UART) driver.
//!
//! Any pin can be routed to any UART signal: each pin `N` is connected to the UART signal
//! `N % S` (`S` being 12 on BL808), and each signal can be assigned to any UART function.
//! This means that two pins sharing the same `N % S` cannot be used for UART at once.
//!
//! The UART IP is nearly the same on all BouffaloLab chips, the instances are declared by the
//! chip modules.
#![macro_use]

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};
use core::task::Poll;

use bflb_hal::uart::{UartConfig, UartFunction};
pub use bflb_hal::uart::{UartDataBits as DataBits, UartParity as Parity, UartStopBits as StopBits};
use embassy_hal_internal::drop::OnDrop;
use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::{self, AnyPin, Drive, Pull};
use crate::interrupt::typelevel::{Binding, Interrupt as _};
use crate::interrupt::{self, Interrupt, InterruptExt};
use crate::pac::Uart as Regs;

/// GLB pin function for UART signals.
const FUNC_UART: u32 = 7;

/// Depth of the TX and RX FIFOs.
const FIFO_DEPTH: u32 = 32;

/// TX FIFO threshold used while writing: wake up when the FIFO is at least half empty.
const TX_THRESHOLD_WRITE: u32 = FIFO_DEPTH / 2 - 1;
/// TX FIFO threshold used while flushing: wake up when the FIFO is fully empty.
const TX_THRESHOLD_FLUSH: u32 = FIFO_DEPTH - 1;

// Bits of the int_sts, int_mask, int_clear and int_en registers.
const INT_UTX_END: u32 = 1 << 0;
const INT_URX_END: u32 = 1 << 1;
const INT_UTX_FIFO: u32 = 1 << 2;
const INT_URX_FIFO: u32 = 1 << 3;
const INT_URX_RTO: u32 = 1 << 4;
const INT_URX_PCE: u32 = 1 << 5;
const INT_ALL: u32 = 0xFFF;

const INT_TX: u32 = INT_UTX_FIFO | INT_UTX_END;
const INT_RX: u32 = INT_URX_FIFO | INT_URX_RTO | INT_URX_PCE | INT_URX_END;

/// UART configuration.
#[non_exhaustive]
#[derive(Debug, Clone)]
pub struct Config {
    /// Baud rate.
    pub baudrate: u32,
    /// Number of data bits.
    pub data_bits: DataBits,
    /// Number of stop bits.
    pub stop_bits: StopBits,
    /// Parity type.
    pub parity: Parity,
    /// Enable the CTS flow control of the transmitter.
    pub cts: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate: 115200,
            data_bits: DataBits::Bits8,
            stop_bits: StopBits::Bits1,
            parity: Parity::None,
            cts: false,
        }
    }
}

/// UART error.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The RX FIFO overflowed, some data was lost.
    Overrun,
    /// A parity error was detected.
    Parity,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let message = match self {
            Self::Overrun => "RX FIFO overrun",
            Self::Parity => "parity error",
        };
        f.write_str(message)
    }
}

impl core::error::Error for Error {}

/// Blocking driver mode.
pub struct Blocking;
/// Async driver mode.
pub struct Async;

trait SealedMode {}
impl SealedMode for Blocking {}
impl SealedMode for Async {}

/// Driver mode.
#[allow(private_bounds)]
pub trait Mode: SealedMode {}
impl Mode for Blocking {}
impl Mode for Async {}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _uart: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::info().regs();
        let state = T::state();
        let pending = regs.int_sts().get() & !regs.int_mask().get();

        // Mask the interrupts that triggered and wake the tasks, they will check the
        // FIFO state themselves and unmask the interrupts again if needed.
        let mut mask = 0;
        if pending & INT_RX != 0 {
            mask |= INT_RX;
            state.rx_waker.wake();
        }
        if pending & INT_TX != 0 {
            mask |= INT_TX;
            state.tx_waker.wake();
        }

        regs.int_mask().modify(|reg| *reg |= mask);
    }
}

/// UART driver.
pub struct Uart<'d, M: Mode> {
    tx: UartTx<'d, M>,
    rx: UartRx<'d, M>,
}

/// UART TX driver.
pub struct UartTx<'d, M: Mode> {
    info: &'static Info,
    state: &'static State,
    tx: Option<Peri<'d, AnyPin>>,
    _mode: PhantomData<M>,
}

/// UART RX driver.
pub struct UartRx<'d, M: Mode> {
    info: &'static Info,
    state: &'static State,
    rx: Option<Peri<'d, AnyPin>>,
    _mode: PhantomData<M>,
}

impl<'d> Uart<'d, Blocking> {
    /// Create a new blocking UART driver with TX and RX.
    pub fn new_blocking<T: Instance>(
        _uart: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(T::info(), T::state(), tx.into(), rx.into(), config)
    }
}

impl<'d> Uart<'d, Async> {
    /// Create a new async UART driver with TX and RX.
    pub fn new<T: Instance>(
        _uart: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        irqs: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        let uart = Self::new_inner(T::info(), T::state(), tx.into(), rx.into(), config);
        enable_interrupt::<T>(&irqs);
        uart
    }
}

impl<'d, M: Mode> Uart<'d, M> {
    fn new_inner(
        info: &'static Info,
        state: &'static State,
        tx: Peri<'d, AnyPin>,
        rx: Peri<'d, AnyPin>,
        config: Config,
    ) -> Self {
        state.refs.store(2, Ordering::Relaxed);
        attach_pin(info.tx_function(), &tx);
        attach_pin(info.rx_function(), &rx);
        configure(info, &config, true, true);
        Self {
            tx: UartTx {
                info,
                state,
                tx: Some(tx),
                _mode: PhantomData,
            },
            rx: UartRx {
                info,
                state,
                rx: Some(rx),
                _mode: PhantomData,
            },
        }
    }

    /// Change the baud rate. This blocks until pending data has been transmitted, and
    /// then clears the FIFOs so that no byte is sent or received at a mixed rate.
    ///
    /// # Panics
    ///
    /// Panics if the baud rate is zero or cannot be reached from the UART clock.
    pub fn set_baudrate(&mut self, baudrate: u32) {
        let _ = self.tx.blocking_flush();
        let regs = self.tx.info.regs();
        bflb_hal::uart::set_baudrate(regs, baudrate);
        regs.fifo_cfg0().modify(|reg| {
            reg.tx_fifo_clear().fill();
            reg.rx_fifo_clear().fill();
        });
    }

    /// Write bytes, blocking until they are all in the TX FIFO.
    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        self.tx.blocking_write(buffer)
    }

    /// Block until all data has been transmitted.
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        self.tx.blocking_flush()
    }

    /// Read bytes, blocking until the buffer is full.
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.rx.blocking_read(buffer)
    }

    /// Split the UART into a transmitter and receiver, which is particularly useful when
    /// having two tasks correlating to transmitting and receiving.
    pub fn split(self) -> (UartTx<'d, M>, UartRx<'d, M>) {
        (self.tx, self.rx)
    }

    /// Split the UART into a transmitter and receiver by mutable reference.
    pub fn split_ref(&mut self) -> (&mut UartTx<'d, M>, &mut UartRx<'d, M>) {
        (&mut self.tx, &mut self.rx)
    }
}

impl<'d> Uart<'d, Async> {
    /// Write bytes, waiting until they are all in the TX FIFO.
    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        self.tx.write(buffer).await
    }

    /// Wait until all data has been transmitted.
    pub async fn flush(&mut self) -> Result<(), Error> {
        self.tx.flush().await
    }

    /// Read bytes, waiting until the buffer is full.
    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.rx.read(buffer).await
    }
}

impl<'d> UartTx<'d, Blocking> {
    /// Create a new blocking TX-only UART driver.
    pub fn new_blocking<T: Instance>(_uart: Peri<'d, T>, tx: Peri<'d, impl TxPin<T>>, config: Config) -> Self {
        Self::new_inner(T::info(), T::state(), tx.into(), config)
    }
}

impl<'d> UartTx<'d, Async> {
    /// Create a new async TX-only UART driver.
    pub fn new<T: Instance>(
        _uart: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        irqs: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        let tx = Self::new_inner(T::info(), T::state(), tx.into(), config);
        enable_interrupt::<T>(&irqs);
        tx
    }

    /// Write bytes, waiting until they are all in the TX FIFO.
    pub async fn write(&mut self, mut buffer: &[u8]) -> Result<(), Error> {
        while !buffer.is_empty() {
            let n = self.write_some(buffer).await;
            buffer = &buffer[n..];
        }
        Ok(())
    }

    /// Write as many bytes as possible to the TX FIFO, waiting until at least one could
    /// be written. Return the number of written bytes.
    async fn write_some(&mut self, buffer: &[u8]) -> usize {
        poll_fn(|cx| {
            self.state.tx_waker.register(cx.waker());
            let n = self.info.write_fifo(buffer);
            if n > 0 {
                return Poll::Ready(n);
            }
            self.info.unmask(INT_UTX_FIFO);
            // Check again, in case the FIFO was emptied before unmasking.
            if self.info.tx_fifo_free() > 0 {
                self.info.mask(INT_UTX_FIFO);
                cx.waker().wake_by_ref();
            }
            Poll::Pending
        })
        .await
    }

    /// Wait until all data has been transmitted.
    pub async fn flush(&mut self) -> Result<(), Error> {
        let regs = self.info.regs();
        self.info.set_tx_threshold(TX_THRESHOLD_FLUSH);
        // Restore the write threshold even if this future is cancelled.
        let info = self.info;
        let _restore = OnDrop::new(move || info.set_tx_threshold(TX_THRESHOLD_WRITE));
        poll_fn(|cx| {
            self.state.tx_waker.register(cx.waker());
            if self.info.tx_fifo_free() >= FIFO_DEPTH {
                return Poll::Ready(());
            }
            self.info.unmask(INT_UTX_FIFO);
            if self.info.tx_fifo_free() >= FIFO_DEPTH {
                self.info.mask(INT_UTX_FIFO);
                cx.waker().wake_by_ref();
            }
            Poll::Pending
        })
        .await;
        // The last character is still being shifted out, this takes at most one frame.
        // There is no interrupt for this, so yield to other tasks while polling.
        poll_fn(|cx| {
            if regs.status().get().utx_bus_busy().get() == 0 {
                Poll::Ready(())
            } else {
                cx.waker().wake_by_ref();
                Poll::Pending
            }
        })
        .await;
        Ok(())
    }
}

impl<'d, M: Mode> UartTx<'d, M> {
    fn new_inner(info: &'static Info, state: &'static State, tx: Peri<'d, AnyPin>, config: Config) -> Self {
        state.refs.store(1, Ordering::Relaxed);
        attach_pin(info.tx_function(), &tx);
        configure(info, &config, true, false);
        Self {
            info,
            state,
            tx: Some(tx),
            _mode: PhantomData,
        }
    }

    /// Write bytes, blocking until they are all in the TX FIFO.
    pub fn blocking_write(&mut self, mut buffer: &[u8]) -> Result<(), Error> {
        while !buffer.is_empty() {
            let n = self.info.write_fifo(buffer);
            buffer = &buffer[n..];
        }
        Ok(())
    }

    /// Block until all data has been transmitted.
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        let regs = self.info.regs();
        while self.info.tx_fifo_free() < FIFO_DEPTH {}
        while regs.status().get().utx_bus_busy().get() != 0 {}
        Ok(())
    }

    /// Return true if the transmitter is busy sending data.
    pub fn busy(&self) -> bool {
        self.info.tx_fifo_free() < FIFO_DEPTH || self.info.regs().status().get().utx_bus_busy().get() != 0
    }
}

impl<M: Mode> Drop for UartTx<'_, M> {
    fn drop(&mut self) {
        // Let the bytes still in the FIFO be transmitted before disabling.
        let _ = self.blocking_flush();
        self.info.mask(INT_TX);
        self.info.regs().utx_cfg().modify(|reg| reg.en().clear());
        if let Some(tx) = &self.tx {
            detach_pin(tx);
        }
        release(self.info, self.state);
    }
}

impl<'d> UartRx<'d, Blocking> {
    /// Create a new blocking RX-only UART driver.
    pub fn new_blocking<T: Instance>(_uart: Peri<'d, T>, rx: Peri<'d, impl RxPin<T>>, config: Config) -> Self {
        Self::new_inner(T::info(), T::state(), rx.into(), config)
    }
}

impl<'d> UartRx<'d, Async> {
    /// Create a new async RX-only UART driver.
    pub fn new<T: Instance>(
        _uart: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        irqs: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        let rx = Self::new_inner(T::info(), T::state(), rx.into(), config);
        enable_interrupt::<T>(&irqs);
        rx
    }

    /// Read bytes, waiting until the buffer is full.
    pub async fn read(&mut self, mut buffer: &mut [u8]) -> Result<(), Error> {
        while !buffer.is_empty() {
            let n = self.read_some(buffer).await?;
            buffer = &mut buffer[n..];
        }
        Ok(())
    }

    /// Read the available bytes, waiting until at least one is received. Return the
    /// number of read bytes.
    async fn read_some(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        let regs = self.info.regs();
        poll_fn(|cx| {
            self.state.rx_waker.register(cx.waker());
            let n = self.info.read_fifo(buffer)?;
            if n > 0 {
                return Poll::Ready(Ok(n));
            }
            // Clear the sticky interrupts before unmasking.
            regs.int_clear().set(INT_URX_RTO | INT_URX_END);
            self.info.unmask(INT_URX_FIFO | INT_URX_RTO | INT_URX_PCE);
            if self.info.rx_fifo_count() > 0 {
                self.info.mask(INT_RX);
                cx.waker().wake_by_ref();
            }
            Poll::Pending
        })
        .await
    }
}

impl<'d, M: Mode> UartRx<'d, M> {
    fn new_inner(info: &'static Info, state: &'static State, rx: Peri<'d, AnyPin>, config: Config) -> Self {
        state.refs.store(1, Ordering::Relaxed);
        attach_pin(info.rx_function(), &rx);
        configure(info, &config, false, true);
        Self {
            info,
            state,
            rx: Some(rx),
            _mode: PhantomData,
        }
    }

    /// Read bytes, blocking until the buffer is full.
    pub fn blocking_read(&mut self, mut buffer: &mut [u8]) -> Result<(), Error> {
        while !buffer.is_empty() {
            let n = self.info.read_fifo(buffer)?;
            buffer = &mut buffer[n..];
        }
        Ok(())
    }
}

impl<M: Mode> Drop for UartRx<'_, M> {
    fn drop(&mut self) {
        self.info.mask(INT_RX);
        self.info.regs().urx_cfg().modify(|reg| reg.en().clear());
        if let Some(rx) = &self.rx {
            detach_pin(rx);
        }
        release(self.info, self.state);
    }
}

/// Register the handler and enable the interrupt of the given instance.
fn enable_interrupt<T: Instance>(irqs: &impl Binding<T::Interrupt, InterruptHandler<T>>) {
    T::Interrupt::register(irqs);
    T::Interrupt::unpend();
    // SAFETY: The handler is registered, and all UART interrupt sources are masked.
    unsafe { T::Interrupt::enable() };
}

/// Configure the UART registers with the HAL initialization, then set up the interrupts
/// and FIFO thresholds used by this driver.
fn configure(info: &Info, config: &Config, enable_tx: bool, enable_rx: bool) {
    let regs = info.regs();

    let mut hal_config = UartConfig::new(config.baudrate);
    hal_config.data_bits = config.data_bits;
    hal_config.stop_bits = config.stop_bits;
    hal_config.parity = config.parity;
    hal_config.flow_control_cts = config.cts;
    // Wake up as soon as one byte is received, see TX_THRESHOLD_* for TX.
    hal_config.tx_fifo_threshold = TX_THRESHOLD_WRITE as u8;
    hal_config.rx_fifo_threshold = 0;

    // This leaves all interrupt sources masked.
    bflb_hal::uart::init_regs(regs, &hal_config, enable_tx, enable_rx);

    regs.int_en().set(INT_ALL);
    regs.int_clear().set(INT_ALL);
}

/// Route the signal of the given pin to the given UART function and configure the pin.
fn attach_pin(function: UartFunction, pin: &AnyPin) {
    let num = gpio::Pin::pin(pin);
    critical_section::with(|_| bflb_hal::uart::attach_signal(num, function));
    gpio::set_function(num, FUNC_UART, Pull::Up, Drive::Drive1);
}

/// Disconnect the signal of the given pin and reset the pin.
fn detach_pin(pin: &AnyPin) {
    let num = gpio::Pin::pin(pin);
    critical_section::with(|_| bflb_hal::uart::detach_signal(num));
    gpio::reset_function(num);
}

/// Shared state of an UART instance.
pub(crate) struct State {
    tx_waker: AtomicWaker,
    rx_waker: AtomicWaker,
    /// Number of living drivers (TX and RX halves) for this instance.
    refs: AtomicU8,
}

impl State {
    pub(crate) const fn new() -> Self {
        Self {
            tx_waker: AtomicWaker::new(),
            rx_waker: AtomicWaker::new(),
            refs: AtomicU8::new(0),
        }
    }
}

/// Type-erased information about an UART instance.
pub(crate) struct Info {
    /// Register block.
    pub(crate) regs: Regs,
    /// Port number, selecting the UART functions that pins can be routed to.
    pub(crate) port: u8,
    /// Interrupt of the instance.
    pub(crate) interrupt: Interrupt,
}

impl Info {
    #[inline]
    fn regs(&self) -> Regs {
        self.regs
    }

    #[inline]
    fn tx_function(&self) -> UartFunction {
        UartFunction::tx(self.port)
    }

    #[inline]
    fn rx_function(&self) -> UartFunction {
        UartFunction::rx(self.port)
    }

    #[inline]
    fn set_tx_threshold(&self, threshold: u32) {
        self.regs().fifo_cfg1().modify(|reg| reg.tx_fifo_th().set(threshold));
    }

    #[inline]
    fn mask(&self, bits: u32) {
        critical_section::with(|_| self.regs().int_mask().modify(|reg| *reg |= bits));
    }

    #[inline]
    fn unmask(&self, bits: u32) {
        critical_section::with(|_| self.regs().int_mask().modify(|reg| *reg &= !bits));
    }

    #[inline]
    fn tx_fifo_free(&self) -> u32 {
        self.regs().fifo_cfg1().get().tx_fifo_count().get()
    }

    #[inline]
    fn rx_fifo_count(&self) -> u32 {
        self.regs().fifo_cfg1().get().rx_fifo_count().get()
    }

    /// Write as many bytes as possible to the TX FIFO, without blocking.
    fn write_fifo(&self, buffer: &[u8]) -> usize {
        let regs = self.regs();
        let n = buffer.len().min(self.tx_fifo_free() as usize);
        for &byte in &buffer[..n] {
            regs.fifo_wdata().set(byte);
        }
        n
    }

    /// Read as many bytes as possible from the RX FIFO, without blocking.
    fn read_fifo(&self, buffer: &mut [u8]) -> Result<usize, Error> {
        let regs = self.regs();

        // The count is sampled before checking the error flags: if no error is flagged
        // after that, all the counted bytes were received without error.
        let count = self.rx_fifo_count();

        // On overflow, received bytes have been dropped, so the stream is broken and the
        // error is reported immediately. Clearing the FIFO also clears the flag.
        if regs.fifo_cfg0().get().rx_fifo_overflow().get() != 0 {
            regs.fifo_cfg0().modify(|reg| reg.rx_fifo_clear().fill());
            return Err(Error::Overrun);
        }

        // The parity error flag isn't associated with a particular byte of the FIFO, so
        // all the bytes received so far are discarded, the corrupted one being among them.
        if regs.int_sts().get() & INT_URX_PCE != 0 {
            regs.fifo_cfg0().modify(|reg| reg.rx_fifo_clear().fill());
            regs.int_clear().set(INT_URX_PCE);
            return Err(Error::Parity);
        }

        let n = buffer.len().min(count as usize);
        for byte in &mut buffer[..n] {
            *byte = regs.fifo_rdata().get();
        }
        Ok(n)
    }
}

/// Release one driver of an instance, disabling the interrupt with the last one.
fn release(info: &Info, state: &State) {
    if state.refs.fetch_sub(1, Ordering::AcqRel) == 1 {
        info.interrupt.disable();
    }
}

pub(crate) trait SealedInstance {
    fn info() -> &'static Info;
    fn state() -> &'static State;
}

/// UART instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {
    /// Interrupt for this instance.
    type Interrupt: interrupt::typelevel::Interrupt;
}

/// Trait for TX pins.
pub trait TxPin<T: Instance>: gpio::Pin {}
/// Trait for RX pins.
pub trait RxPin<T: Instance>: gpio::Pin {}

// Any pin can be routed to any UART function.
impl<T: Instance, P: gpio::Pin> TxPin<T> for P {}
impl<T: Instance, P: gpio::Pin> RxPin<T> for P {}

macro_rules! impl_uart {
    ($type:ident, $pac_type:ident, $irq:ident, $port:expr) => {
        impl crate::uart::SealedInstance for peripherals::$type {
            fn info() -> &'static crate::uart::Info {
                static INFO: crate::uart::Info = crate::uart::Info {
                    regs: pac::$pac_type,
                    port: $port,
                    interrupt: crate::interrupt::$irq,
                };
                &INFO
            }
            fn state() -> &'static crate::uart::State {
                static STATE: crate::uart::State = crate::uart::State::new();
                &STATE
            }
        }
        impl crate::uart::Instance for peripherals::$type {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
}

// ====================== embedded-io ====================== //

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Self::Overrun => embedded_io::ErrorKind::Other,
            Self::Parity => embedded_io::ErrorKind::InvalidData,
        }
    }
}

impl<M: Mode> embedded_io::ErrorType for Uart<'_, M> {
    type Error = Error;
}

impl<M: Mode> embedded_io::ErrorType for UartTx<'_, M> {
    type Error = Error;
}

impl<M: Mode> embedded_io::ErrorType for UartRx<'_, M> {
    type Error = Error;
}

impl<M: Mode> embedded_io::Write for UartTx<'_, M> {
    fn write(&mut self, buffer: &[u8]) -> Result<usize, Self::Error> {
        if buffer.is_empty() {
            return Ok(0);
        }
        loop {
            let n = self.info.write_fifo(buffer);
            if n > 0 {
                return Ok(n);
            }
        }
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.blocking_flush()
    }
}

impl<M: Mode> embedded_io::WriteReady for UartTx<'_, M> {
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.info.tx_fifo_free() > 0)
    }
}

impl<M: Mode> embedded_io::Read for UartRx<'_, M> {
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        if buffer.is_empty() {
            return Ok(0);
        }
        loop {
            let n = self.info.read_fifo(buffer)?;
            if n > 0 {
                return Ok(n);
            }
        }
    }
}

impl<M: Mode> embedded_io::ReadReady for UartRx<'_, M> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.info.rx_fifo_count() > 0)
    }
}

impl<M: Mode> embedded_io::Write for Uart<'_, M> {
    fn write(&mut self, buffer: &[u8]) -> Result<usize, Self::Error> {
        embedded_io::Write::write(&mut self.tx, buffer)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io::Write::flush(&mut self.tx)
    }
}

impl<M: Mode> embedded_io::WriteReady for Uart<'_, M> {
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        embedded_io::WriteReady::write_ready(&mut self.tx)
    }
}

impl<M: Mode> embedded_io::Read for Uart<'_, M> {
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io::Read::read(&mut self.rx, buffer)
    }
}

impl<M: Mode> embedded_io::ReadReady for Uart<'_, M> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        embedded_io::ReadReady::read_ready(&mut self.rx)
    }
}

impl embedded_io_async::Write for UartTx<'_, Async> {
    async fn write(&mut self, buffer: &[u8]) -> Result<usize, Self::Error> {
        if buffer.is_empty() {
            return Ok(0);
        }
        Ok(self.write_some(buffer).await)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        UartTx::flush(self).await
    }
}

impl embedded_io_async::Read for UartRx<'_, Async> {
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        if buffer.is_empty() {
            return Ok(0);
        }
        self.read_some(buffer).await
    }
}

impl embedded_io_async::Write for Uart<'_, Async> {
    async fn write(&mut self, buffer: &[u8]) -> Result<usize, Self::Error> {
        embedded_io_async::Write::write(&mut self.tx, buffer).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io_async::Write::flush(&mut self.tx).await
    }
}

impl embedded_io_async::Read for Uart<'_, Async> {
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io_async::Read::read(&mut self.rx, buffer).await
    }
}

impl<M: Mode> core::fmt::Write for UartTx<'_, M> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.blocking_write(s.as_bytes()).map_err(|_| core::fmt::Error)
    }
}

impl<M: Mode> core::fmt::Write for Uart<'_, M> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.tx.write_str(s)
    }
}
