//! GPIO loopback test, GPIO16 (pin 32) and GPIO17 (pin 31) must be connected together.
//!
//! The tests run each time `g` is received on UART0 (GPIO14/GPIO15), they print one
//! `PASS`/`FAIL` line per check, then `DONE <passed>/<total>`.

#![no_std]
#![no_main]

use core::fmt::Write as _;

use bflb_embassy::gpio::{Flex, Pull};
use bflb_embassy::uart::{self, Async, Uart};
use bflb_embassy::{bind_interrupts, peripherals};
use embassy_executor::Executor;
use embassy_futures::join::join;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::Read;
use static_cell::StaticCell;

// Link the runtime, which calls `main` after initializing the HAL.
use bflb_rt as _;

bind_interrupts!(struct Irqs {
    UART0 => uart::InterruptHandler<peripherals::UART0>;
});

/// Delay before the other pin changes, a wait must not complete before it.
const DELAY: Duration = Duration::from_millis(20);
/// Timeout of the waits expected to complete.
const TIMEOUT: Duration = Duration::from_millis(500);
/// Timeout of the waits expected to never complete.
const NO_EVENT: Duration = Duration::from_millis(50);

#[unsafe(no_mangle)]
pub fn main() {
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    EXECUTOR
        .init(Executor::new())
        .run(|spawner| spawner.spawn(run().unwrap()));
}

struct Reporter<'a, 'd> {
    uart: &'a mut Uart<'d, Async>,
    passed: u32,
    total: u32,
}

impl Reporter<'_, '_> {
    fn check(&mut self, name: &str, ok: bool, detail: core::fmt::Arguments) {
        self.total += 1;
        self.passed += ok as u32;
        let status = if ok { "PASS" } else { "FAIL" };
        let _ = write!(self.uart, "{status} {name} ({detail})\n");
    }
}

/// Let the level propagate through the input synchronizer.
async fn settle() {
    Timer::after_micros(50).await;
}

/// Wait on `$inp` while `$action` is done after [`DELAY`], the wait must complete after it.
macro_rules! delayed {
    ($r:expr, $name:expr, $inp:expr, $wait:ident, $action:expr) => {{
        let start = Instant::now();
        let (res, _) = join(with_timeout(TIMEOUT, $inp.$wait()), async {
            Timer::after(DELAY).await;
            $action;
        })
        .await;
        let elapsed = start.elapsed();
        $r.check(
            $name,
            res.is_ok() && elapsed >= DELAY,
            format_args!("timeout={} after {} us", res.is_err(), elapsed.as_micros()),
        );
    }};
}

/// The wait on `$inp` must complete immediately.
macro_rules! immediate {
    ($r:expr, $name:expr, $inp:expr, $wait:ident) => {{
        let start = Instant::now();
        let res = with_timeout(NO_EVENT, $inp.$wait()).await;
        let elapsed = start.elapsed();
        $r.check(
            $name,
            res.is_ok() && elapsed < DELAY,
            format_args!("timeout={} after {} us", res.is_err(), elapsed.as_micros()),
        );
    }};
}

/// The wait on `$inp` must not complete.
macro_rules! never {
    ($r:expr, $name:expr, $inp:expr, $wait:ident) => {{
        let res = with_timeout(NO_EVENT, $inp.$wait()).await;
        $r.check(
            $name,
            res.is_err(),
            format_args!("timeout={}", res.is_err()),
        );
    }};
}

/// Drive `out` low, high, low, and check that `inp` follows.
async fn static_levels(
    r: &mut Reporter<'_, '_>,
    name: &str,
    out: &mut Flex<'_>,
    inp: &mut Flex<'_>,
) {
    inp.set_pull(Pull::None);
    inp.set_as_input();
    out.set_low();
    out.set_as_output();
    settle().await;
    let low = inp.is_low();
    out.set_high();
    settle().await;
    let high = inp.is_high();
    out.set_low();
    settle().await;
    let low2 = inp.is_low();
    r.check(
        name,
        low && high && low2,
        format_args!("low={low} high={high} low={low2}"),
    );
    out.set_as_input();
}

async fn run_tests(r: &mut Reporter<'_, '_>, a: &mut Flex<'_>, b: &mut Flex<'_>) {
    // ---- Static levels, in both directions. ----
    static_levels(r, "16->17", a, b).await;
    static_levels(r, "17->16", b, a).await;

    // ---- Output state readback, and toggle driving the pin. ----
    b.set_as_input();
    a.set_as_output();
    a.set_low();
    settle().await;
    let ok1 = a.is_set_low() && b.is_low();
    a.toggle();
    settle().await;
    let ok2 = a.is_set_high() && b.is_high();
    a.toggle();
    settle().await;
    let ok3 = a.is_set_low() && b.is_low();
    r.check(
        "output readback/toggle",
        ok1 && ok2 && ok3,
        format_args!("{ok1} {ok2} {ok3}"),
    );

    // ---- Pulls, with the other pin released. ----
    a.set_as_disconnected();
    b.set_as_input();
    b.set_pull(Pull::Up);
    settle().await;
    let up = b.is_high();
    b.set_pull(Pull::Down);
    settle().await;
    let down = b.is_low();
    r.check(
        "pull up/down",
        up && down,
        format_args!("up={up} down={down}"),
    );
    b.set_pull(Pull::None);

    // From here, GPIO16 drives and GPIO17 waits.
    a.set_low();
    a.set_as_output();
    settle().await;

    // ---- Level waits. ----
    delayed!(r, "wait_for_high", b, wait_for_high, a.set_high());
    immediate!(r, "wait_for_high (already high)", b, wait_for_high);
    delayed!(r, "wait_for_low", b, wait_for_low, a.set_low());
    immediate!(r, "wait_for_low (already low)", b, wait_for_low);

    // ---- Edge waits. ----
    delayed!(
        r,
        "wait_for_rising_edge",
        b,
        wait_for_rising_edge,
        a.set_high()
    );
    never!(
        r,
        "rising edge not re-triggered while high",
        b,
        wait_for_rising_edge
    );
    delayed!(
        r,
        "wait_for_falling_edge",
        b,
        wait_for_falling_edge,
        a.set_low()
    );
    never!(
        r,
        "falling edge not re-triggered while low",
        b,
        wait_for_falling_edge
    );
    delayed!(
        r,
        "wait_for_any_edge (rising)",
        b,
        wait_for_any_edge,
        a.set_high()
    );
    delayed!(
        r,
        "wait_for_any_edge (falling)",
        b,
        wait_for_any_edge,
        a.set_low()
    );
    never!(r, "any edge without change", b, wait_for_any_edge);

    // ---- Cancellation: an edge happening while no wait is armed must not complete a
    // later wait, and a cancelled wait must leave the pin usable. ----
    never!(r, "cancelled rising wait", b, wait_for_rising_edge);
    a.set_high();
    settle().await;
    a.set_low();
    settle().await;
    never!(
        r,
        "stale edge ignored after cancel",
        b,
        wait_for_rising_edge
    );
    delayed!(
        r,
        "rising wait after cancel",
        b,
        wait_for_rising_edge,
        a.set_high()
    );

    // ---- Many fast edges. ----
    let mut fails = 0;
    for _ in 0..200 {
        a.set_low();
        settle().await;
        let (res, _) = join(with_timeout(TIMEOUT, b.wait_for_rising_edge()), async {
            embassy_futures::yield_now().await;
            a.set_high();
        })
        .await;
        fails += res.is_err() as u32;
    }
    r.check(
        "200 fast rising edges",
        fails == 0,
        format_args!("{fails} missed"),
    );

    // ---- Reverse direction for the interrupt path of GPIO16. ----
    b.set_low();
    b.set_as_output();
    a.set_as_input();
    settle().await;
    delayed!(
        r,
        "reverse wait_for_rising_edge",
        a,
        wait_for_rising_edge,
        b.set_high()
    );
    delayed!(
        r,
        "reverse wait_for_falling_edge",
        a,
        wait_for_falling_edge,
        b.set_low()
    );

    // Release both pins.
    a.set_as_disconnected();
    b.set_as_disconnected();
}

#[embassy_executor::task]
async fn run() {
    let p = bflb_embassy::init(Default::default());

    let mut uart = Uart::new(p.UART0, p.PIN_14, p.PIN_15, Irqs, Default::default());
    let mut a = Flex::new(p.PIN_16);
    let mut b = Flex::new(p.PIN_17);

    let _ = uart.write(b"\nGPIO loopback test, send 'g' to run\n").await;

    let mut buf = [0u8; 1];
    loop {
        if Read::read(&mut uart, &mut buf).await != Ok(1) {
            continue;
        }
        if buf[0] != b'g' {
            continue;
        }
        let _ = uart.write(b"START\n").await;
        let mut r = Reporter {
            uart: &mut uart,
            passed: 0,
            total: 0,
        };
        run_tests(&mut r, &mut a, &mut b).await;
        let (passed, total) = (r.passed, r.total);
        let _ = write!(uart, "DONE {passed}/{total}\n");
    }
}
