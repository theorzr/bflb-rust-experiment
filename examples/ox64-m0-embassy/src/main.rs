#![no_std]
#![no_main]

use bflb_embassy::gpio::{Level, Output};
use bflb_embassy::uart::{self, Uart};
use bflb_embassy::{bind_interrupts, peripherals};
use embassy_executor::{Executor, Spawner};
use embassy_time::{Duration, Ticker};
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;

// Link the runtime, which calls `main` after initializing the HAL.
use bflb_rt as _;

bind_interrupts!(struct Irqs {
    UART0 => uart::InterruptHandler<peripherals::UART0>;
});

#[unsafe(no_mangle)]
pub fn main() {
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    EXECUTOR
        .init(Executor::new())
        .run(|spawner| spawner.spawn(run(spawner).unwrap()));
}

#[embassy_executor::task]
async fn run(spawner: Spawner) {
    let p = bflb_embassy::init(Default::default());

    let led = Output::new(p.PIN_8, Level::Low);
    spawner.spawn(blink(led).unwrap());

    let mut uart = Uart::new(p.UART0, p.PIN_14, p.PIN_15, Irqs, Default::default());
    let _ = uart
        .write_all(b"\nHello from Embassy! Type something...\n")
        .await;

    // Echo everything received.
    let mut buf = [0u8; 32];
    loop {
        match Read::read(&mut uart, &mut buf).await {
            Ok(n) => {
                let _ = uart.write_all(&buf[..n]).await;
            }
            Err(e) => {
                let _ = core::fmt::Write::write_fmt(&mut uart, format_args!("\nerror: {e}\n"));
            }
        }
    }
}

#[embassy_executor::task]
async fn blink(mut led: Output<'static>) {
    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        led.toggle();
        ticker.next().await;
    }
}
