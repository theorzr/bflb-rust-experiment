//! `embassy-time` driver based on the RISC-V machine timer (mtime/mtimecmp), which is
//! configured by [`bflb_hal`] to tick at 1 MHz.

use core::cell::RefCell;
use core::task::Waker;

use bflb_hal::interrupt::MACHINE_TIMER;
use bflb_hal::time;
use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

struct MtimerDriver {
    queue: Mutex<RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: MtimerDriver = MtimerDriver {
    queue: Mutex::new(RefCell::new(Queue::new())),
});

impl MtimerDriver {
    /// Wake expired timers and program the alarm for the next one.
    fn update_alarm(&self, cs: CriticalSection) {
        let next = self.queue.borrow_ref_mut(cs).next_expiration(self.now());
        // The machine timer interrupt is level-triggered on `mtime >= mtimecmp`, so an
        // alarm set in the past fires immediately, there is no race to handle here.
        // SAFETY: This driver owns the machine timer compare register.
        unsafe { time::set_time_cmp(next) };
    }
}

impl Driver for MtimerDriver {
    #[inline]
    fn now(&self) -> u64 {
        time::get_time()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            if self.queue.borrow_ref_mut(cs).schedule_wake(at, waker) {
                self.update_alarm(cs);
            }
        })
    }
}

fn mtimer_handler(_code: usize, cs: CriticalSection) {
    DRIVER.update_alarm(cs);
}

pub(crate) fn init(cs: CriticalSection) {
    // SAFETY: The time driver takes ownership of the machine timer, replacing the
    // callback queue of the HAL, whose API is disabled so it cannot fight over it.
    unsafe {
        time::disable_callbacks();
        bflb_hal::interrupt::set_handler(MACHINE_TIMER, mtimer_handler, cs);
        MACHINE_TIMER.set_enabled(true);
    }
    // Timers may have been scheduled before init, while the HAL handler was still
    // installed and may have discarded the alarm, so program it from our queue.
    DRIVER.update_alarm(cs);
}
