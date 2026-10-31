//! Main real time clock management, can be used for both synchronized and callback-based
//! waiting. This module is

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::CriticalSection;
use smallvec::SmallVec;

use crate::arch::bl808::addr;
use crate::clock;
use crate::hart::HartLocalCell;
use crate::interrupt::MACHINE_TIMER;

/// The tick frequency of the core timer.
const FREQ: u32 = 1_000_000;

/// The value to set to time cmp in order to disable it (it's still
/// theoretically enabled but set to the maximum u64 micros, which
/// equals 584'868 years).
const DISABLED_TIME_CMP: u64 = u64::MAX;

#[cfg(any(feature = "bl808-m0", feature = "bl808-lp"))]
const RV32_MTIME: *mut u64 = addr::T_HEAD_RV32_MTIME_BASE as _;

#[cfg(any(feature = "bl808-m0", feature = "bl808-lp"))]
const RV32_MTIMECMP: *mut u64 = addr::T_HEAD_RV32_MTIMECMP_BASE as _;

#[cfg(feature = "bl808-d0")]
const RV64_MTIMECMP: *mut u64 = addr::T_HEAD_RV64_MTIMECMP0_BASE as _;

/// Internal function to initialize the timer clock.
pub(crate) fn init() {
    let divider = clock::get_mtimer_source_freq() / FREQ;

    // SAFETY: This is called at the HAL init, our cores (M0/D0/LP) have only one hart so
    // they should be the only one enabling their own RTC clock.
    unsafe {
        clock::enable_mtimer_clock(divider);
    }

    // SAFETY: We are in the init phase and each CPU has its own timer interrupt. We need
    // this interrupt to be able to use the wait_callback function.
    unsafe {
        MACHINE_TIMER.set_enabled(true);
    }
}

/// Get the current time in microseconds.
#[inline]
pub fn get_time() -> u64 {
    #[cfg(feature = "bl808-d0")]
    unsafe {
        let time: u64;
        core::arch::asm!("csrr {}, 0xC01", out(reg) time);
        time
    }
    #[cfg(any(feature = "bl808-m0", feature = "bl808-lp"))]
    unsafe {
        // The 64-bit register is read in two halves, the high half is read again to
        // detect a carry from the low half between the two reads, and retry if so.
        let ptr = RV32_MTIME as *const u32;
        loop {
            let hi = ptr.add(1).read_volatile();
            let lo = ptr.read_volatile();
            if ptr.add(1).read_volatile() == hi {
                break ((hi as u64) << 32) | lo as u64;
            }
        }
    }
}

/// Set the time in microseconds. This function is unsafe because this may (and will)
/// break the currently waiting callbacks.
#[inline]
pub unsafe fn set_time(time: u64) {
    #[cfg(feature = "bl808-d0")]
    unsafe {
        core::arch::asm!("csrw 0xC01, {}", in(reg) time);
    }
    #[cfg(any(feature = "bl808-m0", feature = "bl808-lp"))]
    unsafe {
        RV32_MTIME.write_volatile(time)
    }
}

/// Get the time compare in microseconds.
#[inline]
pub fn get_time_cmp() -> u64 {
    #[cfg(feature = "bl808-d0")]
    unsafe {
        RV64_MTIMECMP.read_volatile()
    }
    #[cfg(any(feature = "bl808-m0", feature = "bl808-lp"))]
    critical_section::with(|_| unsafe {
        // The 64-bit register is read in two halves, interrupts are disabled so that
        // the handler cannot rewrite it between the two reads.
        let ptr = RV32_MTIMECMP as *const u32;
        let lo = ptr.read_volatile();
        let hi = ptr.add(1).read_volatile();
        ((hi as u64) << 32) | lo as u64
    })
}

/// Set the time compare in microseconds. A machine timer interrupt
/// will be triggered whenever the time is greater or equal to this
/// value.
///
/// *Note that you might need to update this value in order to
/// reset the interrupt pending bit. Only resetting the time will
/// not clear this bit.*
///
/// This function is unsafe because the time compare is used by the callback queue of
/// this module, changing it outside of it will break waiting callbacks.
#[inline]
pub unsafe fn set_time_cmp(cmp: u64) {
    #[cfg(feature = "bl808-d0")]
    unsafe {
        RV64_MTIMECMP.write_volatile(cmp)
    }
    #[cfg(any(feature = "bl808-m0", feature = "bl808-lp"))]
    unsafe {
        // The 64-bit register is written in two halves, the high half is first set to
        // its maximum so that no spurious interrupt is triggered between the writes.
        let ptr = RV32_MTIMECMP as *mut u32;
        ptr.add(1).write_volatile(u32::MAX);
        ptr.write_volatile(cmp as u32);
        ptr.add(1).write_volatile((cmp >> 32) as u32);
    }
}

/// State of the callback after being called or not.
#[derive(Debug, Clone, Copy)]
enum TimerCallbackState {
    /// The callback has been consumed and should be removed from the queue.
    Consumed,
    /// The callback has been updated so it should be reordered in the queue.
    Updated,
    /// The callback is still pending for target time.
    Waiting,
}

/// Timer callback abstraction.
trait TimerCallback: Send {
    /// Return the target time.
    fn target_time(&self) -> u64;

    /// Try to call it, returning its (new) state.
    fn call(&mut self, time: u64) -> TimerCallbackState;
}

/// Descriptor for a callback and when to call it.
struct TimerCallbackImpl<F> {
    /// The target time for calling this callback.
    target_time: u64,
    /// The actual callback to call.
    callback: F,
}

impl<F> TimerCallback for TimerCallbackImpl<F>
where
    F: FnMut() -> Option<u64>,
    F: Send + 'static,
{
    #[inline]
    fn target_time(&self) -> u64 {
        self.target_time
    }

    #[inline]
    fn call(&mut self, time: u64) -> TimerCallbackState {
        if self.target_time <= time {
            if let Some(duration) = (self.callback)() {
                self.target_time = time + duration;
                TimerCallbackState::Updated
            } else {
                TimerCallbackState::Consumed
            }
        } else {
            TimerCallbackState::Waiting
        }
    }
}

/// Set when the machine timer has been taken over by another timer implementation, in
/// which case the callback queue is no longer serviced.
static CALLBACKS_DISABLED: AtomicBool = AtomicBool::new(false);

/// Disable the callback-based API of this module, [`wait_callback`] will panic after
/// this. This must be called by any implementation that takes ownership of the machine
/// timer (interrupt handler and time compare register), such as an async time driver.
///
/// This function is unsafe because callbacks already waiting will never be called.
#[inline]
pub unsafe fn disable_callbacks() {
    CALLBACKS_DISABLED.store(true, Ordering::Relaxed);
}

/// The ordered queue of callbacks. It's hart local because timer interrupts are.
static CALLBACK_QUEUE: HartLocalCell<VecDeque<Box<dyn TimerCallback>>> =
    HartLocalCell::new_cell(VecDeque::new());

/// Internal function to insert the given callback into the queue. **This function
/// requires to be executed in an interrupt-free context to avoid deadlocking.**
fn insert_callback(queue: &mut VecDeque<Box<dyn TimerCallback>>, callback: Box<dyn TimerCallback>) {
    let target_time = callback.target_time();

    // Search for the index to insert at.
    let insert_idx = match queue.binary_search_by(|desc| desc.target_time().cmp(&target_time)) {
        Ok(idx) => idx,
        Err(idx) => idx,
    };

    // The first queue element is the next one to be called.
    if insert_idx == 0 {
        // SAFETY: We are the callback queue.
        unsafe {
            set_time_cmp(target_time);
        }
    }

    queue.insert(insert_idx, callback);
}

/// Synchronized wait, this function will block the current thread until the given
/// duration (micros) has been waited. Prefer [`wait_callback`] or [`repeat_callback`]
/// to use an interrupt-driven callback.
#[inline]
pub fn wait(duration: u64) {
    let start = get_time();
    while get_time() - start < duration {}
}

/// Wait for the given duration and then call the given callback. Note that it will be
/// called in an interrupt-free context. The callback can return `None` to just be
/// consumed and removed from the queue, but it can also return `Some` new duration to
/// be called again in the future (returning 0 means that it will be called on next int).
///
/// # Panics
///
/// Panics if the callbacks have been disabled by [`disable_callbacks`].
pub fn wait_callback<F>(duration: u64, callback: F)
where
    F: FnMut() -> Option<u64>,
    F: Send + 'static,
{
    assert!(
        !CALLBACKS_DISABLED.load(Ordering::Relaxed),
        "the machine timer is owned by another time driver, callbacks are disabled"
    );
    critical_section::with(|cs| {
        let mut queue = CALLBACK_QUEUE.borrow_ref_mut(cs);
        insert_callback(
            &mut queue,
            Box::new(TimerCallbackImpl {
                target_time: get_time() + duration,
                callback,
            }),
        )
    });
}

/// This handler is called when the core time reaches the time cmp register.
pub(crate) fn mtimer_handler(_code: usize, cs: CriticalSection) {
    let mut queue = CALLBACK_QUEUE.borrow_ref_mut(cs);
    let time = get_time();

    // Temporary array of updated callbacks.
    let mut updated_callbacks = SmallVec::<[_; 4]>::new();

    // Call every callback that reached their target time and take the index of
    // the last consumed one.
    while let Some(callback) = queue.front_mut() {
        match callback.call(time) {
            TimerCallbackState::Consumed => {
                queue.pop_front();
            }
            TimerCallbackState::Updated => {
                updated_callbacks.push(queue.pop_front().unwrap());
            }
            TimerCallbackState::Waiting => {
                // Callbacks are ordered in the queue so we can just break here.
                break;
            }
        }
    }

    // Consume the update callbacks vector and insert them again.
    for update_callback in updated_callbacks {
        insert_callback(&mut queue, update_callback);
    }

    // Then we get the front to update time compare register to the next callback,
    // or just disable it by setting it to the maximum value.
    // SAFETY: We are the callback queue.
    unsafe {
        if let Some(front) = queue.front() {
            set_time_cmp(front.target_time());
        } else {
            set_time_cmp(DISABLED_TIME_CMP);
        }
    }
}
