#![no_std]

use atomic_polyfill::AtomicBool;
use core::cell::RefCell;
use core::mem::{transmute, MaybeUninit};

pub struct OnceMut<T> {
    used: AtomicBool,
    data: RefCell<MaybeUninit<T>>,
}

impl<T> OnceMut<T> {
    pub const fn new() -> Self {
        Self {
            used: AtomicBool::new(false),
            data: RefCell::new(MaybeUninit::uninit()),
        }
    }

    pub fn take<F>(&'static self, func: F) -> Option<&'static mut T>
    where
        F: FnOnce() -> T,
    {
        let result = self.used.compare_exchange(
            false,
            true,
            core::sync::atomic::Ordering::SeqCst,
            core::sync::atomic::Ordering::SeqCst,
        );
        result.ok().map(|_| {
            let mut d = self.data.borrow_mut();
            let d = d.write(func());
            unsafe { transmute::<&mut T, &'static mut T>(d) }
        })
    }
}

unsafe impl<T> Sync for OnceMut<T> {}
