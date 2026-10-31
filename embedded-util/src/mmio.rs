//! Utilities for MMIO structures.


/// This macro can be used to generate MMIO structures. These structures are
/// special because they are actually a single pointer in size, but provides
/// many methods to access in a volatile way the internal registers that are
/// defined with the macro.
/// 
/// Note that field's type is expected to be copy, but is not checked by 
/// this macro. Read the documentation of [`core::ptr::read_volatile`] for
/// more information about the constraints applied to the type.
/// 
/// Example:
/// ```
/// embedded_util::mmio! {
/// 
///     pub struct MmioPeripheral {
///         [0x00] ro version: u32,          // A simple field at 0x00
///         [0x04] ro state: u32,            // Another field at 0x04
///         [0x04] wo set_state: u32,        // Write access to field at 0x04
///         [0x04] ro state_ext: u64,        // Read access to field at 0x04 but as an u64
///         [0x08] sub sub0: MmioSub,         // You can also define sub structures
///         [0x0A] sub sub1: MmioSub,         // ..multiple time.
///         [0x0C] ro bits: MyReg,
///     }
/// 
///     pub struct MmioSub {
///         [0x00] ro a: bool,
///         [0x01] ro b: bool,
///     }
/// 
/// }
/// 
/// embedded_util::reg! {
///     pub struct MyReg: u32 {}
/// }
/// ```
#[macro_export]
macro_rules! mmio {
    (

        $(#[$struct_meta:meta])*
        $vis:vis struct $name:ident {
            $(
                $(#[$field_meta:meta])*
                [$field_index:literal] $field_mode:ident $field_name:ident : $field_type:ty
            ),*
            $(,)?
        }

        $($t:tt)*

    ) => {

        $(#[$struct_meta])*
        #[derive(Clone, Copy, Eq, PartialEq)]
        $vis struct $name(pub *mut u8);

        // SAFETY: This is only an address, accessing the registers is volatile.
        unsafe impl Send for $name {}
        unsafe impl Sync for $name {}

        impl $name {

            #[inline]
            pub const fn new(ptr: *mut u8) -> Self {
                Self(ptr)
            }

            $(
                $crate::__mmio_field!(
                    $field_mode, 
                    $field_index, 
                    $field_name, 
                    $field_type,
                    $($field_meta),*
                );
            )*

        }

        impl core::fmt::Debug for $name {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                f.debug_struct(stringify!($name))
                    .field("@", &self.0)
                    $( 
                    .field(stringify!($field_name), &self.$field_name()) 
                    )*
                    .finish()
            }
        }

        $crate::mmio! {
            $($t)*
        }

    };
    () => {};
}

#[macro_export]
macro_rules! __mmio_field {
    (rw, $field_index:literal, $field_name:ident, $field_type:ty, $($field_meta:meta),*) => {
        $(#[$field_meta])*
        #[must_use]
        #[inline(always)]
        pub fn $field_name(self) -> $crate::PtrRw<$field_type> {
            unsafe { $crate::PtrRw(self.0.add($field_index) as _) }
        }
    };
    (ro, $field_index:literal, $field_name:ident, $field_type:ty, $($field_meta:meta),*) => {
        $(#[$field_meta])*
        #[must_use]
        #[inline(always)]
        pub fn $field_name(self) -> $crate::PtrRo<$field_type> {
            unsafe { $crate::PtrRo(self.0.add($field_index) as _) }
        }
    };
    (wo, $field_index:literal, $field_name:ident, $field_type:ty, $($field_meta:meta),*) => {
        $(#[$field_meta])*
        #[must_use]
        #[inline(always)]
        pub fn $field_name(self) -> $crate::PtrWo<$field_type> {
            unsafe { $crate::PtrWo(self.0.add($field_index) as _) }
        }
    };
    (sub, $field_index:literal, $field_name:ident, $field_type:ty, $($field_meta:meta),*) => {
        $(#[$field_meta])*
        #[must_use]
        #[inline(always)]
        pub const fn $field_name(self) -> $field_type {
            unsafe { <$field_type>::new(self.0.add($field_index)) }
        }
    };
}


use core::fmt::{self, Debug};

/// A read-only pointer to some value in an MMIO struct.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct PtrRo<T: Debug + Copy>(pub *const T);

impl<T: Debug + Copy> PtrRo<T> {

    /// Get the value referenced by the pointer.
    #[inline(always)]
    pub fn get(self) -> T {
        unsafe { self.0.read_volatile() }
    }

}

/// A write-only pointer to some value in an MMIO struct.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PtrWo<T: Debug + Copy>(pub *mut T);

impl<T: Debug + Copy> PtrWo<T> {

    /// Set the value referenced by the pointer.
    #[inline(always)]
    pub fn set(self, val: T) {
        unsafe { self.0.write_volatile(val) }
    }

    /// Set the value by modifying the default value using a function.
    #[inline(always)]
    pub fn set_with<F: FnOnce(&mut T)>(self, func: F)
    where
        T: Default
    {
        let mut val = T::default();
        func(&mut val);
        self.set(val);
    }

}

/// A read/write pointer to some value in an MMIO struct.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct PtrRw<T: Debug + Copy>(pub *mut T);

impl<T: Debug + Copy> PtrRw<T> {

    /// Get the value referenced by the pointer.
    #[inline(always)]
    pub fn get(self) -> T {
        unsafe { self.0.read_volatile() }
    }

    /// Set the value referenced by the pointer.
    #[inline(always)]
    pub fn set(self, val: T) {
        unsafe { self.0.write_volatile(val) }
    }

    /// Set the value by modifying the default value using a function.
    #[inline(always)]
    pub fn set_with<F: FnOnce(&mut T)>(self, func: F)
    where
        T: Default
    {
        let mut val = T::default();
        func(&mut val);
        self.set(val);
    }

    /// Modify the value referenced by the pointer using a function.
    /// 
    /// You can use this together with registers in order to modify
    /// some bit fields in it and then apply changes. For example:
    /// 
    /// ```compile_fail
    /// let reg_ptr: PtrRw<MyReg>;
    /// reg_ptr.modify(|reg| reg.myfield().set(123));
    /// ```
    /// 
    /// *Synchronization is obviously not guarenteed and race 
    /// modifications can happen depending on your case.*
    #[inline(always)]
    pub fn modify<F: FnOnce(&mut T)>(self, func: F) {
        let mut val = self.get();
        func(&mut val);
        self.set(val);
    }

}

impl<T: Debug + Copy> Debug for PtrRo<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("PtrRo").field(&self.0).field(&self.get()).finish()
    }
}

impl<T: Debug + Copy> Debug for PtrRw<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("PtrRw").field(&self.0).field(&self.get()).finish()
    }
}
