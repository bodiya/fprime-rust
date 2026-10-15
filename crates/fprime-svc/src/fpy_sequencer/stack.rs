//! The sequence stack — port of `FpySequencer::Stack`
//! (`FpySequencerStack.cpp`).
//!
//! A byte array of [`MAX_STACK_SIZE`] bytes with a size (the top) and the
//! start of the current function's frame. Typed values are stored
//! big-endian; byte arrays are copied as they are. Every accessor asserts
//! its bounds (C++ `FW_ASSERT`): the directive handlers check the stack
//! before touching it, so an out-of-bounds access here is a programmer
//! error.

use super::types::StackSizeType;
use fprime_config::fpy_sequencer::MAX_STACK_SIZE;
use fprime_fw::fw_assert;

/// A fixed-width value that can live on the stack (C++ `Stack::push<T>` /
/// `pop<T>` are instantiated for U8..U64, I8..I64, F32 and F64).
pub trait StackValue: Copy {
    /// Size in bytes.
    const SIZE: usize;
    /// Write the big-endian bytes into `out` (`SIZE` bytes long).
    fn to_be(self, out: &mut [u8]);
    /// Read a value from `SIZE` big-endian bytes.
    fn from_be(bytes: &[u8]) -> Self;
}

macro_rules! stack_value {
    ($($ty:ty),*) => {$(
        impl StackValue for $ty {
            const SIZE: usize = size_of::<$ty>();
            fn to_be(self, out: &mut [u8]) {
                out.copy_from_slice(&self.to_be_bytes());
            }
            fn from_be(bytes: &[u8]) -> Self {
                let mut raw = [0u8; size_of::<$ty>()];
                raw.copy_from_slice(bytes);
                <$ty>::from_be_bytes(raw)
            }
        }
    )*};
}

// F32/F64 go through their bit patterns, as the C++ specializations do via
// the U32/U64 paths.
stack_value!(u8, u16, u32, u64, i8, i16, i32, i64, f32, f64);

/// C++ `FpySequencer::Stack`.
#[derive(Debug)]
pub struct Stack {
    /// The program stack: local variables, operands and call frames
    /// (allocated once).
    pub bytes: Box<[u8]>,
    /// How many bytes high the stack is.
    pub size: StackSizeType,
    /// Where the current function's locals begin (the frame pointer).
    pub current_frame_start: StackSizeType,
}

impl Stack {
    /// A zeroed, empty stack. Allocates [`MAX_STACK_SIZE`] bytes.
    #[must_use]
    pub fn new() -> Self {
        Self {
            bytes: vec![0u8; MAX_STACK_SIZE].into_boxed_slice(),
            size: 0,
            current_frame_start: 0,
        }
    }

    /// Back to a fresh stack (zeroed, empty) without reallocating — what
    /// the C++ `Runtime` reconstruction does to its stack member.
    pub fn reset(&mut self) {
        self.bytes.fill(0);
        self.size = 0;
        self.current_frame_start = 0;
    }

    /// Pop a value off the top, converting from big-endian (C++
    /// `pop<T>`). Asserts that the stack holds at least `T::SIZE` bytes.
    pub fn pop<T: StackValue>(&mut self) -> T {
        let size = self.size as usize;
        fw_assert!(size >= T::SIZE, size as i32, T::SIZE as i32);
        let value = T::from_be(&self.bytes[size - T::SIZE..size]);
        self.size -= T::SIZE as StackSizeType;
        value
    }

    /// Push a value onto the top, big-endian (C++ `push<T>`). Asserts that
    /// it fits.
    pub fn push<T: StackValue>(&mut self, value: T) {
        let size = self.size as usize;
        fw_assert!(
            size + T::SIZE <= MAX_STACK_SIZE,
            size as i32,
            T::SIZE as i32
        );
        value.to_be(&mut self.bytes[size..size + T::SIZE]);
        self.size += T::SIZE as StackSizeType;
    }

    /// Pop `dest.len()` bytes off the top into `dest`, unconverted (C++
    /// `pop(U8*, size)`).
    pub fn pop_bytes(&mut self, dest: &mut [u8]) {
        let size = self.size as usize;
        let n = dest.len();
        fw_assert!(size >= n, size as i32, n as i32);
        dest.copy_from_slice(&self.bytes[size - n..size]);
        self.size -= n as StackSizeType;
    }

    /// Push `src` onto the top, unconverted (C++ `push(const U8*, size)`).
    pub fn push_bytes(&mut self, src: &[u8]) {
        let size = self.size as usize;
        let n = src.len();
        fw_assert!(size + n <= MAX_STACK_SIZE, size as i32, n as i32);
        self.bytes[size..size + n].copy_from_slice(src);
        self.size += n as StackSizeType;
    }

    /// Push `byte_count` zero bytes (C++ `pushZeroes`).
    pub fn push_zeroes(&mut self, byte_count: StackSizeType) {
        let size = self.size as usize;
        let n = byte_count as usize;
        fw_assert!(size + n <= MAX_STACK_SIZE, size as i32, n as i32);
        self.bytes[size..size + n].fill(0);
        self.size += byte_count;
    }

    /// The offset of the next unused byte (C++ `top()`, a pointer there).
    #[must_use]
    pub fn top(&self) -> usize {
        self.size as usize
    }

    /// Copy `copy_size` bytes from `src_offset` to `dest_offset`. Asserts
    /// that both regions are in the array and do not overlap; does not
    /// change the size (C++ `copy`).
    pub fn copy(
        &mut self,
        dest_offset: StackSizeType,
        src_offset: StackSizeType,
        copy_size: StackSizeType,
    ) {
        let (dest, src, n) = (
            u64::from(dest_offset),
            u64::from(src_offset),
            u64::from(copy_size),
        );
        let max = MAX_STACK_SIZE as u64;
        fw_assert!(dest + n <= max, dest_offset as i32, copy_size as i32);
        fw_assert!(src + n <= max, src_offset as i32, copy_size as i32);
        fw_assert!(
            n == 0 || dest + n <= src || src + n <= dest,
            dest_offset as i32,
            src_offset as i32
        );
        if n > 0 {
            let (dest, src, n) = (dest as usize, src as usize, n as usize);
            self.bytes.copy_within(src..src + n, dest);
        }
    }

    /// Move `move_size` bytes from `src_offset` to `dest_offset`; the
    /// regions may overlap. Asserts that the destination is in the array
    /// and the source is below the top; does not change the size (C++
    /// `move`).
    pub fn r#move(
        &mut self,
        dest_offset: StackSizeType,
        src_offset: StackSizeType,
        move_size: StackSizeType,
    ) {
        let (dest, src, n) = (
            u64::from(dest_offset),
            u64::from(src_offset),
            u64::from(move_size),
        );
        fw_assert!(
            dest + n <= MAX_STACK_SIZE as u64,
            dest_offset as i32,
            move_size as i32
        );
        fw_assert!(
            src + n <= u64::from(self.size),
            src_offset as i32,
            move_size as i32
        );
        if n > 0 {
            let (dest, src, n) = (dest as usize, src as usize, n as usize);
            self.bytes.copy_within(src..src + n, dest);
        }
    }
}

impl Default for Stack {
    fn default() -> Self {
        Self::new()
    }
}
