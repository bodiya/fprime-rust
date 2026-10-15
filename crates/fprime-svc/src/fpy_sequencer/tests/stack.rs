//! The stack (upstream `StackPushPop*`, `StackPushZeroes`, `StackTop`,
//! `StackCopy*`, `StackMove*`, `StackBigEndianByteOrder`,
//! `StackMixedTypes`).

use super::*;

fn round_trip<V: StackValue + PartialEq + std::fmt::Debug>(value: V) {
    let mut stack = Stack::new();
    stack.push(value);
    assert_eq!(stack.size as usize, V::SIZE);
    assert_eq!(stack.pop::<V>(), value);
    assert_eq!(stack.size, 0);
}

#[test]
fn push_pop_every_type() {
    round_trip(0xABu8);
    round_trip(0xABCDu16);
    round_trip(0xDEAD_BEEFu32);
    round_trip(0xDEAD_BEEF_CAFE_BABEu64);
    round_trip(-42i8);
    round_trip(-12345i16);
    round_trip(-123_456_789i32);
    round_trip(-1_234_567_890_123_456_789i64);
    round_trip(1.25e-3f32);
    round_trip(std::f64::consts::E);
}

#[test]
fn push_pop_multiple_is_lifo() {
    let mut stack = Stack::new();
    stack.push(0x1111_1111u32);
    stack.push(0x2222_2222u32);
    stack.push(0x3333_3333u32);
    assert_eq!(stack.size, 12);
    assert_eq!(stack.pop::<u32>(), 0x3333_3333);
    assert_eq!(stack.pop::<u32>(), 0x2222_2222);
    assert_eq!(stack.pop::<u32>(), 0x1111_1111);
    assert_eq!(stack.size, 0);
}

#[test]
fn push_pop_byte_array() {
    let mut stack = Stack::new();
    let src: Vec<u8> = (0..10).collect();
    stack.push_bytes(&src);
    assert_eq!(stack.size, 10);
    let mut dest = [0u8; 10];
    stack.pop_bytes(&mut dest);
    assert_eq!(stack.size, 0);
    assert_eq!(dest.to_vec(), src);
}

#[test]
fn push_zeroes() {
    let mut stack = Stack::new();
    stack.bytes[..4].fill(0xFF);
    stack.push_zeroes(4);
    assert_eq!(stack.size, 4);
    assert_eq!(&stack.bytes[..4], &[0, 0, 0, 0]);
}

#[test]
fn top_is_the_size() {
    let mut stack = Stack::new();
    assert_eq!(stack.top(), 0);
    stack.size = 10;
    assert_eq!(stack.top(), 10);
    stack.size = 100;
    assert_eq!(stack.top(), 100);
}

#[test]
fn copy_non_overlapping() {
    let mut stack = Stack::new();
    let src = [0xAAu8, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22];
    stack.bytes[..8].copy_from_slice(&src);
    stack.size = 16;
    stack.copy(8, 0, 8);
    assert_eq!(&stack.bytes[..8], &src);
    assert_eq!(&stack.bytes[8..16], &src);
}

#[test]
fn copy_zero_size_is_a_no_op() {
    let mut stack = Stack::new();
    stack.size = 16;
    stack.copy(0, 8, 0);
    // a zero-size copy may even "overlap"
    stack.copy(4, 4, 0);
}

#[test]
#[should_panic(expected = "Assert")]
fn copy_of_overlapping_regions_asserts() {
    let mut stack = Stack::new();
    stack.size = 16;
    stack.copy(2, 0, 4);
}

#[test]
#[should_panic(expected = "Assert")]
fn copy_past_the_array_asserts() {
    let mut stack = Stack::new();
    stack.copy(MAX_STACK_SIZE as StackSizeType - 1, 0, 2);
}

#[test]
fn move_overlapping_dest_before_src() {
    let mut stack = Stack::new();
    stack.size = 16;
    stack.bytes[4..8].copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD]);
    stack.r#move(2, 4, 4);
    assert_eq!(&stack.bytes[2..6], &[0xAA, 0xBB, 0xCC, 0xDD]);
}

#[test]
fn move_overlapping_dest_after_src() {
    let mut stack = Stack::new();
    stack.size = 16;
    stack.bytes[..4].copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD]);
    stack.r#move(2, 0, 4);
    assert_eq!(&stack.bytes[2..6], &[0xAA, 0xBB, 0xCC, 0xDD]);
}

#[test]
fn move_zero_size_is_a_no_op() {
    let mut stack = Stack::new();
    stack.size = 16;
    stack.r#move(0, 8, 0);
}

#[test]
fn move_non_overlapping() {
    let mut stack = Stack::new();
    stack.size = 16;
    stack.bytes[..4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
    stack.r#move(8, 0, 4);
    assert_eq!(&stack.bytes[..4], &[0x11, 0x22, 0x33, 0x44]);
    assert_eq!(&stack.bytes[8..12], &[0x11, 0x22, 0x33, 0x44]);
}

#[test]
#[should_panic(expected = "Assert")]
fn move_from_above_the_top_asserts() {
    let mut stack = Stack::new();
    stack.size = 4;
    stack.r#move(0, 2, 4);
}

#[test]
fn big_endian_byte_order() {
    let mut stack = Stack::new();
    stack.push(0x1234_5678u32);
    assert_eq!(&stack.bytes[..4], &[0x12, 0x34, 0x56, 0x78]);
    stack.push(-2i16);
    assert_eq!(&stack.bytes[4..6], &[0xFF, 0xFE]);
    stack.push(1.0f32);
    assert_eq!(&stack.bytes[6..10], &[0x3F, 0x80, 0x00, 0x00]);
}

#[test]
fn mixed_types() {
    let mut stack = Stack::new();
    stack.push(0xABu8);
    stack.push(0xCDEFu16);
    stack.push(0x1234_5678u32);
    assert_eq!(stack.size, 7);
    assert_eq!(stack.pop::<u32>(), 0x1234_5678);
    assert_eq!(stack.pop::<u16>(), 0xCDEF);
    assert_eq!(stack.pop::<u8>(), 0xAB);
    assert_eq!(stack.size, 0);
}

#[test]
#[should_panic(expected = "Assert")]
fn pop_from_an_empty_stack_asserts() {
    let mut stack = Stack::new();
    stack.pop::<u8>();
}

#[test]
#[should_panic(expected = "Assert")]
fn push_onto_a_full_stack_asserts() {
    let mut stack = Stack::new();
    stack.size = MAX_STACK_SIZE as StackSizeType;
    stack.push(1u8);
}

#[test]
fn reset_zeroes_and_empties_the_stack() {
    let mut stack = Stack::new();
    stack.push(0xFFFF_FFFFu32);
    stack.current_frame_start = 4;
    stack.reset();
    assert_eq!(stack.size, 0);
    assert_eq!(stack.current_frame_start, 0);
    assert!(stack.bytes.iter().all(|&b| b == 0));
    assert_eq!(stack.bytes.len(), MAX_STACK_SIZE);
}

use fprime_config::fpy_sequencer::MAX_STACK_SIZE;
