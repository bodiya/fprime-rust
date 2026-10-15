//! The stack operations (upstream `ieq` … `itrunc_64_32`, the
//! `*_underflow`, overflow, domain-error and Python-semantics tests).

use super::*;

type Op = fn(&mut FpyState) -> DirectiveError;

fn op(t: &Tester, f: Op) -> DirectiveError {
    f(&mut t.st())
}

const T: u8 = fprime_config::FW_SERIALIZE_TRUE_VALUE;
const F: u8 = fprime_config::FW_SERIALIZE_FALSE_VALUE;

/// Push the two operands, run `f`, return the result byte (and empty the
/// stack).
fn cmp<V: StackValue>(t: &Tester, f: Op, lhs: V, rhs: V) -> u8 {
    t.set_stack_size(0);
    t.push(lhs);
    t.push(rhs);
    assert_eq!(op(t, f), DirectiveError::NoError);
    assert_eq!(t.stack_size(), 1);
    t.pop::<u8>()
}

#[test]
fn ieq() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_ieq, -1i64, -1i64), T);
    assert_eq!(cmp(&t, FpyState::op_ieq, -1i64, 1i64), F);
}

#[test]
fn ine() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_ine, -1i64, -1i64), F);
    assert_eq!(cmp(&t, FpyState::op_ine, -1i64, 1i64), T);
}

#[test]
fn or() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_or, T, T), T);
    assert_eq!(cmp(&t, FpyState::op_or, T, F), T);
    assert_eq!(cmp(&t, FpyState::op_or, F, F), F);
    // bitwise, as the stackOp test checks
    assert_eq!(cmp(&t, FpyState::op_or, 5u8, 10u8), 15);
}

#[test]
fn and() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_and, F, F), F);
    assert_eq!(cmp(&t, FpyState::op_and, T, F), F);
    assert_eq!(cmp(&t, FpyState::op_and, T, T), T);
    assert_eq!(cmp(&t, FpyState::op_and, 12u8, 10u8), 8);
}

#[test]
fn ult() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_ult, 0u64, u64::MAX), T);
    assert_eq!(cmp(&t, FpyState::op_ult, 0u64, 0u64), F);
    assert_eq!(cmp(&t, FpyState::op_ult, 0u64, 1u64), T);
    // unsigned: -1 is the largest value
    assert_eq!(cmp(&t, FpyState::op_ult, 5i64, 10i64), T);
}

#[test]
fn ule() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_ule, 0u64, u64::MAX), T);
    assert_eq!(cmp(&t, FpyState::op_ule, 0u64, 0u64), T);
    assert_eq!(cmp(&t, FpyState::op_ule, 0u64, 1u64), T);
    assert_eq!(cmp(&t, FpyState::op_ule, 2u64, 1u64), F);
}

#[test]
fn ugt() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_ugt, u64::MAX, 0u64), T);
    assert_eq!(cmp(&t, FpyState::op_ugt, 0u64, 0u64), F);
    assert_eq!(cmp(&t, FpyState::op_ugt, 1u64, 0u64), T);
}

#[test]
fn uge() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_uge, u64::MAX, 0u64), T);
    assert_eq!(cmp(&t, FpyState::op_uge, 0u64, 0u64), T);
    assert_eq!(cmp(&t, FpyState::op_uge, 1u64, 0u64), T);
    assert_eq!(cmp(&t, FpyState::op_uge, 1u64, 2u64), F);
}

#[test]
fn slt() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_slt, 0i64, i64::MAX), T);
    assert_eq!(cmp(&t, FpyState::op_slt, 0i64, 0i64), F);
    assert_eq!(cmp(&t, FpyState::op_slt, 0i64, 1i64), T);
    assert_eq!(cmp(&t, FpyState::op_slt, -5i64, 10i64), T);
}

#[test]
fn sle() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_sle, 0i64, i64::MAX), T);
    assert_eq!(cmp(&t, FpyState::op_sle, 0i64, 0i64), T);
    assert_eq!(cmp(&t, FpyState::op_sle, 0i64, -1i64), F);
}

#[test]
fn sgt() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_sgt, 0i64, i64::MAX), F);
    assert_eq!(cmp(&t, FpyState::op_sgt, 0i64, 0i64), F);
    assert_eq!(cmp(&t, FpyState::op_sgt, 0i64, -1i64), T);
}

#[test]
fn sge() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_sge, 0i64, i64::MAX), F);
    assert_eq!(cmp(&t, FpyState::op_sge, 0i64, 0i64), T);
    assert_eq!(cmp(&t, FpyState::op_sge, 0i64, -1i64), T);
}

#[test]
fn flt() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_flt, 0.0f64, f64::MAX), T);
    assert_eq!(cmp(&t, FpyState::op_flt, 0.0f64, 0.0f64), F);
    assert_eq!(cmp(&t, FpyState::op_flt, 0.0f64, -1.0f64), F);
    assert_eq!(cmp(&t, FpyState::op_flt, 0.0f64, f64::NAN), F);
    assert_eq!(cmp(&t, FpyState::op_flt, 5.5f64, 10.1f64), T);
}

#[test]
fn fle() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_fle, 0.0f64, f64::MAX), T);
    assert_eq!(cmp(&t, FpyState::op_fle, 0.0f64, 0.0f64), T);
    assert_eq!(cmp(&t, FpyState::op_fle, 0.0f64, -1.0f64), F);
    assert_eq!(cmp(&t, FpyState::op_fle, f64::NAN, f64::NAN), F);
}

#[test]
fn fgt() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_fgt, 0.0f64, f64::MAX), F);
    assert_eq!(cmp(&t, FpyState::op_fgt, 0.0f64, 0.0f64), F);
    assert_eq!(cmp(&t, FpyState::op_fgt, 0.0f64, -1.0f64), T);
    assert_eq!(cmp(&t, FpyState::op_fgt, 0.0f64, f64::NAN), F);
}

#[test]
fn fge() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_fge, 0.0f64, f64::MAX), F);
    assert_eq!(cmp(&t, FpyState::op_fge, 0.0f64, 0.0f64), T);
    assert_eq!(cmp(&t, FpyState::op_fge, 0.0f64, -1.0f64), T);
    assert_eq!(cmp(&t, FpyState::op_fge, f64::NAN, f64::NAN), F);
    assert_eq!(cmp(&t, FpyState::op_fge, 10.1f64, 10.1f64), T);
}

#[test]
fn feq() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_feq, 0.0f64, f64::MAX), F);
    assert_eq!(cmp(&t, FpyState::op_feq, 0.0f64, 0.0f64), T);
    assert_eq!(cmp(&t, FpyState::op_feq, 0.0f64, -1.0f64), F);
    assert_eq!(cmp(&t, FpyState::op_feq, f64::NAN, f64::NAN), F);
}

#[test]
fn fne() {
    let t = Tester::new();
    assert_eq!(cmp(&t, FpyState::op_fne, 0.0f64, f64::MAX), T);
    assert_eq!(cmp(&t, FpyState::op_fne, 0.0f64, 0.0f64), F);
    assert_eq!(cmp(&t, FpyState::op_fne, 0.0f64, -1.0f64), T);
    assert_eq!(cmp(&t, FpyState::op_fne, f64::NAN, f64::NAN), T);
}

#[test]
fn not() {
    let t = Tester::new();
    t.push(T);
    assert_eq!(op(&t, FpyState::op_not), DirectiveError::NoError);
    assert_eq!(t.pop::<u8>(), F);
    t.push(F);
    assert_eq!(op(&t, FpyState::op_not), DirectiveError::NoError);
    assert_eq!(t.pop::<u8>(), T);
    // any non-zero byte is true
    t.push(255u8);
    assert_eq!(op(&t, FpyState::op_not), DirectiveError::NoError);
    assert_eq!(t.pop::<u8>(), F);
}

/// Push `value`, run `f`, pop a `R`.
fn unary<V: StackValue, R: StackValue>(t: &Tester, f: Op, value: V) -> R {
    t.push(value);
    assert_eq!(op(t, f), DirectiveError::NoError);
    let result = t.pop::<R>();
    assert_eq!(t.stack_size(), 0);
    result
}

/// Push both operands, run `f`, pop a `R`.
fn binary<V: StackValue, R: StackValue>(t: &Tester, f: Op, lhs: V, rhs: V) -> R {
    t.push(lhs);
    t.push(rhs);
    assert_eq!(op(t, f), DirectiveError::NoError);
    let result = t.pop::<R>();
    assert_eq!(t.stack_size(), 0);
    result
}

fn binary_err<V: StackValue>(t: &Tester, f: Op, lhs: V, rhs: V) -> DirectiveError {
    t.set_stack_size(0);
    t.push(lhs);
    t.push(rhs);
    op(t, f)
}

#[test]
fn fptrunc() {
    let t = Tester::new();
    let src = 123.123f64;
    assert_eq!(unary::<f64, f32>(&t, FpyState::op_fptrunc, src), src as f32);
}

#[test]
fn fpext() {
    let t = Tester::new();
    let src = 123.123f32;
    assert_eq!(
        unary::<f32, f64>(&t, FpyState::op_fpext, src),
        f64::from(src)
    );
}

#[test]
fn fptosi() {
    let t = Tester::new();
    assert_eq!(unary::<f64, i64>(&t, FpyState::op_fptosi, 123.123), 123);
}

#[test]
fn fptosi_saturates() {
    let t = Tester::new();
    let f = FpyState::op_fptosi;
    assert_eq!(unary::<f64, i64>(&t, f, f64::NAN), 0);
    assert_eq!(unary::<f64, i64>(&t, f, f64::INFINITY), i64::MAX);
    assert_eq!(unary::<f64, i64>(&t, f, f64::NEG_INFINITY), i64::MIN);
    assert_eq!(unary::<f64, i64>(&t, f, 1e300), i64::MAX);
    assert_eq!(unary::<f64, i64>(&t, f, -1e300), i64::MIN);
    // truncation toward zero for in-range values
    assert_eq!(unary::<f64, i64>(&t, f, -3.7), -3);
    // 2^63 is one past the maximum; -2^63 is exactly the minimum
    assert_eq!(
        unary::<f64, i64>(&t, f, 9_223_372_036_854_775_808.0),
        i64::MAX
    );
    assert_eq!(
        unary::<f64, i64>(&t, f, -9_223_372_036_854_775_808.0),
        i64::MIN
    );
}

#[test]
fn sitofp() {
    let t = Tester::new();
    assert_eq!(unary::<i64, f64>(&t, FpyState::op_sitofp, 123), 123.0);
}

#[test]
fn fptoui() {
    let t = Tester::new();
    assert_eq!(unary::<f64, u64>(&t, FpyState::op_fptoui, 123.123), 123);
}

#[test]
fn fptoui_saturates() {
    let t = Tester::new();
    let f = FpyState::op_fptoui;
    assert_eq!(unary::<f64, u64>(&t, f, f64::NAN), 0);
    assert_eq!(unary::<f64, u64>(&t, f, -5.5), 0);
    assert_eq!(unary::<f64, u64>(&t, f, f64::NEG_INFINITY), 0);
    assert_eq!(unary::<f64, u64>(&t, f, f64::INFINITY), u64::MAX);
    assert_eq!(unary::<f64, u64>(&t, f, 1e300), u64::MAX);
    assert_eq!(
        unary::<f64, u64>(&t, f, 18_446_744_073_709_551_616.0),
        u64::MAX
    );
}

#[test]
fn uitofp() {
    let t = Tester::new();
    assert_eq!(unary::<u64, f64>(&t, FpyState::op_uitofp, 123), 123.0);
}

#[test]
fn add() {
    let t = Tester::new();
    assert_eq!(binary::<i64, i64>(&t, FpyState::op_add, 100, 23), 123);
}

#[test]
fn sub() {
    let t = Tester::new();
    assert_eq!(binary::<i64, i64>(&t, FpyState::op_sub, 150, 27), 123);
}

#[test]
fn mul() {
    let t = Tester::new();
    assert_eq!(binary::<i64, i64>(&t, FpyState::op_mul, 41, 3), 123);
}

#[test]
fn udiv() {
    let t = Tester::new();
    assert_eq!(binary::<u64, u64>(&t, FpyState::op_udiv, 246, 2), 123);
}

#[test]
fn sdiv() {
    let t = Tester::new();
    assert_eq!(binary::<i64, i64>(&t, FpyState::op_sdiv, -246, -2), 123);
}

#[test]
fn sdiv_floors_toward_negative_infinity() {
    let t = Tester::new();
    let f = FpyState::op_sdiv;
    assert_eq!(binary::<i64, i64>(&t, f, -7, 2), -4);
    assert_eq!(binary::<i64, i64>(&t, f, 7, -2), -4);
    assert_eq!(binary::<i64, i64>(&t, f, -7, -2), 3);
    // exact quotients need no adjustment
    assert_eq!(binary::<i64, i64>(&t, f, -6, 2), -3);
}

#[test]
fn sdiv_int_min_by_minus_one_overflows() {
    let t = Tester::new();
    assert_eq!(
        binary_err(&t, FpyState::op_sdiv, i64::MIN, -1i64),
        DirectiveError::ArithmeticOverflow
    );
}

#[test]
fn umod() {
    let t = Tester::new();
    assert_eq!(binary::<u64, u64>(&t, FpyState::op_umod, 123, 10), 3);
}

#[test]
fn smod() {
    let t = Tester::new();
    assert_eq!(binary::<i64, i64>(&t, FpyState::op_smod, -7, -3), -1);
}

#[test]
fn fadd() {
    let t = Tester::new();
    assert_eq!(
        binary::<f64, f64>(&t, FpyState::op_fadd, 100.5, 22.5),
        123.0
    );
}

#[test]
fn fsub() {
    let t = Tester::new();
    assert_eq!(
        binary::<f64, f64>(&t, FpyState::op_fsub, 150.5, 27.5),
        123.0
    );
}

#[test]
fn fmul() {
    let t = Tester::new();
    assert_eq!(binary::<f64, f64>(&t, FpyState::op_fmul, 41.0, 3.0), 123.0);
}

#[test]
fn fdiv() {
    let t = Tester::new();
    assert_eq!(binary::<f64, f64>(&t, FpyState::op_fdiv, 246.0, 2.0), 123.0);
}

#[test]
fn fpow() {
    let t = Tester::new();
    assert_eq!(binary::<f64, f64>(&t, FpyState::op_fpow, 3.0, 2.0), 9.0);
}

#[test]
fn flog() {
    let t = Tester::new();
    assert_eq!(
        unary::<f64, f64>(&t, FpyState::op_flog, std::f64::consts::E),
        1.0
    );
}

#[test]
fn fmod() {
    let t = Tester::new();
    assert_eq!(binary::<f64, f64>(&t, FpyState::op_fmod, -8.5, -2.5), -1.0);
}

#[test]
fn fmod_exact_multiple_zero_sign() {
    // Python float %: a zero result takes the divisor's sign
    let t = Tester::new();
    let res = binary::<f64, f64>(&t, FpyState::op_fmod, 2.0, -1.0);
    assert_eq!(res, 0.0);
    assert!(res.is_sign_negative());
    let res = binary::<f64, f64>(&t, FpyState::op_fmod, -1.5, 0.5);
    assert_eq!(res, 0.0);
    assert!(res.is_sign_positive());
}

#[test]
fn ffloor() {
    let t = Tester::new();
    let f = FpyState::op_ffloor;
    assert_eq!(unary::<f64, f64>(&t, f, 7.5), 7.0);
    // toward -inf, not toward zero
    assert_eq!(unary::<f64, f64>(&t, f, -0.5), -1.0);
    assert_eq!(unary::<f64, f64>(&t, f, 0.5), 0.0);
}

#[test]
fn ffloor_preserves_zero_sign() {
    let t = Tester::new();
    let res = unary::<f64, f64>(&t, FpyState::op_ffloor, -0.0);
    assert_eq!(res, 0.0);
    assert!(res.is_sign_negative());
    let res = unary::<f64, f64>(&t, FpyState::op_ffloor, 0.0);
    assert_eq!(res, 0.0);
    assert!(res.is_sign_positive());
}

#[test]
fn ffloor_inf_nan_passthrough() {
    let t = Tester::new();
    let f = FpyState::op_ffloor;
    assert_eq!(unary::<f64, f64>(&t, f, f64::INFINITY), f64::INFINITY);
    assert_eq!(
        unary::<f64, f64>(&t, f, f64::NEG_INFINITY),
        f64::NEG_INFINITY
    );
    assert!(unary::<f64, f64>(&t, f, f64::NAN).is_nan());
}

#[test]
fn iabs() {
    let t = Tester::new();
    let f = FpyState::op_iabs;
    assert_eq!(unary::<i64, i64>(&t, f, -5), 5);
    assert_eq!(unary::<i64, i64>(&t, f, 5), 5);
    assert_eq!(unary::<i64, i64>(&t, f, i64::MAX), i64::MAX);
}

#[test]
fn iabs_int_min_overflows() {
    let t = Tester::new();
    t.push(i64::MIN);
    assert_eq!(
        op(&t, FpyState::op_iabs),
        DirectiveError::ArithmeticOverflow
    );
}

#[test]
fn fabs() {
    let t = Tester::new();
    assert_eq!(unary::<f64, f64>(&t, FpyState::op_fabs, -2.5), 2.5);
    assert_eq!(unary::<f64, f64>(&t, FpyState::op_fabs, 2.5), 2.5);
}

#[test]
fn fabs_clears_zero_sign() {
    let t = Tester::new();
    let res = unary::<f64, f64>(&t, FpyState::op_fabs, -0.0);
    assert_eq!(res, 0.0);
    assert!(res.is_sign_positive());
}

#[test]
fn fabs_inf_nan() {
    let t = Tester::new();
    assert_eq!(
        unary::<f64, f64>(&t, FpyState::op_fabs, f64::NEG_INFINITY),
        f64::INFINITY
    );
    assert!(unary::<f64, f64>(&t, FpyState::op_fabs, f64::NAN).is_nan());
}

#[test]
fn siext_8_64() {
    let t = Tester::new();
    let f = FpyState::op_siext_8_64;
    assert_eq!(unary::<i8, i64>(&t, f, 42), 42);
    assert_eq!(unary::<i8, i64>(&t, f, -42), -42);
    assert_eq!(unary::<i8, i64>(&t, f, i8::MIN), i64::from(i8::MIN));
}

#[test]
fn siext_16_64() {
    let t = Tester::new();
    let f = FpyState::op_siext_16_64;
    assert_eq!(unary::<i16, i64>(&t, f, 1234), 1234);
    assert_eq!(unary::<i16, i64>(&t, f, -1234), -1234);
    assert_eq!(unary::<i16, i64>(&t, f, i16::MIN), i64::from(i16::MIN));
}

#[test]
fn siext_32_64() {
    let t = Tester::new();
    let f = FpyState::op_siext_32_64;
    assert_eq!(unary::<i32, i64>(&t, f, 123_456), 123_456);
    assert_eq!(unary::<i32, i64>(&t, f, -123_456), -123_456);
    assert_eq!(unary::<i32, i64>(&t, f, i32::MIN), i64::from(i32::MIN));
}

#[test]
fn ziext_8_64() {
    let t = Tester::new();
    let f = FpyState::op_ziext_8_64;
    assert_eq!(unary::<u8, u64>(&t, f, 200), 200);
    assert_eq!(unary::<u8, u64>(&t, f, 0), 0);
    assert_eq!(unary::<u8, u64>(&t, f, u8::MAX), u64::from(u8::MAX));
}

#[test]
fn ziext_16_64() {
    let t = Tester::new();
    let f = FpyState::op_ziext_16_64;
    assert_eq!(unary::<u16, u64>(&t, f, 50000), 50000);
    assert_eq!(unary::<u16, u64>(&t, f, u16::MAX), u64::from(u16::MAX));
}

#[test]
fn ziext_32_64() {
    let t = Tester::new();
    let f = FpyState::op_ziext_32_64;
    assert_eq!(unary::<u32, u64>(&t, f, 3_000_000_000), 3_000_000_000);
    assert_eq!(unary::<u32, u64>(&t, f, u32::MAX), u64::from(u32::MAX));
}

#[test]
fn itrunc_64_8() {
    let t = Tester::new();
    let f = FpyState::op_itrunc_64_8;
    assert_eq!(unary::<u64, u8>(&t, f, 0x1234_5678_9ABC_DE42), 0x42);
    assert_eq!(unary::<u64, u8>(&t, f, 255), 255);
}

#[test]
fn itrunc_64_16() {
    let t = Tester::new();
    let f = FpyState::op_itrunc_64_16;
    assert_eq!(unary::<u64, u16>(&t, f, 0x1234_5678_9ABC_DEFF), 0xDEFF);
    assert_eq!(unary::<u64, u16>(&t, f, 65535), 65535);
}

#[test]
fn itrunc_64_32() {
    let t = Tester::new();
    let f = FpyState::op_itrunc_64_32;
    assert_eq!(unary::<u64, u32>(&t, f, 0x1234_5678_9ABC_DEF0), 0x9ABC_DEF0);
    assert_eq!(unary::<u64, u32>(&t, f, 0xFFFF_FFFF), 0xFFFF_FFFF);
}

// -- stack underflow: every op on an empty stack and with one operand -------

/// Each binary op fails on an empty stack and with only one operand of its
/// width (upstream `<op>_underflow`).
#[test]
fn binary_ops_underflow() {
    let ops: &[(Op, usize)] = &[
        (FpyState::op_or, 1),
        (FpyState::op_and, 1),
        (FpyState::op_ieq, 8),
        (FpyState::op_ine, 8),
        (FpyState::op_ult, 8),
        (FpyState::op_ule, 8),
        (FpyState::op_ugt, 8),
        (FpyState::op_uge, 8),
        (FpyState::op_slt, 8),
        (FpyState::op_sle, 8),
        (FpyState::op_sgt, 8),
        (FpyState::op_sge, 8),
        (FpyState::op_feq, 8),
        (FpyState::op_fne, 8),
        (FpyState::op_flt, 8),
        (FpyState::op_fle, 8),
        (FpyState::op_fgt, 8),
        (FpyState::op_fge, 8),
        (FpyState::op_add, 8),
        (FpyState::op_sub, 8),
        (FpyState::op_mul, 8),
        (FpyState::op_sdiv, 8),
        (FpyState::op_smod, 8),
        (FpyState::op_udiv, 8),
        (FpyState::op_umod, 8),
        (FpyState::op_fadd, 8),
        (FpyState::op_fsub, 8),
        (FpyState::op_fmul, 8),
        (FpyState::op_fdiv, 8),
        (FpyState::op_fpow, 8),
        (FpyState::op_fmod, 8),
    ];
    for (i, (f, width)) in ops.iter().enumerate() {
        let t = Tester::new();
        assert_eq!(op(&t, *f), DirectiveError::StackUnderflow, "op {i} empty");
        t.set_stack_size(*width as StackSizeType);
        assert_eq!(
            op(&t, *f),
            DirectiveError::StackUnderflow,
            "op {i} one operand"
        );
        // nothing was popped
        assert_eq!(t.stack_size(), *width as StackSizeType);
    }
}

/// Each unary op fails on an empty stack (upstream `<op>_underflow`), and
/// `FFLOOR`/`IABS`/`FABS` also with only 4 bytes.
#[test]
fn unary_ops_underflow() {
    let ops: &[Op] = &[
        FpyState::op_not,
        FpyState::op_fpext,
        FpyState::op_fptrunc,
        FpyState::op_fptosi,
        FpyState::op_sitofp,
        FpyState::op_fptoui,
        FpyState::op_uitofp,
        FpyState::op_flog,
        FpyState::op_siext_8_64,
        FpyState::op_siext_16_64,
        FpyState::op_siext_32_64,
        FpyState::op_ziext_8_64,
        FpyState::op_ziext_16_64,
        FpyState::op_ziext_32_64,
        FpyState::op_itrunc_64_8,
        FpyState::op_itrunc_64_16,
        FpyState::op_itrunc_64_32,
        FpyState::op_ffloor,
        FpyState::op_iabs,
        FpyState::op_fabs,
    ];
    for (i, f) in ops.iter().enumerate() {
        let t = Tester::new();
        assert_eq!(op(&t, *f), DirectiveError::StackUnderflow, "op {i}");
    }
    for f in [FpyState::op_ffloor, FpyState::op_iabs, FpyState::op_fabs] {
        let t = Tester::new();
        t.push(0u32);
        assert_eq!(op(&t, f), DirectiveError::StackUnderflow);
    }
}

// -- arithmetic overflow and underflow --------------------------------------

#[test]
fn add_overflow_and_underflow() {
    let t = Tester::new();
    assert_eq!(
        binary_err(&t, FpyState::op_add, i64::MAX, 1i64),
        DirectiveError::ArithmeticOverflow
    );
    assert_eq!(
        binary_err(&t, FpyState::op_add, i64::MIN, -1i64),
        DirectiveError::ArithmeticUnderflow
    );
}

#[test]
fn sub_overflow_and_underflow() {
    let t = Tester::new();
    assert_eq!(
        binary_err(&t, FpyState::op_sub, i64::MAX, -1i64),
        DirectiveError::ArithmeticOverflow
    );
    assert_eq!(
        binary_err(&t, FpyState::op_sub, i64::MIN, 1i64),
        DirectiveError::ArithmeticUnderflow
    );
}

#[test]
fn mul_overflow_and_underflow() {
    let t = Tester::new();
    let f = FpyState::op_mul;
    let overflow = DirectiveError::ArithmeticOverflow;
    let underflow = DirectiveError::ArithmeticUnderflow;
    assert_eq!(binary_err(&t, f, i64::MAX, 2i64), overflow);
    assert_eq!(binary_err(&t, f, i64::MIN / 2 - 1, -3i64), overflow);
    // I64 min has no positive counterpart: negation-based guards miss these
    assert_eq!(binary_err(&t, f, i64::MIN, -1i64), overflow);
    assert_eq!(binary_err(&t, f, -1i64, i64::MIN), overflow);
    assert_eq!(binary_err(&t, f, i64::MIN, -2i64), overflow);
    assert_eq!(binary_err(&t, f, -2i64, i64::MIN), overflow);
    assert_eq!(binary_err(&t, f, i64::MIN, i64::MIN), overflow);
    assert_eq!(
        binary_err(&t, f, -3_037_000_500i64, -3_037_000_500i64),
        overflow
    );
    assert_eq!(binary_err(&t, f, i64::MIN, 2i64), underflow);
    assert_eq!(binary_err(&t, f, i64::MAX, -2i64), underflow);
}

#[test]
fn mul_neg_neg_boundaries() {
    let t = Tester::new();
    // (-3037000499)^2 is the largest both-negative square within I64 max
    assert_eq!(
        binary::<i64, i64>(&t, FpyState::op_mul, -3_037_000_499, -3_037_000_499),
        9_223_372_030_926_249_001
    );
    assert_eq!(binary::<i64, i64>(&t, FpyState::op_mul, -1, -1), 1);
}

// -- domain errors -------------------------------------------------------------

#[test]
fn division_and_modulo_by_zero_are_domain_errors() {
    let t = Tester::new();
    assert_eq!(
        binary_err(&t, FpyState::op_udiv, 42u64, 0u64),
        DirectiveError::DomainError
    );
    assert_eq!(
        binary_err(&t, FpyState::op_sdiv, 42i64, 0i64),
        DirectiveError::DomainError
    );
    assert_eq!(
        binary_err(&t, FpyState::op_umod, 42u64, 0u64),
        DirectiveError::DomainError
    );
    // C++ parity: the modulo ops check the divisor before popping the
    // dividend, which stays on the stack
    assert_eq!(t.stack_size(), 8);
    assert_eq!(
        binary_err(&t, FpyState::op_smod, 42i64, 0i64),
        DirectiveError::DomainError
    );
    assert_eq!(t.stack_size(), 8);
}

#[test]
fn smod_int_min_by_minus_one_is_zero() {
    let t = Tester::new();
    assert_eq!(binary::<i64, i64>(&t, FpyState::op_smod, i64::MIN, -1), 0);
}

#[test]
fn flog_domain_error() {
    let t = Tester::new();
    t.push(0.0f64);
    assert_eq!(op(&t, FpyState::op_flog), DirectiveError::DomainError);
    t.push(-1.0f64);
    assert_eq!(op(&t, FpyState::op_flog), DirectiveError::DomainError);
}

#[test]
fn fmod_zero_divisor_yields_nan() {
    let t = Tester::new();
    assert!(binary::<f64, f64>(&t, FpyState::op_fmod, 1.0, 0.0).is_nan());
}

#[test]
fn smod_python_adjustment() {
    let t = Tester::new();
    // C++: 7 % -3 = 1; Python: -2
    assert_eq!(binary::<i64, i64>(&t, FpyState::op_smod, 7, -3), -2);
    // C++: -7 % 3 = -1; Python: 2
    assert_eq!(binary::<i64, i64>(&t, FpyState::op_smod, -7, 3), 2);
}

#[test]
fn fmod_python_adjustment() {
    let t = Tester::new();
    assert_eq!(binary::<f64, f64>(&t, FpyState::op_fmod, 7.0, -3.0), -2.0);
    assert_eq!(binary::<f64, f64>(&t, FpyState::op_fmod, -7.0, 3.0), 2.0);
}
