//! `configure()` misconfigurations and the heap guards (upstream's
//! `WasmSequencerConfigTester` death tests; here `#[should_panic]`, or a
//! caught panic where the component must then be leaked).

use super::*;

/// Requirement: none. `configure` is one-shot.
#[test]
#[should_panic(expected = "Assert")]
fn configure_twice_asserts() {
    let t = Tester::unconfigured();
    t.configure_with(&Tester::standard_config());
    t.configure_with(&Tester::standard_config());
}

/// WASM-SEQ-003: more heap pages than `SPACEWASM_MAX_PAGES`.
#[test]
#[should_panic(expected = "Assert")]
fn configure_too_many_dynamic_pages_asserts() {
    let t = Tester::unconfigured();
    let mut cfg = Tester::standard_config();
    cfg.heap_pages = fprime_config::wasm_sequencer::SPACEWASM_MAX_PAGES + 1;
    t.configure_with(&cfg);
}

/// WASM-SEQ-003: a stack the heap cannot back fails store creation.
#[test]
#[should_panic(expected = "Assert")]
fn configure_stack_too_large_for_pool_asserts() {
    let t = Tester::unconfigured();
    let mut cfg = Tester::standard_config();
    cfg.stack_size = 1 << 20;
    t.configure_with(&cfg);
}

/// WASM-SEQ-003: after any page free, a page allocation asserts (spacewasm
/// frees pages out of order, so the bump index may point at a live page).
/// The heap is left inconsistent on purpose, so the tester is leaked
/// rather than torn down (upstream runs this in a forked child).
#[test]
fn page_free_then_re_alloc_poison_asserts() {
    let t = Tester::unconfigured();
    t.configure_with(&Tester::standard_config());
    let comp = t.comp.clone();
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let heap = comp.heap.lock().unwrap();
        heap.as_ref().unwrap().poke_page_free_then_realloc();
    }));
    let err = result.expect_err("the poisoned pool must assert");
    let msg = err
        .downcast_ref::<&str>()
        .map(ToString::to_string)
        .or_else(|| err.downcast_ref::<String>().cloned())
        .unwrap_or_default();
    assert!(msg.contains("page allocation after a page free"), "{msg}");
    drop(comp);
    t.leak();
}

/// Requirement: none. Tear-down tolerates a never-configured instance.
#[test]
fn unconfigured_deinit_is_harmless() {
    let t = Tester::unconfigured();
    assert!(!t.comp.is_store_created());
    t.comp.deinit();
}

/// The heap pages and guest pool are recycled in place by every store
/// reset: no page leaks across RUNs (not upstream).
#[test]
fn store_reset_recycles_every_page() {
    let t = Tester::new();
    let used = t.dynamic_pages_used();
    assert!(used > 0);
    let file = t.stage("globals.wasm");
    for seq in 0..5 {
        t.send_run(seq, &file, BLOCK, &SeqArgs::default());
        t.dispatch_all();
    }
    t.send_cancel(9);
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    assert_eq!(t.dynamic_pages_used(), used);
    assert_eq!(t.guest_offset(), 0);
}
