#!/usr/bin/env bash
# CI-style check of the `no_std` + `alloc` builds (ROADMAP item 13 and
# "no_std beyond the core").
#
# Proves that fprime-config, fprime-fw, fprime-os (its no_std subset,
# including Os::Mutex / Os::ConditionVariable), fprime-comp, fprime-utils
# (all but the file-based modules) and fprime-svc (every component that
# needs no file system or console) build and test without `std`, that
# nothing in them pulls `std` in (a build for a bare-metal target, which
# has no `std` at all — including the two no_std examples), that the MSRV
# holds for those builds, that fprime-svc's optional `zlib` / `aes-gcm` /
# `wasm` features compose with no_std (`wasm` builds the fprime-spacewasm
# glue crate and spacewasm without std; the WasmSequencer component itself
# needs std and is absent from these builds), and that the default build
# still has zero third-party dependencies while the no_std build adds only
# `spin` (plus, with the optional features on, exactly those features'
# crates). Every
# build and test runs with both `Os::Queue` delegates: the default
# lock-based PriorityQueue and the ISR-safe LocklessPriorityQueue
# (`lockless-queue` feature, the recommended no_std choice when interrupts
# send to queues; its std build is tested here too).
#
# Usage: tools/check-no-std.sh [bare-metal-target]   (default thumbv7em-none-eabihf)
# Needs `rustup target add <target>` (and `--toolchain 1.87`) once.
set -euo pipefail
cd "$(dirname "$0")/.."

TARGET="${1:-thumbv7em-none-eabihf}"
NOSTD=(--no-default-features --features spin)
PKGS=(-p fprime-fw -p fprime-os -p fprime-comp -p fprime-utils -p fprime-svc)
# Per-package spellings for invocations that also select fprime-config
# (which has no features: it is unconditionally no_std).
SPIN_ALL=fprime-fw/spin,fprime-os/spin,fprime-comp/spin,fprime-utils/spin,fprime-svc/spin
NOSTD_ALL=(--no-default-features --features "$SPIN_ALL")
# The same with the lockless queue delegate selected.
LOCKLESS_PKGS=(-p fprime-os -p fprime-comp -p fprime-svc)
NOSTD_LOCKLESS=(--no-default-features --features spin,lockless-queue)
NOSTD_ALL_LOCKLESS=(--no-default-features --features "$SPIN_ALL,fprime-os/lockless-queue")
# fprime-svc's optional third-party features, with and without std.
SVC_OPT=zlib,aes-gcm,wasm
NOSTD_SVC_OPT=(--no-default-features --features "spin,$SVC_OPT")
NOSTD_ALL_OPT=(--no-default-features --features "$SPIN_ALL,fprime-svc/zlib,fprime-svc/aes-gcm,fprime-svc/wasm")
EXAMPLES=(--example no_std_component --example no_std_deployment)

step() { printf '\n== %s\n' "$*"; }

step "host build + tests without std"
cargo build "${PKGS[@]}" "${NOSTD[@]}"
cargo test "${PKGS[@]}" "${NOSTD[@]}"
cargo clippy "${PKGS[@]}" --all-targets "${NOSTD[@]}" -- -D warnings

step "host build + tests without std, lockless queue delegate"
cargo test "${LOCKLESS_PKGS[@]}" "${NOSTD_LOCKLESS[@]}"
cargo clippy "${LOCKLESS_PKGS[@]}" --all-targets "${NOSTD_LOCKLESS[@]}" -- -D warnings

step "host tests with std, lockless queue delegate"
cargo test "${LOCKLESS_PKGS[@]}" --features lockless-queue
cargo clippy "${LOCKLESS_PKGS[@]}" --all-targets --features lockless-queue -- -D warnings

step "host build + tests without std, fprime-svc optional features ($SVC_OPT)"
cargo test -p fprime-svc "${NOSTD_SVC_OPT[@]}"
cargo clippy -p fprime-svc --all-targets "${NOSTD_SVC_OPT[@]}" -- -D warnings

step "bare-metal build ($TARGET): nothing may pull in std"
cargo build --target "$TARGET" -p fprime-config "${PKGS[@]}" --lib \
    "${EXAMPLES[@]}" "${NOSTD_ALL[@]}"
cargo build --target "$TARGET" -p fprime-config "${PKGS[@]}" --lib \
    "${EXAMPLES[@]}" "${NOSTD_ALL_LOCKLESS[@]}"
# The optional features too (getrandom's `custom` backend stands in for
# the OS RNG there; the deployment registers its source).
cargo build --target "$TARGET" -p fprime-config "${PKGS[@]}" --lib \
    "${EXAMPLES[@]}" "${NOSTD_ALL_OPT[@]}"

step "MSRV (1.87) for the no_std builds"
cargo +1.87 check "${PKGS[@]}" --all-targets "${NOSTD[@]}"
cargo +1.87 check "${LOCKLESS_PKGS[@]}" --all-targets "${NOSTD_LOCKLESS[@]}"
cargo +1.87 check -p fprime-svc --all-targets "${NOSTD_SVC_OPT[@]}"
cargo +1.87 build --target "$TARGET" -p fprime-config "${PKGS[@]}" --lib \
    "${EXAMPLES[@]}" "${NOSTD_ALL[@]}"
cargo +1.87 build --target "$TARGET" -p fprime-config "${PKGS[@]}" --lib \
    "${EXAMPLES[@]}" "${NOSTD_ALL_LOCKLESS[@]}"
cargo +1.87 build --target "$TARGET" -p fprime-config "${PKGS[@]}" --lib \
    "${EXAMPLES[@]}" "${NOSTD_ALL_OPT[@]}"

step "dependency trees"
thirdparty_default=$(cargo tree --workspace -e all --prefix none | grep -v -e '^fprime' -e '^$' | sort -u || true)
if [[ -n "$thirdparty_default" ]]; then
    echo "default build has third-party dependencies:" >&2
    echo "$thirdparty_default" >&2
    exit 1
fi
echo "default build: no third-party crates"
thirdparty() { # cargo tree args... -> sorted third-party crate names
    cargo tree "$@" -e normal --prefix none | grep -v -e '^fprime' -e '^$' | awk '{print $1}' \
        | sort -u
}
thirdparty_nostd=$(thirdparty "${PKGS[@]}" "${NOSTD[@]}")
if [[ "$thirdparty_nostd" != "spin" ]]; then
    echo "no_std build must depend on spin only, got: $thirdparty_nostd" >&2
    exit 1
fi
echo "no_std build: spin only"
# With fprime-svc's optional features: spin plus exactly the crates those
# features pull into the std build, for the host and the bare-metal target.
for target_args in "" "--target $TARGET"; do
    # shellcheck disable=SC2086 # target_args is deliberately word-split.
    opt_std=$(thirdparty -p fprime-svc --features "$SVC_OPT" $target_args)
    # shellcheck disable=SC2086
    opt_nostd=$(thirdparty -p fprime-svc "${NOSTD_SVC_OPT[@]}" $target_args)
    expected=$(printf '%s\nspin\n' "$opt_std" | sort -u)
    if [[ "$opt_nostd" != "$expected" ]]; then
        echo "no_std + $SVC_OPT ($target_args) must add only spin to the feature crates:" >&2
        diff <(echo "$expected") <(echo "$opt_nostd") >&2 || true
        exit 1
    fi
done
echo "no_std build with $SVC_OPT: spin + the feature crates only"

step "ok"
