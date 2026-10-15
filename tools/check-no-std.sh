#!/usr/bin/env bash
# CI-style check of the `no_std` + `alloc` builds (ROADMAP item 13).
#
# Proves that fprime-config, fprime-fw, fprime-os (its no_std subset) and
# fprime-comp build and test without `std`, that nothing in them pulls
# `std` in (a build for a bare-metal target, which has no `std` at all),
# that the MSRV holds for those builds, and that the default build still
# has zero third-party dependencies while the no_std build adds only `spin`.
#
# Usage: tools/check-no-std.sh [bare-metal-target]   (default thumbv7em-none-eabihf)
# Needs `rustup target add <target>` (and `--toolchain 1.85`) once.
set -euo pipefail
cd "$(dirname "$0")/.."

TARGET="${1:-thumbv7em-none-eabihf}"
NOSTD=(--no-default-features --features spin)
PKGS=(-p fprime-fw -p fprime-os -p fprime-comp)
# Per-package spellings for invocations that also select fprime-config
# (which has no features: it is unconditionally no_std).
NOSTD_ALL=(--no-default-features --features fprime-fw/spin,fprime-os/spin,fprime-comp/spin)

step() { printf '\n== %s\n' "$*"; }

step "host build + tests without std"
cargo build "${PKGS[@]}" "${NOSTD[@]}"
cargo test "${PKGS[@]}" "${NOSTD[@]}"
cargo clippy "${PKGS[@]}" --all-targets "${NOSTD[@]}" -- -D warnings

step "bare-metal build ($TARGET): nothing may pull in std"
cargo build --target "$TARGET" -p fprime-config "${PKGS[@]}" --lib \
    --example no_std_component "${NOSTD_ALL[@]}"

step "MSRV (1.85) for the no_std builds"
cargo +1.85 check "${PKGS[@]}" --all-targets "${NOSTD[@]}"
cargo +1.85 build --target "$TARGET" -p fprime-config "${PKGS[@]}" --lib \
    --example no_std_component "${NOSTD_ALL[@]}"

step "dependency trees"
thirdparty_default=$(cargo tree --workspace -e all --prefix none | grep -v -e '^fprime' -e '^$' | sort -u || true)
if [[ -n "$thirdparty_default" ]]; then
    echo "default build has third-party dependencies:" >&2
    echo "$thirdparty_default" >&2
    exit 1
fi
echo "default build: no third-party crates"
thirdparty_nostd=$(cargo tree "${PKGS[@]}" -e normal "${NOSTD[@]}" --prefix none \
    | grep -v -e '^fprime' -e '^$' | awk '{print $1}' | sort -u)
if [[ "$thirdparty_nostd" != "spin" ]]; then
    echo "no_std build must depend on spin only, got: $thirdparty_nostd" >&2
    exit 1
fi
echo "no_std build: spin only"

step "ok"
