#!/usr/bin/env bash
# Regenerate the Rust reference deployment's ground dictionary from the
# upstream F Prime FPP model plus this directory's model of the Rust
# instances and the Rust SignalGen.
#
#   crates/fprime-ref/fpp/generate-dictionary.sh <path to an fprime checkout>
#
# Writes crates/fprime-ref/dictionary/RefTopologyDictionary.json. The
# framework version recorded in the dictionary is the checkout's
# `git describe`.
set -euo pipefail
FPRIME=${1:?usage: generate-dictionary.sh <fprime checkout>}
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
OUT="$ROOT/crates/fprime-ref/dictionary"
mkdir -p "$OUT"
IMPORTS=()
while IFS= read -r f; do IMPORTS+=(-i "$f"); done < <(
  find "$FPRIME/default/config" "$FPRIME/Fw" "$FPRIME/Svc" "$FPRIME/Drv" "$FPRIME/Os" \
    -name '*.fpp' -not -path '*/test/*' | sort
)
IMPORTS+=(-i "$FPRIME/cmake/platform/unix/Platform/PlatformTypes.fpp")
VERSION=$(git -C "$FPRIME" describe --tags --always 2>/dev/null || echo unknown)
cd "$ROOT"
cargo run -q -p fprime-fpp --bin fpp-to-rust -- --dict "$OUT" \
  -p "$(grep -m1 '^version' Cargo.toml | sed 's/.*"\(.*\)"/\1/')" -f "$VERSION" \
  "${IMPORTS[@]}" "$HERE/SignalGen.fpp" "$HERE/Ref.fpp"
echo "wrote $OUT/RefTopologyDictionary.json"
