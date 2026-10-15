#!/usr/bin/env bash
# Regenerate the Rust reference deployment's ground dictionary from the
# upstream F Prime FPP model plus this directory's model of the Rust
# instances and the Rust SignalGen.
#
#   crates/fprime-ref/fpp/generate-dictionary.sh [--allow-other-commit] <fprime checkout>
#
# Writes crates/fprime-ref/dictionary/RefTopologyDictionary.json.
#
# The port is pinned to one upstream commit (FPRIME_UPSTREAM_COMMIT at the
# repository root). The checkout must be a clean git work tree at exactly
# that commit; `--allow-other-commit` lifts both checks (for the first step
# of a sync, see docs/ROADMAP.md "Syncing with upstream"). The dictionary's
# metadata.frameworkVersion is the first seven hex digits of the commit the
# checkout is actually at, so a pinned run always writes the pin.
set -euo pipefail
usage() {
  echo "usage: generate-dictionary.sh [--allow-other-commit] <fprime checkout>" >&2
  exit 2
}
ALLOW_OTHER=0
FPRIME=
for arg in "$@"; do
  case "$arg" in
    --allow-other-commit) ALLOW_OTHER=1 ;;
    -h|--help) usage ;;
    -*) echo "unknown option: $arg" >&2; usage ;;
    *) [ -z "$FPRIME" ] || usage; FPRIME=$arg ;;
  esac
done
[ -n "$FPRIME" ] || usage
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
OUT="$ROOT/crates/fprime-ref/dictionary"

PIN=$(grep -v '^[[:space:]]*#' "$ROOT/FPRIME_UPSTREAM_COMMIT" | tr -d '[:space:]')
if ! [[ "$PIN" =~ ^[0-9a-f]{40}$ ]]; then
  echo "FPRIME_UPSTREAM_COMMIT does not hold one full commit hash" >&2
  exit 1
fi
if ! COMMIT=$(git -C "$FPRIME" rev-parse --verify -q HEAD 2>/dev/null); then
  echo "$FPRIME is not a git checkout; cannot check it against the pin $PIN" >&2
  exit 1
fi
if [ "$ALLOW_OTHER" -eq 0 ]; then
  if [ "$COMMIT" != "$PIN" ]; then
    echo "$FPRIME is at $COMMIT, but the port is pinned to $PIN" >&2
    echo "(FPRIME_UPSTREAM_COMMIT). Check out the pinned commit, or pass" >&2
    echo "--allow-other-commit while syncing to a new upstream." >&2
    exit 1
  fi
  if ! git -C "$FPRIME" diff --quiet HEAD --; then
    echo "$FPRIME has uncommitted changes to tracked files; the dictionary" >&2
    echo "would not match the pinned commit. Pass --allow-other-commit to" >&2
    echo "generate from it anyway." >&2
    exit 1
  fi
elif [ "$COMMIT" != "$PIN" ]; then
  echo "note: generating from $COMMIT, not the pinned $PIN" >&2
fi
VERSION=${COMMIT:0:7}

mkdir -p "$OUT"
IMPORTS=()
while IFS= read -r f; do IMPORTS+=(-i "$f"); done < <(
  find "$FPRIME/default/config" "$FPRIME/Fw" "$FPRIME/Svc" "$FPRIME/Drv" "$FPRIME/Os" \
    -name '*.fpp' -not -path '*/test/*' | sort
)
IMPORTS+=(-i "$FPRIME/cmake/platform/unix/Platform/PlatformTypes.fpp")
cd "$ROOT"
cargo run -q -p fprime-fpp --bin fpp-to-rust -- --dict "$OUT" \
  -p "$(grep -m1 '^version' Cargo.toml | sed 's/.*"\(.*\)"/\1/')" -f "$VERSION" \
  "${IMPORTS[@]}" "$HERE/SignalGen.fpp" "$HERE/Ref.fpp"
echo "wrote $OUT/RefTopologyDictionary.json (frameworkVersion $VERSION)"
