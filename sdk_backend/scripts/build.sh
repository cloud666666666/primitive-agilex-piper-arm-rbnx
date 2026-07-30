#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
VENV="$PKG/rbnx-build/venv"

mkdir -p "$PKG/rbnx-build"
if [[ ! -x "$VENV/bin/python" ]]; then
  uv venv --python /usr/bin/python3 "$VENV"
fi

ROBONIX_API="$(rbnx path robonix-api)"
uv pip install --python "$VENV/bin/python" --quiet \
  "$ROBONIX_API" "grpcio-tools>=1.50" "python-can>=3.3.4" "typing_extensions"

FLAGS=(--mcp)
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)
PATH="$VENV/bin:$PATH" rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[piper_arm/build] done"
