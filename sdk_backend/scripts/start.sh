#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
VENV="$PKG/rbnx-build/venv"
PIPER_SDK_PARENT="${PIPER_SDK_PARENT:-}"

if [[ ! -x "$VENV/bin/python" ]]; then
  echo "piper_arm venv missing; run rbnx build first" >&2
  exit 2
fi
if [[ -z "$PIPER_SDK_PARENT" ]]; then
  echo "PIPER_SDK_PARENT is required and must contain the piper_sdk checkout" >&2
  exit 2
fi
if [[ ! -f "$PIPER_SDK_PARENT/piper_sdk/version.py" ]]; then
  echo "Piper SDK missing at $PIPER_SDK_PARENT/piper_sdk" >&2
  exit 2
fi

export PYTHONPATH="$PIPER_SDK_PARENT:$PKG:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
exec "$VENV/bin/python" -m piper_arm.main
