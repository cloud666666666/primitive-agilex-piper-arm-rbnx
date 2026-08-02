#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
VENV="$PKG/rbnx-build/venv"
ROS_DISTRO="${ROS_DISTRO:-humble}"

cd "$PKG"
if [[ ! -x "$VENV/bin/python" ]]; then
    echo "[piper_arm/start] ERR: build environment missing; run rbnx build first" >&2
    exit 2
fi

# CMD_INIT delivers backend config only after this process registers, so the
# Python lifecycle dispatcher selects ros2 or sdk. Source ROS opportunistically:
# it is required by ros2 and harmless for sdk.
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u
fi
if [[ -f "$PKG/rbnx-build/ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u; source "$PKG/rbnx-build/ws/install/setup.bash"; set -u
fi

if [[ -z "${PIPER_SDK_PARENT:-}" && -f "$PKG/rbnx-build/piper_sdk.env" ]]; then
    # build.sh records its pinned checkout here. An explicit caller value wins.
    # shellcheck disable=SC1091
    set +u; source "$PKG/rbnx-build/piper_sdk.env"; set -u
fi
if [[ -n "${PIPER_SDK_PARENT:-}" ]]; then
    export PYTHONPATH="$PIPER_SDK_PARENT:${PYTHONPATH:-}"
fi

if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PKG:${PYTHONPATH:-}"
else
    export PYTHONPATH="$PKG:${PYTHONPATH:-}"
fi
export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:$PYTHONPATH"

exec "$VENV/bin/python" -m piper_ctl.main
