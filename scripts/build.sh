#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
BUILD="$PKG/rbnx-build"
VENV="$BUILD/venv"
CLEAN="${RBNX_BUILD_CLEAN:-}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
PIPER_SDK_ENV="$BUILD/piper_sdk.env"
PIPER_SDK_REF="${RBNX_PIPER_SDK_REF:-4eddfcf817cd87de9acee316a72cf5b988025378}"
PIPER_SDK_REPO="${RBNX_PIPER_SDK_REPO:-https://github.com/agilexrobotics/piper_sdk.git}"

cd "$PKG"
if [[ "$CLEAN" == "1" ]]; then
    echo "[piper_arm/build] clean: removing $BUILD"
    rm -rf -- "$BUILD"
fi
mkdir -p "$BUILD/ws/src" "$BUILD/data"

# Build the original ROS backend whenever its host toolchain is available.
# SDK-only hosts can still build and run the direct backend without ROS.
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" && -x /usr/bin/colcon ]]; then
    ln -snf "$PKG/src/piper" "$BUILD/ws/src/piper"
    ln -snf "$PKG/src/piper_msgs" "$BUILD/ws/src/piper_msgs"
    ln -snf "$PKG/src/graspnet_msgs" "$BUILD/ws/src/graspnet_msgs"
    echo "[piper_arm/build] colcon build (ROS 2 backend)"
    (
        unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER PYTHONPATH
        export PATH="/opt/ros/${ROS_DISTRO}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
        # shellcheck disable=SC1091
        set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u
        cd "$BUILD/ws"
        /usr/bin/colcon build --symlink-install \
            --packages-select piper piper_msgs graspnet_msgs \
            --event-handlers console_direct+ \
            --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
    )
else
    echo "[piper_arm/build] WARN: ROS ${ROS_DISTRO}/colcon unavailable; skipping ROS backend build" >&2
fi

command -v uv >/dev/null 2>&1 || {
    echo "[piper_arm/build] ERR: uv is required" >&2
    exit 2
}
RBNX_BIN="$(command -v rbnx)"
if [[ ! -x "$VENV/bin/python" ]]; then
    uv venv --python /usr/bin/python3 --system-site-packages "$VENV"
fi

ROBONIX_API="$($RBNX_BIN path robonix-api)"
uv pip install --python "$VENV/bin/python" --quiet \
    "$ROBONIX_API" "grpcio-tools>=1.50" "python-can>=3.3.4" "typing_extensions"

# The SDK backend uses constructor options that are not present in older
# piper_sdk releases. Keep the tested source revision beside the build output
# and make start.sh load it before any system-installed SDK.
rm -f -- "$PIPER_SDK_ENV"
if [[ "${RBNX_PIPER_SDK_BUILD:-1}" != "0" ]]; then
    if [[ ! "$PIPER_SDK_REF" =~ ^[0-9a-fA-F]{40}$ ]]; then
        echo "[piper_arm/build] ERR: RBNX_PIPER_SDK_REF must be a full 40-character commit SHA" >&2
        exit 3
    fi

    if [[ -n "${RBNX_PIPER_SDK_PARENT:-}" ]]; then
        if [[ ! -d "$RBNX_PIPER_SDK_PARENT/piper_sdk" ]]; then
            echo "[piper_arm/build] ERR: RBNX_PIPER_SDK_PARENT=$RBNX_PIPER_SDK_PARENT does not contain piper_sdk/" >&2
            exit 3
        fi
        PIPER_SDK_PARENT="$(cd "$RBNX_PIPER_SDK_PARENT" && pwd)"
        echo "[piper_arm/build] using caller-provided piper_sdk: $PIPER_SDK_PARENT/piper_sdk"
    else
        PIPER_SDK_PARENT="$BUILD/piper_sdk_parent"
        PIPER_SDK_DIR="$PIPER_SDK_PARENT/piper_sdk"
        mkdir -p "$PIPER_SDK_PARENT"

        if [[ -e "$PIPER_SDK_DIR" && ! -d "$PIPER_SDK_DIR/.git" ]]; then
            echo "[piper_arm/build] ERR: $PIPER_SDK_DIR exists but is not a Git checkout; remove it or run a clean build" >&2
            exit 3
        fi
        if [[ ! -d "$PIPER_SDK_DIR/.git" ]]; then
            command -v git >/dev/null 2>&1 || {
                echo "[piper_arm/build] ERR: git is required to fetch piper_sdk" >&2
                exit 3
            }
            echo "[piper_arm/build] creating piper_sdk checkout from $PIPER_SDK_REPO"
            git init --quiet "$PIPER_SDK_DIR"
            git -C "$PIPER_SDK_DIR" remote add origin "$PIPER_SDK_REPO"
        fi

        if [[ -n "$(git -C "$PIPER_SDK_DIR" status --porcelain)" ]]; then
            echo "[piper_arm/build] ERR: cached piper_sdk checkout is dirty; remove $PIPER_SDK_DIR or run a clean build" >&2
            exit 3
        fi
        git -C "$PIPER_SDK_DIR" remote set-url origin "$PIPER_SDK_REPO"
        if ! git -C "$PIPER_SDK_DIR" cat-file -e "${PIPER_SDK_REF}^{commit}" 2>/dev/null; then
            echo "[piper_arm/build] fetching piper_sdk commit $PIPER_SDK_REF"
            git -C "$PIPER_SDK_DIR" fetch --quiet --depth 1 origin "$PIPER_SDK_REF" || \
                git -C "$PIPER_SDK_DIR" fetch --quiet origin
        fi
        if ! git -C "$PIPER_SDK_DIR" cat-file -e "${PIPER_SDK_REF}^{commit}" 2>/dev/null; then
            echo "[piper_arm/build] ERR: piper_sdk commit $PIPER_SDK_REF is unavailable from $PIPER_SDK_REPO" >&2
            exit 3
        fi
        git -C "$PIPER_SDK_DIR" -c advice.detachedHead=false \
            checkout --quiet --detach "$PIPER_SDK_REF"
        if [[ "$(git -C "$PIPER_SDK_DIR" rev-parse HEAD)" != "$PIPER_SDK_REF" ]]; then
            echo "[piper_arm/build] ERR: piper_sdk checkout did not resolve to $PIPER_SDK_REF" >&2
            exit 3
        fi
    fi

    # Verify both the source selected at runtime and the constructor surface
    # used by piper_arm_sdk.main. This catches stale site packages and bad
    # caller-provided checkouts without opening a CAN interface.
    if ! PIPER_SDK_EXPECTED_DIR="$PIPER_SDK_PARENT/piper_sdk" \
        PYTHONPATH="$PIPER_SDK_PARENT${PYTHONPATH:+:$PYTHONPATH}" \
        "$VENV/bin/python" - <<'PY'
import inspect
import os
from pathlib import Path
import sys

import piper_sdk
from piper_sdk import C_PiperInterface_V2

module_file = getattr(piper_sdk, "__file__", None)
expected_dir = Path(os.environ["PIPER_SDK_EXPECTED_DIR"]).resolve()
if module_file is None or Path(module_file).resolve().parent != expected_dir:
    print(
        f"expected piper_sdk from {expected_dir}, loaded {module_file!r}",
        file=sys.stderr,
    )
    sys.exit(1)

required = {
    "can_name",
    "judge_flag",
    "can_auto_init",
    "start_sdk_joint_limit",
    "start_sdk_gripper_limit",
}
missing = sorted(required - set(inspect.signature(C_PiperInterface_V2).parameters))
if missing:
    print(f"piper_sdk is missing required constructor options: {missing}", file=sys.stderr)
    sys.exit(1)
PY
    then
        echo "[piper_arm/build] ERR: piper_sdk validation failed for $PIPER_SDK_PARENT/piper_sdk" >&2
        exit 4
    fi

    {
        echo "# Generated by scripts/build.sh; do not edit."
        printf 'export PIPER_SDK_PARENT=%q\n' "$PIPER_SDK_PARENT"
    } > "$PIPER_SDK_ENV"
    echo "[piper_arm/build] piper_sdk ready at $PIPER_SDK_PARENT/piper_sdk"
else
    echo "[piper_arm/build] WARN: RBNX_PIPER_SDK_BUILD=0; SDK backend requires PIPER_SDK_PARENT at runtime" >&2
fi

FLAGS=(--mcp --out-dir "$BUILD/codegen")
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[piper_arm/build] rbnx codegen ${FLAGS[*]}"
PATH="$VENV/bin:$PATH" "$RBNX_BIN" codegen -p "$PKG" "${FLAGS[@]}"

touch "$BUILD/.rbnx-built"
echo "[piper_arm/build] done"
