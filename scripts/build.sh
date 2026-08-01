#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
BUILD="$PKG/rbnx-build"
VENV="$BUILD/venv"
CLEAN="${RBNX_BUILD_CLEAN:-}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

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

FLAGS=(--mcp --out-dir "$BUILD/codegen")
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[piper_arm/build] rbnx codegen ${FLAGS[*]}"
PATH="$VENV/bin:$PATH" "$RBNX_BIN" codegen -p "$PKG" "${FLAGS[@]}"

touch "$BUILD/.rbnx-built"
echo "[piper_arm/build] done"
