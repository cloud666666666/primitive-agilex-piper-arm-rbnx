#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Unified AgileX Piper primitive with ROS 2 and direct-SDK backends.

The shared lifecycle selects `backend: ros2 | sdk` during CMD_INIT. The
default ROS path wraps the vendored `piper` driver; the SDK path delegates to
`piper_arm_sdk.main` and exposes its MCP, gripper and health handlers.

Lifecycle (per Robonix developer guide §5):
    on_init       — light: validate cfg, cache for activate.
    on_activate   — heavy: optionally run scripts/can_activate.sh
                    (auto_can_setup=true), spawn
                    `ros2 launch piper start_single_piper.launch.py …`,
                    wait for the first JointState on
                    /<ns>/joint_states_single as proof the CAN link
                    is up, declare 5 ROS 2 topics on atlas
                    (joint_states / joint_command / end_pose /
                    pos_command / arm_status) — `arm/driver` is
                    auto-declared by the framework via the generated
                    lifecycle Servicer.
    on_deactivate — symmetric: kill piper subprocess.
    on_shutdown   — last-chance kill (idempotent w/ on_deactivate).

Config (from manifest's primitive[].config block, delivered via
Driver(CMD_INIT, config_json)):
    can_port            default "can_piper"
    can_bitrate         default 1000000
    can_usb_address     default ""             — pin via USB bus path
    auto_can_setup      default false          — run scripts/can_activate.sh
    auto_enable         default true
    gripper_exist       default true
    gripper_val_mutiple default 2              (upstream typo preserved)
    arm_namespace       default "/arm"         — see note below
    joint_states_topic   default "/<ns>/joint_states_single"
    joint_command_topic  default "/<ns>/joint_command"
    arm_status_topic     default "/<ns>/arm_status"
    end_pose_topic       default "/<ns>/end_pose"
    pos_command_topic    default "/<ns>/pos_command"
    sentinel_timeout_s   default 30.0

NOTE on `arm_namespace`: the upstream `start_single_piper.launch.py`
hard-codes `namespace='/arm'` (no launch_arg). We expose
`arm_namespace` in cfg only so the operator can override the
DERIVED topic names (joint_states_topic etc.) when they manually
remap with `ros2 run` outside this package — the launch file we
spawn always uses `/arm`. Don't bother changing arm_namespace
unless you also fork the launch file.
"""
from __future__ import annotations

import logging
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Optional

from robonix_api import Ok, Err

from piper_provider import piper_arm as piper_ctl

logging.basicConfig(
    level=os.environ.get("PIPER_LOG_LEVEL", "INFO"),
    format="[piper] %(message)s",
)
log = logging.getLogger("piper")

_pkg_root: Path = Path(__file__).resolve().parent.parent

# Subprocess + cached cfg. Allocated in on_activate, released in
# on_deactivate / on_shutdown. Module-level so the kill helper is
# reachable from every lifecycle callback.
_piper_proc: Optional[subprocess.Popen] = None
_piper_pgid: Optional[int] = None
_resolved_cfg: Optional[dict[str, Any]] = None


def _bool_arg(v: Any) -> str:
    """Coerce truthy Python values into the upstream piper launch
    file's expected casing — start_single_piper.launch.py default
    values are 'true'/'false' (lowercase) but the original run.sh
    passes 'True' (capitalised). Either form parses the same in
    rclpy's launch arg evaluator, but keep lowercase to match the
    declared default and avoid surprising operators."""
    return "True" if bool(v) else "False"


def _can_activate(cfg: dict) -> Optional[str]:
    """Run the vendored scripts/can_activate.sh once. Returns None on
    success, an error string on failure.

    Keep this OFF by default. The script needs sudo, which means
    running it from inside `rbnx boot` requires the operator to have
    set up passwordless sudo. The recommended path is to bring up
    can_piper once at host boot via systemd / udev and leave
    `auto_can_setup=false` here. This helper exists for dev laptops
    where USB-CAN paths shift between sessions and the convenience
    is worth the sudo coupling.
    """
    script = _pkg_root / "scripts" / "can_activate.sh"
    if not script.is_file():
        return f"scripts/can_activate.sh missing at {script}"
    can_port = str(cfg.get("can_port", "can_piper"))
    can_bitrate = str(cfg.get("can_bitrate", 1000000))
    can_usb_address = str(cfg.get("can_usb_address", ""))
    args = ["bash", str(script), can_port, can_bitrate]
    if can_usb_address:
        args.append(can_usb_address)
    log.info("auto_can_setup: %s", " ".join(args))
    try:
        out = subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=15.0,
            check=False,
        )
    except FileNotFoundError as e:
        return f"can_activate.sh: spawn failed: {e}"
    except subprocess.TimeoutExpired:
        return (
            "can_activate.sh: timed out after 15s (sudo prompt blocked?). "
            "For manual diagnosis run "
            "`bash scripts/find_all_can_port.sh` in piper_ctl_rbnx."
        )
    if out.returncode != 0:
        # The script's own stderr is the most useful diagnostic.
        tail = (out.stderr or out.stdout or "").strip().splitlines()[-5:]
        return (
            f"can_activate.sh: exit {out.returncode}; tail: "
            + " | ".join(tail)
            + "; run `bash scripts/find_all_can_port.sh` to inspect "
            "the current CAN interface/USB-port mapping."
        )
    return None


def _spawn_piper(cfg: dict) -> None:
    """Launch ros2 launch piper start_single_piper.launch.py with config args.

    start_new_session=True so the whole process group can be torn
    down by signalling its PGID — matters because the launch spawns
    the actual driver node which itself spawns CAN-side helper
    threads. A flat SIGTERM only kills the parent.
    """
    global _piper_proc, _piper_pgid
    args = [
        "ros2", "launch", "piper", "start_single_piper.launch.py",
        f"can_port:={cfg.get('can_port', 'can_piper')}",
        f"auto_enable:={_bool_arg(cfg.get('auto_enable', True))}",
        f"gripper_exist:={_bool_arg(cfg.get('gripper_exist', True))}",
        f"gripper_val_mutiple:={int(cfg.get('gripper_val_mutiple', 2))}",
        f"gripper_stroke_m:={float(cfg.get('gripper_stroke_m', 0.10))}",
    ]
    log_path = _pkg_root / "rbnx-build" / "data" / "piper.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_fh = open(log_path, "ab", buffering=0)
    log.info("spawning piper driver (can_port=%s) → %s",
             cfg.get("can_port", "can_piper"), log_path)
    log.debug("launch args: %s", " ".join(args))
    _piper_proc = subprocess.Popen(
        args, stdout=log_fh, stderr=log_fh, start_new_session=True,
    )
    _piper_pgid = os.getpgid(_piper_proc.pid)


def _kill_piper() -> None:
    """Tear down the launched ros2 process group. Idempotent — safe
    to call from on_deactivate followed by on_shutdown without raising
    on the second call."""
    global _piper_proc, _piper_pgid
    p = _piper_proc
    pgid = _piper_pgid
    if pgid is None and p is not None:
        try:
            pgid = os.getpgid(p.pid)
        except ProcessLookupError:
            pgid = None
    if pgid is None:
        _piper_proc = None
        _piper_pgid = None
        return

    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        _piper_proc = None
        _piper_pgid = None
        return

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            _piper_proc = None
            _piper_pgid = None
            return
        time.sleep(0.1)

    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    if p is not None:
        try:
            p.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            pass
    _piper_proc = None
    _piper_pgid = None


def _wait_for_joint_states(topic: str, timeout_s: float) -> bool:
    """Spin up a one-shot rclpy node, subscribe to `topic`, return
    True when the first sensor_msgs/JointState arrives within timeout.

    JointState is published by piper_ctrl_single_node from a separate
    `publish_thread` which depends on the CAN read loop succeeding —
    so receiving the first message is the proof we want that:
        (a) the launch process is up
        (b) the CAN interface accepted reads from the arm
        (c) the arm's MCU answered our enable request

    QoS RELIABLE because the upstream publisher uses depth=1 default
    QoS (which is reliable). BEST_EFFORT here would still receive
    messages but RELIABLE is the unambiguous match. Mirrors
    ranger_chassis/main.py::_wait_for_odom.
    """
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import JointState
    except ImportError as e:
        log.warning("rclpy unavailable (%s); skipping sentinel wait", e)
        return True
    rclpy.init(args=None)
    node = Node("piper_atlas_sentinel")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    seen = threading.Event()
    node.create_subscription(JointState, topic, lambda _m: seen.set(), qos)
    log.info("waiting for first JointState on %s — up to %.1fs",
             topic, timeout_s)
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if seen.is_set():
                break
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return seen.is_set()


# ── lifecycle handlers ───────────────────────────────────────────────────
def _ros_init(cfg: dict):
    """REGISTERED → INACTIVE. Validate cfg + cache for activate.

    Light only — DO NOT touch CAN, DO NOT spawn ros2, DO NOT declare
    on atlas. Heavy work belongs in on_activate so a CMD_DEACTIVATE
    → CMD_ACTIVATE re-cycle works without a half-baked init side
    effect."""
    global _resolved_cfg
    cfg = cfg or {}
    try:
        sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))
        if sentinel_timeout <= 0:
            return Err(f"sentinel_timeout_s must be > 0, got {sentinel_timeout}")
    except (TypeError, ValueError) as e:
        return Err(f"sentinel_timeout_s not numeric: {e}")
    try:
        bitrate = int(cfg.get("can_bitrate", 1000000))
        if bitrate <= 0:
            return Err(f"can_bitrate must be > 0, got {bitrate}")
    except (TypeError, ValueError) as e:
        return Err(f"can_bitrate not integer: {e}")
    _resolved_cfg = dict(cfg)
    log.info("CMD_INIT ok (can_port=%s, auto_can_setup=%s, gripper_exist=%s)",
             cfg.get("can_port", "can_piper"),
             cfg.get("auto_can_setup", False),
             cfg.get("gripper_exist", True))
    return Ok()


def _ros_activate():
    """INACTIVE → ACTIVE. (Optionally) bring up CAN, spawn
    start_single_piper.launch.py, wait for the first JointState, then
    atlas-declare the four data topics.

    On any failure between spawn and declare, the piper subprocess is
    torn down before returning Err so the next CMD_ACTIVATE starts
    from a clean state."""
    cfg = _resolved_cfg or {}

    # Topic names — derive from arm_namespace, allow override.
    ns = str(cfg.get("arm_namespace", "/arm"))
    if not ns.startswith("/"):
        ns = "/" + ns
    joint_states_topic = str(cfg.get(
        "joint_states_topic", f"{ns}/joint_states_single",
    ))
    joint_command_topic = str(cfg.get(
        "joint_command_topic", f"{ns}/joint_command",
    ))
    arm_status_topic = str(cfg.get("arm_status_topic", f"{ns}/arm_status"))
    end_pose_topic = str(cfg.get("end_pose_topic", f"{ns}/end_pose"))
    pos_command_topic = str(cfg.get("pos_command_topic", f"{ns}/pos_command"))
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))

    # Optional CAN bring-up. Off by default (sudo coupling); operators
    # are expected to have can_piper UP before `rbnx boot`. See README.
    if bool(cfg.get("auto_can_setup", False)):
        err = _can_activate(cfg)
        if err is not None:
            return Err(f"auto_can_setup failed: {err}")

    try:
        _spawn_piper(cfg)
    except Exception as e:  # noqa: BLE001
        return Err(f"spawn piper failed: {e}")

    if not _wait_for_joint_states(joint_states_topic, sentinel_timeout):
        _kill_piper()
        return Err(
            f"no JointState on {joint_states_topic} within "
            f"{sentinel_timeout:.1f}s (check rbnx-build/data/piper.log; "
            f"is can_piper UP? `ip link show can_piper`. is the arm "
            f"powered + USB-CAN attached?)"
        )

    try:
        piper_ctl.declare_ros2_topic(
            "robonix/primitive/arm/joint_states",
            topic=joint_states_topic,
            qos="reliable",
            description=(
                f"Piper arm joint feedback (sensor_msgs/JointState, "
                f"6 joints + gripper). Use this for FK / "
                f"robot_state_publisher / MoveIt state monitor."
            ),
        )
        # joint_command is topic_in (driver subscribes; consumers publish).
        piper_ctl.declare_ros2_topic(
            "robonix/primitive/arm/joint_command",
            topic=joint_command_topic,
            qos="reliable",
            description=(
                f"Named joint target (sensor_msgs/JointState). PUBLISH "
                f"here to drive the arm in joint space. A `gripper` "
                f"entry in the JointState commands the finger opening; "
                f"joint velocities beyond a small deadband switch the "
                f"driver into MIT-style ctrl."
            ),
        )
        piper_ctl.declare_ros2_topic(
            "robonix/primitive/arm/end_pose",
            topic=end_pose_topic,
            qos="reliable",
            description=(
                f"End-effector pose (geometry_msgs/Pose) computed by "
                f"the upstream driver. Frame: arm/base_link."
            ),
        )
        # pos_command is topic_in (driver subscribes; consumers publish).
        piper_ctl.declare_ros2_topic(
            "robonix/primitive/arm/pos_command",
            topic=pos_command_topic,
            qos="reliable",
            description=(
                f"Cartesian end-effector target (geometry_msgs/Pose) "
                f"in arm/base_link. PUBLISH here to drive the arm "
                f"through the driver's internal Cartesian controller. "
                f"Gripper is NOT part of this contract; command it "
                f"through joint_command."
            ),
        )
        # arm_status is vendor-specific (piper_msgs/PiperStatusMsg) and
        # is NOT part of the robonix global primitive/arm/* set. Kept
        # because pick_skill_rbnx / piper_moveit_rbnx rely on it.
        piper_ctl.declare_ros2_topic(
            "robonix/primitive/arm/arm_status",
            topic=arm_status_topic,
            qos="reliable",
            description=(
                f"Piper arm status word (piper_msgs/PiperStatusMsg). "
                f"`arm_status==0` means idle/ready; >0 means busy or "
                f"faulted (see ctrl_mode / err_code fields). pick.py "
                f"polls this between consecutive grasp commands."
            ),
        )
    except Exception as e:  # noqa: BLE001
        _kill_piper()
        return Err(f"declare_ros2_topic failed: {e}")

    log.info(
        "CMD_ACTIVATE ok: joint_states=%s joint_command=%s end_pose=%s "
        "pos_command=%s arm_status=%s",
        joint_states_topic, joint_command_topic, end_pose_topic,
        pos_command_topic, arm_status_topic,
    )
    return Ok()


def _ros_deactivate():
    """ACTIVE → INACTIVE. Kill the piper subprocess. Idempotent."""
    _kill_piper()
    log.info("CMD_DEACTIVATE ok")
    return Ok()


def _ros_shutdown():
    """any → TERMINATED. Last-chance kill. Idempotent w/ on_deactivate."""
    _kill_piper()
    return Ok()


# Importing the SDK module registers its MCP and health handlers on the shared
# provider. It does not import or connect piper_sdk until SDK activation.
from piper_arm_sdk import main as sdk_backend  # noqa: E402


_backend = "ros2"


@piper_ctl.on_init
def init(cfg: dict):
    """Select one backend from CMD_INIT config and validate its options."""
    global _backend
    cfg = dict(cfg or {})
    backend = str(cfg.get("backend", "ros2")).strip().lower()
    if backend not in {"ros2", "sdk"}:
        return Err("backend must be 'ros2' or 'sdk'")
    _backend = backend
    log.info("selected backend=%s", backend)
    if backend == "sdk":
        return sdk_backend.init(cfg)
    return _ros_init(cfg)


@piper_ctl.on_activate
def activate():
    if _backend == "sdk":
        return sdk_backend.activate()
    return _ros_activate()


@piper_ctl.on_deactivate
def deactivate():
    if _backend == "sdk":
        return sdk_backend.deactivate()
    return _ros_deactivate()


@piper_ctl.on_shutdown
def shutdown():
    if _backend == "sdk":
        return sdk_backend.shutdown()
    return _ros_shutdown()


if __name__ == "__main__":
    piper_ctl.run()
