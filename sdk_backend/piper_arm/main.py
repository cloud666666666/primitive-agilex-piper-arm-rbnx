"""Robonix MCP primitive for one locally connected Piper arm."""

from __future__ import annotations

import logging
import os
from pathlib import Path
import threading
from typing import Any

from robonix_api import Err, Ok, Primitive

from piper_arm_mcp import (  # type: ignore[import-not-found]
    DisableArm_Request,
    DisableArm_Response,
    DisableGripper_Request,
    DisableGripper_Response,
    EmergencyStop_Request,
    EmergencyStop_Response,
    EnableArm_Request,
    EnableArm_Response,
    EnableGripper_Request,
    EnableGripper_Response,
    ExecuteJointTrajectory_Request,
    ExecuteJointTrajectory_Response,
    GetStatus_Request,
    GetStatus_Response,
    JogJoint_Request,
    JogJoint_Response,
    SetEnabled_Request,
    SetEnabled_Response,
    SetGripper_Request,
    SetGripper_Response,
    SetGripperZero_Request,
    SetGripperZero_Response,
)

from .controller import (
    JointTrajectoryExecutionError,
    JointTrajectoryWaypoint,
    PiperControlError,
    PiperController,
)

import health_pb2  # type: ignore[import-not-found]  # noqa: E402


logging.basicConfig(
    level=os.environ.get("PIPER_LOG_LEVEL", "INFO"),
    format="[%(name)s] %(levelname)s %(message)s",
)
log = logging.getLogger("piper_arm")

piper_arm = Primitive(id="piper_arm", namespace="robonix/primitive/arm")

_config: dict[str, Any] = {}
_controller: PiperController | None = None
_health_stop = threading.Event()


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _motion_unlocked(cfg: dict[str, Any]) -> bool:
    arm_label = str(cfg.get("arm_label", "")).strip().upper()
    arm_override = os.environ.get(f"ROBONIX_ALLOW_{arm_label}_MOTION")
    if arm_override is not None:
        return _as_bool(arm_override)
    override = os.environ.get("ROBONIX_ALLOW_MOTION")
    return _as_bool(override) if override is not None else _as_bool(cfg.get("allow_motion", False))


def _operator_confirmed_gripper_zero(cfg: dict[str, Any]) -> bool:
    arm_label = str(cfg.get("arm_label", "")).strip().upper()
    return _as_bool(os.environ.get(f"ROBONIX_CONFIRM_{arm_label}_GRIPPER_ZERO", "0"))


def _require_controller() -> PiperController:
    if _controller is None:
        raise PiperControlError("Piper primitive is not active")
    return _controller


def _health_state():
    """Convert a read-only SDK snapshot into the shared health contract."""
    snapshot = _require_controller().health_snapshot()
    readings = [health_pb2.SensorReading(**reading) for reading in snapshot["readings"]]
    return health_pb2.HealthState(
        voltage=snapshot["voltage"],
        charging=snapshot["charging"],
        remaining_s=snapshot["remaining_s"],
        readings=readings,
    )


@piper_arm.grpc("robonix/primitive/health/state")
def health_state(_request):
    """Return one live Piper telemetry frame without enabling or moving it."""
    return health_pb2.GetHealthState_Response(state=_health_state())


@piper_arm.grpc("robonix/primitive/health/stream")
def health_stream(_request, context):
    """Stream live Piper telemetry at 2 Hz until the consumer disconnects."""
    while context.is_active() and not _health_stop.is_set():
        yield _health_state()
        _health_stop.wait(0.5)


@piper_arm.mcp("robonix/primitive/arm/status")
def status(_req: GetStatus_Request) -> GetStatus_Response:
    """Read this provider's live arm and factory-gripper state without moving it.
    Call before motion and choose left_piper or right_piper by provider name."""
    try:
        state = _require_controller().status()
        return GetStatus_Response(
            connected=state.connected,
            enabled=state.enabled,
            motion_unlocked=state.motion_unlocked,
            arm=state.arm,
            can_port=state.can_port,
            joints_deg=list(state.joints_deg),
            telemetry_valid=state.telemetry_valid,
            joint_voltages_v=list(state.joint_voltages_v),
            joint_currents_a=list(state.joint_currents_a),
            joint_motor_temperatures_c=list(state.joint_motor_temperatures_c),
            joint_driver_temperatures_c=list(state.joint_driver_temperatures_c),
            joint_driver_enabled=list(state.joint_driver_enabled),
            joint_communication_ok=list(state.joint_communication_ok),
            joint_fault_codes=list(state.joint_fault_codes),
            gripper_opening_m=state.gripper_opening_m,
            gripper_enabled=state.gripper_enabled,
            gripper_homed=state.gripper_homed,
            gripper_fault=state.gripper_fault,
            gripper_status_code=state.gripper_status_code,
            error_code=state.error_code,
            detail=state.detail,
        )
    except Exception as exc:  # noqa: BLE001
        return GetStatus_Response(
            connected=False,
            enabled=False,
            motion_unlocked=False,
            arm=str(_config.get("arm_label", "unknown")),
            can_port=str(_config.get("can_port", "")),
            joints_deg=[],
            telemetry_valid=False,
            joint_voltages_v=[],
            joint_currents_a=[],
            joint_motor_temperatures_c=[],
            joint_driver_temperatures_c=[],
            joint_driver_enabled=[],
            joint_communication_ok=[],
            joint_fault_codes=[],
            gripper_opening_m=0.0,
            gripper_enabled=False,
            gripper_homed=False,
            gripper_fault=True,
            gripper_status_code=-1,
            error_code=-1,
            detail=str(exc),
        )


@piper_arm.mcp("robonix/primitive/arm/set_enabled")
def set_enabled(req: SetEnabled_Request) -> SetEnabled_Response:
    """Backward-compatible arm enable switch. Prefer the explicit enable_arm
    and disable_arm capabilities for new plans."""
    arm = str(_config.get("arm_label", "unknown"))
    try:
        enabled = _require_controller().set_enabled(bool(req.enabled))
        return SetEnabled_Response(
            success=True,
            enabled=enabled,
            arm=arm,
            detail=f"{arm} arm {'enabled' if enabled else 'disabled'}",
        )
    except Exception as exc:  # noqa: BLE001
        return SetEnabled_Response(success=False, enabled=False, arm=arm, detail=str(exc))


@piper_arm.mcp("robonix/primitive/arm/enable_arm")
def enable_arm(_req: EnableArm_Request) -> EnableArm_Response:
    """Enable this arm if it is disabled. This is idempotent, does not command
    a pose, and leaves the arm enabled until an explicit disable or stop."""
    arm = str(_config.get("arm_label", "unknown"))
    try:
        enabled = _require_controller().set_enabled(True)
        return EnableArm_Response(
            success=True, enabled=enabled, arm=arm, detail=f"{arm} arm enabled"
        )
    except Exception as exc:  # noqa: BLE001
        return EnableArm_Response(success=False, enabled=False, arm=arm, detail=str(exc))


@piper_arm.mcp("robonix/primitive/arm/disable_arm")
def disable_arm(_req: DisableArm_Request) -> DisableArm_Response:
    """Disable this arm only when explicitly requested by the user. This is
    idempotent and sends no gripper-disable command."""
    arm = str(_config.get("arm_label", "unknown"))
    try:
        enabled = _require_controller().set_enabled(False)
        return DisableArm_Response(
            success=True, enabled=enabled, arm=arm, detail=f"{arm} arm disabled"
        )
    except Exception as exc:  # noqa: BLE001
        return DisableArm_Response(success=False, enabled=True, arm=arm, detail=str(exc))


@piper_arm.mcp("robonix/primitive/arm/enable_gripper")
def enable_gripper(_req: EnableGripper_Request) -> EnableGripper_Response:
    """Enable this arm's factory gripper if it is disabled. The arm joint
    drivers remain unchanged."""
    arm = str(_config.get("arm_label", "unknown"))
    try:
        enabled = _require_controller().set_gripper_enabled(True)
        return EnableGripper_Response(
            success=True,
            enabled=enabled,
            arm=arm,
            detail=f"{arm} gripper enabled",
        )
    except Exception as exc:  # noqa: BLE001
        return EnableGripper_Response(
            success=False, enabled=False, arm=arm, detail=str(exc)
        )


@piper_arm.mcp("robonix/primitive/arm/disable_gripper")
def disable_gripper(_req: DisableGripper_Request) -> DisableGripper_Response:
    """Disable this arm's factory gripper only when explicitly requested by the
    user. The arm drivers remain unchanged."""
    arm = str(_config.get("arm_label", "unknown"))
    try:
        enabled = _require_controller().set_gripper_enabled(False)
        return DisableGripper_Response(
            success=True,
            enabled=enabled,
            arm=arm,
            detail=f"{arm} gripper disabled",
        )
    except Exception as exc:  # noqa: BLE001
        return DisableGripper_Response(
            success=False, enabled=True, arm=arm, detail=str(exc)
        )


@piper_arm.mcp("robonix/primitive/arm/jog_joint")
def jog_joint(req: JogJoint_Request) -> JogJoint_Response:
    """Move one joint of this arm by a small relative angle. Joint is 1..6,
    delta_deg is signed and bounded by deployment policy, and speed_percent=0
    selects the low default speed. A disabled arm is enabled first and remains
    enabled afterward. Never use this as a Cartesian pose command."""
    arm = str(_config.get("arm_label", "unknown"))
    try:
        targets = _require_controller().jog_joint(
            int(req.joint), float(req.delta_deg), int(req.speed_percent)
        )
        return JogJoint_Response(
            success=True,
            arm=arm,
            target_joints_deg=list(targets),
            detail=f"{arm} joint{int(req.joint)} jog completed",
        )
    except Exception as exc:  # noqa: BLE001
        return JogJoint_Response(success=False, arm=arm, target_joints_deg=[], detail=str(exc))


@piper_arm.mcp("robonix/primitive/arm/execute_joint_trajectory")
def execute_joint_trajectory(
    req: ExecuteJointTrajectory_Request,
) -> ExecuteJointTrajectory_Response:
    """Execute recorded absolute J1-J6 waypoints in order.

    All waypoints are validated before motion. This joint-space operation has
    no collision or perception awareness and must only use operator-verified
    paths. A disabled arm is enabled first and remains enabled afterward; the
    gripper is not commanded.
    """
    arm = str(_config.get("arm_label", "unknown"))
    try:
        waypoints = [
            JointTrajectoryWaypoint(
                name=str(waypoint.name),
                joints_deg=tuple(float(value) for value in waypoint.joints_deg),
            )
            for waypoint in req.waypoints
        ]
        completed, final_joints = _require_controller().execute_joint_trajectory(
            waypoints, int(req.speed_percent)
        )
        return ExecuteJointTrajectory_Response(
            success=True,
            arm=arm,
            completed_waypoints=completed,
            failed_waypoint="",
            final_joints_deg=list(final_joints),
            detail=f"{arm} completed {completed} joint trajectory waypoints",
        )
    except JointTrajectoryExecutionError as exc:
        return ExecuteJointTrajectory_Response(
            success=False,
            arm=arm,
            completed_waypoints=exc.completed_waypoints,
            failed_waypoint=exc.failed_waypoint,
            final_joints_deg=list(exc.final_joints_deg),
            detail=str(exc),
        )
    except Exception as exc:  # noqa: BLE001
        return ExecuteJointTrajectory_Response(
            success=False,
            arm=arm,
            completed_waypoints=0,
            failed_waypoint="",
            final_joints_deg=[],
            detail=str(exc),
        )


@piper_arm.mcp("robonix/primitive/arm/set_gripper")
def set_gripper(req: SetGripper_Request) -> SetGripper_Response:
    """Set this arm's factory parallel gripper opening in metres. Use 0.0 to
    close and at most 0.07 to open; effort_nm=0 selects the conservative default.
    Disabled arm or gripper drivers are enabled first and remain enabled."""
    arm = str(_config.get("arm_label", "unknown"))
    try:
        result = _require_controller().set_gripper(
            float(req.opening_m), float(req.effort_nm)
        )
        detail = (
            f"{arm} stable closing contact detected"
            if result.contact_detected
            else f"{arm} gripper command completed"
        )
        return SetGripper_Response(
            success=True,
            arm=arm,
            opening_m=result.opening_m,
            contact_detected=result.contact_detected,
            detail=detail,
        )
    except Exception as exc:  # noqa: BLE001
        return SetGripper_Response(
            success=False,
            arm=arm,
            opening_m=0.0,
            contact_detected=False,
            detail=str(exc),
        )


@piper_arm.mcp("robonix/primitive/arm/set_gripper_zero")
def set_gripper_zero(req: SetGripperZero_Request) -> SetGripperZero_Response:
    """Set the current fully closed factory-gripper position to zero. Call only
    after the operator manually closes this provider's gripper and explicitly
    confirms that state. The gripper remains disabled and the arm does not move."""
    arm = str(_config.get("arm_label", "unknown"))
    try:
        opening = _require_controller().set_gripper_zero(
            bool(req.fully_closed_confirmed)
        )
        return SetGripperZero_Response(
            success=True,
            arm=arm,
            opening_m=opening,
            homed=True,
            detail=f"{arm} gripper zero set and verified",
        )
    except Exception as exc:  # noqa: BLE001
        return SetGripperZero_Response(
            success=False,
            arm=arm,
            opening_m=0.0,
            homed=False,
            detail=str(exc),
        )


@piper_arm.mcp("robonix/primitive/arm/emergency_stop")
def emergency_stop(_req: EmergencyStop_Request) -> EmergencyStop_Response:
    """Immediately issue the Piper fast emergency-stop command to this arm.
    This remains available even when normal hardware motion is locked."""
    arm = str(_config.get("arm_label", "unknown"))
    try:
        _require_controller().emergency_stop()
        return EmergencyStop_Response(success=True, arm=arm, detail=f"{arm} emergency stop sent")
    except Exception as exc:  # noqa: BLE001
        return EmergencyStop_Response(success=False, arm=arm, detail=str(exc))


@piper_arm.on_init
def init(cfg: dict[str, Any]):
    global _config
    cfg = dict(cfg or {})
    can_port = str(cfg.get("can_port", "")).strip()
    arm_label = str(cfg.get("arm_label", "")).strip().lower()
    if arm_label not in {"left", "right"}:
        return Err("arm_label must be 'left' or 'right'")
    if not can_port:
        return Err("can_port is required")
    if not Path("/sys/class/net", can_port).exists():
        return Err(f"CAN interface {can_port!r} does not exist")
    _config = cfg
    log.info(
        "init arm=%s can=%s motion_unlocked=%s",
        arm_label,
        can_port,
        _motion_unlocked(cfg),
    )
    return Ok()


@piper_arm.on_activate
def activate():
    global _controller
    try:
        from piper_sdk import C_PiperInterface_V2

        sdk = C_PiperInterface_V2(
            can_name=str(_config["can_port"]),
            judge_flag=True,
            can_auto_init=True,
            start_sdk_joint_limit=False,
            start_sdk_gripper_limit=False,
        )
        _controller = PiperController(
            sdk,
            arm_label=str(_config["arm_label"]),
            can_port=str(_config["can_port"]),
            allow_motion=_motion_unlocked(_config),
            max_jog_deg=float(_config.get("max_jog_deg", 5.0)),
            max_trajectory_waypoints=int(
                _config.get("max_trajectory_waypoints", 32)
            ),
            default_speed_percent=int(_config.get("default_speed_percent", 8)),
            max_speed_percent=int(_config.get("max_speed_percent", 15)),
            feedback_timeout_s=float(_config.get("feedback_timeout_s", 6.0)),
            motion_timeout_s=float(_config.get("motion_timeout_s", 12.0)),
            joint_position_tolerance_deg=float(
                _config.get("joint_position_tolerance_deg", 1.5)
            ),
            gripper_max_opening_m=float(_config.get("gripper_max_opening_m", 0.07)),
            gripper_multiplier=float(_config.get("gripper_multiplier", 1.0)),
            max_gripper_effort_nm=float(_config.get("max_gripper_effort_nm", 1.5)),
            gripper_zero_max_offset_m=float(
                _config.get("gripper_zero_max_offset_m", 0.01)
            ),
            gripper_zero_settle_s=float(
                _config.get("gripper_zero_settle_s", 1.5)
            ),
            gripper_position_tolerance_m=float(
                _config.get("gripper_position_tolerance_m", 0.001)
            ),
            gripper_arm_stability_s=float(
                _config.get("gripper_arm_stability_s", 1.25)
            ),
            gripper_contact_max_target_m=float(
                _config.get("gripper_contact_max_target_m", 0.035)
            ),
            gripper_contact_min_closure_m=float(
                _config.get("gripper_contact_min_closure_m", 0.003)
            ),
            gripper_contact_stability_s=float(
                _config.get("gripper_contact_stability_s", 1.0)
            ),
            gripper_contact_position_epsilon_m=float(
                _config.get("gripper_contact_position_epsilon_m", 0.001)
            ),
            joint_limit_tolerance_deg=float(
                _config.get("joint_limit_tolerance_deg", 3.0)
            ),
            operator_confirmed_gripper_zero=_operator_confirmed_gripper_zero(_config),
            command_period_s=float(_config.get("command_period_s", 0.02)),
        )
        _controller.connect()
        _health_stop.clear()
        log.info("active arm=%s can=%s (no enable or motion sent)", _config["arm_label"], _config["can_port"])
        return Ok()
    except Exception as exc:  # noqa: BLE001
        _controller = None
        return Err(f"Piper activation failed: {exc}")


def _disconnect() -> None:
    global _controller
    controller = _controller
    _controller = None
    _health_stop.set()
    if controller is not None:
        controller.disconnect(emergency_stop=_as_bool(_config.get("stop_on_shutdown", True)))


@piper_arm.on_deactivate
def deactivate():
    _disconnect()
    return Ok()


@piper_arm.on_shutdown
def shutdown():
    _disconnect()
    return Ok()


def main() -> int:
    piper_arm.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
