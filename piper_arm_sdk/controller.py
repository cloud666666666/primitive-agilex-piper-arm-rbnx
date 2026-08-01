# SPDX-License-Identifier: MIT
"""Bounded, testable control logic around piper_sdk."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import math
import threading
import time
from typing import Any, Sequence


log = logging.getLogger(__name__)


LOCAL_JOINT_LIMITS_DEG: tuple[tuple[float, float], ...] = (
    (-150.0, 150.0),
    (0.0, math.degrees(3.14)),
    (math.degrees(-2.967), 0.0),
    (math.degrees(-1.745), math.degrees(1.745)),
    (math.degrees(-1.22), math.degrees(1.22)),
    (math.degrees(-2.0944), math.degrees(2.0944)),
)


class PiperControlError(RuntimeError):
    """A rejected or failed Piper command."""


class JointTrajectoryExecutionError(PiperControlError):
    """A trajectory failure with structured partial-progress information."""

    def __init__(
        self,
        message: str,
        *,
        completed_waypoints: int,
        failed_waypoint: str,
        final_joints_deg: Sequence[float],
    ) -> None:
        super().__init__(message)
        self.completed_waypoints = int(completed_waypoints)
        self.failed_waypoint = failed_waypoint
        self.final_joints_deg = tuple(float(value) for value in final_joints_deg)


@dataclass(frozen=True)
class PiperState:
    connected: bool
    enabled: bool
    motion_unlocked: bool
    arm: str
    can_port: str
    joints_deg: tuple[float, ...]
    telemetry_valid: bool
    joint_voltages_v: tuple[float, ...]
    joint_currents_a: tuple[float, ...]
    joint_motor_temperatures_c: tuple[float, ...]
    joint_driver_temperatures_c: tuple[float, ...]
    joint_driver_enabled: tuple[bool, ...]
    joint_communication_ok: tuple[bool, ...]
    joint_fault_codes: tuple[int, ...]
    gripper_opening_m: float
    gripper_enabled: bool
    gripper_homed: bool
    gripper_fault: bool
    gripper_status_code: int
    error_code: int
    detail: str


@dataclass(frozen=True)
class JointTrajectoryWaypoint:
    """One named absolute J1-J6 target in degrees."""

    name: str
    joints_deg: tuple[float, ...]


@dataclass(frozen=True)
class GripperCommandResult:
    opening_m: float
    contact_detected: bool


@dataclass(frozen=True)
class _JointTelemetry:
    valid: bool
    voltages_v: tuple[float, ...]
    currents_a: tuple[float, ...]
    motor_temperatures_c: tuple[float, ...]
    driver_temperatures_c: tuple[float, ...]
    driver_enabled: tuple[bool, ...]
    communication_ok: tuple[bool, ...]
    fault_codes: tuple[int, ...]


class PiperController:
    """Own one SDK object and enforce the deployment's motion limits."""

    def __init__(
        self,
        sdk: Any,
        *,
        arm_label: str,
        can_port: str,
        allow_motion: bool,
        max_jog_deg: float = 5.0,
        max_trajectory_waypoints: int = 32,
        default_speed_percent: int = 8,
        max_speed_percent: int = 15,
        feedback_timeout_s: float = 6.0,
        motion_timeout_s: float = 12.0,
        joint_position_tolerance_deg: float = 1.5,
        gripper_max_opening_m: float = 0.07,
        gripper_multiplier: float = 1.0,
        max_gripper_effort_nm: float = 1.5,
        gripper_zero_max_offset_m: float = 0.01,
        gripper_zero_settle_s: float = 1.5,
        gripper_position_tolerance_m: float = 0.001,
        gripper_arm_stability_s: float = 1.25,
        gripper_contact_max_target_m: float = 0.035,
        gripper_contact_min_closure_m: float = 0.003,
        gripper_contact_stability_s: float = 1.0,
        gripper_contact_position_epsilon_m: float = 0.001,
        joint_limit_tolerance_deg: float = 3.0,
        operator_confirmed_gripper_zero: bool = False,
        command_period_s: float = 0.02,
        joint_limits_deg: Sequence[Sequence[float]] = LOCAL_JOINT_LIMITS_DEG,
    ) -> None:
        if len(joint_limits_deg) != 6:
            raise ValueError("joint_limits_deg must contain six [lower, upper] pairs")
        limits = tuple((float(v[0]), float(v[1])) for v in joint_limits_deg)
        if any(lower >= upper for lower, upper in limits):
            raise ValueError("each joint lower limit must be less than its upper limit")
        if max_jog_deg <= 0.0:
            raise ValueError("max_jog_deg must be positive")
        if not 1 <= max_trajectory_waypoints <= 256:
            raise ValueError("max_trajectory_waypoints must be between 1 and 256")
        if not 1 <= default_speed_percent <= max_speed_percent <= 100:
            raise ValueError("speed percentages must satisfy 1 <= default <= max <= 100")
        if feedback_timeout_s <= 0.0 or motion_timeout_s <= 0.0:
            raise ValueError("timeouts must be positive")
        if not 0.1 <= joint_position_tolerance_deg <= 5.0:
            raise ValueError("joint_position_tolerance_deg must be between 0.1 and 5")
        if gripper_max_opening_m <= 0.0 or gripper_multiplier <= 0.0:
            raise ValueError("gripper range and multiplier must be positive")
        if not 0.5 <= max_gripper_effort_nm <= 5.0:
            raise ValueError("max_gripper_effort_nm must be between 0.5 and 5.0")
        if not 0.001 <= gripper_zero_max_offset_m <= gripper_max_opening_m:
            raise ValueError(
                "gripper_zero_max_offset_m must be between 0.001 and the maximum opening"
            )
        if not 0.0 <= gripper_zero_settle_s <= 10.0:
            raise ValueError("gripper_zero_settle_s must be between 0 and 10")
        if not 0.0001 <= gripper_position_tolerance_m <= 0.005:
            raise ValueError(
                "gripper_position_tolerance_m must be between 0.0001 and 0.005"
            )
        if not 0.0 <= gripper_arm_stability_s < feedback_timeout_s:
            raise ValueError(
                "gripper_arm_stability_s must be non-negative and less than feedback_timeout_s"
            )
        if not 0.0 <= gripper_contact_max_target_m <= gripper_max_opening_m:
            raise ValueError(
                "gripper_contact_max_target_m must be between 0 and the maximum opening"
            )
        if not 0.00005 <= gripper_contact_position_epsilon_m <= 0.001:
            raise ValueError(
                "gripper_contact_position_epsilon_m must be between 0.00005 and 0.001"
            )
        if not (
            max(
                gripper_position_tolerance_m,
                gripper_contact_position_epsilon_m,
            )
            < gripper_contact_min_closure_m
            <= gripper_max_opening_m
        ):
            raise ValueError(
                "gripper_contact_min_closure_m must exceed both gripper position "
                "thresholds and must not exceed the maximum opening"
            )
        if not 0.01 <= gripper_contact_stability_s < motion_timeout_s:
            raise ValueError(
                "gripper_contact_stability_s must be between 0.01 and the motion timeout"
            )
        if not 0.0 <= joint_limit_tolerance_deg <= 5.0:
            raise ValueError("joint_limit_tolerance_deg must be between 0 and 5")
        if not 0.005 <= command_period_s <= 0.1:
            raise ValueError("command_period_s must be between 0.005 and 0.1")

        self._sdk = sdk
        self.arm_label = arm_label
        self.can_port = can_port
        self.allow_motion = allow_motion
        self.max_jog_deg = float(max_jog_deg)
        self.max_trajectory_waypoints = int(max_trajectory_waypoints)
        self.default_speed_percent = int(default_speed_percent)
        self.max_speed_percent = int(max_speed_percent)
        self.feedback_timeout_s = float(feedback_timeout_s)
        self.motion_timeout_s = float(motion_timeout_s)
        self.joint_position_tolerance_deg = float(joint_position_tolerance_deg)
        self.gripper_max_opening_m = float(gripper_max_opening_m)
        self.gripper_multiplier = float(gripper_multiplier)
        self.max_gripper_effort_nm = float(max_gripper_effort_nm)
        self.gripper_zero_max_offset_m = float(gripper_zero_max_offset_m)
        self.gripper_zero_settle_s = float(gripper_zero_settle_s)
        self.gripper_position_tolerance_m = float(gripper_position_tolerance_m)
        self.gripper_arm_stability_s = float(gripper_arm_stability_s)
        self.gripper_contact_max_target_m = float(gripper_contact_max_target_m)
        self.gripper_contact_min_closure_m = float(
            gripper_contact_min_closure_m
        )
        self.gripper_contact_stability_s = float(gripper_contact_stability_s)
        self.gripper_contact_position_epsilon_m = float(
            gripper_contact_position_epsilon_m
        )
        self.joint_limit_tolerance_deg = float(joint_limit_tolerance_deg)
        self.operator_confirmed_gripper_zero = bool(operator_confirmed_gripper_zero)
        self.command_period_s = float(command_period_s)
        self.joint_limits_deg = limits
        self._connected = False
        self._actuation_issued = False
        self._gripper_actuation_issued = False
        self._lock = threading.RLock()

    def connect(self) -> None:
        with self._lock:
            if self._connected:
                return
            self._sdk.ConnectPort()
            deadline = time.monotonic() + self.feedback_timeout_s
            while time.monotonic() < deadline:
                if self._feedback_ready():
                    self._connected = True
                    return
                time.sleep(0.05)
            self._sdk.DisconnectPort()
            raise PiperControlError(
                f"no joint/gripper feedback on {self.can_port} within "
                f"{self.feedback_timeout_s:.1f}s"
            )

    def disconnect(self, *, emergency_stop: bool) -> None:
        with self._lock:
            if not self._connected:
                return
            if emergency_stop and self._actuation_issued:
                try:
                    self._sdk.EmergencyStop(0x01)
                except Exception:
                    pass
                if self._gripper_actuation_issued:
                    try:
                        self._disable_gripper_at_current_position()
                    except Exception:
                        pass
                for motor_num in range(6, 0, -1):
                    try:
                        self._sdk.DisableArm(motor_num)
                    except Exception:
                        pass
                    time.sleep(self.command_period_s)
            self._sdk.DisconnectPort()
            self._connected = False

    def status(self) -> PiperState:
        with self._lock:
            if not self._connected:
                return PiperState(
                    connected=False,
                    enabled=False,
                    motion_unlocked=self.allow_motion,
                    arm=self.arm_label,
                    can_port=self.can_port,
                    joints_deg=(),
                    telemetry_valid=False,
                    joint_voltages_v=(),
                    joint_currents_a=(),
                    joint_motor_temperatures_c=(),
                    joint_driver_temperatures_c=(),
                    joint_driver_enabled=(),
                    joint_communication_ok=(),
                    joint_fault_codes=(),
                    gripper_opening_m=0.0,
                    gripper_enabled=False,
                    gripper_homed=False,
                    gripper_fault=True,
                    gripper_status_code=-1,
                    error_code=-1,
                    detail="SDK is not connected",
                )
            joints = self._read_joints_deg()
            telemetry = self._read_joint_telemetry()
            gripper_msg = self._read_gripper_feedback()
            gripper = float(gripper_msg.grippers_angle) / 1_000_000.0
            gripper_status_code = int(gripper_msg.status_code)
            gripper_enabled = bool(gripper_msg.foc_status.driver_enable_status)
            hardware_gripper_homed = bool(gripper_msg.foc_status.homing_status)
            gripper_homed = (
                hardware_gripper_homed or self.operator_confirmed_gripper_zero
            )
            gripper_fault = bool(gripper_status_code & 0x3F)
            error_code = int(self._sdk.GetArmStatus().arm_status.err_code)
            enabled = self._motors_enabled()
            detail = "ready" if error_code == 0 else f"Piper error_code={error_code}"
            if not hardware_gripper_homed and self.operator_confirmed_gripper_zero:
                detail += "; gripper zero accepted from explicit operator confirmation"
            elif not hardware_gripper_homed:
                detail += "; gripper is not homed"
            if gripper_fault:
                detail += f"; gripper fault status=0x{gripper_status_code:02x}"
            if not self.allow_motion:
                detail += "; motion locked (set ROBONIX_ALLOW_MOTION=1 before boot)"
            return PiperState(
                connected=True,
                enabled=enabled,
                motion_unlocked=self.allow_motion,
                arm=self.arm_label,
                can_port=self.can_port,
                joints_deg=joints,
                telemetry_valid=telemetry.valid,
                joint_voltages_v=telemetry.voltages_v,
                joint_currents_a=telemetry.currents_a,
                joint_motor_temperatures_c=telemetry.motor_temperatures_c,
                joint_driver_temperatures_c=telemetry.driver_temperatures_c,
                joint_driver_enabled=telemetry.driver_enabled,
                joint_communication_ok=telemetry.communication_ok,
                joint_fault_codes=telemetry.fault_codes,
                gripper_opening_m=gripper,
                gripper_enabled=gripper_enabled,
                gripper_homed=gripper_homed,
                gripper_fault=gripper_fault,
                gripper_status_code=gripper_status_code,
                error_code=error_code,
                detail=detail,
            )

    def health_snapshot(self) -> dict[str, Any]:
        """Read live Piper telemetry without writing to the CAN interface."""
        with self._lock:
            self._require_connected()
            telemetry = self._read_joint_telemetry()
            arm_status = self._sdk.GetArmStatus().arm_status
            gripper = self._read_gripper_feedback()
            arm_path = "body/arm"
            readings: list[dict[str, float | str]] = []

            for index in range(1, 7):
                offset = index - 1
                component = f"{arm_path}/joint_{index}"
                readings.extend(
                    [
                        {
                            "name": f"{component}/motor_temp",
                            "temp_c": telemetry.motor_temperatures_c[offset],
                        },
                        {
                            "name": f"{component}/driver_temp",
                            "temp_c": telemetry.driver_temperatures_c[offset],
                        },
                        {
                            "name": f"{component}/voltage",
                            "voltage": telemetry.voltages_v[offset],
                            "current_a": telemetry.currents_a[offset],
                        },
                        {
                            "name": f"{component}/error",
                            "current_a": float(telemetry.fault_codes[offset]),
                        },
                        {
                            "name": f"{component}/enabled",
                            "current_a": float(telemetry.driver_enabled[offset]),
                        },
                        {
                            "name": f"{component}/communication_ok",
                            "current_a": float(telemetry.communication_ok[offset]),
                        },
                    ]
                )

            gripper_path = f"{arm_path}/gripper"
            gripper_fault = int(gripper.status_code) & 0x3F
            readings.extend(
                [
                    {
                        "name": f"{arm_path}/state",
                        "current_a": float(int(arm_status.err_code) != 0),
                    },
                    {
                        "name": f"{arm_path}/error",
                        "current_a": float(int(arm_status.err_code)),
                    },
                    {
                        "name": f"{arm_path}/communication_ok",
                        "current_a": float(telemetry.valid),
                    },
                    {
                        "name": f"{gripper_path}/error",
                        "current_a": float(gripper_fault),
                    },
                    {
                        "name": f"{gripper_path}/enabled",
                        "current_a": float(
                            bool(gripper.foc_status.driver_enable_status)
                        ),
                    },
                    {
                        "name": f"{gripper_path}/homed",
                        "current_a": float(
                            bool(gripper.foc_status.homing_status)
                            or self.operator_confirmed_gripper_zero
                        ),
                    },
                    {
                        "name": f"{gripper_path}/opening_m",
                        "current_a": float(gripper.grippers_angle) / 1_000_000.0,
                    },
                ]
            )
            return {
                "voltage": (
                    sum(telemetry.voltages_v) / len(telemetry.voltages_v)
                    if telemetry.voltages_v
                    else -1.0
                ),
                "charging": False,
                "remaining_s": -1,
                "readings": readings,
            }

    def _read_joint_telemetry(self) -> _JointTelemetry:
        """Read the six low-speed motor frames; callers hold the controller lock."""
        low_speed = self._sdk.GetArmLowSpdInfoMsgs()
        valid = self._feedback_ready() and float(low_speed.Hz) > 0.0
        motors = [getattr(low_speed, f"motor_{index}") for index in range(1, 7)]
        return _JointTelemetry(
            valid=valid,
            voltages_v=tuple(round(float(motor.vol) * 0.1, 1) for motor in motors),
            currents_a=tuple(
                round(float(getattr(motor, "bus_current", 0)) * 0.001, 3)
                for motor in motors
            ),
            motor_temperatures_c=tuple(float(motor.motor_temp) for motor in motors),
            driver_temperatures_c=tuple(float(motor.foc_temp) for motor in motors),
            driver_enabled=tuple(
                bool(motor.foc_status.driver_enable_status) for motor in motors
            ),
            communication_ok=(valid,) * 6,
            fault_codes=tuple(int(motor.foc_status_code) & 0xBF for motor in motors),
        )

    def set_enabled(self, enabled: bool) -> bool:
        with self._lock:
            self._require_connected()
            if enabled:
                self._require_motion_unlocked()
                if self._motors_enabled():
                    return True
                self._actuation_issued = True
                self._sdk.EmergencyStop(0x02)
                self._ensure_can_command_mode(self.default_speed_percent)
                deadline = time.monotonic() + self.feedback_timeout_s
                while time.monotonic() < deadline:
                    motor_states = self._motor_enable_states()
                    if all(motor_states):
                        return True
                    # Selector 7 also changes the gripper state on this Piper.
                    # Enable only joint drivers 1..6 so the gripper remains an
                    # independently controlled actuator.
                    for motor_num in range(1, 7):
                        if not motor_states[motor_num - 1]:
                            self._sdk.EnableArm(motor_num)
                            time.sleep(self.command_period_s)
                    if self._motors_enabled():
                        return True
                    time.sleep(0.1)
                raise PiperControlError("arm enable timed out")

            if not any(self._motor_enable_states()):
                return False
            self._actuation_issued = True
            deadline = time.monotonic() + self.feedback_timeout_s
            while time.monotonic() < deadline:
                motor_states = self._motor_enable_states()
                if not any(motor_states):
                    return False
                # Piper selector 7 couples the arm and gripper on this hardware.
                # Address only joint drivers 1..6 so arm disable preserves the
                # independently controlled gripper state.
                for motor_num in range(6, 0, -1):
                    if motor_states[motor_num - 1]:
                        self._sdk.DisableArm(motor_num)
                        time.sleep(self.command_period_s)
                if not any(self._motor_enable_states()):
                    return False
                time.sleep(0.1)
            raise PiperControlError("arm disable timed out")

    def set_gripper_enabled(self, enabled: bool, effort_nm: float = 1.0) -> bool:
        with self._lock:
            self._require_connected()
            feedback = self._read_gripper_feedback()
            current_enabled = bool(feedback.foc_status.driver_enable_status)
            if enabled:
                self._require_motion_unlocked()
                if current_enabled:
                    return True
                effort = float(effort_nm)
                if not math.isfinite(effort) or not 0.5 <= effort <= self.max_gripper_effort_nm:
                    raise PiperControlError(
                        f"effort_nm must be between 0.5 and {self.max_gripper_effort_nm:.2f}"
                    )
                self._ensure_can_command_mode(self.default_speed_percent)
                self._actuation_issued = True
                self._gripper_actuation_issued = True
                self._ensure_gripper_enabled(
                    round(effort * 1000.0),
                    clear_errors=bool(int(feedback.status_code) & 0x3F),
                )
                return True

            if not current_enabled:
                return False
            self._actuation_issued = True
            self._gripper_actuation_issued = True
            self._disable_gripper_at_current_position()
            deadline = time.monotonic() + self.feedback_timeout_s
            while time.monotonic() < deadline:
                if not bool(
                    self._read_gripper_feedback().foc_status.driver_enable_status
                ):
                    return False
                time.sleep(self.command_period_s)
            raise PiperControlError("gripper disable timed out")

    def set_gripper_zero(self, fully_closed_confirmed: bool) -> float:
        with self._lock:
            self._require_connected()
            self._require_motion_unlocked()
            if not fully_closed_confirmed:
                raise PiperControlError(
                    "fully_closed_confirmed must be true after the operator manually "
                    "closes the gripper"
                )

            feedback = self._read_gripper_feedback()
            status_code = int(feedback.status_code)
            if status_code & 0x3F:
                raise PiperControlError(
                    f"refusing gripper zero with fault status=0x{status_code:02x}"
                )
            error_code = int(self._sdk.GetArmStatus().arm_status.err_code)
            if error_code != 0:
                raise PiperControlError(
                    f"refusing gripper zero with Piper error_code={error_code}"
                )

            opening_m = float(feedback.grippers_angle) / 1_000_000.0
            if abs(opening_m) > self.gripper_zero_max_offset_m:
                raise PiperControlError(
                    f"current gripper offset {opening_m:.4f} m exceeds the "
                    f"zeroing limit of +/-{self.gripper_zero_max_offset_m:.4f} m; "
                    "manually close the gripper before setting zero"
                )

            # Piper requires a disabled command before accepting 0xAE as zero.
            self._sdk.GripperCtrl(0, 1000, 0x00, 0)
            time.sleep(self.gripper_zero_settle_s)
            self._sdk.GripperCtrl(0, 1000, 0x00, 0xAE)

            settled_samples = 0
            deadline = time.monotonic() + self.feedback_timeout_s
            while time.monotonic() < deadline:
                current_feedback = self._read_gripper_feedback()
                current_status = int(current_feedback.status_code)
                current_opening_m = (
                    float(current_feedback.grippers_angle) / 1_000_000.0
                )
                if current_status & 0x3F:
                    raise PiperControlError(
                        f"gripper fault after setting zero: 0x{current_status:02x}"
                    )
                if (
                    not bool(current_feedback.foc_status.driver_enable_status)
                    and abs(current_opening_m) <= 0.0005
                ):
                    settled_samples += 1
                    if settled_samples >= 3:
                        self.operator_confirmed_gripper_zero = True
                        return current_opening_m
                else:
                    settled_samples = 0
                time.sleep(self.command_period_s)
            raise PiperControlError("gripper zero command was not confirmed by feedback")

    def jog_joint(self, joint: int, delta_deg: float, speed_percent: int) -> tuple[float, ...]:
        with self._lock:
            self._require_connected()
            self._require_motion_unlocked()
            if joint < 1 or joint > 6:
                raise PiperControlError("joint must be an integer from 1 through 6")
            if not math.isfinite(delta_deg) or abs(delta_deg) < 0.01:
                raise PiperControlError("delta_deg must be finite and at least 0.01 degree")
            if abs(delta_deg) > self.max_jog_deg:
                raise PiperControlError(
                    f"abs(delta_deg) must not exceed {self.max_jog_deg:.2f} degrees"
                )
            speed = self._resolve_speed(speed_percent)

            state = self.status()
            if state.error_code != 0:
                raise PiperControlError(f"refusing motion with Piper error_code={state.error_code}")
            targets = list(state.joints_deg)
            self._validate_current_joint_envelope(targets)
            current = targets[joint - 1]
            targets[joint - 1] += float(delta_deg)
            lower, upper = self.joint_limits_deg[joint - 1]
            target = targets[joint - 1]
            if lower <= current <= upper and not lower <= target <= upper:
                raise PiperControlError(
                    f"joint{joint} target {target:.3f} deg is outside "
                    f"local limit [{lower:.3f}, {upper:.3f}]"
                )
            if current < lower and target < current:
                raise PiperControlError(
                    f"joint{joint} is below its local limit; only a positive jog is allowed"
                )
            if current > upper and target > current:
                raise PiperControlError(
                    f"joint{joint} is above its local limit; only a negative jog is allowed"
                )
            self._validate_joint_envelope(joint - 1, target)

            if not state.enabled:
                self.set_enabled(True)
            self._actuation_issued = True
            raw_targets = tuple(round(value * 1000.0) for value in targets)
            self._drive_joint_to_target(
                joint - 1, targets[joint - 1], raw_targets, speed
            )
            return tuple(targets)

    def execute_joint_trajectory(
        self,
        waypoints: Sequence[JointTrajectoryWaypoint],
        speed_percent: int,
    ) -> tuple[int, tuple[float, ...]]:
        """Execute prevalidated absolute joint targets in order.

        The complete request is validated before any enable or motion command.
        Each target must settle on all six axes before the next target starts.
        A runtime failure reports completed progress and leaves the arm enabled.
        """
        with self._lock:
            self._require_connected()
            self._require_motion_unlocked()
            trajectory = self._validate_joint_trajectory(waypoints)
            speed = self._resolve_speed(speed_percent)

            state = self.status()
            if state.error_code != 0:
                raise PiperControlError(
                    f"refusing motion with Piper error_code={state.error_code}"
                )
            self._validate_current_joint_envelope(state.joints_deg)
            if not state.enabled:
                self.set_enabled(True)
            self._actuation_issued = True

            completed = 0
            for waypoint in trajectory:
                raw_targets = tuple(
                    round(value * 1000.0) for value in waypoint.joints_deg
                )
                try:
                    self._drive_joints_to_target(
                        waypoint.name,
                        waypoint.joints_deg,
                        raw_targets,
                        speed,
                    )
                except Exception as exc:
                    try:
                        final_joints = self._read_joints_deg()
                    except Exception:  # noqa: BLE001
                        final_joints = ()
                    raise JointTrajectoryExecutionError(
                        f"waypoint {waypoint.name!r} failed: {exc}",
                        completed_waypoints=completed,
                        failed_waypoint=waypoint.name,
                        final_joints_deg=final_joints,
                    ) from exc
                completed += 1

            return completed, self._read_joints_deg()

    def set_gripper(
        self, opening_m: float, effort_nm: float
    ) -> GripperCommandResult:
        with self._lock:
            self._require_connected()
            self._require_motion_unlocked()
            if not math.isfinite(opening_m) or not 0.0 <= opening_m <= self.gripper_max_opening_m:
                raise PiperControlError(
                    f"opening_m must be between 0 and {self.gripper_max_opening_m:.3f}"
                )
            effort = 1.0 if effort_nm == 0.0 else float(effort_nm)
            if not math.isfinite(effort) or not 0.5 <= effort <= self.max_gripper_effort_nm:
                raise PiperControlError(
                    f"effort_nm must be 0 (default) or 0.5..{self.max_gripper_effort_nm:.2f}"
                )
            feedback = self._read_gripper_feedback()
            initial_opening_m = float(feedback.grippers_angle) / 1_000_000.0
            if not (
                bool(feedback.foc_status.homing_status)
                or self.operator_confirmed_gripper_zero
            ):
                raise PiperControlError(
                    "gripper is not homed; set its zero at the fully closed mechanical "
                    "position before sending opening commands"
                )
            status_code = int(feedback.status_code)
            if status_code & 0x3F:
                raise PiperControlError(
                    f"refusing gripper motion with fault status=0x{status_code:02x}"
                )
            error_code = int(self._sdk.GetArmStatus().arm_status.err_code)
            if error_code != 0:
                raise PiperControlError(
                    f"refusing gripper motion with Piper error_code={error_code}"
                )
            raw_opening = round(opening_m * 1_000_000.0 * self.gripper_multiplier)
            raw_effort = round(effort * 1000.0)
            self._actuation_issued = True
            self._gripper_actuation_issued = True
            if self._motors_enabled():
                self._ensure_can_command_mode(self.default_speed_percent)
            else:
                self.set_enabled(True)
            if not bool(
                self._read_gripper_feedback().foc_status.driver_enable_status
            ):
                self.set_gripper_enabled(True, effort)
            try:
                contact_detected = self._drive_gripper_to_target(
                    opening_m,
                    raw_opening,
                    raw_effort,
                    initial_opening_m=initial_opening_m,
                )
            except Exception as motion_error:
                try:
                    self._ensure_arm_enabled_after_gripper()
                except Exception as recovery_error:
                    raise PiperControlError(
                        f"{motion_error}; arm enable recovery also failed: {recovery_error}"
                    ) from recovery_error
                raise
            self._ensure_arm_enabled_after_gripper()
            return GripperCommandResult(
                opening_m=self._read_gripper_opening_m(),
                contact_detected=contact_detected,
            )

    def emergency_stop(self) -> None:
        with self._lock:
            self._require_connected()
            self._sdk.EmergencyStop(0x01)

    def _validate_current_joint_envelope(self, joints_deg: Sequence[float]) -> None:
        if len(joints_deg) != 6:
            raise PiperControlError("joint feedback did not contain six values")
        for index, value in enumerate(joints_deg):
            self._validate_joint_envelope(index, value)

    def _validate_joint_trajectory(
        self, waypoints: Sequence[JointTrajectoryWaypoint]
    ) -> tuple[JointTrajectoryWaypoint, ...]:
        """Normalize and validate every waypoint before hardware actuation."""
        if not waypoints:
            raise PiperControlError("trajectory must contain at least one waypoint")
        if len(waypoints) > self.max_trajectory_waypoints:
            raise PiperControlError(
                f"trajectory contains {len(waypoints)} waypoints; maximum is "
                f"{self.max_trajectory_waypoints}"
            )

        normalized: list[JointTrajectoryWaypoint] = []
        names: set[str] = set()
        for waypoint_index, waypoint in enumerate(waypoints, start=1):
            name = str(waypoint.name).strip() or f"waypoint_{waypoint_index}"
            if len(name) > 64:
                raise PiperControlError(
                    f"waypoint {waypoint_index} name must not exceed 64 characters"
                )
            if name in names:
                raise PiperControlError(f"duplicate waypoint name {name!r}")
            names.add(name)
            if len(waypoint.joints_deg) != 6:
                raise PiperControlError(
                    f"waypoint {name!r} must contain exactly six joint angles"
                )
            try:
                targets = tuple(float(value) for value in waypoint.joints_deg)
            except (TypeError, ValueError) as exc:
                raise PiperControlError(
                    f"waypoint {name!r} contains a non-numeric joint angle"
                ) from exc
            for joint_index, target in enumerate(targets):
                if not math.isfinite(target):
                    raise PiperControlError(
                        f"waypoint {name!r} joint{joint_index + 1} must be finite"
                    )
                self._validate_absolute_joint_target(name, joint_index, target)
            normalized.append(JointTrajectoryWaypoint(name, targets))
        return tuple(normalized)

    def _validate_absolute_joint_target(
        self, waypoint_name: str, joint_index: int, value: float
    ) -> None:
        """Require commanded absolute targets to stay inside nominal limits."""
        lower, upper = self.joint_limits_deg[joint_index]
        if not lower <= value <= upper:
            raise PiperControlError(
                f"waypoint {waypoint_name!r} joint{joint_index + 1} target "
                f"{value:.3f} deg is outside local limit [{lower:.3f}, {upper:.3f}]"
            )

    def _validate_joint_envelope(self, joint_index: int, value: float) -> None:
        lower, upper = self.joint_limits_deg[joint_index]
        tolerance = self.joint_limit_tolerance_deg
        if value < lower - tolerance or value > upper + tolerance:
            raise PiperControlError(
                f"joint{joint_index + 1} value {value:.3f} deg is outside the "
                f"commandable local limit [{lower:.3f}, {upper:.3f}] by more "
                f"than {tolerance:.2f} deg; refusing the grouped joint command "
                "because Piper firmware would clamp that joint"
            )

    def _resolve_speed(self, speed_percent: int) -> int:
        """Apply the configured default and enforce the deployment speed cap."""
        speed = self.default_speed_percent if speed_percent == 0 else int(speed_percent)
        if speed < 1 or speed > self.max_speed_percent:
            raise PiperControlError(
                f"speed_percent must be 0 (default) or 1..{self.max_speed_percent}"
            )
        return speed

    def _feedback_ready(self) -> bool:
        try:
            return (
                float(self._sdk.GetArmJointMsgs().Hz) > 0.0
                and float(self._sdk.GetArmGripperMsgs().Hz) > 0.0
            )
        except (AttributeError, TypeError, ValueError):
            return False

    def _read_joints_deg(self) -> tuple[float, ...]:
        msg = self._sdk.GetArmJointMsgs().joint_state
        return tuple(
            float(getattr(msg, f"joint_{index}")) / 1000.0
            for index in range(1, 7)
        )

    def _read_gripper_opening_m(self) -> float:
        msg = self._read_gripper_feedback()
        return float(msg.grippers_angle) / 1_000_000.0

    def _read_gripper_feedback(self) -> Any:
        return self._sdk.GetArmGripperMsgs().gripper_state

    def _disable_gripper_at_current_position(self) -> None:
        current = int(self._read_gripper_feedback().grippers_angle)
        self._sdk.GripperCtrl(current, 1000, 0x00, 0)

    def _motors_enabled(self) -> bool:
        return all(self._motor_enable_states())

    def _motor_enable_states(self) -> tuple[bool, ...]:
        low_speed = self._sdk.GetArmLowSpdInfoMsgs()
        return tuple(
            bool(getattr(low_speed, f"motor_{index}").foc_status.driver_enable_status)
            for index in range(1, 7)
        )

    def _ensure_can_command_mode(self, speed_percent: int) -> None:
        deadline = time.monotonic() + self.feedback_timeout_s
        while time.monotonic() < deadline:
            self._sdk.MotionCtrl_2(0x01, 0x01, speed_percent, 0x00)
            if int(self._sdk.GetArmStatus().arm_status.ctrl_mode) == 0x01:
                return
            time.sleep(self.command_period_s)
        raise PiperControlError("arm did not enter CAN command mode")

    def _ensure_gripper_enabled(
        self, raw_effort: int, *, clear_errors: bool = False
    ) -> None:
        if bool(self._read_gripper_feedback().foc_status.driver_enable_status):
            return
        enable_code = 0x03 if clear_errors else 0x01
        deadline = time.monotonic() + self.feedback_timeout_s
        while time.monotonic() < deadline:
            current = int(self._read_gripper_feedback().grippers_angle)
            self._sdk.GripperCtrl(current, raw_effort, enable_code, 0)
            if bool(self._read_gripper_feedback().foc_status.driver_enable_status):
                return
            time.sleep(self.command_period_s)
        raise PiperControlError("gripper enable timed out")

    def _ensure_arm_enabled_after_gripper(self) -> None:
        """Keep the movement capability's arm-enable postcondition.

        This Piper firmware can report the old joint-driver state briefly and
        then drop all six drivers after a gripper command. Wait through that
        delayed feedback window and restore only disabled joint drivers.
        """
        deadline = time.monotonic() + self.feedback_timeout_s
        stable_since: float | None = None
        restored = False
        transient_communication_error = False
        while time.monotonic() < deadline:
            motor_states = self._motor_enable_states()
            arm_status = self._sdk.GetArmStatus().arm_status
            error_code = int(arm_status.err_code)
            if error_code == 0x3F:
                stable_since = None
                if not transient_communication_error:
                    log.warning(
                        "%s gripper motion produced transient joint communication "
                        "error 0x3f; waiting for fresh feedback",
                        self.arm_label,
                    )
                    transient_communication_error = True
                time.sleep(self.command_period_s)
                continue
            if error_code != 0:
                raise PiperControlError(
                    "arm reported an error while preserving enable state after gripper motion: "
                    f"error_code={error_code}"
                )

            now = time.monotonic()
            if all(motor_states):
                if self.gripper_arm_stability_s == 0.0:
                    return
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= self.gripper_arm_stability_s:
                    return
            else:
                stable_since = None
                if not restored:
                    disabled = [
                        str(index)
                        for index, enabled in enumerate(motor_states, start=1)
                        if not enabled
                    ]
                    log.warning(
                        "%s gripper motion dropped arm joint drivers %s; restoring them",
                        self.arm_label,
                        ",".join(disabled),
                    )
                    restored = True
                for motor_num in range(1, 7):
                    if not motor_states[motor_num - 1]:
                        self._sdk.EnableArm(motor_num)
                        time.sleep(self.command_period_s)
            time.sleep(self.command_period_s)

        raise PiperControlError(
            "arm joint drivers did not remain enabled after gripper motion"
        )

    def _drive_joint_to_target(
        self,
        joint_index: int,
        target_deg: float,
        raw_targets: Sequence[int],
        speed_percent: int,
    ) -> None:
        self._ensure_can_command_mode(speed_percent)
        deadline = time.monotonic() + self.motion_timeout_s
        while time.monotonic() < deadline:
            self._sdk.MotionCtrl_2(0x01, 0x01, speed_percent, 0x00)
            self._sdk.JointCtrl(*raw_targets)
            current = self._read_joints_deg()[joint_index]
            if abs(current - target_deg) <= self.joint_position_tolerance_deg:
                return
            time.sleep(self.command_period_s)
        raise PiperControlError(
            f"joint{joint_index + 1} did not reach {target_deg:.3f} deg within "
            f"{self.motion_timeout_s:.1f}s"
        )

    def _drive_joints_to_target(
        self,
        waypoint_name: str,
        targets_deg: Sequence[float],
        raw_targets: Sequence[int],
        speed_percent: int,
    ) -> None:
        """Command one six-axis waypoint and require three settled samples."""
        self._ensure_can_command_mode(speed_percent)
        deadline = time.monotonic() + self.motion_timeout_s
        settled_samples = 0
        last_errors = (math.inf,) * 6
        while time.monotonic() < deadline:
            error_code = int(self._sdk.GetArmStatus().arm_status.err_code)
            if error_code != 0:
                raise PiperControlError(
                    f"Piper error_code={error_code} while moving to {waypoint_name!r}"
                )
            self._sdk.MotionCtrl_2(0x01, 0x01, speed_percent, 0x00)
            self._sdk.JointCtrl(*raw_targets)
            current = self._read_joints_deg()
            last_errors = tuple(
                abs(actual - target)
                for actual, target in zip(current, targets_deg, strict=True)
            )
            if all(
                error <= self.joint_position_tolerance_deg
                for error in last_errors
            ):
                settled_samples += 1
                if settled_samples >= 3:
                    return
            else:
                settled_samples = 0
            time.sleep(self.command_period_s)

        error_summary = ", ".join(
            f"J{index}={error:.3f}deg"
            for index, error in enumerate(last_errors, start=1)
        )
        raise PiperControlError(
            f"did not reach waypoint {waypoint_name!r} within "
            f"{self.motion_timeout_s:.1f}s; errors: {error_summary}"
        )

    def _drive_gripper_to_target(
        self,
        target_m: float,
        raw_opening: int,
        raw_effort: int,
        *,
        initial_opening_m: float,
    ) -> bool:
        settled_samples = 0
        contact_eligible = (
            target_m <= self.gripper_contact_max_target_m
            and target_m
            < initial_opening_m - self.gripper_position_tolerance_m
        )
        contact_stable_since: float | None = None
        contact_anchor_m = math.inf
        current_m = initial_opening_m
        max_closure_m = 0.0
        deadline = time.monotonic() + self.motion_timeout_s
        while time.monotonic() < deadline:
            self._sdk.MotionCtrl_2(
                0x01, 0x01, self.default_speed_percent, 0x00
            )
            self._sdk.GripperCtrl(raw_opening, raw_effort, 0x01, 0)
            feedback = self._read_gripper_feedback()
            status_code = int(feedback.status_code)
            if status_code & 0x3F:
                raise PiperControlError(
                    f"gripper fault while moving: status=0x{status_code:02x}"
                )
            if not bool(feedback.foc_status.driver_enable_status):
                raise PiperControlError("gripper became disabled while moving")

            current_m = float(feedback.grippers_angle) / 1_000_000.0
            error_m = current_m - target_m
            closure_m = initial_opening_m - current_m
            max_closure_m = max(max_closure_m, closure_m)
            if abs(error_m) <= self.gripper_position_tolerance_m:
                settled_samples += 1
                if settled_samples >= 5:
                    return False
            else:
                settled_samples = 0

            significant_closure = (
                closure_m >= self.gripper_contact_min_closure_m
            )
            stopped_before_target = error_m > self.gripper_position_tolerance_m
            if contact_eligible and significant_closure and stopped_before_target:
                now = time.monotonic()
                if (
                    contact_stable_since is None
                    or abs(current_m - contact_anchor_m)
                    > self.gripper_contact_position_epsilon_m
                ):
                    contact_stable_since = now
                    contact_anchor_m = current_m
                elif (
                    now - contact_stable_since
                    >= self.gripper_contact_stability_s
                ):
                    log.info(
                        "%s gripper stable contact: initial=%.4f m current=%.4f m "
                        "target=%.4f m closure=%.4f m",
                        self.arm_label,
                        initial_opening_m,
                        current_m,
                        target_m,
                        closure_m,
                    )
                    return True
            else:
                contact_stable_since = None
                contact_anchor_m = math.inf
            time.sleep(self.command_period_s)
        raise PiperControlError(
            f"gripper did not reach {target_m:.3f} m or establish stable contact "
            f"within {self.motion_timeout_s:.1f}s; final={current_m:.4f} m, "
            f"max_closure={max_closure_m:.4f} m, "
            f"required_closure={self.gripper_contact_min_closure_m:.4f} m"
        )

    def _require_connected(self) -> None:
        if not self._connected:
            raise PiperControlError("SDK is not connected")

    def _require_motion_unlocked(self) -> None:
        if not self.allow_motion:
            raise PiperControlError(
                "hardware motion is locked; set ROBONIX_ALLOW_MOTION=1 before rbnx boot"
            )
