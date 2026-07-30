from __future__ import annotations

import itertools
import unittest

from piper_arm.controller import (
    JointTrajectoryExecutionError,
    JointTrajectoryWaypoint,
    PiperControlError,
    PiperController,
)


class Box:
    def __init__(self, **values):
        self.__dict__.update(values)


class FakeSdk:
    def __init__(self):
        self.joints = [0, 90000, -90000, 0, 0, 0]
        self.gripper = 80000
        self.gripper_status = 0xC0
        self.motor_enabled = [False] * 6
        self.ctrl_mode = 0
        self.error_code = 0
        self.calls = []

    def ConnectPort(self):
        self.calls.append(("connect",))

    def DisconnectPort(self):
        self.calls.append(("disconnect",))

    def GetArmJointMsgs(self):
        state = Box(**{f"joint_{i + 1}": value for i, value in enumerate(self.joints)})
        return Box(Hz=10.0, joint_state=state)

    def GetArmGripperMsgs(self):
        foc = Box(
            driver_enable_status=bool(self.gripper_status & 0x40),
            homing_status=bool(self.gripper_status & 0x80),
        )
        return Box(
            Hz=10.0,
            gripper_state=Box(
                grippers_angle=self.gripper,
                status_code=self.gripper_status,
                foc_status=foc,
            ),
        )

    def GetArmStatus(self):
        return Box(
            arm_status=Box(err_code=self.error_code, ctrl_mode=self.ctrl_mode)
        )

    def GetArmLowSpdInfoMsgs(self):
        values = {
            f"motor_{index}": Box(
                vol=240 + index,
                bus_current=100 + index,
                motor_temp=30 + index,
                foc_temp=35 + index,
                foc_status_code=0x40 if self.motor_enabled[index - 1] else 0,
                foc_status=Box(driver_enable_status=self.motor_enabled[index - 1]),
            )
            for index in range(1, 7)
        }
        return Box(Hz=10.0, **values)

    def EmergencyStop(self, code):
        self.calls.append(("emergency", code))

    def EnableArm(self, motor):
        self.calls.append(("enable", motor))
        if motor == 7:
            self.enabled = True
        else:
            self.motor_enabled[motor - 1] = True

    def DisableArm(self, motor):
        self.calls.append(("disable", motor))
        if motor == 7:
            self.enabled = False
        else:
            self.motor_enabled[motor - 1] = False

    def MotionCtrl_2(self, *args):
        self.calls.append(("motion", *args))
        self.ctrl_mode = args[0]

    def JointCtrl(self, *targets):
        self.calls.append(("joints", *targets))
        self.joints = list(targets)

    def GripperCtrl(self, opening, effort, code, zero):
        self.calls.append(("gripper", opening, effort, code, zero))
        if code in (0x01, 0x03):
            self.gripper_status |= 0x40
        elif code in (0x00, 0x02):
            self.gripper_status &= ~0x40
        if code == 0x01:
            self.gripper = opening
        if zero == 0xAE:
            self.gripper = 0

    @property
    def enabled(self):
        return all(self.motor_enabled)

    @enabled.setter
    def enabled(self, enabled):
        self.motor_enabled = [bool(enabled)] * 6


def controller(
    *,
    allow_motion: bool,
    operator_confirmed_gripper_zero: bool = False,
    gripper_arm_stability_s: float = 0.0,
    gripper_position_tolerance_m: float = 0.001,
    gripper_contact_min_closure_m: float = 0.003,
    gripper_contact_stability_s: float = 0.02,
    joint_position_tolerance_deg: float = 1.5,
    feedback_timeout_s: float = 0.1,
    max_trajectory_waypoints: int = 32,
) -> tuple[PiperController, FakeSdk]:
    sdk = FakeSdk()
    ctl = PiperController(
        sdk,
        arm_label="left",
        can_port="can_left",
        allow_motion=allow_motion,
        max_trajectory_waypoints=max_trajectory_waypoints,
        feedback_timeout_s=feedback_timeout_s,
        motion_timeout_s=0.1,
        joint_position_tolerance_deg=joint_position_tolerance_deg,
        gripper_zero_settle_s=0.0,
        gripper_position_tolerance_m=gripper_position_tolerance_m,
        gripper_arm_stability_s=gripper_arm_stability_s,
        gripper_contact_min_closure_m=gripper_contact_min_closure_m,
        gripper_contact_stability_s=gripper_contact_stability_s,
        operator_confirmed_gripper_zero=operator_confirmed_gripper_zero,
    )
    ctl.connect()
    return ctl, sdk


class PiperControllerTest(unittest.TestCase):
    def test_health_snapshot_is_live_and_read_only(self):
        ctl, sdk = controller(allow_motion=False)

        snapshot = ctl.health_snapshot()

        self.assertAlmostEqual(snapshot["voltage"], 24.35)
        self.assertEqual(snapshot["remaining_s"], -1)
        self.assertEqual(len(snapshot["readings"]), 43)
        first_voltage = next(
            reading
            for reading in snapshot["readings"]
            if reading["name"] == "body/arm/joint_1/voltage"
        )
        self.assertAlmostEqual(first_voltage["voltage"], 24.1)
        self.assertAlmostEqual(first_voltage["current_a"], 0.101)
        self.assertEqual(sdk.calls, [("connect",)])

    def test_connect_and_status_do_not_send_motion(self):
        ctl, sdk = controller(allow_motion=False)
        state = ctl.status()
        self.assertTrue(state.connected)
        self.assertFalse(state.motion_unlocked)
        self.assertEqual(state.joints_deg, (0.0, 90.0, -90.0, 0.0, 0.0, 0.0))
        self.assertTrue(state.telemetry_valid)
        self.assertEqual(state.joint_voltages_v, (24.1, 24.2, 24.3, 24.4, 24.5, 24.6))
        self.assertEqual(
            state.joint_currents_a, (0.101, 0.102, 0.103, 0.104, 0.105, 0.106)
        )
        self.assertEqual(
            state.joint_motor_temperatures_c, (31.0, 32.0, 33.0, 34.0, 35.0, 36.0)
        )
        self.assertEqual(
            state.joint_driver_temperatures_c, (36.0, 37.0, 38.0, 39.0, 40.0, 41.0)
        )
        self.assertEqual(state.joint_driver_enabled, (False,) * 6)
        self.assertEqual(state.joint_communication_ok, (True,) * 6)
        self.assertEqual(state.joint_fault_codes, (0,) * 6)
        self.assertEqual(sdk.calls, [("connect",)])

    def test_motion_lock_rejects_jog_and_gripper(self):
        ctl, sdk = controller(allow_motion=False)
        with self.assertRaises(PiperControlError):
            ctl.jog_joint(1, 1.0, 5)
        with self.assertRaises(PiperControlError):
            ctl.execute_joint_trajectory(
                [
                    JointTrajectoryWaypoint(
                        "locked", (0.0, 90.0, -90.0, 0.0, 0.0, 0.0)
                    )
                ],
                5,
            )
        with self.assertRaises(PiperControlError):
            ctl.set_gripper(0.04, 1.0)
        self.assertFalse(any(call[0] in ("motion", "enable", "joints") for call in sdk.calls))

    def test_jog_enables_and_preserves_other_joints(self):
        ctl, sdk = controller(allow_motion=True)
        target = ctl.jog_joint(1, 2.0, 5)
        self.assertEqual(target, (2.0, 90.0, -90.0, 0.0, 0.0, 0.0))
        self.assertIn(("joints", 2000, 90000, -90000, 0, 0, 0), sdk.calls)
        self.assertTrue(sdk.enabled)
        self.assertFalse(any(call[0] == "disable" for call in sdk.calls))

    def test_joint_position_tolerance_is_one_and_a_half_degrees(self):
        ctl, sdk = controller(allow_motion=True)
        original_joint_ctrl = sdk.JointCtrl

        def settle_with_error(error_millideg):
            def joint_ctrl(*targets):
                original_joint_ctrl(*targets)
                sdk.joints[0] -= error_millideg

            return joint_ctrl

        sdk.JointCtrl = settle_with_error(1400)
        ctl.jog_joint(1, 2.0, 5)

        ctl, sdk = controller(allow_motion=True)
        original_joint_ctrl = sdk.JointCtrl
        sdk.JointCtrl = settle_with_error(1600)
        with self.assertRaisesRegex(PiperControlError, "did not reach"):
            ctl.jog_joint(1, 2.0, 5)

    def test_absolute_joint_trajectory_executes_all_waypoints_and_stays_enabled(self):
        ctl, sdk = controller(allow_motion=True)
        waypoints = [
            JointTrajectoryWaypoint(
                "pre_pick", (10.0, 80.0, -80.0, 5.0, 10.0, 15.0)
            ),
            JointTrajectoryWaypoint(
                "pick", (20.0, 70.0, -70.0, 10.0, 20.0, 30.0)
            ),
        ]

        completed, final_joints = ctl.execute_joint_trajectory(waypoints, 5)

        self.assertEqual(completed, 2)
        self.assertEqual(final_joints, waypoints[-1].joints_deg)
        self.assertIn(("joints", 10000, 80000, -80000, 5000, 10000, 15000), sdk.calls)
        self.assertIn(("joints", 20000, 70000, -70000, 10000, 20000, 30000), sdk.calls)
        self.assertTrue(sdk.enabled)
        self.assertFalse(any(call[0] == "disable" for call in sdk.calls))
        self.assertFalse(any(call[0] == "gripper" for call in sdk.calls))

    def test_absolute_joint_trajectory_prevalidates_every_waypoint(self):
        ctl, sdk = controller(allow_motion=True)
        waypoints = [
            JointTrajectoryWaypoint(
                "valid", (10.0, 80.0, -80.0, 5.0, 10.0, 15.0)
            ),
            JointTrajectoryWaypoint(
                "invalid", (20.0, -0.1, -70.0, 10.0, 20.0, 30.0)
            ),
        ]

        with self.assertRaisesRegex(PiperControlError, "outside local limit"):
            ctl.execute_joint_trajectory(waypoints, 5)

        self.assertFalse(
            any(call[0] in ("emergency", "motion", "enable", "joints") for call in sdk.calls)
        )

    def test_absolute_joint_trajectory_requires_six_finite_angles(self):
        ctl, sdk = controller(allow_motion=True)

        with self.assertRaisesRegex(PiperControlError, "exactly six"):
            ctl.execute_joint_trajectory(
                [JointTrajectoryWaypoint("short", (0.0, 90.0, -90.0))], 5
            )
        with self.assertRaisesRegex(PiperControlError, "must be finite"):
            ctl.execute_joint_trajectory(
                [
                    JointTrajectoryWaypoint(
                        "nan", (0.0, 90.0, -90.0, 0.0, float("nan"), 0.0)
                    )
                ],
                5,
            )

        self.assertFalse(any(call[0] in ("motion", "enable", "joints") for call in sdk.calls))

    def test_absolute_joint_trajectory_reports_partial_progress(self):
        ctl, sdk = controller(allow_motion=True)
        original_joint_ctrl = sdk.JointCtrl

        def leave_second_waypoint_outside_tolerance(*targets):
            original_joint_ctrl(*targets)
            if targets[0] == 20000:
                sdk.joints[5] = targets[5] - 1600

        sdk.JointCtrl = leave_second_waypoint_outside_tolerance
        waypoints = [
            JointTrajectoryWaypoint(
                "first", (10.0, 80.0, -80.0, 5.0, 10.0, 15.0)
            ),
            JointTrajectoryWaypoint(
                "second", (20.0, 70.0, -70.0, 10.0, 20.0, 30.0)
            ),
        ]

        with self.assertRaises(JointTrajectoryExecutionError) as caught:
            ctl.execute_joint_trajectory(waypoints, 5)

        self.assertEqual(caught.exception.completed_waypoints, 1)
        self.assertEqual(caught.exception.failed_waypoint, "second")
        self.assertAlmostEqual(caught.exception.final_joints_deg[5], 28.4)
        self.assertTrue(sdk.enabled)
        self.assertFalse(any(call[0] == "disable" for call in sdk.calls))

    def test_absolute_joint_trajectory_enforces_count_and_speed_before_motion(self):
        ctl, sdk = controller(allow_motion=True, max_trajectory_waypoints=1)
        waypoint = JointTrajectoryWaypoint(
            "only", (10.0, 80.0, -80.0, 5.0, 10.0, 15.0)
        )

        with self.assertRaisesRegex(PiperControlError, "maximum is 1"):
            ctl.execute_joint_trajectory([waypoint, waypoint], 5)
        with self.assertRaisesRegex(PiperControlError, "1..15"):
            ctl.execute_joint_trajectory([waypoint], 16)

        self.assertFalse(any(call[0] in ("motion", "enable", "joints") for call in sdk.calls))

    def test_arm_enable_and_disable_are_idempotent(self):
        ctl, sdk = controller(allow_motion=True)
        self.assertTrue(ctl.set_enabled(True))
        self.assertTrue(ctl.set_enabled(True))
        self.assertEqual(
            [call for call in sdk.calls if call[0] == "enable"],
            [("enable", motor) for motor in range(1, 7)],
        )

        self.assertFalse(ctl.set_enabled(False))
        self.assertFalse(ctl.set_enabled(False))
        self.assertEqual(
            [call for call in sdk.calls if call[0] == "disable"],
            [("disable", motor) for motor in range(6, 0, -1)],
        )

    def test_arm_enable_preserves_disabled_gripper(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.gripper_status = 0x80

        self.assertTrue(ctl.set_enabled(True))

        self.assertTrue(sdk.enabled)
        self.assertFalse(sdk.gripper_status & 0x40)
        self.assertFalse(any(call == ("enable", 7) for call in sdk.calls))

    def test_arm_enable_completes_partially_enabled_arm(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.motor_enabled = [True, False, True, False, True, False]

        self.assertTrue(ctl.set_enabled(True))

        self.assertEqual(sdk.motor_enabled, [True] * 6)
        self.assertEqual(
            [call for call in sdk.calls if call[0] == "enable"],
            [("enable", 2), ("enable", 4), ("enable", 6)],
        )

    def test_arm_disable_preserves_enabled_gripper(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.enabled = True
        sdk.gripper_status = 0xC0

        self.assertFalse(ctl.set_enabled(False))

        self.assertFalse(sdk.enabled)
        self.assertTrue(sdk.gripper_status & 0x40)
        self.assertFalse(any(call == ("disable", 7) for call in sdk.calls))

    def test_arm_disable_finishes_partially_disabled_arm(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.motor_enabled = [True, False, True, False, True, True]

        self.assertFalse(ctl.set_enabled(False))

        self.assertEqual(sdk.motor_enabled, [False] * 6)
        self.assertEqual(
            [call for call in sdk.calls if call[0] == "disable"],
            [("disable", 6), ("disable", 5), ("disable", 3), ("disable", 1)],
        )

    def test_explicit_gripper_enable_and_disable_are_idempotent(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.gripper_status = 0x80

        self.assertTrue(ctl.set_gripper_enabled(True))
        self.assertTrue(ctl.set_gripper_enabled(True))
        self.assertFalse(sdk.enabled)
        self.assertFalse(any(call[0] == "enable" for call in sdk.calls))
        self.assertNotIn(("emergency", 2), sdk.calls)
        self.assertEqual(
            [call for call in sdk.calls if call[0] == "gripper" and call[3] == 1],
            [("gripper", 80000, 1000, 1, 0)],
        )

        self.assertFalse(ctl.set_gripper_enabled(False))
        self.assertFalse(ctl.set_gripper_enabled(False))
        self.assertEqual(
            [call for call in sdk.calls if call[0] == "gripper" and call[3] == 0],
            [("gripper", 80000, 1000, 0, 0)],
        )
        self.assertFalse(sdk.enabled)

    def test_gripper_enable_preserves_partial_arm_enable_state(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.motor_enabled = [True, False, True, False, True, False]
        sdk.gripper_status = 0x80

        self.assertTrue(ctl.set_gripper_enabled(True))

        self.assertEqual(
            sdk.motor_enabled, [True, False, True, False, True, False]
        )
        self.assertFalse(any(call[0] == "enable" for call in sdk.calls))

    def test_jog_allows_encoder_offset_at_three_degree_tolerance(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.joints = [4824, -3000, -5231, 11427, 45747, 96275]
        target = ctl.jog_joint(1, 1.0, 5)
        self.assertEqual(target[0], 5.824)
        self.assertIn(
            ("joints", 5824, -3000, -5231, 11427, 45747, 96275),
            sdk.calls,
        )

    def test_offset_beyond_three_degrees_rejects_all_grouped_motion(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.joints[1] = -3001
        with self.assertRaisesRegex(PiperControlError, "firmware would clamp"):
            ctl.jog_joint(2, -0.5, 5)
        with self.assertRaisesRegex(PiperControlError, "firmware would clamp"):
            ctl.jog_joint(2, 0.5, 5)
        self.assertFalse(any(call[0] == "motion" for call in sdk.calls))

    def test_excessive_encoder_offset_rejects_all_joint_motion(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.joints[2] = 3001
        with self.assertRaisesRegex(PiperControlError, "firmware would clamp"):
            ctl.jog_joint(1, 1.0, 5)
        self.assertFalse(any(call[0] == "motion" for call in sdk.calls))

    def test_jog_limit_is_enforced_before_sdk_motion(self):
        ctl, sdk = controller(allow_motion=True)
        with self.assertRaises(PiperControlError):
            ctl.jog_joint(1, 5.1, 5)
        self.assertFalse(any(call[0] == "motion" for call in sdk.calls))

    def test_gripper_units_match_local_sdk_millimetres(self):
        ctl, sdk = controller(allow_motion=True)
        result = ctl.set_gripper(0.04, 1.0)
        self.assertAlmostEqual(result.opening_m, 0.04)
        self.assertFalse(result.contact_detected)
        target_calls = [
            call
            for call in sdk.calls
            if call == ("gripper", 40000, 1000, 1, 0)
        ]
        self.assertGreaterEqual(len(target_calls), 5)

    def test_gripper_rejects_target_above_physical_limit_before_actuation(self):
        ctl, sdk = controller(allow_motion=True)
        with self.assertRaisesRegex(PiperControlError, "between 0 and 0.070"):
            ctl.set_gripper(0.1, 1.0)
        self.assertFalse(
            any(call[0] in ("emergency", "motion", "enable", "gripper") for call in sdk.calls)
        )

    def test_gripper_enters_can_mode_and_enables_arm_before_target(self):
        ctl, sdk = controller(allow_motion=True)
        ctl.set_gripper(0.04, 1.0)
        resume_index = sdk.calls.index(("emergency", 2))
        mode_index = sdk.calls.index(("motion", 1, 1, 8, 0))
        enable_index = sdk.calls.index(("enable", 6))
        target_index = sdk.calls.index(("gripper", 40000, 1000, 1, 0))
        self.assertLess(resume_index, mode_index)
        self.assertLess(mode_index, enable_index)
        self.assertLess(enable_index, target_index)
        self.assertTrue(sdk.enabled)
        self.assertEqual(sdk.ctrl_mode, 1)

    def test_gripper_rejects_target_when_can_mode_is_not_confirmed(self):
        ctl, sdk = controller(allow_motion=True)

        def ignore_mode_command(*args):
            sdk.calls.append(("motion", *args))

        sdk.MotionCtrl_2 = ignore_mode_command
        with self.assertRaisesRegex(PiperControlError, "CAN command mode"):
            ctl.set_gripper(0.04, 1.0)
        self.assertNotIn(("gripper", 40000, 1000, 1, 0), sdk.calls)
        self.assertFalse(any(call[0] == "gripper" and call[3] == 0 for call in sdk.calls))

    def test_gripper_is_enabled_without_arm_resume_before_target(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.enabled = True
        sdk.gripper_status = 0x80
        result = ctl.set_gripper(0.04, 1.0)
        self.assertAlmostEqual(result.opening_m, 0.04)
        self.assertFalse(result.contact_detected)
        gripper_calls = [call for call in sdk.calls if call[0] == "gripper"]
        self.assertEqual(gripper_calls[0], ("gripper", 80000, 1000, 1, 0))
        self.assertEqual(gripper_calls[1], ("gripper", 40000, 1000, 1, 0))
        self.assertNotIn(("emergency", 2), sdk.calls)
        self.assertTrue(sdk.gripper_status & 0x40)

    def test_gripper_accepts_local_physical_max_with_one_mm_tolerance(self):
        ctl, sdk = controller(allow_motion=True)
        original_gripper_ctrl = sdk.GripperCtrl

        def clamp_at_local_max(opening, effort, code, zero):
            if code == 0x01 and opening == 70000:
                opening = 69100
            original_gripper_ctrl(opening, effort, code, zero)

        sdk.GripperCtrl = clamp_at_local_max

        result = ctl.set_gripper(0.07, 1.0)

        self.assertAlmostEqual(result.opening_m, 0.0691)
        self.assertFalse(result.contact_detected)

    def test_stable_contact_accepts_verified_deep_closing_target(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.gripper = 60000
        original_gripper_ctrl = sdk.GripperCtrl

        def block_on_object(opening, effort, code, zero):
            if code == 0x01 and opening == 30000:
                opening = 37500
            original_gripper_ctrl(opening, effort, code, zero)

        sdk.GripperCtrl = block_on_object

        result = ctl.set_gripper(0.03, 1.0)

        self.assertAlmostEqual(result.opening_m, 0.0375)
        self.assertTrue(result.contact_detected)

    def test_stable_contact_accepts_sub_one_mm_feedback_jitter(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.gripper = 60000
        original_gripper_ctrl = sdk.GripperCtrl
        samples = iter((37500, 38300))

        def block_with_feedback_jitter(opening, effort, code, zero):
            if code == 0x01 and opening == 30000:
                try:
                    opening = next(samples)
                except StopIteration:
                    opening = 38300
            original_gripper_ctrl(opening, effort, code, zero)

        sdk.GripperCtrl = block_with_feedback_jitter

        result = ctl.set_gripper(0.03, 1.0)

        self.assertAlmostEqual(result.opening_m, 0.0383)
        self.assertTrue(result.contact_detected)

    def test_stable_contact_accepts_wide_object_independent_of_target_gap(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.gripper = 60000
        original_gripper_ctrl = sdk.GripperCtrl

        def block_on_wide_object(opening, effort, code, zero):
            if code == 0x01 and opening == 30000:
                opening = 50000
            original_gripper_ctrl(opening, effort, code, zero)

        sdk.GripperCtrl = block_on_wide_object

        result = ctl.set_gripper(0.03, 1.0)

        self.assertAlmostEqual(result.opening_m, 0.05)
        self.assertTrue(result.contact_detected)

    def test_stable_contact_requires_significant_closing_travel(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.gripper = 60000
        original_gripper_ctrl = sdk.GripperCtrl

        def stop_after_tiny_closure(opening, effort, code, zero):
            if code == 0x01 and opening == 30000:
                opening = 58500
            original_gripper_ctrl(opening, effort, code, zero)

        sdk.GripperCtrl = stop_after_tiny_closure

        with self.assertRaisesRegex(
            PiperControlError,
            r"max_closure=0\.0015 m, required_closure=0\.0030 m",
        ):
            ctl.set_gripper(0.03, 1.0)

    def test_stable_contact_rejects_large_feedback_oscillation(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.gripper = 60000
        original_gripper_ctrl = sdk.GripperCtrl
        samples = itertools.cycle((50000, 52000))

        def oscillate_after_closure(opening, effort, code, zero):
            if code == 0x01 and opening == 30000:
                opening = next(samples)
            original_gripper_ctrl(opening, effort, code, zero)

        sdk.GripperCtrl = oscillate_after_closure

        with self.assertRaisesRegex(PiperControlError, "establish stable contact"):
            ctl.set_gripper(0.03, 1.0)

    def test_stable_contact_does_not_accept_shallow_closing_target(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.gripper = 60000
        original_gripper_ctrl = sdk.GripperCtrl

        def stop_without_secure_grasp(opening, effort, code, zero):
            if code == 0x01 and opening == 38000:
                opening = 39700
            original_gripper_ctrl(opening, effort, code, zero)

        sdk.GripperCtrl = stop_without_secure_grasp

        with self.assertRaisesRegex(PiperControlError, "did not reach"):
            ctl.set_gripper(0.038, 1.0)

    def test_stable_contact_does_not_accept_blocked_opening(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.gripper = 30000
        original_gripper_ctrl = sdk.GripperCtrl

        def block_opening(opening, effort, code, zero):
            if code == 0x01 and opening == 60000:
                opening = 40000
            original_gripper_ctrl(opening, effort, code, zero)

        sdk.GripperCtrl = block_opening

        with self.assertRaisesRegex(PiperControlError, "did not reach"):
            ctl.set_gripper(0.06, 1.0)

    def test_gripper_motion_skips_enable_when_already_enabled(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.enabled = True
        ctl.set_gripper(0.04, 1.0)
        self.assertNotIn(("emergency", 2), sdk.calls)
        self.assertFalse(any(call[0] == "enable" for call in sdk.calls))
        self.assertFalse(any(call[0] == "gripper" and call[3] == 3 for call in sdk.calls))

    def test_gripper_open_and_close_restore_delayed_arm_driver_drop(self):
        for target_m in (0.04, 0.0):
            with self.subTest(target_m=target_m):
                ctl, sdk = controller(
                    allow_motion=True,
                    gripper_arm_stability_s=0.04,
                    feedback_timeout_s=0.4,
                )
                sdk.enabled = True
                original_feedback = sdk.GetArmLowSpdInfoMsgs
                post_target_reads = 0
                dropped = False

                def delayed_driver_drop():
                    nonlocal post_target_reads, dropped
                    if any(
                        call[0] == "gripper"
                        and call[1] == round(target_m * 1_000_000.0)
                        and call[3] == 1
                        for call in sdk.calls
                    ):
                        post_target_reads += 1
                        if post_target_reads == 2 and not dropped:
                            sdk.enabled = False
                            sdk.error_code = 0x3F
                            dropped = True
                        elif post_target_reads == 4:
                            sdk.error_code = 0
                    return original_feedback()

                sdk.GetArmLowSpdInfoMsgs = delayed_driver_drop

                ctl.set_gripper(target_m, 1.0)

                self.assertTrue(dropped)
                self.assertTrue(sdk.enabled)
                self.assertNotIn(("emergency", 2), sdk.calls)
                self.assertEqual(
                    [call for call in sdk.calls if call[0] == "enable"],
                    [("enable", motor) for motor in range(1, 7)],
                )

    def test_unhomed_gripper_is_rejected_without_actuation(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.gripper_status = 0
        with self.assertRaisesRegex(PiperControlError, "not homed"):
            ctl.set_gripper(0.04, 1.0)
        self.assertFalse(any(call[0] == "gripper" for call in sdk.calls))

    def test_operator_confirmed_zero_allows_unreported_homing_bit(self):
        ctl, sdk = controller(
            allow_motion=True, operator_confirmed_gripper_zero=True
        )
        sdk.gripper_status = 0
        result = ctl.set_gripper(0.04, 1.0)
        self.assertAlmostEqual(result.opening_m, 0.04)
        self.assertFalse(result.contact_detected)
        self.assertIn(("gripper", 40000, 1000, 1, 0), sdk.calls)

    def test_set_gripper_zero_requires_confirmation_before_actuation(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.gripper = 0
        sdk.gripper_status = 0
        with self.assertRaisesRegex(PiperControlError, "must be true"):
            ctl.set_gripper_zero(False)
        self.assertFalse(any(call[0] == "gripper" for call in sdk.calls))

    def test_set_gripper_zero_requires_motion_unlock(self):
        ctl, sdk = controller(allow_motion=False)
        sdk.gripper = 0
        sdk.gripper_status = 0
        with self.assertRaisesRegex(PiperControlError, "motion is locked"):
            ctl.set_gripper_zero(True)
        self.assertFalse(any(call[0] == "gripper" for call in sdk.calls))

    def test_set_gripper_zero_rejects_open_gripper_before_actuation(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.gripper = 10001
        sdk.gripper_status = 0
        with self.assertRaisesRegex(PiperControlError, "manually close"):
            ctl.set_gripper_zero(True)
        self.assertFalse(any(call[0] == "gripper" for call in sdk.calls))

    def test_set_gripper_zero_uses_sdk_sequence_and_enables_session(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.gripper = 8700
        sdk.gripper_status = 0

        opening = ctl.set_gripper_zero(True)

        self.assertAlmostEqual(opening, 0.0)
        self.assertEqual(
            [call for call in sdk.calls if call[0] == "gripper"],
            [
                ("gripper", 0, 1000, 0, 0),
                ("gripper", 0, 1000, 0, 0xAE),
            ],
        )
        self.assertTrue(ctl.status().gripper_homed)
        self.assertFalse(ctl.status().gripper_enabled)
        self.assertFalse(
            any(
                call[0] in ("emergency", "motion", "enable", "joints")
                for call in sdk.calls
            )
        )

        result = ctl.set_gripper(0.04, 1.0)
        self.assertAlmostEqual(result.opening_m, 0.04)
        self.assertFalse(result.contact_detected)

    def test_gripper_timeout_keeps_existing_enable_state(self):
        ctl, sdk = controller(allow_motion=True)

        def ignore_enabled_command(opening, effort, code, zero):
            sdk.calls.append(("gripper", opening, effort, code, zero))

        sdk.GripperCtrl = ignore_enabled_command
        with self.assertRaisesRegex(PiperControlError, "did not reach"):
            ctl.set_gripper(0.04, 1.0)
        enabled_commands = [
            call for call in sdk.calls if call[0] == "gripper" and call[3] == 1
        ]
        self.assertGreaterEqual(len(enabled_commands), 2)
        self.assertFalse(any(call[0] == "gripper" and call[3] == 0 for call in sdk.calls))
        self.assertTrue(sdk.gripper_status & 0x40)

    def test_gripper_timeout_restores_dropped_arm_drivers(self):
        ctl, sdk = controller(allow_motion=True)
        sdk.enabled = True

        def timeout_and_drop_arm(opening, effort, code, zero):
            sdk.calls.append(("gripper", opening, effort, code, zero))
            if code == 0x01 and opening == 40000:
                sdk.enabled = False

        sdk.GripperCtrl = timeout_and_drop_arm

        with self.assertRaisesRegex(PiperControlError, "did not reach"):
            ctl.set_gripper(0.04, 1.0)

        self.assertTrue(sdk.enabled)
        self.assertEqual(
            [call for call in sdk.calls if call[0] == "enable"],
            [("enable", motor) for motor in range(1, 7)],
        )

    def test_disable_is_allowed_while_motion_is_locked(self):
        ctl, sdk = controller(allow_motion=False)
        sdk.enabled = True
        self.assertFalse(ctl.set_enabled(False))
        self.assertEqual(
            [call for call in sdk.calls if call[0] == "disable"],
            [("disable", motor) for motor in range(6, 0, -1)],
        )

        sdk.gripper_status = 0xC0
        self.assertFalse(ctl.set_gripper_enabled(False))
        self.assertIn(("gripper", 80000, 1000, 0, 0), sdk.calls)

    def test_emergency_stop_is_allowed_while_locked(self):
        ctl, sdk = controller(allow_motion=False)
        ctl.emergency_stop()
        self.assertIn(("emergency", 1), sdk.calls)

    def test_read_only_disconnect_does_not_change_hardware_state(self):
        ctl, sdk = controller(allow_motion=False)
        ctl.disconnect(emergency_stop=True)
        self.assertEqual(sdk.calls, [("connect",), ("disconnect",)])

    def test_disconnect_stops_after_actuation(self):
        ctl, sdk = controller(allow_motion=True)
        ctl.set_gripper(0.04, 1.0)
        ctl.disconnect(emergency_stop=True)
        self.assertEqual(
            sdk.calls[-9:],
            [
                ("emergency", 1),
                ("gripper", 40000, 1000, 0, 0),
                *[("disable", motor) for motor in range(6, 0, -1)],
                ("disconnect",),
            ],
        )
        self.assertFalse(sdk.enabled)
        self.assertFalse(sdk.gripper_status & 0x40)


if __name__ == "__main__":
    unittest.main()
