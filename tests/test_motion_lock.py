from __future__ import annotations

import stat
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import piper_arm_pb2

from piper_arm_sdk import main
from piper_provider import piper_arm


class FakeController:
    def __init__(self, *, motion_unlocked: bool) -> None:
        self.allow_motion = motion_unlocked
        self.calls: list[bool] = []

    def set_motion_locked(self, locked: bool) -> bool:
        self.calls.append(locked)
        self.allow_motion = not locked
        return locked


class LockedMcpController:
    def __init__(self) -> None:
        self.allow_motion = False
        self.enable_calls: list[bool] = []

    def status(self):
        return SimpleNamespace(
            connected=True,
            enabled=False,
            motion_unlocked=False,
            arm="left",
            can_port="can_left",
            joints_deg=(0.0,) * 6,
            telemetry_valid=True,
            joint_voltages_v=(24.0,) * 6,
            joint_currents_a=(0.0,) * 6,
            joint_motor_temperatures_c=(30.0,) * 6,
            joint_driver_temperatures_c=(30.0,) * 6,
            joint_driver_enabled=(False,) * 6,
            joint_communication_ok=(True,) * 6,
            joint_fault_codes=(0,) * 6,
            gripper_opening_m=0.0,
            gripper_enabled=False,
            gripper_homed=True,
            gripper_fault=False,
            gripper_status_code=0x80,
            error_code=0,
            detail="ready; motion locked",
        )

    def set_enabled(self, enabled: bool) -> bool:
        self.enable_calls.append(enabled)
        raise main.PiperControlError(
            "hardware motion is locked; operator authorization is required to unlock"
        )


class MotionLockAuthorizationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.previous_controller = main._controller
        self.previous_config = main._config
        self.previous_token = main._operator_token
        self.previous_token_path = main._operator_token_path
        main._config = {"arm_label": "left"}
        main._operator_token_path = None

    def tearDown(self) -> None:
        main._revoke_operator_token()
        main._controller = self.previous_controller
        main._config = self.previous_config
        main._operator_token = self.previous_token
        main._operator_token_path = self.previous_token_path

    def test_unlock_rejects_missing_or_invalid_operator_token(self) -> None:
        controller = FakeController(motion_unlocked=False)
        main._controller = controller
        main._operator_token = "expected-token"

        for supplied in ("", "wrong-token"):
            response = main.motion_lock(
                piper_arm_pb2.SetMotionLock_Request(
                    locked=False,
                    operator_token=supplied,
                )
            )
            self.assertFalse(response.success)
            self.assertTrue(response.locked)
            self.assertFalse(response.motion_unlocked)
            self.assertEqual(response.detail, "operator authorization required")

        self.assertEqual(controller.calls, [])

    def test_invalid_unlock_reports_already_unlocked_state_without_controller_call(
        self,
    ) -> None:
        controller = FakeController(motion_unlocked=True)
        main._controller = controller
        main._operator_token = "expected-token"

        response = main.motion_lock(
            piper_arm_pb2.SetMotionLock_Request(
                locked=False,
                operator_token="wrong-token",
            )
        )

        self.assertFalse(response.success)
        self.assertFalse(response.locked)
        self.assertTrue(response.motion_unlocked)
        self.assertEqual(response.detail, "operator authorization required")
        self.assertEqual(controller.calls, [])

    def test_motion_lock_is_registered_for_grpc_only(self) -> None:
        contract = "robonix/primitive/arm/motion_lock"
        grpc_contracts = {item[0] for item in piper_arm._grpc_handlers}
        mcp_contracts = {
            getattr(handler, "_robonix_contract_id", "")
            for handler in piper_arm._mcp_handlers
        }

        self.assertIn(contract, grpc_contracts)
        self.assertNotIn(contract, mcp_contracts)

    def test_locked_status_gives_pilot_exact_operator_handoff(self) -> None:
        main._controller = LockedMcpController()

        with patch.dict(main.os.environ, {"RBNX_INSTANCE_NAME": "left_piper"}):
            response = main.status(object())

        self.assertFalse(response.motion_unlocked)
        self.assertIn("Pilot cannot unlock motion", response.detail)
        self.assertIn("scripts/motion_lock.py unlock --provider left_piper", response.detail)
        self.assertIn("require motion_unlocked=true", response.detail)
        self.assertNotIn("current-token", response.detail)

    def test_lock_rejected_mcp_call_gives_same_operator_handoff(self) -> None:
        controller = LockedMcpController()
        main._controller = controller

        with patch.dict(main.os.environ, {"RBNX_INSTANCE_NAME": "left_piper"}):
            response = main.enable_arm(object())

        self.assertFalse(response.success)
        self.assertEqual(controller.enable_calls, [True])
        self.assertIn("Pilot cannot unlock motion", response.detail)
        self.assertIn("scripts/motion_lock.py unlock --provider left_piper", response.detail)

    def test_unrelated_error_does_not_suggest_unlock(self) -> None:
        main._controller = FakeController(motion_unlocked=False)

        detail = main._mcp_error_detail(RuntimeError("CAN write failed"))

        self.assertEqual(detail, "CAN write failed")

    def test_unlock_accepts_current_operator_token(self) -> None:
        controller = FakeController(motion_unlocked=False)
        main._controller = controller
        main._operator_token = "current-token"

        response = main.motion_lock(
            piper_arm_pb2.SetMotionLock_Request(
                locked=False,
                operator_token="current-token",
            )
        )

        self.assertTrue(response.success)
        self.assertFalse(response.locked)
        self.assertTrue(response.motion_unlocked)
        self.assertEqual(controller.calls, [False])

    def test_lock_is_fail_safe_and_does_not_require_token(self) -> None:
        controller = FakeController(motion_unlocked=True)
        main._controller = controller
        main._operator_token = "current-token"

        response = main.motion_lock(
            piper_arm_pb2.SetMotionLock_Request(
                locked=True,
                operator_token="",
            )
        )

        self.assertTrue(response.success)
        self.assertTrue(response.locked)
        self.assertFalse(response.motion_unlocked)
        self.assertEqual(controller.calls, [True])

    def test_controller_failure_remains_on_hardware_error_path(self) -> None:
        controller = FakeController(motion_unlocked=False)
        main._controller = controller

        with patch.object(
            controller,
            "set_motion_locked",
            side_effect=RuntimeError("CAN write failed"),
        ) as set_motion_locked:
            response = main.motion_lock(
                piper_arm_pb2.SetMotionLock_Request(
                    locked=True,
                    operator_token="",
                )
            )

        self.assertFalse(response.success)
        self.assertTrue(response.locked)
        self.assertFalse(response.motion_unlocked)
        self.assertEqual(response.detail, "CAN write failed")
        set_motion_locked.assert_called_once_with(True)

    def test_operator_token_is_owner_only_and_revoked(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "left_piper.motion-token"
            with (
                patch.object(main, "_motion_token_file", return_value=path),
                patch.object(
                    main.secrets,
                    "token_urlsafe",
                    return_value="session-token",
                ),
            ):
                provisioned = main._provision_operator_token()

            self.assertEqual(provisioned, path)
            self.assertEqual(path.read_text(encoding="utf-8"), "session-token\n")
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
            main._revoke_operator_token()
            self.assertFalse(path.exists())


if __name__ == "__main__":
    unittest.main()
