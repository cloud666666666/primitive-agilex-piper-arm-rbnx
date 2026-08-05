from __future__ import annotations

import unittest
from unittest.mock import patch

from robonix_api import Err, Ok

from piper_ctl import main


class BackendDispatchTests(unittest.TestCase):
    def tearDown(self) -> None:
        main._backend = "ros2"

    def test_backend_defaults_to_ros2(self) -> None:
        with patch.object(main, "_ros_init", return_value=Ok()) as ros_init:
            result = main.init({"can_port": "can_piper"})
        self.assertIsInstance(result, Ok)
        self.assertEqual(main._backend, "ros2")
        ros_init.assert_called_once_with({"can_port": "can_piper"})

    def test_sdk_backend_is_selected_during_init(self) -> None:
        cfg = {"backend": "sdk", "arm_label": "left", "can_port": "can_left"}
        with patch.object(main.sdk_backend, "init", return_value=Ok()) as sdk_init:
            result = main.init(cfg)
        self.assertIsInstance(result, Ok)
        self.assertEqual(main._backend, "sdk")
        sdk_init.assert_called_once_with(cfg)

    def test_unknown_backend_is_rejected(self) -> None:
        result = main.init({"backend": "other"})
        self.assertIsInstance(result, Err)
        self.assertEqual(main._backend, "ros2")

    def test_sdk_lifecycle_routes_to_sdk_backend(self) -> None:
        main._backend = "sdk"
        with (
            patch.object(main.sdk_backend, "activate", return_value=Ok()) as activate,
            patch.object(main.sdk_backend, "deactivate", return_value=Ok()) as deactivate,
            patch.object(main.sdk_backend, "shutdown", return_value=Ok()) as shutdown,
        ):
            self.assertIsInstance(main.activate(), Ok)
            self.assertIsInstance(main.deactivate(), Ok)
            self.assertIsInstance(main.shutdown(), Ok)
        activate.assert_called_once_with()
        deactivate.assert_called_once_with()
        shutdown.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
