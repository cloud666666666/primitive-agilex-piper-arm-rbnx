from __future__ import annotations

import unittest

from robonix_api import Ok

from piper_arm_sdk import main as sdk_backend
from piper_ctl import main


class LifecycleTests(unittest.TestCase):
    def test_deactivate_and_shutdown_are_registered_no_hardware_callbacks(self) -> None:
        self.assertIs(main.piper_ctl._on_deactivate, main.deactivate)
        self.assertIs(main.piper_ctl._on_shutdown, main.shutdown)
        self.assertIsNone(sdk_backend._controller)
        previous_backend = main._backend
        try:
            main._backend = "sdk"
            self.assertIsInstance(main.deactivate(), Ok)
            self.assertIsInstance(main.shutdown(), Ok)
        finally:
            main._backend = previous_backend
        self.assertIsNone(sdk_backend._controller)


if __name__ == "__main__":
    unittest.main()
