from __future__ import annotations

import unittest

from robonix_api import Ok

from piper_arm import main


class LifecycleTests(unittest.TestCase):
    def test_deactivate_and_shutdown_are_registered_no_hardware_callbacks(self) -> None:
        self.assertIs(main.piper_arm._on_deactivate, main.deactivate)
        self.assertIs(main.piper_arm._on_shutdown, main.shutdown)
        self.assertIsNone(main._controller)
        self.assertIsInstance(main.deactivate(), Ok)
        self.assertIsInstance(main.shutdown(), Ok)
        self.assertIsNone(main._controller)


if __name__ == "__main__":
    unittest.main()
