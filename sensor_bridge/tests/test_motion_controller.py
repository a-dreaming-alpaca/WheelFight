import sys
import types
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch


sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "0_FinalSourceCode")
)

from motion_controller import MotionController  # noqa: E402
from robot_config import HardwareConfig  # noqa: E402


class FakeUpTech:
    def __init__(self):
        self.calls = []

    def CDS_Open(self):
        self.calls.append(("open",))

    def CDS_Close(self):
        self.calls.append(("close",))

    def CDS_SetMode(self, device_id, mode):
        self.calls.append(("mode", device_id, mode))

    def CDS_SetSpeed(self, device_id, speed):
        self.calls.append(("speed", device_id, speed))

    def CDS_SetAngle(self, device_id, angle, speed):
        self.calls.append(("angle", device_id, angle, speed))


class MotionControllerTests(unittest.TestCase):
    def test_real_hardware_branch_initializes_project_path_before_uptech_import(self):
        uptech = FakeUpTech()
        uptech_module = types.ModuleType("uptech")
        uptech_module.UpTech = lambda: uptech

        with patch.dict(sys.modules, {"uptech": uptech_module}):
            with patch(
                "motion_controller.add_project_root_to_path"
            ) as add_project_root:
                controller = MotionController(open_bus=False)

        add_project_root.assert_called_once_with()
        self.assertIs(controller.uptech, uptech)

    def test_motor_mapping_right_inversion_and_servo_modes(self):
        uptech = FakeUpTech()
        controller = MotionController(uptech=uptech, open_bus=False)
        controller.move_cmd(400, 500)

        mode_calls = [call for call in uptech.calls if call[0] == "mode"]
        self.assertEqual(mode_calls, [("mode", 5, 0), ("mode", 6, 0)])
        self.assertIn(("speed", 2, 400), uptech.calls)
        self.assertIn(("speed", 1, -500), uptech.calls)

    def test_shovel_uses_ids_five_and_six_when_interlock_enabled(self):
        uptech = FakeUpTech()
        config = replace(
            HardwareConfig(),
            shovel_motion_enabled=True,
            shovel_lowered_left=640,
            shovel_lowered_right=384,
        )
        controller = MotionController(uptech=uptech, config=config, open_bus=False)
        self.assertTrue(controller.lower_shovel())
        self.assertIn(("angle", 5, 640, config.servo_speed), uptech.calls)
        self.assertIn(("angle", 6, 384, config.servo_speed), uptech.calls)


if __name__ == "__main__":
    unittest.main()
