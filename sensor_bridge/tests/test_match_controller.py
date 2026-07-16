import sys
import unittest
from dataclasses import replace
from pathlib import Path


sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "0_FinalSourceCode")
)

from energy_vision import EnergyClass, VisionResult  # noqa: E402
from match_demo_state_machine import MatchController, RobotState  # noqa: E402
from mega_sensor_reader import SensorFrame  # noqa: E402
from motion_controller import DriveCommand  # noqa: E402
from robot_config import DEFAULT_CONFIG, RobotConfig  # noqa: E402


class FakeClock:
    def __init__(self):
        self.value = 10.0

    def __call__(self):
        return self.value

    def advance(self, seconds=0.02):
        self.value += seconds


class FakeReader:
    def __init__(self):
        self.frame = None

    def latest_frame(self):
        return self.frame

    def start(self):
        pass

    def stop(self):
        pass


class FakeMotion:
    def __init__(self):
        self.commands = []
        self.shovel_pose = "raised"

    def apply(self, command, force=False):
        self.commands.append(command)

    def stop(self, force=False):
        self.commands.append(DriveCommand(label="forced-stop"))

    def raise_shovel(self):
        self.shovel_pose = "raised"
        return True

    def lower_shovel(self):
        self.shovel_pose = "lowered"
        return True


class FakeVision:
    def __init__(self, clock):
        self.clock = clock
        self.result = VisionResult.none(clock())

    def latest_result(self):
        return self.result

    def start(self):
        pass

    def stop(self):
        pass


def test_config():
    sensors = replace(
        DEFAULT_CONFIG.sensors,
        analog_filter_window=1,
        edge_clear_frames=1,
        rear_high_confirm_frames=1,
        platform_confirm_frames=1,
    )
    timing = replace(
        DEFAULT_CONFIG.timing,
        sensor_stop_after=0.20,
        start_clear_time=0.01,
        start_hand_confirm_time=0.01,
        start_release_confirm_time=0.01,
        start_release_delay=0.01,
        shovel_settle_time=0.01,
        ground_candidate_confirm=0.01,
        platform_verify_time=0.01,
        target_classify_timeout=0.20,
        fault_recover_time=0.01,
        match_duration=0.10,
        status_publish_interval=999.0,
    )
    vision = replace(
        DEFAULT_CONFIG.vision,
        classify_votes=2,
        no_marker_votes_for_enemy=2,
    )
    return RobotConfig(
        hardware=DEFAULT_CONFIG.hardware,
        sensors=sensors,
        motion=DEFAULT_CONFIG.motion,
        timing=timing,
        vision=vision,
    )


class ControllerHarness:
    def __init__(self):
        self.clock = FakeClock()
        self.reader = FakeReader()
        self.motion = FakeMotion()
        self.vision = FakeVision(self.clock)
        self.controller = MatchController(
            sensor_reader=self.reader,
            motion_controller=self.motion,
            vision_detector=self.vision,
            config=test_config(),
            clock=self.clock,
            wall_clock=self.clock,
        )
        self.controller._publish_status = lambda force=False: None
        self.sequence = 0

    def step(
        self,
        *,
        ir=None,
        gray=(100, 100),
        digital=(0, 0, 1),
        vision=None,
        received_age=0.0,
    ):
        # The first ranging calibration measured an unobstructed IR value near 0.
        analog = [0] * 14
        analog[12], analog[13] = gray
        if ir:
            for index, value in ir.items():
                analog[index] = value
        if vision is not None:
            self.vision.result = VisionResult(
                classification=vision,
                confidence=1.0,
                center_x=320.0,
                bbox_width=80.0,
                tag_id=2 if vision == EnergyClass.HARMFUL else None,
                timestamp=self.clock(),
                frame_width=640,
            )
        self.sequence += 1
        self.reader.frame = SensorFrame(
            sequence=self.sequence,
            mega_millis=self.sequence * 20,
            analog=tuple(analog),
            digital=tuple(digital),
            received_monotonic=self.clock() - received_age,
        )
        command = self.controller.step_once(self.clock())
        self.clock.advance()
        return command


class MatchControllerTests(unittest.TestCase):
    def test_two_hand_press_and_release_generates_start_event(self):
        h = ControllerHarness()
        h.step()  # boot -> wait clear
        h.step()
        h.step()  # clear held -> wait hands
        h.step(ir={3: 800, 9: 800})
        h.step(ir={3: 800, 9: 800})  # hands confirmed
        self.assertEqual(h.controller.state, RobotState.WAIT_START_RELEASE)
        h.step()
        h.step()  # release confirmed
        self.assertEqual(h.controller.state, RobotState.START_RELEASE_DELAY)
        self.assertTrue(h.controller.match_started)
        h.step()
        h.step()
        self.assertEqual(h.controller.state, RobotState.GROUND_SEARCH)
        self.assertEqual(h.motion.shovel_pose, "lowered")

    def test_platform_and_fence_verification_use_rear_high_sensor(self):
        platform = ControllerHarness()
        platform.controller.state = RobotState.VERIFY_PLATFORM
        platform.controller.state_entered = platform.clock()
        platform.step(ir={6: 800}, digital=(0, 0, 1))
        platform.step(ir={6: 800}, digital=(0, 0, 1))
        self.assertEqual(platform.controller.state, RobotState.CLIMB_BACKWARD)

        fence = ControllerHarness()
        fence.controller.state = RobotState.VERIFY_PLATFORM
        fence.controller.state_entered = fence.clock()
        command = fence.step(ir={6: 800}, digital=(0, 0, 0))
        self.assertEqual(fence.controller.state, RobotState.FENCE_ESCAPE)
        self.assertEqual(command.label, "fence-confirmed-stop")

    def test_climb_requires_rear_then_both_grayscale_on_platform(self):
        h = ControllerHarness()
        h.controller.state = RobotState.CLIMB_BACKWARD
        h.controller.state_entered = h.clock()
        h.step(ir={6: 800}, gray=(100, 700))
        self.assertEqual(h.controller.state, RobotState.CLIMB_BACKWARD)
        h.step(ir={6: 800}, gray=(700, 700))
        self.assertEqual(h.controller.state, RobotState.CLIMB_CLEAR_EDGE)

    def test_partial_fall_recovery_moves_toward_platform(self):
        front_off = ControllerHarness()
        front_off.controller.state = RobotState.PARTIAL_FALL_RECOVER
        front_off.controller.state_entered = front_off.clock()
        command = front_off.step(gray=(100, 700))
        self.assertLess(command.left_speed, 0)
        self.assertLess(command.right_speed, 0)

        rear_off = ControllerHarness()
        rear_off.controller.state = RobotState.PARTIAL_FALL_RECOVER
        rear_off.controller.state_entered = rear_off.clock()
        command = rear_off.step(gray=(700, 100))
        self.assertGreater(command.left_speed, 0)
        self.assertGreater(command.right_speed, 0)

    def test_stale_sensor_stops_motion_and_enters_fault(self):
        h = ControllerHarness()
        h.controller.state = RobotState.ATTACK_ENEMY
        h.controller.state_entered = h.clock()
        h.controller.match_started = True
        h.controller.match_start_time = h.clock()
        command = h.step(gray=(700, 700), received_age=0.21)
        self.assertEqual(h.controller.state, RobotState.FAULT_STOP)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))
        self.assertEqual(command.label, "sensor-stale-stop")

    def test_match_duration_is_a_terminal_stop(self):
        h = ControllerHarness()
        h.controller.state = RobotState.ATTACK_ENEMY
        h.controller.state_entered = h.clock()
        h.controller.match_started = True
        h.controller.match_start_time = h.clock() - 0.11
        command = h.step(gray=(700, 700))
        self.assertEqual(h.controller.state, RobotState.MATCH_END)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))

    def test_edge_immediately_preempts_enemy_attack(self):
        h = ControllerHarness()
        h.controller.state = RobotState.ATTACK_ENEMY
        h.controller.state_entered = h.clock()
        h.controller.match_started = True
        h.controller.match_start_time = h.clock()
        command = h.step(ir={0: 800}, gray=(700, 700), digital=(1, 0, 1))
        self.assertEqual(h.controller.state, RobotState.EDGE_RECOVER)
        self.assertEqual(command.left_speed, 0)
        self.assertEqual(command.right_speed, 0)

    def test_rear_high_object_preempts_climb_as_fence(self):
        h = ControllerHarness()
        h.controller.state = RobotState.CLIMB_BACKWARD
        h.controller.state_entered = h.clock()
        h.controller.match_started = True
        h.controller.match_start_time = h.clock()
        command = h.step(ir={6: 800}, gray=(100, 100), digital=(0, 0, 0))
        self.assertEqual(h.controller.state, RobotState.FENCE_ESCAPE)
        self.assertEqual(command.label, "climb-fence-stop")

    def test_harmful_tag_is_avoided_after_multiple_votes(self):
        h = ControllerHarness()
        h.controller.state = RobotState.TARGET_CLASSIFY
        h.controller.state_entered = h.clock()
        h.controller.match_started = True
        h.controller.match_start_time = h.clock()
        h.step(ir={0: 800}, gray=(700, 700), vision=EnergyClass.HARMFUL)
        h.step(ir={0: 800}, gray=(700, 700), vision=EnergyClass.HARMFUL)
        self.assertEqual(h.controller.state, RobotState.AVOID_BLOCK)

    def test_gain_tag_enters_push_after_multiple_votes(self):
        h = ControllerHarness()
        h.controller.state = RobotState.TARGET_CLASSIFY
        h.controller.state_entered = h.clock()
        h.controller.match_started = True
        h.controller.match_start_time = h.clock()
        h.step(ir={0: 800}, gray=(700, 700), vision=EnergyClass.GAIN)
        h.step(ir={0: 800}, gray=(700, 700), vision=EnergyClass.GAIN)
        self.assertEqual(h.controller.state, RobotState.PUSH_GAIN_BLOCK)

    def test_repeated_good_frames_without_marker_classify_enemy(self):
        h = ControllerHarness()
        h.controller.state = RobotState.TARGET_CLASSIFY
        h.controller.state_entered = h.clock()
        h.controller.match_started = True
        h.controller.match_start_time = h.clock()
        h.step(ir={0: 800}, gray=(700, 700), vision=EnergyClass.NO_BLOCK_MARKER)
        h.step(ir={0: 800}, gray=(700, 700), vision=EnergyClass.NO_BLOCK_MARKER)
        self.assertEqual(h.controller.state, RobotState.ATTACK_ENEMY)

    def test_far_no_marker_target_is_not_assumed_to_be_enemy(self):
        h = ControllerHarness()
        h.controller.state = RobotState.TARGET_CLASSIFY
        h.controller.state_entered = h.clock()
        h.controller.match_started = True
        h.controller.match_start_time = h.clock()
        h.step(ir={0: 300}, gray=(700, 700), vision=EnergyClass.NO_BLOCK_MARKER)
        h.step(ir={0: 300}, gray=(700, 700), vision=EnergyClass.NO_BLOCK_MARKER)
        self.assertEqual(h.controller.state, RobotState.TARGET_CLASSIFY)


if __name__ == "__main__":
    unittest.main()
