import sys
import unittest
from dataclasses import replace
from pathlib import Path


sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "0_FinalSourceCode")
)

from energy_vision import VisionResult  # noqa: E402
from match_demo_state_machine import MatchController, RobotState  # noqa: E402
from mega_sensor_reader import SensorFrame  # noqa: E402
from motion_controller import DriveCommand  # noqa: E402
from perception import PerceptionEngine, PlatformState  # noqa: E402
from robot_config import DEFAULT_CONFIG, RobotConfig, SensorConfig  # noqa: E402


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

    def apply(self, command, force=False):
        self.commands.append(command)

    def stop(self, force=False):
        self.commands.append(DriveCommand(label="forced-stop"))

    def raise_shovel(self):
        return True

    def lower_shovel(self):
        return True


class FakeVision:
    def __init__(self, clock):
        self.clock = clock

    def latest_result(self):
        return VisionResult.none(self.clock())

    def start(self):
        pass

    def stop(self):
        pass


def sensor_frame(sequence, gray, digital=(0, 0), timestamp=1.0):
    analog = [0] * 15
    analog[12], analog[13] = gray
    return SensorFrame(
        sequence=sequence,
        mega_millis=sequence * 20,
        analog=tuple(analog),
        digital=tuple(digital),
        received_monotonic=timestamp,
    )


def safety_config(
    *,
    sensor_overrides=None,
    timing_overrides=None,
) -> RobotConfig:
    sensor_values = {
        "analog_filter_window": 1,
        "force_platform_on": False,
        "gray_on_is_high": True,
        "gray_on_enter": 600,
        "gray_off_exit": 500,
        "platform_confirm_frames": 1,
        "edge_confirm_frames": 1,
        "edge_clear_frames": 1,
        "full_fall_confirm_frames": 3,
    }
    sensor_values.update(sensor_overrides or {})
    sensors = replace(DEFAULT_CONFIG.sensors, **sensor_values)
    timing = replace(DEFAULT_CONFIG.timing, **(timing_overrides or {}))
    return replace(DEFAULT_CONFIG, sensors=sensors, timing=timing)


class ControllerHarness:
    def __init__(self, config=None):
        self.config = config or safety_config()
        self.clock = FakeClock()
        self.reader = FakeReader()
        self.motion = FakeMotion()
        self.controller = MatchController(
            sensor_reader=self.reader,
            motion_controller=self.motion,
            vision_detector=FakeVision(self.clock),
            config=self.config,
            clock=self.clock,
            wall_clock=self.clock,
        )
        self.controller._publish_status = lambda force=False: None
        self.controller.state = RobotState.ARENA_SEARCH
        self.controller.state_entered = self.clock()
        self.controller.match_started = True
        self.sequence = 0

    def step(self, gray=(700, 700), digital=(0, 0), sequence=None):
        if sequence is None:
            self.sequence += 1
            sequence = self.sequence
        else:
            self.sequence = max(self.sequence, sequence)
        self.reader.frame = sensor_frame(
            sequence,
            gray,
            digital,
            timestamp=self.clock(),
        )
        command = self.controller.step_once(self.clock())
        self.clock.advance()
        return command

    def repeat_current_frame(self):
        command = self.controller.step_once(self.clock())
        self.clock.advance()
        return command

    def remember_linear_command(self, direction):
        speed = 123 * direction
        self.controller._finish_step(
            DriveCommand(speed, speed, "test-linear-history"),
            self.clock(),
        )


class PlatformHysteresisTests(unittest.TestCase):
    def test_high_values_mean_on_platform_with_hysteresis(self):
        config = SensorConfig(
            analog_filter_window=1,
            force_platform_on=False,
            gray_on_is_high=True,
            gray_on_enter=600,
            gray_off_exit=500,
            platform_confirm_frames=1,
        )
        engine = PerceptionEngine(config)

        below_exit = engine.update(
            sensor_frame(1, (499, 499), timestamp=1.00), now=1.00
        )
        in_band_while_off = engine.update(
            sensor_frame(2, (550, 550), timestamp=1.02), now=1.02
        )
        entered = engine.update(
            sensor_frame(3, (600, 600), timestamp=1.04), now=1.04
        )
        in_band_while_on = engine.update(
            sensor_frame(4, (550, 550), timestamp=1.06), now=1.06
        )
        exited = engine.update(
            sensor_frame(5, (499, 499), timestamp=1.08), now=1.08
        )

        self.assertEqual(below_exit.platform_state, PlatformState.OFF)
        self.assertEqual(in_band_while_off.platform_state, PlatformState.OFF)
        self.assertEqual(entered.platform_state, PlatformState.ON)
        self.assertEqual(in_band_while_on.platform_state, PlatformState.ON)
        self.assertEqual(exited.platform_state, PlatformState.OFF)


class FilteredEdgeEventTests(unittest.TestCase):
    def test_raw_di_pulse_does_not_stop_or_enter_edge_recovery(self):
        harness = ControllerHarness(
            safety_config(
                sensor_overrides={
                    "edge_confirm_frames": 2,
                    "edge_clear_frames": 1,
                }
            )
        )

        command = harness.step(gray=(700, 700), digital=(1, 0))

        self.assertTrue(harness.controller.last_perception.front_left_edge_raw)
        self.assertFalse(harness.controller.last_perception.front_left_edge)
        self.assertEqual(harness.controller.state, RobotState.ARENA_SEARCH)
        self.assertGreater(command.left_speed, 0)
        self.assertGreater(command.right_speed, 0)

    def test_filtered_di_rise_enters_edge_recovery(self):
        harness = ControllerHarness(
            safety_config(sensor_overrides={"edge_confirm_frames": 2})
        )

        harness.step(gray=(700, 700), digital=(1, 0))
        command = harness.step(gray=(700, 700), digital=(1, 0))

        self.assertTrue(harness.controller.last_perception.front_left_edge)
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))

    def test_held_filtered_di_is_consumed_only_once_until_cleared(self):
        config = safety_config(
            timing_overrides={
                "edge_stop_time": 0.0,
                "edge_reverse_time": 0.0,
                "edge_turn_time": 0.0,
            }
        )
        harness = ControllerHarness(config)

        harness.step(gray=(700, 700), digital=(1, 0))
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)

        # A safe grayscale state is sufficient to finish recovery even while
        # the already-consumed filtered DI level remains asserted.
        harness.step(gray=(700, 700), digital=(1, 0))
        self.assertEqual(harness.controller.state, RobotState.ARENA_SEARCH)

        harness.step(gray=(700, 700), digital=(1, 0))
        self.assertEqual(harness.controller.state, RobotState.ARENA_SEARCH)

        # Clearing the filtered input rearms the edge event. A later rise is a
        # new physical crossing event and must preempt the arena state again.
        harness.step(gray=(700, 700), digital=(0, 0))
        command = harness.step(gray=(700, 700), digital=(1, 0))
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))


class CompleteFallTests(unittest.TestCase):
    def _begin_fall_without_prior_platform_on(self, harness):
        # The first controller frame is already below the grayscale threshold.
        # The filtered DI rise alone provides the boundary-crossing evidence;
        # no earlier PlatformState.ON observation is required.
        harness.step(gray=(400, 400), digital=(1, 0))
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)
        self.assertTrue(harness.controller._fall_evidence_latched)

    def test_full_fall_needs_no_prior_on_or_platform_state_off(self):
        harness = ControllerHarness(
            safety_config(
                sensor_overrides={
                    # Keep the aggregate state UNKNOWN throughout this test.
                    # The decision deliberately uses the two filtered
                    # grayscale flags instead of reconfirming PlatformState.OFF.
                    "platform_confirm_frames": 99,
                }
            )
        )
        self._begin_fall_without_prior_platform_on(harness)

        for expected_count in (1, 2):
            harness.step(gray=(400, 400), digital=(0, 0))
            self.assertEqual(
                harness.controller.last_perception.platform_state,
                PlatformState.UNKNOWN,
            )
            self.assertEqual(
                harness.controller._full_fall_confirm_count,
                expected_count,
            )
            self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)

        harness.step(gray=(400, 400), digital=(0, 0))
        self.assertEqual(harness.controller.state, RobotState.GROUND_SEARCH)

    def test_full_fall_votes_count_only_distinct_mega_sequences(self):
        harness = ControllerHarness(
            safety_config(sensor_overrides={"platform_confirm_frames": 99})
        )
        self._begin_fall_without_prior_platform_on(harness)

        harness.step(gray=(400, 400), digital=(0, 0))
        self.assertEqual(harness.controller._full_fall_confirm_count, 1)

        for _ in range(8):
            harness.repeat_current_frame()

        self.assertEqual(harness.controller._full_fall_confirm_count, 1)
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)

        harness.step(gray=(400, 400), digital=(0, 0))
        harness.step(gray=(400, 400), digital=(0, 0))
        self.assertEqual(harness.controller.state, RobotState.GROUND_SEARCH)

    def test_raw_di_high_does_not_block_vote_when_filtered_di_is_clear(self):
        harness = ControllerHarness(
            safety_config(
                sensor_overrides={
                    "edge_confirm_frames": 2,
                    "edge_clear_frames": 1,
                    "platform_confirm_frames": 99,
                }
            )
        )

        # Two raw frames are required to create the one filtered rise event.
        harness.step(gray=(400, 400), digital=(1, 0))
        harness.step(gray=(400, 400), digital=(1, 0))
        self.assertTrue(harness.controller._fall_evidence_latched)

        harness.step(gray=(400, 400), digital=(0, 0))
        self.assertEqual(harness.controller._full_fall_confirm_count, 1)

        # This is a new raw high sample, but it has not passed the two-frame DI
        # filter. Control and full-fall confirmation must use the filtered bit.
        harness.step(gray=(400, 400), digital=(1, 0))
        perception = harness.controller.last_perception
        self.assertTrue(perception.front_left_edge_raw)
        self.assertFalse(perception.front_left_edge)
        self.assertEqual(harness.controller._full_fall_confirm_count, 2)

        harness.step(gray=(400, 400), digital=(0, 0))
        self.assertEqual(harness.controller.state, RobotState.GROUND_SEARCH)

    def test_votes_wait_for_both_filtered_di_inputs_to_clear(self):
        harness = ControllerHarness(
            safety_config(sensor_overrides={"platform_confirm_frames": 99})
        )
        self._begin_fall_without_prior_platform_on(harness)

        for _ in range(4):
            harness.step(gray=(400, 400), digital=(1, 0))

        self.assertEqual(harness.controller._full_fall_confirm_count, 0)
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)

        harness.step(gray=(400, 400), digital=(0, 0))
        self.assertEqual(harness.controller._full_fall_confirm_count, 1)

    def test_stable_gray_on_during_recovery_does_not_erase_di_evidence(self):
        harness = ControllerHarness()

        harness.step(gray=(700, 700), digital=(1, 0))
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)
        self.assertTrue(harness.controller._fall_evidence_latched)

        # The forward DI can see the drop before either bottom grayscale
        # channel reaches the rim. This intermediate ON frame must not erase
        # the evidence while the recovery action is still in progress.
        harness.step(gray=(700, 700), digital=(1, 0))
        self.assertTrue(harness.controller._fall_evidence_latched)

        for _ in range(harness.config.sensors.full_fall_confirm_frames):
            harness.step(gray=(400, 400), digital=(0, 0))

        self.assertEqual(harness.controller.state, RobotState.GROUND_SEARCH)

    def test_a_grayscale_on_flag_resets_incomplete_full_fall_votes(self):
        harness = ControllerHarness(
            safety_config(sensor_overrides={"platform_confirm_frames": 99})
        )
        self._begin_fall_without_prior_platform_on(harness)

        harness.step(gray=(400, 400), digital=(0, 0))
        self.assertEqual(harness.controller._full_fall_confirm_count, 1)

        harness.step(gray=(700, 400), digital=(0, 0))
        self.assertEqual(harness.controller._full_fall_confirm_count, 0)
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)

        for _ in range(harness.config.sensors.full_fall_confirm_frames):
            harness.step(gray=(400, 400), digital=(0, 0))
        self.assertEqual(harness.controller.state, RobotState.GROUND_SEARCH)

    def test_dual_gray_off_without_filtered_di_event_never_becomes_ground(self):
        harness = ControllerHarness(
            safety_config(
                timing_overrides={"edge_recover_timeout": 5.0},
            )
        )

        for _ in range(10):
            harness.step(gray=(400, 400), digital=(0, 0))

        self.assertFalse(harness.controller._fall_evidence_latched)
        self.assertEqual(harness.controller._full_fall_confirm_count, 0)
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)


class GrayscaleEdgeRecoveryTests(unittest.TestCase):
    def _direction_after_gray_preemption(self, gray, *, history=0):
        harness = ControllerHarness(
            safety_config(timing_overrides={"edge_stop_time": 0.0})
        )
        if history:
            harness.remember_linear_command(history)

        stop = harness.step(gray=gray)
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)
        self.assertEqual((stop.left_speed, stop.right_speed), (0, 0))
        command = harness.step(gray=gray)
        return harness, command

    def test_front_gray_off_reverses_toward_platform_interior(self):
        _, command = self._direction_after_gray_preemption((400, 700))

        self.assertLess(command.left_speed, 0)
        self.assertLess(command.right_speed, 0)

    def test_rear_gray_off_drives_forward_toward_platform_interior(self):
        _, command = self._direction_after_gray_preemption((700, 400))

        self.assertGreater(command.left_speed, 0)
        self.assertGreater(command.right_speed, 0)

    def test_dual_gray_off_reverses_last_forward_direction(self):
        _, command = self._direction_after_gray_preemption(
            (400, 400), history=1
        )

        self.assertLess(command.left_speed, 0)
        self.assertLess(command.right_speed, 0)

    def test_dual_gray_off_reverses_last_backward_direction(self):
        _, command = self._direction_after_gray_preemption(
            (400, 400), history=-1
        )

        self.assertGreater(command.left_speed, 0)
        self.assertGreater(command.right_speed, 0)

    def test_stop_and_in_place_turn_do_not_replace_linear_history(self):
        harness = ControllerHarness(
            safety_config(timing_overrides={"edge_stop_time": 0.0})
        )
        harness.remember_linear_command(1)
        harness.controller._finish_step(
            DriveCommand(200, -200, "test-turn"), harness.clock()
        )
        harness.controller._finish_step(
            DriveCommand(label="test-stop"), harness.clock()
        )

        harness.step(gray=(400, 400))
        command = harness.step(gray=(400, 400))

        self.assertLess(command.left_speed, 0)
        self.assertLess(command.right_speed, 0)

    def test_stable_gray_on_can_finish_di_recovery_while_di_remains_high(self):
        harness = ControllerHarness(
            safety_config(
                timing_overrides={
                    "edge_stop_time": 0.0,
                    "edge_reverse_time": 0.0,
                    "edge_turn_time": 0.0,
                }
            )
        )

        harness.step(gray=(700, 700), digital=(1, 0))
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)
        harness.step(gray=(700, 700), digital=(1, 0))

        self.assertTrue(harness.controller.last_perception.front_left_edge)
        self.assertEqual(harness.controller.state, RobotState.ARENA_SEARCH)

    def test_backward_gray_recovery_turns_inward_before_arena_patrol(self):
        turn_time = 0.05
        harness = ControllerHarness(
            safety_config(
                timing_overrides={
                    "edge_stop_time": 0.0,
                    "edge_turn_time": turn_time,
                }
            )
        )

        harness.step(gray=(400, 700))
        reverse = harness.step(gray=(400, 700))
        self.assertLess(reverse.left_speed, 0)
        self.assertLess(reverse.right_speed, 0)

        safe_stop = harness.step(gray=(700, 700))
        self.assertEqual((safe_stop.left_speed, safe_stop.right_speed), (0, 0))
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)

        turn = harness.step(gray=(700, 700))
        self.assertGreater(turn.left_speed, 0)
        self.assertLess(turn.right_speed, 0)
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)

        harness.clock.value = harness.controller._edge_turn_started_at + turn_time
        complete = harness.step(gray=(700, 700))
        self.assertEqual((complete.left_speed, complete.right_speed), (0, 0))
        self.assertEqual(harness.controller.state, RobotState.ARENA_SEARCH)

    def test_ambiguous_dual_off_timeout_replans_in_opposite_direction(self):
        timeout = 0.05
        harness = ControllerHarness(
            safety_config(
                timing_overrides={
                    "edge_stop_time": 0.0,
                    "edge_recover_timeout": timeout,
                }
            )
        )
        harness.remember_linear_command(1)

        harness.step(gray=(400, 400))
        first_attempt = harness.step(gray=(400, 400))
        self.assertLess(first_attempt.left_speed, 0)
        self.assertLess(first_attempt.right_speed, 0)

        harness.clock.value = harness.controller.state_entered + timeout + 0.01
        replan_stop = harness.step(gray=(400, 400))
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)
        self.assertNotEqual(harness.controller.state, RobotState.FAULT_STOP)
        self.assertEqual((replan_stop.left_speed, replan_stop.right_speed), (0, 0))

        second_attempt = harness.step(gray=(400, 400))
        self.assertGreater(second_attempt.left_speed, 0)
        self.assertGreater(second_attempt.right_speed, 0)

        # A second failed attempt is also replanned instead of becoming a
        # permanent fault. The guess alternates back to the original side.
        harness.clock.value = harness.controller.state_entered + timeout + 0.01
        harness.step(gray=(400, 400))
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)
        third_attempt = harness.step(gray=(400, 400))
        self.assertLess(third_attempt.left_speed, 0)
        self.assertLess(third_attempt.right_speed, 0)


class FaultRecoveryTests(unittest.TestCase):
    def test_deploy_shovel_fault_recovers_to_ground_search(self):
        harness = ControllerHarness(
            safety_config(timing_overrides={"fault_recover_time": 0.0})
        )
        harness.controller.state = RobotState.DEPLOY_SHOVEL
        harness.controller._transition(
            RobotState.FAULT_STOP,
            "test shovel deployment fault",
            harness.clock(),
        )

        command = harness.step(gray=(400, 400))

        self.assertEqual(command.label, "fault-stop")
        self.assertEqual(harness.controller.state, RobotState.GROUND_SEARCH)

    def test_ground_state_fault_recovers_to_ground_search_without_replay(self):
        harness = ControllerHarness(
            safety_config(timing_overrides={"fault_recover_time": 0.0})
        )
        harness.controller.state = RobotState.CLIMB_BACKWARD
        harness.controller._transition(
            RobotState.FAULT_STOP,
            "test ground fault",
            harness.clock(),
        )

        command = harness.step(gray=(700, 700))

        self.assertEqual(command.label, "fault-stop")
        self.assertEqual(harness.controller.state, RobotState.GROUND_SEARCH)
        self.assertNotEqual(
            harness.controller.state,
            RobotState.STARTUP_CLIMB_BURST,
        )

    def test_arena_fault_outside_stable_on_initializes_edge_recovery(self):
        harness = ControllerHarness(
            safety_config(timing_overrides={"fault_recover_time": 0.0})
        )
        harness.controller._transition(
            RobotState.FAULT_STOP,
            "test arena fault",
            harness.clock(),
        )

        command = harness.step(gray=(400, 700))

        self.assertEqual(command.label, "fault-stop")
        self.assertEqual(harness.controller.state, RobotState.EDGE_RECOVER)
        self.assertEqual(harness.controller._edge_recovery_mode, "gray")
        self.assertEqual(harness.controller._edge_recovery_drive_direction, -1)


class ConfigurationValidationTests(unittest.TestCase):
    def test_full_fall_confirmation_frames_must_be_a_positive_integer(self):
        for value in (True, 0, -1, 1.5):
            with self.subTest(value=value):
                config = safety_config(
                    sensor_overrides={"full_fall_confirm_frames": value}
                )
                with self.assertRaisesRegex(
                    ValueError, "full_fall_confirm_frames"
                ):
                    ControllerHarness(config)


if __name__ == "__main__":
    unittest.main()
