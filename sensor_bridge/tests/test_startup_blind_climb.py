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
from perception import PlatformState  # noqa: E402
from robot_config import DEFAULT_CONFIG, RobotConfig  # noqa: E402


class FakeClock:
    def __init__(self):
        self.value = 10.0

    def __call__(self):
        return self.value


class FakeReader:
    def __init__(self):
        self.frame = None

    def latest_frame(self):
        return self.frame


class FakeMotion:
    def __init__(self):
        self.commands = []
        self.shovel_pose = "raised"

    def apply(self, command, force=False):
        self.commands.append(command)

    def lower_shovel(self):
        self.shovel_pose = "lowered"
        return True


class FakeVision:
    def __init__(self, clock):
        self.result = VisionResult.none(clock())

    def latest_result(self):
        return self.result


def startup_config(**timing_updates):
    sensors = replace(
        DEFAULT_CONFIG.sensors,
        analog_filter_window=1,
        force_platform_on=True,
        rear_high_confirm_frames=1,
        rear_high_clear_frames=1,
        platform_confirm_frames=1,
    )
    values = {
        "shovel_settle_time": 0.06,
        "startup_climb_backward_time": 0.10,
        "stuck_timeout": 0.03,
        "status_publish_interval": 999.0,
    }
    values.update(timing_updates)
    timing = replace(DEFAULT_CONFIG.timing, **values)
    return RobotConfig(
        hardware=DEFAULT_CONFIG.hardware,
        sensors=sensors,
        motion=DEFAULT_CONFIG.motion,
        timing=timing,
        vision=DEFAULT_CONFIG.vision,
    )


class StartupHarness:
    def __init__(self, config=None):
        self.clock = FakeClock()
        self.reader = FakeReader()
        self.motion = FakeMotion()
        self.vision = FakeVision(self.clock)
        self.controller = MatchController(
            sensor_reader=self.reader,
            motion_controller=self.motion,
            vision_detector=self.vision,
            config=config or startup_config(),
            clock=self.clock,
            wall_clock=self.clock,
        )
        self.controller._publish_status = lambda force=False: None
        self.sequence = 0

    def step(
        self,
        *,
        hands=False,
        gray=(300, 300),
        rear_high=0,
        digital=(0, 0),
        sensor_age=0.0,
    ):
        analog = [0] * 15
        analog[12], analog[13] = gray
        analog[14] = rear_high
        if hands:
            analog[3] = 800
            analog[9] = 800
        self.sequence += 1
        self.reader.frame = SensorFrame(
            sequence=self.sequence,
            mega_millis=self.sequence * 20,
            analog=tuple(analog),
            digital=digital,
            received_monotonic=self.clock() - sensor_age,
        )
        return self.controller.step_once(self.clock())

    def enter_startup_burst(self):
        self.step()
        self.step(hands=True)
        self.clock.value = (
            self.controller.state_entered
            + self.controller.config.timing.shovel_settle_time
        )
        command = self.step(hands=True)
        return command, self.controller.state_entered


class StartupBlindClimbTests(unittest.TestCase):
    def test_startup_climb_time_requires_a_finite_positive_number(self):
        invalid_values = (
            True,
            0,
            -0.1,
            float("nan"),
            float("inf"),
            -float("inf"),
        )
        for invalid in invalid_values:
            with self.subTest(invalid=invalid):
                with self.assertRaisesRegex(
                    ValueError,
                    "startup_climb_backward_time must be a finite positive number",
                ):
                    StartupHarness(
                        startup_config(startup_climb_backward_time=invalid)
                    )

    def test_boot_does_not_wait_for_grayscale_platform_semantics(self):
        base = startup_config()
        sensors = replace(
            base.sensors,
            force_platform_on=False,
            platform_confirm_frames=3,
        )
        h = StartupHarness(replace(base, sensors=sensors))

        command = h.step(gray=(600, 600))

        self.assertEqual(
            h.controller.last_perception.platform_state,
            PlatformState.UNKNOWN,
        )
        self.assertEqual(h.controller.state, RobotState.WAIT_START_GESTURE)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))

    def test_shovel_settle_transitions_to_dedicated_startup_burst(self):
        h = StartupHarness()

        h.step()
        trigger = h.step(hands=True)
        self.assertEqual(h.controller.state, RobotState.DEPLOY_SHOVEL)
        self.assertEqual((trigger.left_speed, trigger.right_speed), (0, 0))
        self.assertEqual(h.motion.shovel_pose, "lowered")

        h.clock.value = (
            h.controller.state_entered
            + h.controller.config.timing.shovel_settle_time
            - 1e-6
        )
        settling = h.step(hands=True)
        self.assertEqual(h.controller.state, RobotState.DEPLOY_SHOVEL)
        self.assertEqual((settling.left_speed, settling.right_speed), (0, 0))

        h.clock.value += 1e-6
        transition = h.step(hands=True)
        speed = h.controller.config.motion.climb_speed
        self.assertEqual(h.controller.state, RobotState.STARTUP_CLIMB_BURST)
        self.assertEqual(
            (transition.left_speed, transition.right_speed),
            (-speed, -speed),
        )
        self.assertEqual(transition.label, "startup-climb-backward")

    def test_on_platform_reading_cannot_end_startup_climb_before_expiry(self):
        base = startup_config()
        sensors = replace(base.sensors, force_platform_on=False)
        h = StartupHarness(replace(base, sensors=sensors))
        transition, entered = h.enter_startup_burst()
        self.assertEqual(transition.label, "startup-climb-backward")

        duration = h.controller.config.timing.startup_climb_backward_time
        h.clock.value = entered + duration - 1e-6
        moving = h.step(gray=(700, 700))
        speed = h.controller.config.motion.climb_speed
        self.assertEqual(h.controller.state, RobotState.STARTUP_CLIMB_BURST)
        self.assertEqual((moving.left_speed, moving.right_speed), (-speed, -speed))
        self.assertEqual(moving.label, "startup-climb-backward")

    def test_startup_burst_ignores_rear_high_until_normal_climb_off_platform(self):
        base = startup_config()
        sensors = replace(base.sensors, force_platform_on=False)
        h = StartupHarness(replace(base, sensors=sensors))
        _, entered = h.enter_startup_burst()

        h.clock.value = entered + 0.02
        command = h.step(rear_high=800)

        speed = h.controller.config.motion.climb_speed
        self.assertEqual(h.controller.state, RobotState.STARTUP_CLIMB_BURST)
        self.assertEqual((command.left_speed, command.right_speed), (-speed, -speed))

        duration = h.controller.config.timing.startup_climb_backward_time
        h.clock.value = entered + duration + 1e-6
        complete = h.step(rear_high=800)
        self.assertEqual(h.controller.state, RobotState.FENCE_ESCAPE)
        self.assertEqual((complete.left_speed, complete.right_speed), (0, 0))
        self.assertEqual(complete.label, "climb-fence-stop")

    def test_startup_burst_expiry_prefers_platform_over_rear_high(self):
        h = StartupHarness()
        _, entered = h.enter_startup_burst()

        h.clock.value = (
            entered
            + h.controller.config.timing.startup_climb_backward_time
            + 1e-6
        )
        command = h.step(rear_high=800)

        self.assertTrue(h.controller.last_perception.rear_high_object)
        self.assertEqual(h.controller.state, RobotState.CLIMB_REORIENT)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))
        self.assertEqual(command.label, "climb-success-stop")

    def test_climb_prepare_prefers_platform_over_rear_high(self):
        h = StartupHarness()
        h.controller.state = RobotState.CLIMB_PREPARE
        h.controller.state_entered = h.clock()

        command = h.step(rear_high=800)

        self.assertTrue(h.controller.last_perception.rear_high_object)
        self.assertEqual(h.controller.state, RobotState.CLIMB_REORIENT)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))
        self.assertEqual(command.label, "climb-prepare-already-on-stop")

    def test_normal_climb_prefers_platform_over_rear_high(self):
        h = StartupHarness()
        h.controller.state = RobotState.CLIMB_PREPARE
        h.controller.state_entered = h.clock()
        h.controller._transition(
            RobotState.CLIMB_BACKWARD,
            "rear-high priority test",
            h.clock(),
        )

        command = h.step(rear_high=800)

        self.assertTrue(h.controller.last_perception.rear_high_object)
        self.assertEqual(h.controller.state, RobotState.CLIMB_REORIENT)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))
        self.assertEqual(command.label, "climb-success-stop")

    def test_normal_grayscale_semantics_apply_on_burst_expiry_frame(self):
        h = StartupHarness()
        _, entered = h.enter_startup_burst()

        h.clock.value = (
            entered
            + h.controller.config.timing.startup_climb_backward_time
            + 1e-6
        )
        command = h.step()

        self.assertEqual(h.controller.state, RobotState.CLIMB_REORIENT)
        self.assertEqual(command.label, "climb-success-stop")

    def test_startup_burst_ignores_edge_inputs(self):
        h = StartupHarness()
        _, entered = h.enter_startup_burst()

        command = None
        for frame_index in range(h.controller.config.sensors.edge_confirm_frames):
            h.clock.value = entered + 0.01 * (frame_index + 1)
            command = h.step(digital=(1, 1))

        speed = h.controller.config.motion.climb_speed
        self.assertTrue(h.controller.last_perception.front_left_edge)
        self.assertTrue(h.controller.last_perception.front_right_edge)
        self.assertEqual(h.controller.state, RobotState.STARTUP_CLIMB_BURST)
        self.assertEqual((command.left_speed, command.right_speed), (-speed, -speed))

    def test_sensor_fault_stops_burst_without_replaying_it(self):
        h = StartupHarness()
        _, entered = h.enter_startup_burst()

        h.clock.value = entered + 0.02
        h.reader.frame = None
        missing = h.controller.step_once(h.clock())
        self.assertEqual(h.controller.state, RobotState.FAULT_STOP)
        self.assertEqual(missing.label, "sensor-missing-stop")

        h.step()
        h.clock.value += h.controller.config.timing.fault_recover_time
        recovered = h.step()
        self.assertEqual(h.controller.state, RobotState.GROUND_SEARCH)
        self.assertNotEqual(h.controller.state, RobotState.STARTUP_CLIMB_BURST)
        self.assertEqual(recovered.label, "fault-stop")

        next_command = h.step()
        self.assertNotEqual(h.controller.state, RobotState.STARTUP_CLIMB_BURST)
        self.assertNotEqual(next_command.label, "startup-climb-backward")

    def test_only_hard_sensor_staleness_preempts_startup_burst(self):
        h = StartupHarness()
        _, entered = h.enter_startup_burst()
        timing = h.controller.config.timing

        h.clock.value = entered + 0.02
        soft_stale = h.step(
            sensor_age=(timing.sensor_warning_after + timing.sensor_stop_after) / 2
        )
        speed = h.controller.config.motion.climb_speed
        self.assertEqual(h.controller.state, RobotState.STARTUP_CLIMB_BURST)
        self.assertEqual(
            (soft_stale.left_speed, soft_stale.right_speed),
            (-speed, -speed),
        )

        hard_stale = h.step(sensor_age=timing.sensor_stop_after + 1e-6)
        self.assertEqual(h.controller.state, RobotState.FAULT_STOP)
        self.assertEqual(hard_stale.label, "sensor-stale-stop")

    def test_startup_burst_is_not_cancelled_by_static_feature_watchdog(self):
        config = startup_config(
            startup_climb_backward_time=0.20,
            stuck_timeout=0.03,
        )
        h = StartupHarness(config)
        _, entered = h.enter_startup_burst()

        h.clock.value = entered + 0.01
        h.step()
        h.clock.value = entered + 0.10
        command = h.step()

        speed = h.controller.config.motion.climb_speed
        self.assertEqual(h.controller.state, RobotState.STARTUP_CLIMB_BURST)
        self.assertEqual((command.left_speed, command.right_speed), (-speed, -speed))

    def test_startup_burst_counts_toward_total_climb_timeout(self):
        base = startup_config(climb_timeout=0.15)
        sensors = replace(base.sensors, force_platform_on=False)
        h = StartupHarness(replace(base, sensors=sensors))
        _, entered = h.enter_startup_burst()

        h.clock.value = (
            entered
            + h.controller.config.timing.startup_climb_backward_time
            + 1e-6
        )
        h.step(gray=(300, 300))
        self.assertEqual(h.controller.state, RobotState.CLIMB_BACKWARD)

        h.clock.value = (
            entered + h.controller.config.timing.climb_timeout + 1e-6
        )
        timed_out = h.step(gray=(300, 300))
        self.assertEqual(h.controller.state, RobotState.GROUND_SEARCH)
        self.assertEqual(timed_out.label, "climb-timeout-stop")

    def test_total_climb_timeout_can_stop_an_overlong_startup_burst(self):
        config = startup_config(
            startup_climb_backward_time=0.20,
            climb_timeout=0.15,
        )
        h = StartupHarness(config)
        _, entered = h.enter_startup_burst()

        h.clock.value = entered + config.timing.climb_timeout
        timed_out = h.step()

        self.assertEqual(h.controller.state, RobotState.GROUND_SEARCH)
        self.assertEqual((timed_out.left_speed, timed_out.right_speed), (0, 0))
        self.assertEqual(timed_out.label, "climb-timeout-stop")

    def test_match_end_still_preempts_startup_burst(self):
        config = startup_config(match_duration=0.08)
        h = StartupHarness(config)
        h.enter_startup_burst()

        h.clock.value = h.controller.match_start_time + config.timing.match_duration
        command = h.step()

        self.assertEqual(h.controller.state, RobotState.MATCH_END)
        self.assertEqual(command.label, "match-end-stop")

    def test_status_derives_startup_flags_from_explicit_state(self):
        h = StartupHarness()
        h.step()
        h.step(hands=True)
        deploy_status = h.controller._status_snapshot()
        self.assertFalse(deploy_status["startup_climb_active"])
        self.assertTrue(deploy_status["startup_climb_pending"])

        h.clock.value = (
            h.controller.state_entered
            + h.controller.config.timing.shovel_settle_time
        )
        h.step(hands=True)
        burst_status = h.controller._status_snapshot()
        self.assertTrue(burst_status["startup_climb_active"])
        self.assertTrue(burst_status["startup_climb_pending"])

        h.clock.value += (
            h.controller.config.timing.startup_climb_backward_time + 1e-6
        )
        h.step()
        climb_status = h.controller._status_snapshot()
        self.assertFalse(climb_status["startup_climb_active"])
        self.assertFalse(climb_status["startup_climb_pending"])

    def test_normal_climb_stops_on_first_double_on_then_waits_for_stable_on(self):
        sensors = replace(
            startup_config().sensors,
            force_platform_on=False,
            gray_on_is_high=True,
            gray_on_enter=650,
            gray_off_exit=600,
            platform_confirm_frames=3,
        )
        config = replace(startup_config(), sensors=sensors)
        h = StartupHarness(config)
        h.controller.state = RobotState.CLIMB_PREPARE
        h.controller.state_entered = h.clock()
        h.controller._transition(
            RobotState.CLIMB_BACKWARD,
            "normal climb test",
            h.clock(),
        )

        speed = h.controller.config.motion.climb_speed
        moving = h.step(gray=(500, 500))
        self.assertEqual(h.controller.state, RobotState.CLIMB_BACKWARD)
        self.assertEqual((moving.left_speed, moving.right_speed), (-speed, -speed))

        h.clock.value += 0.02
        one_on = h.step(gray=(700, 500))
        self.assertTrue(h.controller.last_perception.front_on_platform)
        self.assertFalse(h.controller.last_perception.rear_on_platform)
        self.assertEqual(h.controller.state, RobotState.CLIMB_BACKWARD)
        self.assertEqual((one_on.left_speed, one_on.right_speed), (-speed, -speed))

        h.clock.value += 0.02
        first_on = h.step(gray=(700, 700))
        self.assertTrue(h.controller.last_perception.front_on_platform)
        self.assertTrue(h.controller.last_perception.rear_on_platform)
        self.assertNotEqual(
            h.controller.last_perception.platform_state,
            PlatformState.ON,
        )
        self.assertEqual(h.controller.state, RobotState.CLIMB_BACKWARD)
        self.assertEqual((first_on.left_speed, first_on.right_speed), (0, 0))

        h.clock.value += 0.02
        confirming = h.step(gray=(700, 700))
        self.assertEqual(h.controller.state, RobotState.CLIMB_BACKWARD)
        self.assertEqual((confirming.left_speed, confirming.right_speed), (0, 0))

        h.clock.value += 0.02
        stable = h.step(gray=(700, 700))
        self.assertEqual(
            h.controller.last_perception.platform_state,
            PlatformState.ON,
        )
        self.assertEqual(h.controller.state, RobotState.CLIMB_REORIENT)
        self.assertEqual((stable.left_speed, stable.right_speed), (0, 0))
        self.assertEqual(stable.label, "climb-success-stop")

    def test_climb_reorient_turns_right_then_enters_arena_search(self):
        base = startup_config(stuck_timeout=0.01)
        sensors = replace(
            base.sensors,
            force_platform_on=False,
            gray_on_is_high=True,
            gray_on_enter=650,
            gray_off_exit=600,
            platform_confirm_frames=1,
        )
        h = StartupHarness(replace(base, sensors=sensors))
        h.controller.state = RobotState.CLIMB_BACKWARD
        h.controller.state_entered = h.clock()
        h.controller._transition(
            RobotState.CLIMB_REORIENT,
            "reorient test",
            h.clock(),
        )

        self.assertEqual(h.controller.config.motion.climb_reorient_turn_speed, 700)
        self.assertAlmostEqual(
            h.controller.config.timing.climb_reorient_turn_time,
            1.40,
        )
        speed = h.controller.config.motion.climb_reorient_turn_speed
        started = h.controller.state_entered

        turning = h.step(gray=(700, 700))
        self.assertEqual(h.controller.state, RobotState.CLIMB_REORIENT)
        self.assertEqual((turning.left_speed, turning.right_speed), (speed, -speed))

        h.clock.value = (
            started + h.controller.config.timing.climb_reorient_turn_time - 1e-6
        )
        still_turning = h.step(gray=(700, 700))
        self.assertEqual(h.controller.state, RobotState.CLIMB_REORIENT)
        self.assertEqual(
            (still_turning.left_speed, still_turning.right_speed),
            (speed, -speed),
        )

        h.clock.value += 1e-6
        complete = h.step(gray=(700, 700))
        self.assertEqual(h.controller.state, RobotState.ARENA_SEARCH)
        self.assertEqual((complete.left_speed, complete.right_speed), (0, 0))

    def test_normal_climb_ignores_stuck_watchdog_but_keeps_total_timeout(self):
        base = startup_config(stuck_timeout=0.03, climb_timeout=0.20)
        sensors = replace(
            base.sensors,
            force_platform_on=False,
            gray_on_is_high=True,
            gray_on_enter=650,
            gray_off_exit=600,
            platform_confirm_frames=1,
        )
        h = StartupHarness(replace(base, sensors=sensors))
        h.controller.state = RobotState.CLIMB_PREPARE
        h.controller.state_entered = h.clock()
        h.controller._transition(
            RobotState.CLIMB_BACKWARD,
            "normal climb watchdog test",
            h.clock(),
        )
        entered = h.controller.state_entered

        speed = h.controller.config.motion.climb_speed
        first = h.step(gray=(500, 500))
        self.assertEqual((first.left_speed, first.right_speed), (-speed, -speed))

        h.clock.value = entered + 0.10
        after_stuck_deadline = h.step(gray=(500, 500))
        self.assertEqual(h.controller.state, RobotState.CLIMB_BACKWARD)
        self.assertEqual(
            (after_stuck_deadline.left_speed, after_stuck_deadline.right_speed),
            (-speed, -speed),
        )
        self.assertNotEqual(after_stuck_deadline.label, "stuck-watchdog-stop")

        h.clock.value = (
            entered + h.controller.config.timing.climb_timeout + 1e-6
        )
        timed_out = h.step(gray=(500, 500))

        self.assertEqual(h.controller.state, RobotState.GROUND_SEARCH)
        self.assertEqual((timed_out.left_speed, timed_out.right_speed), (0, 0))
        self.assertEqual(timed_out.label, "climb-timeout-stop")


if __name__ == "__main__":
    unittest.main()
