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
        # State-machine scenarios use fixed synthetic readings; keep their
        # thresholds independent from the current on-robot calibration.
        ir_detect_enter=200,
        ir_detect_exit=150,
        no_marker_enemy_ir_threshold=350,
        # These state-machine tests consume semantic platform states and keep
        # their legacy synthetic levels independent from hardware polarity.
        gray_on_is_high=True,
        gray_on_enter=550,
        gray_off_exit=450,
        edge_clear_frames=1,
        rear_high_detect_enter=350,
        rear_high_detect_exit=300,
        rear_high_confirm_frames=1,
        rear_high_clear_frames=1,
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
        climb_prepare_forward_time=0.04,
        climb_prepare_settle_time=0.06,
        target_center_confirm_time=0.05,
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
    def __init__(self, config=None):
        self.clock = FakeClock()
        self.reader = FakeReader()
        self.motion = FakeMotion()
        self.vision = FakeVision(self.clock)
        self.controller = MatchController(
            sensor_reader=self.reader,
            motion_controller=self.motion,
            vision_detector=self.vision,
            config=config or test_config(),
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
        rear_high=0,
        digital=(0, 0),
        vision=None,
        vision_confidence=1.0,
        received_age=0.0,
    ):
        # The first ranging calibration measured an unobstructed IR value near 0.
        analog = [0] * 15
        analog[12], analog[13] = gray
        analog[14] = rear_high
        if ir:
            for index, value in ir.items():
                analog[index] = value
        if vision is not None:
            self.vision.result = VisionResult(
                classification=vision,
                confidence=vision_confidence,
                center_x=320.0,
                bbox_width=80.0,
                tag_id=None,
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
    def test_invalid_rear_ir_channel_config_fails_before_startup(self):
        for rear_indices in ((), (12,), (-1,), ("6",)):
            with self.subTest(rear_indices=rear_indices):
                config = test_config()
                sensors = replace(
                    config.sensors,
                    rear_platform_ir_indices=rear_indices,
                )
                clock = FakeClock()
                with self.assertRaisesRegex(
                    ValueError,
                    "rear_platform_ir_indices",
                ):
                    MatchController(
                        sensor_reader=FakeReader(),
                        motion_controller=FakeMotion(),
                        vision_detector=FakeVision(clock),
                        config=replace(config, sensors=sensors),
                        clock=clock,
                        wall_clock=clock,
                    )

    def test_default_reader_uses_configured_serial_timings(self):
        config = test_config()
        timing = replace(
            config.timing,
            sensor_stop_after=0.19,
            sensor_read_timeout=0.037,
            sensor_reconnect_interval=0.42,
        )
        config = replace(config, timing=timing)
        clock = FakeClock()

        controller = MatchController(
            motion_controller=FakeMotion(),
            vision_detector=FakeVision(clock),
            config=config,
            clock=clock,
            wall_clock=clock,
            mega_port="TEST_PORT",
        )

        self.assertEqual(controller.sensor_reader.port, "TEST_PORT")
        self.assertEqual(controller.sensor_reader.stale_after, 0.19)
        self.assertEqual(controller.sensor_reader.read_timeout, 0.037)
        self.assertEqual(controller.sensor_reader.reconnect_interval, 0.42)

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
        platform.step(ir={6: 800}, digital=(0, 0))
        platform.step(ir={6: 800}, digital=(0, 0))
        self.assertEqual(platform.controller.state, RobotState.CLIMB_PREPARE)

        fence = ControllerHarness()
        fence.controller.state = RobotState.VERIFY_PLATFORM
        fence.controller.state_entered = fence.clock()
        command = fence.step(ir={6: 800}, rear_high=800, digital=(0, 0))
        self.assertEqual(fence.controller.state, RobotState.FENCE_ESCAPE)
        self.assertEqual(command.label, "fence-confirmed-stop")

    def test_rear_alignment_holds_still_before_platform_verification(self):
        h = ControllerHarness()
        h.controller.state = RobotState.ALIGN_REAR
        h.controller.state_entered = h.clock()

        command = h.step(ir={6: 300})
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))
        self.assertEqual(command.label, "rear-align-hold")
        self.assertEqual(h.controller.state, RobotState.ALIGN_REAR)

        command = h.step(ir={6: 300})
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))
        self.assertEqual(h.controller.state, RobotState.VERIFY_PLATFORM)

        turning = ControllerHarness()
        turning.controller.state = RobotState.ALIGN_REAR
        turning.controller.state_entered = turning.clock()
        command = turning.step(ir={5: 300})
        self.assertNotEqual(command.left_speed, 0)
        self.assertEqual(command.left_speed, -command.right_speed)

    def test_rear_candidate_loss_uses_configured_grace_time(self):
        config = test_config()
        timing = replace(config.timing, rear_candidate_lost_grace=0.07)
        h = ControllerHarness(replace(config, timing=timing))
        h.controller.state = RobotState.ALIGN_REAR
        h.controller.state_entered = h.clock()

        h.step(ir={5: 800})
        h.clock.value = h.controller.state_entered + 0.50
        h.step(ir={5: 800})
        loss_started = h.clock()
        command = h.step()
        self.assertEqual(h.controller.state, RobotState.ALIGN_REAR)
        self.assertEqual(command.label, "rear-align-target-lost")

        h.clock.value = loss_started + timing.rear_candidate_lost_grace - 1e-6
        command = h.step()
        self.assertEqual(h.controller.state, RobotState.ALIGN_REAR)
        self.assertEqual(command.label, "rear-align-target-lost")

        h.clock.value = loss_started + timing.rear_candidate_lost_grace + 1e-6
        command = h.step()
        self.assertEqual(h.controller.state, RobotState.GROUND_SEARCH)
        self.assertEqual(command.label, "rear-align-target-lost")

    def test_platform_verification_uses_configured_rear_ir_channels(self):
        config = test_config()
        sensors = replace(config.sensors, rear_platform_ir_indices=(4,))
        h = ControllerHarness(replace(config, sensors=sensors))
        h.controller.state = RobotState.VERIFY_PLATFORM
        h.controller.state_entered = h.clock()

        h.step(ir={4: 800})
        command = h.step(ir={4: 800})

        self.assertEqual(h.controller.state, RobotState.CLIMB_PREPARE)
        self.assertEqual(command.label, "platform-verified-stop")

    def test_rear_alignment_uses_configured_rear_ir_channels(self):
        config = test_config()
        sensors = replace(
            config.sensors,
            rear_platform_ir_indices=(4,),
            alignment_tolerance_deg=70.0,
        )
        h = ControllerHarness(replace(config, sensors=sensors))
        h.controller.state = RobotState.ALIGN_REAR
        h.controller.state_entered = h.clock()

        command = h.step(ir={4: 800})
        self.assertEqual(command.label, "rear-align-hold")
        command = h.step(ir={4: 800})

        self.assertEqual(h.controller.state, RobotState.VERIFY_PLATFORM)
        self.assertEqual(command.label, "rear-aligned-stop")

    def test_climb_prepare_creates_runup_then_settles_before_reversing(self):
        h = ControllerHarness()
        h.controller.state = RobotState.CLIMB_PREPARE
        h.controller.state_entered = h.clock()
        h.controller._climb_seen_rear_on = True

        command = h.step(ir={6: 800})
        prepare_speed = h.controller.config.motion.climb_prepare_speed
        self.assertEqual(
            (command.left_speed, command.right_speed),
            (prepare_speed, prepare_speed),
        )
        self.assertEqual(h.controller.state, RobotState.CLIMB_PREPARE)

        entered = h.controller.state_entered
        h.clock.value = (
            entered + h.controller.config.timing.climb_prepare_forward_time
            + 1e-6
        )
        command = h.step(ir={6: 800})
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))
        self.assertEqual(h.controller.state, RobotState.CLIMB_PREPARE)

        h.clock.value = (
            entered
            + h.controller.config.timing.climb_prepare_forward_time
            + h.controller.config.timing.climb_prepare_settle_time
            + 1e-6
        )
        command = h.step(ir={6: 800})
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))
        self.assertEqual(h.controller.state, RobotState.CLIMB_BACKWARD)
        self.assertFalse(h.controller._climb_seen_rear_on)

        command = h.step(ir={6: 800})
        climb_speed = h.controller.config.motion.climb_speed
        self.assertEqual(
            (command.left_speed, command.right_speed),
            (-climb_speed, -climb_speed),
        )

    def test_climb_prepare_continues_when_rear_ir_disappears(self):
        h = ControllerHarness()
        h.controller.state = RobotState.CLIMB_PREPARE
        h.controller.state_entered = h.clock()

        command = h.step()

        self.assertEqual(h.controller.state, RobotState.CLIMB_PREPARE)
        self.assertGreater(command.left_speed, 0)
        self.assertEqual(command.left_speed, command.right_speed)

    def test_rear_high_object_preempts_climb_prepare_as_fence(self):
        h = ControllerHarness()
        h.controller.state = RobotState.CLIMB_PREPARE
        h.controller.state_entered = h.clock()

        command = h.step(ir={6: 800}, rear_high=800, digital=(0, 0))

        self.assertEqual(h.controller.state, RobotState.FENCE_ESCAPE)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))
        self.assertEqual(command.label, "climb-fence-stop")

    def test_climb_prepare_skips_runup_when_already_on_platform(self):
        h = ControllerHarness()
        h.controller.state = RobotState.CLIMB_PREPARE
        h.controller.state_entered = h.clock()

        command = h.step(gray=(700, 700))

        self.assertEqual(h.controller.state, RobotState.CLIMB_CLEAR_EDGE)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))

    def test_climb_prepare_prefers_on_platform_over_rear_high_object(self):
        h = ControllerHarness()
        h.controller.state = RobotState.CLIMB_PREPARE
        h.controller.state_entered = h.clock()

        command = h.step(gray=(700, 700), rear_high=800, digital=(0, 0))

        self.assertEqual(h.controller.state, RobotState.CLIMB_CLEAR_EDGE)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))
        self.assertEqual(command.label, "climb-prepare-already-on-stop")

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

    def test_arena_search_patrols_straight_forward_without_target(self):
        h = ControllerHarness()
        h.controller.state = RobotState.ARENA_SEARCH
        h.controller.state_entered = h.clock()

        command = h.step(gray=(700, 700))

        patrol_speed = h.controller.config.motion.arena_patrol_speed
        self.assertGreater(command.left_speed, 0)
        self.assertEqual(
            (command.left_speed, command.right_speed),
            (patrol_speed, patrol_speed),
        )

    def test_arena_search_stops_and_aligns_when_target_appears(self):
        h = ControllerHarness()
        h.controller.state = RobotState.ARENA_SEARCH
        h.controller.state_entered = h.clock()

        command = h.step(ir={2: 300}, gray=(700, 700))

        self.assertEqual(h.controller.state, RobotState.TARGET_ALIGN)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))

    def test_target_align_prefers_cluster_closest_to_front_over_strongest(self):
        h = ControllerHarness()
        h.controller.state = RobotState.TARGET_ALIGN
        h.controller.state_entered = h.clock()

        command = h.step(ir={1: 300, 9: 800}, gray=(700, 700))

        self.assertEqual(h.controller.state, RobotState.TARGET_ALIGN)
        self.assertGreater(command.left_speed, 0)
        self.assertLess(command.right_speed, 0)
        self.assertEqual(command.label, "target-align")

    def test_target_align_stops_while_center_confirmation_is_held(self):
        h = ControllerHarness()
        h.controller.state = RobotState.TARGET_ALIGN
        h.controller.state_entered = h.clock()
        hold_started = h.clock()
        confirm_time = h.controller.config.timing.target_center_confirm_time

        command = h.step(ir={0: 300}, gray=(700, 700))

        self.assertEqual(h.controller.state, RobotState.TARGET_ALIGN)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))
        self.assertEqual(command.label, "target-align-hold")

        h.clock.value = hold_started + confirm_time - 1e-6
        command = h.step(ir={0: 300}, gray=(700, 700))

        self.assertEqual(h.controller.state, RobotState.TARGET_ALIGN)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))
        self.assertEqual(command.label, "target-align-hold")

        h.clock.value = hold_started + confirm_time + 1e-6
        command = h.step(ir={0: 300}, gray=(700, 700))

        self.assertEqual(h.controller.state, RobotState.TARGET_CLASSIFY)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))
        self.assertEqual(command.label, "target-centered-stop")

    def test_classification_loss_angle_uses_configured_limit(self):
        config = test_config()
        sensors = replace(
            config.sensors,
            target_classify_loss_bearing_deg=70.0,
        )
        h = ControllerHarness(replace(config, sensors=sensors))
        h.controller.state = RobotState.TARGET_CLASSIFY
        h.controller.state_entered = h.clock()

        command = h.step(ir={2: 800}, gray=(700, 700))

        self.assertEqual(h.controller.state, RobotState.TARGET_ALIGN)
        self.assertEqual(command.label, "classify-realign-stop")

    def test_attack_tracking_angle_uses_configured_limit(self):
        config = test_config()
        sensors = replace(config.sensors, attack_target_max_bearing_deg=45.0)
        h = ControllerHarness(replace(config, sensors=sensors))
        h.controller.state = RobotState.ATTACK_ENEMY
        h.controller.state_entered = h.clock()
        h.controller._target_last_seen = (
            h.clock() - config.timing.target_lost_grace - 0.01
        )

        command = h.step(ir={2: 800}, gray=(700, 700))

        self.assertEqual(h.controller.state, RobotState.ARENA_SEARCH)
        self.assertEqual(command.label, "attack-target-lost-stop")

    def test_stuck_watchdog_uses_configured_analog_bin_size(self):
        config = test_config()
        sensors = replace(config.sensors, stuck_analog_bin_size=100)
        timing = replace(config.timing, stuck_timeout=0.05)
        h = ControllerHarness(
            replace(config, sensors=sensors, timing=timing)
        )
        h.controller.state = RobotState.ATTACK_ENEMY
        h.controller.state_entered = h.clock()

        h.step(ir={0: 300}, gray=(700, 700))
        h.clock.value = (
            h.controller._feature_changed_at + timing.stuck_timeout + 1e-6
        )
        command = h.step(ir={0: 340}, gray=(700, 700))

        self.assertEqual(h.controller.state, RobotState.AVOID_BLOCK)
        self.assertEqual(command.label, "stuck-watchdog-stop")

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
        self.assertEqual(
            h.controller.state_reason,
            f"{h.controller.config.timing.match_duration:g} second match end",
        )

    def test_edge_immediately_preempts_enemy_attack(self):
        h = ControllerHarness()
        h.controller.state = RobotState.ATTACK_ENEMY
        h.controller.state_entered = h.clock()
        h.controller.match_started = True
        h.controller.match_start_time = h.clock()
        command = h.step(ir={0: 800}, gray=(700, 700), digital=(1, 0))
        self.assertEqual(h.controller.state, RobotState.EDGE_RECOVER)
        self.assertEqual(command.left_speed, 0)
        self.assertEqual(command.right_speed, 0)

    def test_double_edge_recovery_always_turns_right_without_alternating(self):
        h = ControllerHarness()
        timing = h.controller.config.timing

        def enter_double_edge_recovery():
            h.controller.state = RobotState.ARENA_SEARCH
            h.controller.state_entered = h.clock()
            command = h.step(gray=(700, 700), digital=(1, 1))
            self.assertEqual(h.controller.state, RobotState.EDGE_RECOVER)
            self.assertEqual((command.left_speed, command.right_speed), (0, 0))
            return h.controller.state_entered

        first_entered = enter_double_edge_recovery()
        h.clock.value = (
            first_entered
            + timing.edge_stop_time
            + timing.edge_reverse_time
            + 1e-6
        )
        first_turn = h.step(gray=(700, 700), digital=(0, 0))
        self.assertGreater(first_turn.left_speed, 0)
        self.assertLess(first_turn.right_speed, 0)

        h.clock.value = (
            first_entered
            + timing.edge_stop_time
            + timing.edge_reverse_time
            + timing.edge_turn_time
            + 1e-6
        )
        h.step(gray=(700, 700), digital=(0, 0))
        self.assertEqual(h.controller.state, RobotState.ARENA_SEARCH)

        repeated_entered = enter_double_edge_recovery()
        h.clock.value = (
            repeated_entered
            + timing.edge_stop_time
            + timing.edge_reverse_time
            + 1e-6
        )
        repeated_turn = h.step(gray=(700, 700), digital=(0, 0))
        self.assertGreater(repeated_turn.left_speed, 0)
        self.assertLess(repeated_turn.right_speed, 0)

    def test_fence_escape_always_turns_right(self):
        h = ControllerHarness()
        timing = h.controller.config.timing

        def run_fence_escape():
            h.controller.state = RobotState.FENCE_ESCAPE
            h.controller.state_entered = h.clock()
            entered = h.controller.state_entered
            h.clock.value = (
                entered + timing.fence_escape_forward_time + 1e-6
            )
            turn_command = h.step()
            h.clock.value = (
                entered
                + timing.fence_escape_forward_time
                + timing.fence_escape_turn_time
                + 1e-6
            )
            completion = h.step()
            self.assertEqual(h.controller.state, RobotState.GROUND_SEARCH)
            self.assertEqual(
                (completion.left_speed, completion.right_speed),
                (0, 0),
            )
            self.assertEqual(completion.label, "fence-escape-complete-stop")
            return turn_command

        first_turn = run_fence_escape()
        repeated_turn = run_fence_escape()

        for command in (first_turn, repeated_turn):
            self.assertGreater(command.left_speed, 0)
            self.assertLess(command.right_speed, 0)
            self.assertEqual(command.label, "fence-escape-turn")
        self.assertFalse(hasattr(h.controller, "_alternate_turn_sign"))

    def test_single_edge_recovery_turn_directions_are_unchanged(self):
        def turn_command_for(digital):
            h = ControllerHarness()
            h.controller.state = RobotState.ARENA_SEARCH
            h.controller.state_entered = h.clock()
            h.step(gray=(700, 700), digital=digital)
            entered = h.controller.state_entered
            timing = h.controller.config.timing
            h.clock.value = (
                entered
                + timing.edge_stop_time
                + timing.edge_reverse_time
                + 1e-6
            )
            return h.step(gray=(700, 700), digital=(0, 0))

        left_edge_turn = turn_command_for((1, 0))
        self.assertGreater(left_edge_turn.left_speed, 0)
        self.assertLess(left_edge_turn.right_speed, 0)

        right_edge_turn = turn_command_for((0, 1))
        self.assertLess(right_edge_turn.left_speed, 0)
        self.assertGreater(right_edge_turn.right_speed, 0)

    def test_rear_high_object_preempts_climb_as_fence(self):
        h = ControllerHarness()
        h.controller.state = RobotState.CLIMB_BACKWARD
        h.controller.state_entered = h.clock()
        h.controller.match_started = True
        h.controller.match_start_time = h.clock()
        command = h.step(
            ir={6: 800},
            gray=(100, 100),
            rear_high=800,
            digital=(0, 0),
        )
        self.assertEqual(h.controller.state, RobotState.FENCE_ESCAPE)
        self.assertEqual(command.label, "climb-fence-stop")

    def test_harmful_color_is_avoided_after_multiple_votes(self):
        h = ControllerHarness()
        h.controller.state = RobotState.TARGET_CLASSIFY
        h.controller.state_entered = h.clock()
        h.controller.match_started = True
        h.controller.match_start_time = h.clock()
        h.step(ir={0: 800}, gray=(700, 700), vision=EnergyClass.HARMFUL)
        h.step(ir={0: 800}, gray=(700, 700), vision=EnergyClass.HARMFUL)
        self.assertEqual(h.controller.state, RobotState.AVOID_BLOCK)

    def test_harmful_votes_must_be_consecutive(self):
        h = ControllerHarness()
        h.controller.state = RobotState.TARGET_CLASSIFY
        h.controller.state_entered = h.clock()

        h.step(ir={0: 800}, gray=(700, 700), vision=EnergyClass.HARMFUL)
        h.step(ir={0: 800}, gray=(700, 700), vision=EnergyClass.UNKNOWN)
        h.step(ir={0: 800}, gray=(700, 700), vision=EnergyClass.HARMFUL)
        self.assertEqual(h.controller.state, RobotState.TARGET_CLASSIFY)

        h.step(ir={0: 800}, gray=(700, 700), vision=EnergyClass.HARMFUL)
        self.assertEqual(h.controller.state, RobotState.AVOID_BLOCK)

    def test_first_avoidance_turns_right_regardless_of_rejected_target_side(self):
        for sensor_index in (1, 11):
            with self.subTest(sensor_index=sensor_index):
                h = ControllerHarness()
                h.controller._transition(
                    RobotState.AVOID_BLOCK,
                    "test first avoidance",
                    h.clock(),
                )

                command = h.step(
                    ir={sensor_index: 800},
                    gray=(700, 700),
                )

                speed = h.controller.config.motion.avoid_turn_speed
                self.assertEqual(h.controller.state, RobotState.AVOID_BLOCK)
                self.assertEqual(
                    (command.left_speed, command.right_speed),
                    (speed, -speed),
                )
                self.assertEqual(command.label, "avoid-block-turn-right")

    def test_avoidance_stays_fixed_then_alternates_after_edge_preemption(self):
        h = ControllerHarness()
        speed = h.controller.config.motion.avoid_turn_speed
        h.controller._transition(
            RobotState.AVOID_BLOCK,
            "test first avoidance",
            h.clock(),
        )
        first_right = h.step(ir={1: 800}, gray=(700, 700))
        same_entry_right = h.step(ir={11: 800}, gray=(700, 700))

        edge_stop = h.step(gray=(700, 700), digital=(0, 1))
        self.assertEqual(h.controller.state, RobotState.EDGE_RECOVER)
        self.assertEqual(edge_stop.label, "edge-stop")

        h.controller._transition(
            RobotState.AVOID_BLOCK,
            "test avoidance after edge recovery",
            h.clock(),
        )
        next_left = h.step(ir={0: 800}, gray=(700, 700))
        h.controller._transition(
            RobotState.ARENA_SEARCH,
            "test second avoidance complete",
            h.clock(),
        )
        h.controller._transition(
            RobotState.AVOID_BLOCK,
            "test third avoidance",
            h.clock(),
        )
        third_right = h.step(ir={0: 800}, gray=(700, 700))

        self.assertEqual(
            [
                (command.left_speed, command.right_speed)
                for command in (
                    first_right,
                    same_entry_right,
                    next_left,
                    third_right,
                )
            ],
            [
                (speed, -speed),
                (speed, -speed),
                (-speed, speed),
                (speed, -speed),
            ],
        )
        self.assertEqual(
            [
                command.label
                for command in (
                    first_right,
                    same_entry_right,
                    next_left,
                    third_right,
                )
            ],
            [
                "avoid-block-turn-right",
                "avoid-block-turn-right",
                "avoid-block-turn-left",
                "avoid-block-turn-right",
            ],
        )

    def test_avoid_block_turns_then_drives_forward_before_search(self):
        h = ControllerHarness()
        h.controller._transition(
            RobotState.AVOID_BLOCK,
            "test avoidance phases",
            h.clock(),
        )
        entered = h.controller.state_entered
        timing = h.controller.config.timing
        motion = h.controller.config.motion

        command = h.step(ir={0: 800}, gray=(700, 700))
        self.assertEqual(
            (command.left_speed, command.right_speed),
            (motion.avoid_turn_speed, -motion.avoid_turn_speed),
        )
        self.assertEqual(command.label, "avoid-block-turn-right")

        h.clock.value = entered + timing.avoid_turn_time + 1e-6
        command = h.step(ir={6: 800}, gray=(700, 700))
        self.assertEqual(h.controller.state, RobotState.AVOID_BLOCK)
        self.assertEqual(
            (command.left_speed, command.right_speed),
            (motion.avoid_depart_speed, motion.avoid_depart_speed),
        )
        self.assertEqual(command.label, "avoid-block-depart-forward")

        h.clock.value = (
            entered
            + timing.avoid_turn_time
            + timing.avoid_depart_time
            + 1e-6
        )
        command = h.step(gray=(700, 700))
        self.assertEqual(h.controller.state, RobotState.ARENA_SEARCH)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))
        self.assertEqual(command.label, "avoid-departure-complete-stop")

        command = h.step(gray=(700, 700))
        self.assertGreater(command.left_speed, 0)
        self.assertEqual(command.left_speed, command.right_speed)

    def test_edge_preempts_avoid_departure(self):
        h = ControllerHarness()
        h.controller._transition(
            RobotState.AVOID_BLOCK,
            "test edge preemption",
            h.clock(),
        )
        h.clock.value += h.controller.config.timing.avoid_turn_time + 1e-6

        command = h.step(gray=(700, 700), digital=(1, 0))

        self.assertEqual(h.controller.state, RobotState.EDGE_RECOVER)
        self.assertEqual((command.left_speed, command.right_speed), (0, 0))
        self.assertEqual(command.label, "edge-stop")

    def test_gain_color_enters_push_after_multiple_votes(self):
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

    def test_classification_realigns_before_voting_when_target_drifts(self):
        for sensor_index in (1, 11):
            with self.subTest(sensor_index=sensor_index):
                h = ControllerHarness()
                h.controller.state = RobotState.TARGET_CLASSIFY
                h.controller.state_entered = h.clock()

                command = h.step(
                    ir={sensor_index: 800},
                    gray=(700, 700),
                    vision=EnergyClass.NO_BLOCK_MARKER,
                )

                self.assertEqual(h.controller.state, RobotState.TARGET_ALIGN)
                self.assertEqual(command.label, "classify-realign-stop")
                self.assertEqual(
                    h.controller._vision_votes[EnergyClass.NO_BLOCK_MARKER],
                    0,
                )

    def test_far_no_marker_target_is_not_assumed_to_be_enemy(self):
        h = ControllerHarness()
        h.controller.state = RobotState.TARGET_CLASSIFY
        h.controller.state_entered = h.clock()
        h.controller.match_started = True
        h.controller.match_start_time = h.clock()
        h.step(ir={0: 300}, gray=(700, 700), vision=EnergyClass.NO_BLOCK_MARKER)
        h.step(ir={0: 300}, gray=(700, 700), vision=EnergyClass.NO_BLOCK_MARKER)
        self.assertEqual(h.controller.state, RobotState.TARGET_CLASSIFY)

    def test_low_confidence_no_marker_result_is_not_assumed_to_be_enemy(self):
        h = ControllerHarness()
        h.controller.state = RobotState.TARGET_CLASSIFY
        h.controller.state_entered = h.clock()
        h.controller.match_started = True
        h.controller.match_start_time = h.clock()
        h.step(
            ir={0: 800},
            gray=(700, 700),
            vision=EnergyClass.NO_BLOCK_MARKER,
            vision_confidence=0.10,
        )
        h.step(
            ir={0: 800},
            gray=(700, 700),
            vision=EnergyClass.NO_BLOCK_MARKER,
            vision_confidence=0.10,
        )
        self.assertEqual(h.controller.state, RobotState.TARGET_CLASSIFY)


if __name__ == "__main__":
    unittest.main()
