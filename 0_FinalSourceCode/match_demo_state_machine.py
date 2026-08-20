"""WheelFight 2026 preemptive hierarchical match controller."""

from __future__ import annotations

import argparse
import json
import os
import time
from collections import Counter
from enum import Enum
from typing import Optional

from energy_vision import ColorEnergyDetector, EnergyClass, VisionResult
from mega_sensor_reader import MegaSensorReader
from motion_controller import DriveCommand, MotionController
from yolo_vision import YoloEnergyDetector
from perception import (
    PerceptionEngine,
    PerceptionSnapshot,
    PlatformState,
    bearing_error,
)
from robot_config import DEFAULT_CONFIG, RobotConfig


IR_SENSOR_COUNT = 12


class RobotState(str, Enum):
    BOOT_SELF_CHECK = "BOOT_SELF_CHECK"
    WAIT_START_GESTURE = "WAIT_START_GESTURE"
    DEPLOY_SHOVEL = "DEPLOY_SHOVEL"

    GROUND_SEARCH = "GROUND_SEARCH"
    ALIGN_REAR = "ALIGN_REAR"
    VERIFY_PLATFORM = "VERIFY_PLATFORM"
    CLIMB_PREPARE = "CLIMB_PREPARE"
    FENCE_ESCAPE = "FENCE_ESCAPE"
    CLIMB_BACKWARD = "CLIMB_BACKWARD"
    CLIMB_CLEAR_EDGE = "CLIMB_CLEAR_EDGE"

    ARENA_SEARCH = "ARENA_SEARCH"
    TARGET_ALIGN = "TARGET_ALIGN"
    TARGET_CLASSIFY = "TARGET_CLASSIFY"
    ATTACK_ENEMY = "ATTACK_ENEMY"
    PUSH_GAIN_BLOCK = "PUSH_GAIN_BLOCK"
    AVOID_BLOCK = "AVOID_BLOCK"
    EDGE_RECOVER = "EDGE_RECOVER"
    PARTIAL_FALL_RECOVER = "PARTIAL_FALL_RECOVER"

    FAULT_STOP = "FAULT_STOP"
    MATCH_END = "MATCH_END"


GROUND_STATES = {
    RobotState.GROUND_SEARCH,
    RobotState.ALIGN_REAR,
    RobotState.VERIFY_PLATFORM,
    RobotState.CLIMB_PREPARE,
    RobotState.FENCE_ESCAPE,
    RobotState.CLIMB_BACKWARD,
    RobotState.CLIMB_CLEAR_EDGE,
}

ARENA_STATES = {
    RobotState.ARENA_SEARCH,
    RobotState.TARGET_ALIGN,
    RobotState.TARGET_CLASSIFY,
    RobotState.ATTACK_ENEMY,
    RobotState.PUSH_GAIN_BLOCK,
    RobotState.AVOID_BLOCK,
}

PREMATCH_STATES = {
    RobotState.BOOT_SELF_CHECK,
    RobotState.WAIT_START_GESTURE,
    RobotState.DEPLOY_SHOVEL,
}


class MatchController:
    STATUS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runtime")
    STATUS_FILE = os.path.join(STATUS_DIR, "match_status.json")
    STATUS_TMP_FILE = STATUS_FILE + ".tmp"

    def __init__(
        self,
        sensor_reader=None,
        motion_controller=None,
        vision_detector=None,
        config: RobotConfig = DEFAULT_CONFIG,
        clock=time.monotonic,
        wall_clock=time.time,
        mega_port: Optional[str] = None,
    ) -> None:
        rear_indices = config.sensors.rear_platform_ir_indices
        if not rear_indices or any(
            isinstance(index, bool)
            or not isinstance(index, int)
            or not 0 <= index < IR_SENSOR_COUNT
            for index in rear_indices
        ):
            raise ValueError(
                "rear_platform_ir_indices must contain A0..A11 indices"
            )
        self.config = config
        self.clock = clock
        self.wall_clock = wall_clock
        self.sensor_reader = sensor_reader or MegaSensorReader(
            port=mega_port,
            stale_after=config.timing.sensor_stop_after,
            read_timeout=config.timing.sensor_read_timeout,
            reconnect_interval=config.timing.sensor_reconnect_interval,
        )
        self.motion_controller = motion_controller or MotionController(
            config=config.hardware
        )
        self.vision_detector = vision_detector or YoloEnergyDetector(
            model_path=os.path.join(os.path.dirname(__file__), "yolo", "out.rknn"),
            config=config.vision,
            clock=clock,
        )
        self.perception = PerceptionEngine(config.sensors)

        now = self.clock()
        self.state = RobotState.BOOT_SELF_CHECK
        self.state_reason = "controller created"
        self.state_entered = now
        self.match_started = False
        self.match_start_time: Optional[float] = None
        self.running = False
        self.vision_available = True
        self.last_command = DriveCommand()
        self.last_perception: Optional[PerceptionSnapshot] = None
        self.last_vision = VisionResult.none(now)

        self._condition_since: dict[str, float] = {}
        self._vision_votes: Counter[EnergyClass] = Counter()
        self._last_vision_vote_timestamp: Optional[float] = None
        self._target_last_seen = now
        self._edge_pattern = (False, False)
        self._edge_recovery_confirm_count = 0
        self._edge_action_started_at: Optional[float] = None
        # A positive sign is a right turn. Lock it per avoidance entry.
        self._avoid_turn_sign = 1
        self._next_avoid_turn_sign = 1
        self._climb_seen_rear_on = False
        self._fault_started = now
        self._last_feature_signature = None
        self._feature_changed_at = now
        self._last_status_publish = 0.0
        self._components_started = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start_components(self) -> None:
        if self._components_started:
            return
        self.sensor_reader.start()
        try:
            self.vision_detector.start()
            self.vision_available = True
        except RuntimeError as exc:
            self.vision_available = False
            print(f"vision degraded mode: {exc}")
        self.motion_controller.raise_shovel()
        self._components_started = True

    def start_match(self) -> None:
        self.start_components()
        self.running = True
        next_tick = self.clock()
        try:
            while self.running:
                now = self.clock()
                if now < next_tick:
                    time.sleep(min(next_tick - now, self.config.timing.control_period))
                    continue
                self.step_once(now)
                next_tick += self.config.timing.control_period
                if now - next_tick > self.config.timing.control_period:
                    next_tick = now + self.config.timing.control_period
        except KeyboardInterrupt:
            print("KeyboardInterrupt received.")
        finally:
            self.stop_match()

    def stop_match(self) -> None:
        self.running = False
        try:
            self.motion_controller.stop(force=True)
        except Exception as exc:
            print(f"motor stop failed: {exc}")
        try:
            self.sensor_reader.stop()
        except Exception:
            pass
        try:
            self.vision_detector.stop()
        except Exception:
            pass
        close_motion = getattr(self.motion_controller, "close", None)
        if callable(close_motion):
            try:
                close_motion()
            except Exception as exc:
                print(f"motion bus close failed: {exc}")
        self._publish_status(force=True)

    # ------------------------------------------------------------------
    # One control iteration
    # ------------------------------------------------------------------
    def step_once(self, now: Optional[float] = None) -> DriveCommand:
        if now is None:
            now = self.clock()
        frame = self.sensor_reader.latest_frame()
        if frame is None:
            command = self._handle_missing_sensor(now)
            return self._finish_step(command, now)

        perception = self.perception.update(frame, now)
        self.last_perception = perception
        self.last_vision = self.vision_detector.latest_result()

        preempt_command = self._evaluate_safety_preemption(perception, now)
        if preempt_command is not None:
            return self._finish_step(preempt_command, now)

        command = self._step_state(perception, self.last_vision, now)
        command = self._apply_stuck_watchdog(command, perception, now)
        return self._finish_step(command, now)

    def _finish_step(self, command: DriveCommand, now: float) -> DriveCommand:
        self.last_command = command
        self.motion_controller.apply(command)
        self._publish_status()
        return command

    # ------------------------------------------------------------------
    # Safety supervisor
    # ------------------------------------------------------------------
    def _handle_missing_sensor(self, now: float) -> DriveCommand:
        if self.state != RobotState.BOOT_SELF_CHECK:
            self._transition(RobotState.FAULT_STOP, "no Mega sensor frame", now)
        return DriveCommand(label="sensor-missing-stop")

    def _evaluate_safety_preemption(
        self, p: PerceptionSnapshot, now: float
    ) -> Optional[DriveCommand]:
        timing = self.config.timing
        if p.sensor_age > timing.sensor_stop_after:
            if self.state != RobotState.FAULT_STOP:
                self._transition(
                    RobotState.FAULT_STOP,
                    f"sensor stale {p.sensor_age:.3f}s",
                    now,
                )
            return DriveCommand(label="sensor-stale-stop")

        if (
            self.match_started
            and self.match_start_time is not None
            and now - self.match_start_time >= timing.match_duration
        ):
            if self.state != RobotState.MATCH_END:
                self._transition(
                    RobotState.MATCH_END,
                    f"{timing.match_duration:g} second match end",
                    now,
                )
            return DriveCommand(label="match-end-stop")

        if self.state in (RobotState.FAULT_STOP, RobotState.MATCH_END):
            return None
        if self.state in PREMATCH_STATES:
            return None

        if (
            self.state == RobotState.CLIMB_PREPARE
            and p.front_on_platform
            and p.rear_on_platform
        ):
            self._transition(
                RobotState.CLIMB_CLEAR_EDGE,
                "already on platform during climb preparation",
                now,
            )
            return DriveCommand(label="climb-prepare-already-on-stop")

        if self.state in (
            RobotState.CLIMB_PREPARE,
            RobotState.CLIMB_BACKWARD,
        ) and p.rear_high_object:
            self._transition(
                RobotState.FENCE_ESCAPE, "high rear object during climb", now
            )
            return DriveCommand(label="climb-fence-stop")

        if self.state in ARENA_STATES:
            if p.platform_state == PlatformState.OFF:
                self._transition(RobotState.GROUND_SEARCH, "fully off platform", now)
                return DriveCommand(label="fall-stop")
            if p.platform_state in (
                PlatformState.FRONT_TRANSITION,
                PlatformState.REAR_TRANSITION,
                PlatformState.UNKNOWN,
            ):
                self._transition(
                    RobotState.PARTIAL_FALL_RECOVER,
                    f"platform {p.platform_state.value}",
                    now,
                )
                return DriveCommand(label="partial-fall-stop")
            if p.front_left_edge or p.front_right_edge:
                self._edge_pattern = (
                    p.front_left_edge,
                    p.front_right_edge,
                )
                self._transition(
                    RobotState.EDGE_RECOVER,
                    f"front edge {int(p.front_left_edge)}/{int(p.front_right_edge)}",
                    now,
                )
                return DriveCommand(label="edge-stop")

        if self.state == RobotState.EDGE_RECOVER and p.platform_state in (
            PlatformState.FRONT_TRANSITION,
            PlatformState.REAR_TRANSITION,
        ):
            self._transition(
                RobotState.PARTIAL_FALL_RECOVER,
                "platform transition during edge recovery",
                now,
            )
            return DriveCommand(label="edge-partial-stop")
        return None

    # ------------------------------------------------------------------
    # State dispatch
    # ------------------------------------------------------------------
    def _step_state(
        self, p: PerceptionSnapshot, vision: VisionResult, now: float
    ) -> DriveCommand:
        handlers = {
            RobotState.BOOT_SELF_CHECK: self._step_boot,
            RobotState.WAIT_START_GESTURE: self._step_wait_start_gesture,
            RobotState.DEPLOY_SHOVEL: self._step_deploy_shovel,
            RobotState.GROUND_SEARCH: self._step_ground_search,
            RobotState.ALIGN_REAR: self._step_align_rear,
            RobotState.VERIFY_PLATFORM: self._step_verify_platform,
            RobotState.CLIMB_PREPARE: self._step_climb_prepare,
            RobotState.FENCE_ESCAPE: self._step_fence_escape,
            RobotState.CLIMB_BACKWARD: self._step_climb_backward,
            RobotState.CLIMB_CLEAR_EDGE: self._step_climb_clear_edge,
            RobotState.ARENA_SEARCH: self._step_arena_search,
            RobotState.TARGET_ALIGN: self._step_target_align,
            RobotState.TARGET_CLASSIFY: self._step_target_classify,
            RobotState.ATTACK_ENEMY: self._step_attack_enemy,
            RobotState.PUSH_GAIN_BLOCK: self._step_push_gain,
            RobotState.AVOID_BLOCK: self._step_avoid_block,
            RobotState.EDGE_RECOVER: self._step_edge_recover,
            RobotState.PARTIAL_FALL_RECOVER: self._step_partial_fall,
            RobotState.FAULT_STOP: self._step_fault_stop,
            RobotState.MATCH_END: self._step_match_end,
        }
        return handlers[self.state](p, vision, now)

    # ------------------------------------------------------------------
    # Prematch states
    # ------------------------------------------------------------------
    def _step_boot(self, p, vision, now) -> DriveCommand:
        if p.platform_state != PlatformState.UNKNOWN:
            self._transition(
                RobotState.WAIT_START_GESTURE, "stable Mega sensor data", now
            )
        return DriveCommand(label="boot-stop")

    def _step_wait_start_gesture(self, p, vision, now) -> DriveCommand:
        both_near = p.start_left_hand_near and p.start_right_hand_near
        if both_near:
            self.match_started = True
            self.match_start_time = now
            self.motion_controller.lower_shovel()
            self._transition(
                RobotState.DEPLOY_SHOVEL, "both start hands detected", now
            )
            return DriveCommand(label="start-triggered-stop")
        return DriveCommand(label="wait-start-gesture")

    def _step_deploy_shovel(self, p, vision, now) -> DriveCommand:
        if self._state_elapsed(now) >= self.config.timing.shovel_settle_time:
            if p.platform_state == PlatformState.ON:
                self._transition(RobotState.ARENA_SEARCH, "started on platform", now)
            else:
                self._transition(RobotState.GROUND_SEARCH, "begin platform search", now)
        return DriveCommand(label="shovel-settle")

    # ------------------------------------------------------------------
    # Ground and climbing states
    # ------------------------------------------------------------------
    def _step_ground_search(self, p, vision, now) -> DriveCommand:
        if p.platform_state == PlatformState.ON:
            self._transition(
                RobotState.CLIMB_CLEAR_EDGE, "platform detected while searching", now
            )
            return DriveCommand(label="ground-found-on-stop")
        if p.clusters:
            self._transition(RobotState.ALIGN_REAR, "ranging candidate found", now)
            return DriveCommand(label="candidate-stop")
        speed = self.config.motion.search_turn_speed
        return DriveCommand(speed, -speed, "ground-search-turn")

    def _step_align_rear(self, p, vision, now) -> DriveCommand:
        target = p.cluster_nearest(180.0)
        candidate_missing = target is None
        if self._held(
            "rear-candidate-missing",
            candidate_missing,
            self.config.timing.rear_candidate_lost_grace,
            now,
        ):
            self._transition(RobotState.GROUND_SEARCH, "candidate lost", now)
            return DriveCommand(label="rear-align-target-lost")
        if target is None:
            return DriveCommand(label="rear-align-target-lost")

        error = bearing_error(target.bearing_deg, 180.0)
        aligned = abs(error) <= self.config.sensors.alignment_tolerance_deg
        rear_active = any(
            p.infrared_active[index]
            for index in self.config.sensors.rear_platform_ir_indices
        )
        if self._held(
            "rear-aligned",
            aligned and rear_active,
            self.config.timing.ground_candidate_confirm,
            now,
        ):
            self._transition(
                RobotState.VERIFY_PLATFORM, "rear candidate aligned", now
            )
            return DriveCommand(label="rear-aligned-stop")
        if aligned and rear_active:
            return DriveCommand(label="rear-align-hold")
        if self._state_elapsed(now) > self.config.timing.align_timeout:
            self._transition(RobotState.GROUND_SEARCH, "rear alignment timeout", now)
            return DriveCommand(label="rear-align-timeout-stop")
        return self._turn_for_error(error, self.config.motion.align_turn_speed, "align-rear")

    def _step_verify_platform(self, p, vision, now) -> DriveCommand:
        if p.rear_high_object:
            self._transition(RobotState.FENCE_ESCAPE, "rear high object is fence", now)
            return DriveCommand(label="fence-confirmed-stop")

        rear_active = any(
            p.infrared_active[index]
            for index in self.config.sensors.rear_platform_ir_indices
        )
        if self._held(
            "low-platform",
            rear_active and not p.rear_high_object,
            self.config.timing.platform_verify_time,
            now,
        ):
            self._transition(
                RobotState.CLIMB_PREPARE, "low rear obstacle verified", now
            )
            return DriveCommand(label="platform-verified-stop")
        if self._state_elapsed(now) > self.config.timing.platform_probe_timeout:
            self._transition(RobotState.GROUND_SEARCH, "platform probe timeout", now)
            return DriveCommand(label="platform-probe-timeout")
        speed = self.config.motion.platform_probe_speed
        return DriveCommand(-speed, -speed, "platform-probe-reverse")

    def _step_climb_prepare(self, p, vision, now) -> DriveCommand:
        if p.front_on_platform and p.rear_on_platform:
            self._transition(
                RobotState.CLIMB_CLEAR_EDGE,
                "already on platform during climb preparation",
                now,
            )
            return DriveCommand(label="climb-prepare-already-on-stop")

        elapsed = self._state_elapsed(now)
        timing = self.config.timing
        if elapsed < timing.climb_prepare_forward_time:
            speed = self.config.motion.climb_prepare_speed
            return DriveCommand(speed, speed, "climb-prepare-forward")
        if elapsed < (
            timing.climb_prepare_forward_time
            + timing.climb_prepare_settle_time
        ):
            return DriveCommand(label="climb-prepare-settle")

        self._transition(
            RobotState.CLIMB_BACKWARD,
            "climb run-up distance prepared",
            now,
        )
        return DriveCommand(label="climb-prepare-complete-stop")

    def _step_fence_escape(self, p, vision, now) -> DriveCommand:
        elapsed = self._state_elapsed(now)
        motion = self.config.motion
        timing = self.config.timing
        if elapsed < timing.fence_escape_forward_time:
            speed = motion.fence_escape_forward_speed
            return DriveCommand(speed, speed, "fence-escape-forward")
        if elapsed < timing.fence_escape_forward_time + timing.fence_escape_turn_time:
            speed = motion.fence_escape_turn_speed
            return DriveCommand(speed, -speed, "fence-escape-turn")
        self._transition(RobotState.GROUND_SEARCH, "fence escape complete", now)
        return DriveCommand(label="fence-escape-complete-stop")

    def _step_climb_backward(self, p, vision, now) -> DriveCommand:
        if p.rear_on_platform:
            self._climb_seen_rear_on = True
        if p.platform_state == PlatformState.ON and self._climb_seen_rear_on:
            self._transition(RobotState.CLIMB_CLEAR_EDGE, "both grayscale on", now)
            return DriveCommand(label="climb-success-stop")
        if self._state_elapsed(now) > self.config.timing.climb_timeout:
            self._transition(RobotState.GROUND_SEARCH, "climb timeout withdraw", now)
            return DriveCommand(label="climb-timeout-stop")
        speed = self.config.motion.climb_speed
        return DriveCommand(-speed, -speed, "climb-backward")

    def _step_climb_clear_edge(self, p, vision, now) -> DriveCommand:
        if p.platform_state == PlatformState.OFF:
            self._transition(RobotState.GROUND_SEARCH, "climb clearance fell off", now)
            return DriveCommand(label="climb-clear-fall-stop")
        if p.platform_state == PlatformState.REAR_TRANSITION:
            speed = self.config.motion.partial_recover_speed
            return DriveCommand(speed, speed, "climb-clear-rear-recover")

        clear = (
            p.platform_state == PlatformState.ON
            and not p.front_left_edge
            and not p.front_right_edge
        )
        if self._held(
            "climb-clear",
            clear,
            self.config.timing.climb_clear_stable_time,
            now,
        ):
            self._transition(RobotState.ARENA_SEARCH, "clear of climb edge", now)
            return DriveCommand(label="climb-clear-complete-stop")
        if self._state_elapsed(now) > self.config.timing.climb_clear_timeout:
            if p.platform_state == PlatformState.ON:
                self._transition(
                    RobotState.ARENA_SEARCH, "climb clearance timeout on platform", now
                )
            else:
                self._transition(
                    RobotState.PARTIAL_FALL_RECOVER,
                    "climb clearance transition timeout",
                    now,
                )
            return DriveCommand(label="climb-clear-timeout-stop")
        speed = self.config.motion.climb_clear_speed
        return DriveCommand(-speed, -speed, "climb-clear-reverse")

    # ------------------------------------------------------------------
    # Arena target behavior
    # ------------------------------------------------------------------
    def _step_arena_search(self, p, vision, now) -> DriveCommand:
        if p.clusters:
            self._transition(RobotState.TARGET_ALIGN, "arena target candidate", now)
            return DriveCommand(label="arena-candidate-stop")
        speed = self.config.motion.arena_patrol_speed
        return DriveCommand(speed, speed, "arena-patrol-forward")

    def _step_target_align(self, p, vision, now) -> DriveCommand:
        target = p.cluster_nearest(0.0)
        if target is None:
            if self._state_elapsed(now) > self.config.timing.target_lost_grace:
                self._transition(RobotState.ARENA_SEARCH, "target lost while aligning", now)
            return DriveCommand(label="target-align-lost-stop")

        self._target_last_seen = now
        error = bearing_error(target.bearing_deg, 0.0)
        centered = abs(error) <= self.config.sensors.front_target_tolerance_deg
        if self._held(
            "target-centered",
            centered,
            self.config.timing.target_center_confirm_time,
            now,
        ):
            self._transition(RobotState.TARGET_CLASSIFY, "front target centered", now)
            return DriveCommand(label="target-centered-stop")
        if centered:
            return DriveCommand(label="target-align-hold")
        if self._state_elapsed(now) > self.config.timing.target_align_timeout:
            self._transition(RobotState.ARENA_SEARCH, "target alignment timeout", now)
            return DriveCommand(label="target-align-timeout-stop")
        return self._turn_for_error(error, self.config.motion.align_turn_speed, "target-align")

    def _step_target_classify(self, p, vision, now) -> DriveCommand:
        target = p.cluster_nearest(0.0)
        if (
            target is None
            or abs(target.bearing_deg)
            > self.config.sensors.target_classify_loss_bearing_deg
        ):
            self._transition(RobotState.ARENA_SEARCH, "classification target lost", now)
            return DriveCommand(label="classify-target-lost")
        if (
            abs(target.bearing_deg)
            > self.config.sensors.front_target_tolerance_deg
        ):
            self._transition(
                RobotState.TARGET_ALIGN,
                "target drifted outside vision center",
                now,
            )
            return DriveCommand(label="classify-realign-stop")

        if vision.is_fresh(now, self.config.timing.camera_stale_after):
            if vision.timestamp != self._last_vision_vote_timestamp:
                self._last_vision_vote_timestamp = vision.timestamp
                vote = vision.classification
                if (
                    vote in (EnergyClass.GAIN, EnergyClass.HARMFUL)
                    and vision.confidence <= self.config.vision.min_color_confidence
                ):
                    vote = EnergyClass.UNKNOWN
                if (
                    vote == EnergyClass.NO_BLOCK_MARKER
                    and (
                        vision.confidence
                        <= self.config.vision.min_color_confidence
                        or not self._good_no_marker_view(target)
                    )
                ):
                    vote = EnergyClass.UNKNOWN
                accepted_votes = (
                    EnergyClass.GAIN,
                    EnergyClass.HARMFUL,
                    EnergyClass.NO_BLOCK_MARKER,
                )
                if vote in accepted_votes:
                    consecutive_count = self._vision_votes[vote] + 1
                    self._vision_votes.clear()
                    self._vision_votes[vote] = consecutive_count
                else:
                    self._vision_votes.clear()

        required = self.config.vision.classify_votes
        if self._vision_votes[EnergyClass.HARMFUL] >= required:
            self._transition(RobotState.AVOID_BLOCK, "harmful red X confirmed", now)
            return DriveCommand(label="harmful-confirmed-stop")
        if self._vision_votes[EnergyClass.GAIN] >= required:
            self._transition(RobotState.PUSH_GAIN_BLOCK, "gain color confirmed", now)
            return DriveCommand(label="gain-confirmed-stop")
        if (
            self._vision_votes[EnergyClass.NO_BLOCK_MARKER]
            >= self.config.vision.no_marker_votes_for_enemy
        ):
            self._target_last_seen = now
            self._transition(
                RobotState.ATTACK_ENEMY,
                "good-view frames contain no complete energy-block marker",
                now,
            )
            return DriveCommand(label="enemy-confirmed-stop")
        if self._state_elapsed(now) > self.config.timing.target_classify_timeout:
            self._transition(RobotState.AVOID_BLOCK, "target classification uncertain", now)
            return DriveCommand(label="classification-timeout-stop")
        return DriveCommand(label="classifying-target-stop")

    def _step_attack_enemy(self, p, vision, now) -> DriveCommand:
        if self._fresh_class(vision, EnergyClass.HARMFUL, now):
            self._transition(RobotState.AVOID_BLOCK, "harmful red X during attack", now)
            return DriveCommand(label="attack-harmful-stop")
        if self._fresh_class(vision, EnergyClass.GAIN, now):
            self._transition(
                RobotState.TARGET_CLASSIFY, "energy-block color appeared during attack", now
            )
            return DriveCommand(label="attack-color-stop")

        target = p.cluster_nearest(0.0)
        if (
            target is not None
            and abs(target.bearing_deg)
            <= self.config.sensors.attack_target_max_bearing_deg
        ):
            self._target_last_seen = now
            error = target.bearing_deg
        elif now - self._target_last_seen > self.config.timing.target_lost_grace:
            self._transition(RobotState.ARENA_SEARCH, "enemy target lost", now)
            return DriveCommand(label="attack-target-lost-stop")
        else:
            error = 0.0

        if self._state_elapsed(now) > self.config.timing.attack_timeout:
            self._transition(RobotState.ARENA_SEARCH, "attack action timeout", now)
            return DriveCommand(label="attack-timeout-stop")
        return self._steered_forward(
            self.config.motion.attack_speed, error, "attack-enemy"
        )

    def _step_push_gain(self, p, vision, now) -> DriveCommand:
        if self._fresh_class(vision, EnergyClass.HARMFUL, now):
            self._transition(RobotState.AVOID_BLOCK, "harmful red X reclassification", now)
            return DriveCommand(label="push-harmful-stop")
        if not vision.is_fresh(now, self.config.timing.camera_stale_after):
            self._transition(RobotState.AVOID_BLOCK, "camera stale while pushing", now)
            return DriveCommand(label="push-camera-stale-stop")

        target = p.cluster_nearest(0.0)
        if target is None:
            if now - self._target_last_seen > self.config.timing.target_lost_grace:
                self._transition(RobotState.ARENA_SEARCH, "gain block departed", now)
                return DriveCommand(label="gain-target-lost-stop")
            error = 0.0
        else:
            self._target_last_seen = now
            error = target.bearing_deg

        if self._state_elapsed(now) > self.config.timing.push_timeout:
            self._transition(RobotState.ARENA_SEARCH, "gain push timeout", now)
            return DriveCommand(label="gain-push-timeout-stop")
        return self._steered_forward(
            self.config.motion.push_gain_speed, error, "push-gain-block"
        )

    def _step_avoid_block(self, p, vision, now) -> DriveCommand:
        elapsed = self._state_elapsed(now)
        timing = self.config.timing
        motion = self.config.motion
        if elapsed < timing.avoid_turn_time:
            speed = motion.avoid_turn_speed * self._avoid_turn_sign
            direction = "right" if self._avoid_turn_sign > 0 else "left"
            return DriveCommand(speed, -speed, f"avoid-block-turn-{direction}")
        if elapsed < timing.avoid_turn_time + timing.avoid_depart_time:
            speed = motion.avoid_depart_speed
            return DriveCommand(speed, speed, "avoid-block-depart-forward")
        self._transition(RobotState.ARENA_SEARCH, "block avoidance departure complete", now)
        return DriveCommand(label="avoid-departure-complete-stop")

    # ------------------------------------------------------------------
    # Edge, fall and fault states
    # ------------------------------------------------------------------
    def _step_edge_recover(self, p, vision, now) -> DriveCommand:
        timing = self.config.timing
        motion = self.config.motion

        if self._edge_action_started_at is None:
            stable_pattern = (p.front_left_edge, p.front_right_edge)
            if stable_pattern == self._edge_pattern:
                self._edge_recovery_confirm_count += 1
            else:
                self._edge_recovery_confirm_count = 0

            if (
                self._edge_recovery_confirm_count
                >= self.config.sensors.edge_recovery_confirm_frames
            ):
                self._edge_action_started_at = now

        if self._state_elapsed(now) > timing.edge_recover_timeout:
            if p.platform_state == PlatformState.ON:
                self._transition(
                    RobotState.ARENA_SEARCH, "edge recovery timeout but on platform", now
                )
            else:
                self._transition(
                    RobotState.PARTIAL_FALL_RECOVER,
                    "edge recovery timeout",
                    now,
                )
            return DriveCommand(label="edge-recovery-timeout-stop")

        if self._edge_action_started_at is None:
            return DriveCommand(label="edge-recovery-confirm-stop")

        elapsed = max(0.0, now - self._edge_action_started_at)
        if elapsed < timing.edge_stop_time:
            return DriveCommand(label="edge-immediate-stop")
        if elapsed < timing.edge_stop_time + timing.edge_reverse_time:
            speed = motion.edge_reverse_speed
            return DriveCommand(-speed, -speed, "edge-short-reverse")

        left, right = self._edge_pattern
        if left and not right:
            turn_sign = 1
        elif right and not left:
            turn_sign = -1
        else:
            # With both front edge sensors active, use a deterministic
            # clockwise/right turn. The angle remains a calibration parameter.
            turn_sign = 1
        speed = motion.edge_turn_speed * turn_sign
        turn_complete = elapsed >= (
            timing.edge_stop_time + timing.edge_reverse_time + timing.edge_turn_time
        )
        clear = (
            p.platform_state == PlatformState.ON
            and not p.front_left_edge
            and not p.front_right_edge
        )
        if turn_complete and clear:
            self._transition(RobotState.ARENA_SEARCH, "edge recovery complete", now)
            return DriveCommand(label="edge-recovery-complete-stop")
        return DriveCommand(speed, -speed, "edge-turn-away")

    def _step_partial_fall(self, p, vision, now) -> DriveCommand:
        if p.platform_state == PlatformState.ON:
            self._transition(RobotState.ARENA_SEARCH, "partial fall recovered", now)
            return DriveCommand(label="partial-recovered-stop")
        if p.platform_state == PlatformState.OFF:
            self._transition(RobotState.GROUND_SEARCH, "partial fall became full fall", now)
            return DriveCommand(label="partial-full-fall-stop")
        if self._state_elapsed(now) > self.config.timing.partial_recover_timeout:
            self._fault_started = now
            self._transition(
                RobotState.FAULT_STOP, "partial fall recovery timeout", now
            )
            return DriveCommand(label="partial-timeout-stop")

        speed = self.config.motion.partial_recover_speed
        if p.platform_state == PlatformState.FRONT_TRANSITION:
            return DriveCommand(-speed, -speed, "front-transition-reverse")
        if p.platform_state == PlatformState.REAR_TRANSITION:
            return DriveCommand(speed, speed, "rear-transition-forward")
        return DriveCommand(label="partial-unknown-stop")

    def _step_fault_stop(self, p, vision, now) -> DriveCommand:
        healthy = (
            p.sensor_age <= self.config.timing.sensor_warning_after
            and p.platform_state != PlatformState.UNKNOWN
        )
        if self._held(
            "fault-healthy",
            healthy,
            self.config.timing.fault_recover_time,
            now,
        ):
            if not self.match_started:
                next_state = RobotState.WAIT_START_GESTURE
            elif p.platform_state == PlatformState.ON:
                next_state = RobotState.ARENA_SEARCH
            elif p.platform_state == PlatformState.OFF:
                next_state = RobotState.GROUND_SEARCH
            else:
                next_state = RobotState.PARTIAL_FALL_RECOVER
            self._transition(next_state, "sensor fault recovered", now)
        return DriveCommand(label="fault-stop")

    def _step_match_end(self, p, vision, now) -> DriveCommand:
        return DriveCommand(label="match-end-stop")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _transition(self, state: RobotState, reason: str, now: float) -> None:
        entering_avoid = (
            state == RobotState.AVOID_BLOCK and self.state != RobotState.AVOID_BLOCK
        )
        if self.state != state:
            print(f"State:{self.state.value}->{state.value} {reason}")
        if entering_avoid:
            # Consume the direction on entry so an edge-preempted attempt still
            # causes the following avoidance attempt to choose the other side.
            self._avoid_turn_sign = self._next_avoid_turn_sign
            self._next_avoid_turn_sign *= -1
        self.state = state
        self.state_reason = reason
        self.state_entered = now
        self._condition_since.clear()
        if state == RobotState.TARGET_CLASSIFY:
            self._vision_votes.clear()
            self._last_vision_vote_timestamp = None
        if state == RobotState.ATTACK_ENEMY:
            self._target_last_seen = now
        if state == RobotState.PUSH_GAIN_BLOCK:
            self._target_last_seen = now
        if state == RobotState.CLIMB_BACKWARD:
            self._climb_seen_rear_on = False
        if state == RobotState.FAULT_STOP:
            self._fault_started = now
        if state == RobotState.EDGE_RECOVER:
            # The triggering stable edge sample is the first confirmation
            # frame; subsequent cycles provide the remaining confirmations.
            self._edge_recovery_confirm_count = 1
            self._edge_action_started_at = None
        self._publish_status(force=True)

    def _state_elapsed(self, now: float) -> float:
        return max(0.0, now - self.state_entered)

    def _held(self, key: str, condition: bool, duration: float, now: float) -> bool:
        if not condition:
            self._condition_since.pop(key, None)
            return False
        since = self._condition_since.setdefault(key, now)
        return now - since >= duration

    @staticmethod
    def _turn_for_error(error: float, speed: int, label: str) -> DriveCommand:
        signed_speed = speed if error >= 0 else -speed
        return DriveCommand(signed_speed, -signed_speed, label)

    def _steered_forward(
        self, base_speed: int, bearing: float, label: str
    ) -> DriveCommand:
        adjustment = int(round(self.config.motion.target_turn_gain * bearing))
        limit = max(0, base_speed - self.config.motion.attack_min_speed)
        adjustment = max(-limit, min(limit, adjustment))
        return DriveCommand(base_speed + adjustment, base_speed - adjustment, label)

    def _fresh_class(
        self, vision: VisionResult, classification: EnergyClass, now: float
    ) -> bool:
        return (
            vision.classification == classification
            and vision.confidence > self.config.vision.min_color_confidence
            and vision.is_fresh(now, self.config.timing.camera_stale_after)
        )

    def _good_no_marker_view(self, target) -> bool:
        threshold = self.config.sensors.no_marker_enemy_ir_threshold
        if self.config.sensors.ir_near_is_high:
            return target.representative_value >= threshold
        return target.representative_value <= threshold

    def _apply_stuck_watchdog(
        self, command: DriveCommand, p: PerceptionSnapshot, now: float
    ) -> DriveCommand:
        signature = p.feature_signature(
            self.config.sensors.stuck_analog_bin_size
        )
        moving = command.left_speed != 0 or command.right_speed != 0
        if signature != self._last_feature_signature:
            self._last_feature_signature = signature
            self._feature_changed_at = now
            return command
        if not moving:
            self._feature_changed_at = now
            return command
        watched_states = {
            RobotState.ALIGN_REAR,
            RobotState.VERIFY_PLATFORM,
            RobotState.CLIMB_PREPARE,
            RobotState.CLIMB_BACKWARD,
            RobotState.ATTACK_ENEMY,
            RobotState.PUSH_GAIN_BLOCK,
        }
        if (
            self.state in watched_states
            and now - self._feature_changed_at > self.config.timing.stuck_timeout
        ):
            if self.state in GROUND_STATES:
                self._transition(RobotState.FENCE_ESCAPE, "stuck watchdog", now)
            else:
                self._transition(RobotState.AVOID_BLOCK, "stuck watchdog", now)
            self._feature_changed_at = now
            return DriveCommand(label="stuck-watchdog-stop")
        return command

    # ------------------------------------------------------------------
    # Status publication
    # ------------------------------------------------------------------
    def _status_snapshot(self) -> dict:
        p = self.last_perception
        vision = self.last_vision
        sensor_link = {}
        reader_status = getattr(self.sensor_reader, "status", None)
        if callable(reader_status):
            try:
                sensor_link = reader_status()
            except Exception as exc:
                sensor_link = {"last_error": f"status unavailable: {exc}"}

        vision_backend = {}
        vision_status = getattr(self.vision_detector, "status", None)
        if callable(vision_status):
            try:
                vision_backend = vision_status()
            except Exception as exc:
                vision_backend = {"last_error": f"status unavailable: {exc}"}

        match_elapsed = None
        if self.match_started and self.match_start_time is not None:
            match_elapsed = self.clock() - self.match_start_time
        status = {
            "timestamp": self.wall_clock(),
            "state": self.state.value,
            "state_reason": self.state_reason,
            "match_running": self.running,
            "match_started": self.match_started,
            "match_elapsed": match_elapsed,
            "command": {
                "left": self.last_command.left_speed,
                "right": self.last_command.right_speed,
                "label": self.last_command.label,
            },
            "shovel_pose": self.motion_controller.shovel_pose,
            "sensor_link": sensor_link,
            "vision_available": self.vision_available,
            "vision_backend": vision_backend,
            "vision": {
                "classification": vision.classification.value,
                "confidence": vision.confidence,
                "gain_color_ratio": vision.gain_color_ratio,
                "harmful_color_ratio": vision.harmful_color_ratio,
                "red_x_score": vision.red_x_score,
                "red_x_detected": vision.red_x_detected,
                "red_x_angle_deg": vision.red_x_angle_deg,
                "age": max(0.0, self.clock() - vision.timestamp),
                "error": vision.error,
            },
        }
        if p is not None:
            status["sensor"] = {
                "sequence": p.sequence,
                "age": p.sensor_age,
                "raw_analog": list(p.raw_analog),
                "raw_digital": list(p.raw_digital),
                "filtered_analog": list(p.filtered_analog),
                "infrared_active": list(p.infrared_active),
                "disabled_ir_indices": list(p.disabled_ir_indices),
                "platform_state": p.platform_state.value,
                "front_on_platform": p.front_on_platform,
                "rear_on_platform": p.rear_on_platform,
                "front_left_edge": p.front_left_edge,
                "front_right_edge": p.front_right_edge,
                "rear_high_object": p.rear_high_object,
                "start_left_hand_near": p.start_left_hand_near,
                "start_right_hand_near": p.start_right_hand_near,
                "clusters": [
                    {
                        "indices": list(cluster.indices),
                        "bearing": cluster.bearing_deg,
                        "strength": cluster.strength,
                        "value": cluster.representative_value,
                    }
                    for cluster in p.clusters
                ],
            }
        return status

    def _publish_status(self, force: bool = False) -> None:
        now = self.clock()
        if (
            not force
            and now - self._last_status_publish
            < self.config.timing.status_publish_interval
        ):
            return
        self._last_status_publish = now
        try:
            os.makedirs(self.STATUS_DIR, exist_ok=True)
            with open(self.STATUS_TMP_FILE, "w", encoding="utf-8") as fp:
                json.dump(self._status_snapshot(), fp, ensure_ascii=False, indent=2)
            os.replace(self.STATUS_TMP_FILE, self.STATUS_FILE)
        except Exception as exc:
            print(f"status publish failed: {exc}")


# Keep the historical class name import-compatible without creating hardware at
# module import time.
Match_demo = MatchController


def build_default_controller(mega_port: Optional[str] = None) -> MatchController:
    return MatchController(mega_port=mega_port)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="WheelFight 2026 match controller")
    parser.add_argument(
        "--mega-port",
        default=None,
        help="Mega serial device, for example /dev/serial/by-id/...; auto-detect if omitted",
    )
    return parser.parse_args()


if __name__ == "__main__":
    arguments = parse_args()
    build_default_controller(arguments.mega_port).start_match()
