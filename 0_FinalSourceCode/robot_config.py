"""Central configuration for the WheelFight 2026 controller.

Every numeric value in this file is a starting value for low-speed bench tests.
It must be calibrated on the final robot before full-power operation.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class HardwareConfig:
    left_motor_id: int = 2
    right_motor_id: int = 1
    left_shovel_servo_id: int = 5
    right_shovel_servo_id: int = 6
    servo_mode: int = 0
    motor_limit: int = 1000
    servo_speed: int = 500

    # Calibrated shovel positions. Motion is enabled, so startup immediately
    # raises the shovel; keep the mechanism clear whenever the bus is opened.
    shovel_motion_enabled: bool = True
    shovel_raised_left: int = 12
    shovel_raised_right: int = 1012
    shovel_lowered_left: int = 512
    shovel_lowered_right: int = 512


@dataclass(frozen=True)
class SensorConfig:
    analog_filter_window: int = 3

    # Temporary 10-bit Mega ADC thresholds based on the first ranging test:
    # about 0 with no target, about 200 at 50 cm, and 500-600 near 10 cm.
    # The response falls again inside roughly 10 cm, so these values still
    # require final on-robot calibration with every sensor installed.
    ir_near_is_high: bool = True
    ir_detect_enter: int = 400
    ir_detect_exit: int = 350
    # A frame with neither complete energy-block marker is enemy evidence only
    # when the centered ranging target is at least this close. This is
    # provisional and must be measured.
    no_marker_enemy_ir_threshold: int = 350
    start_hand_enter: int = 400
    start_hand_exit: int = 350

    # Grayscale is confirmed by the team to be larger on the platform.
    gray_on_enter: int = 550
    gray_off_exit: int = 450

    edge_clear_frames: int = 3
    rear_high_confirm_frames: int = 3
    platform_confirm_frames: int = 3
    # Rear low-object sector used for platform alignment/verification.
    rear_platform_ir_indices: tuple[int, ...] = (5, 6, 7)
    alignment_tolerance_deg: float = 18.0
    front_target_tolerance_deg: float = 20.0
    # Outside the centering tolerance the classifier realigns; outside this
    # wider limit it abandons the candidate entirely.
    target_classify_loss_bearing_deg: float = 35.0
    # Forward sector in which attack steering is allowed to follow a cluster.
    attack_target_max_bearing_deg: float = 65.0
    # ADC counts per feature bin for the no-motion/stuck watchdog.
    stuck_analog_bin_size: int = 16


@dataclass(frozen=True)
class MotionConfig:
    search_turn_speed: int = 220
    align_turn_speed: int = 180
    fence_escape_forward_speed: int = 280
    fence_escape_turn_speed: int = 240
    platform_probe_speed: int = 180
    climb_prepare_speed: int = 300
    climb_speed: int = 800
    climb_clear_speed: int = 350
    arena_patrol_speed: int = 220
    attack_speed: int = 700
    attack_min_speed: int = 350
    push_gain_speed: int = 430
    avoid_turn_speed: int = 320
    avoid_depart_speed: int = 220
    edge_reverse_speed: int = 380
    edge_turn_speed: int = 320
    partial_recover_speed: int = 320
    target_turn_gain: float = 4.0


@dataclass(frozen=True)
class TimingConfig:
    control_period: float = 0.02
    sensor_warning_after: float = 0.06
    sensor_stop_after: float = 0.10
    # Mega receiver thread timing; separate from the controller's stale-data
    # warning and emergency-stop thresholds above.
    sensor_read_timeout: float = 0.10
    sensor_reconnect_interval: float = 1.0
    camera_stale_after: float = 0.25
    status_publish_interval: float = 0.10
    match_duration: float = 120.0

    start_clear_time: float = 0.50
    start_hand_confirm_time: float = 0.10
    start_gesture_timeout: float = 2.00
    start_release_confirm_time: float = 0.10
    start_release_delay: float = 0.40
    shovel_settle_time: float = 0.60

    ground_candidate_confirm: float = 0.10
    # Ignore a brief ranging dropout while turning a candidate toward A6.
    rear_candidate_lost_grace: float = 0.30
    align_timeout: float = 4.0
    platform_verify_time: float = 0.12
    platform_probe_timeout: float = 0.60
    climb_prepare_forward_time: float = 0.30
    climb_prepare_settle_time: float = 0.08
    fence_escape_forward_time: float = 0.40
    fence_escape_turn_time: float = 0.55
    climb_timeout: float = 2.20
    climb_clear_stable_time: float = 0.20
    climb_clear_timeout: float = 1.20

    target_center_confirm_time: float = 0.08
    target_align_timeout: float = 3.0
    target_classify_timeout: float = 0.80
    target_lost_grace: float = 0.25
    attack_timeout: float = 2.50
    push_timeout: float = 3.00
    # Provisional open-loop values: tune the turn for roughly 180 degrees,
    # then tune the forward phase far enough to leave the rejected target.
    avoid_turn_time: float = 1.00
    avoid_depart_time: float = 0.60

    edge_stop_time: float = 0.06
    edge_reverse_time: float = 0.28
    edge_turn_time: float = 0.50
    edge_recover_timeout: float = 1.40
    partial_recover_timeout: float = 1.20
    fault_recover_time: float = 0.50
    stuck_timeout: float = 2.00


@dataclass(frozen=True)
class VisionConfig:
    camera_indices: tuple[int, ...] = (0, 1)
    frame_width: int = 640
    frame_height: int = 480
    classify_votes: int = 3
    no_marker_votes_for_enemy: int = 5
    min_color_confidence: float = 0.35

    # The IR layer centers the target first, so color recognition only uses a
    # fixed central image region. All HSV values use OpenCV's H=0..179 scale.
    roi_x_min: float = 0.20
    roi_x_max: float = 0.80
    roi_y_min: float = 0.15
    roi_y_max: float = 0.85
    yellow_green_h_min: int = 25
    yellow_green_h_max: int = 50
    red_h_low_min: int = 0
    red_h_low_max: int = 12
    red_h_high_min: int = 168
    red_h_high_max: int = 179
    min_saturation: int = 70
    min_value: int = 60
    # Provisional: lower this only after checking false positives on the final
    # camera; raise it if small colored noise is classified as a block.
    min_color_area_ratio: float = 0.015

    # A harmful block must also form a complete four-arm red cross. The two
    # crossing lines are searched over one 90-degree period, so an X that
    # appears as a plus from an oblique view can still match. The candidate is
    # normalized first, making the score largely size-independent. These are
    # provisional calibration values.
    red_x_diagonal_band_ratio: float = 0.14
    red_x_center_size_ratio: float = 0.22
    # Smaller steps tolerate more image rotation but evaluate more angles;
    # the detector clamps this value to the safe 1..15 degree range.
    red_x_angle_step_deg: float = 5.0
    min_red_x_score: float = 0.20

    reconnect_interval: float = 1.0
    show_debug_window: bool = False


@dataclass(frozen=True)
class RobotConfig:
    hardware: HardwareConfig = field(default_factory=HardwareConfig)
    sensors: SensorConfig = field(default_factory=SensorConfig)
    motion: MotionConfig = field(default_factory=MotionConfig)
    timing: TimingConfig = field(default_factory=TimingConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)


DEFAULT_CONFIG = RobotConfig()
