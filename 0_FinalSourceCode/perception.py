"""Sensor filtering and semantic perception for the WheelFight controller."""

from __future__ import annotations

import math
import statistics
import time
from collections import deque
from dataclasses import dataclass, replace
from enum import Enum
from typing import Optional

from mega_sensor_reader import SensorFrame
from robot_config import DEFAULT_CONFIG, SensorConfig


REAR_CENTER_IR_INDEX = 6


class PlatformState(str, Enum):
    UNKNOWN = "UNKNOWN"
    ON = "ON"
    OFF = "OFF"
    FRONT_TRANSITION = "FRONT_TRANSITION"
    REAR_TRANSITION = "REAR_TRANSITION"


def normalize_bearing(degrees: float) -> float:
    """Return a bearing in [-180, 180), positive clockwise/right."""
    return (degrees + 180.0) % 360.0 - 180.0


def bearing_error(current: float, desired: float) -> float:
    return normalize_bearing(current - desired)


@dataclass(frozen=True)
class IRCluster:
    indices: tuple[int, ...]
    bearing_deg: float
    strength: float
    representative_value: int


@dataclass(frozen=True)
class PerceptionSnapshot:
    sequence: int
    received_monotonic: float
    sensor_age: float
    raw_analog: tuple[int, ...]
    raw_digital: tuple[int, ...]
    filtered_analog: tuple[int, ...]
    infrared: tuple[int, ...]
    infrared_active: tuple[bool, ...]
    clusters: tuple[IRCluster, ...]
    gray_front: int
    gray_rear: int
    front_on_platform: bool
    rear_on_platform: bool
    platform_state: PlatformState
    front_left_edge: bool
    front_right_edge: bool
    rear_high_object: bool
    start_left_hand_near: bool
    start_right_hand_near: bool

    def cluster_nearest(self, desired_bearing: float) -> Optional[IRCluster]:
        if not self.clusters:
            return None
        return min(
            self.clusters,
            key=lambda cluster: (
                abs(bearing_error(cluster.bearing_deg, desired_bearing)),
                -cluster.strength,
            ),
        )

    def strongest_cluster(self) -> Optional[IRCluster]:
        if not self.clusters:
            return None
        return max(self.clusters, key=lambda cluster: cluster.strength)

    def feature_signature(
        self,
        analog_bin_size: int = DEFAULT_CONFIG.sensors.stuck_analog_bin_size,
    ) -> tuple:
        bin_size = max(1, int(analog_bin_size))
        return (
            tuple(
                round(value / bin_size) for value in self.filtered_analog
            ),
            self.raw_digital,
            self.platform_state,
        )


class PerceptionEngine:
    def __init__(self, config: SensorConfig = DEFAULT_CONFIG.sensors) -> None:
        if config.gray_on_is_high:
            valid_gray_hysteresis = config.gray_on_enter > config.gray_off_exit
        else:
            valid_gray_hysteresis = config.gray_on_enter < config.gray_off_exit
        if not valid_gray_hysteresis:
            polarity = "high" if config.gray_on_is_high else "low"
            raise ValueError(
                "invalid grayscale hysteresis thresholds for "
                f"{polarity}-on-platform polarity"
            )

        self.config = config
        window = max(1, config.analog_filter_window)
        self._analog_windows = [deque(maxlen=window) for _ in range(14)]
        self._ir_active = [False] * 12
        self._hand_near = [False, False]  # left A9, right A3
        self._gray_on = [False, False]

        self._edge_state = [False, False]
        self._edge_clear_counts = [0, 0]
        self._rear_high_state = False
        self._rear_high_detect_count = 0
        self._rear_high_clear_count = 0

        self._platform_state = PlatformState.UNKNOWN
        self._pending_platform_state = PlatformState.UNKNOWN
        self._pending_platform_count = 0
        self._last_sequence: Optional[int] = None
        self._last_snapshot: Optional[PerceptionSnapshot] = None

    def reset(self) -> None:
        self.__init__(self.config)

    def update(
        self, frame: SensorFrame, now: Optional[float] = None
    ) -> PerceptionSnapshot:
        if now is None:
            now = time.monotonic()
        if self._last_sequence == frame.sequence and self._last_snapshot is not None:
            self._last_snapshot = replace(
                self._last_snapshot,
                sensor_age=max(0.0, now - frame.received_monotonic),
            )
            return self._last_snapshot

        self._last_sequence = frame.sequence
        for index, value in enumerate(frame.analog):
            self._analog_windows[index].append(value)
        filtered = tuple(
            int(round(statistics.median(values))) for values in self._analog_windows
        )

        for index in range(12):
            enter, exit_value = self._ir_thresholds(index)
            self._ir_active[index] = self._update_hysteresis(
                self._ir_active[index],
                filtered[index],
                enter,
                exit_value,
                self.config.ir_near_is_high,
            )

        self._hand_near[0] = self._update_hysteresis(
            self._hand_near[0],
            filtered[9],
            self.config.start_hand_enter,
            self.config.start_hand_exit,
            self.config.ir_near_is_high,
        )
        self._hand_near[1] = self._update_hysteresis(
            self._hand_near[1],
            filtered[3],
            self.config.start_hand_enter,
            self.config.start_hand_exit,
            self.config.ir_near_is_high,
        )

        self._gray_on[0] = self._update_hysteresis(
            self._gray_on[0],
            filtered[12],
            self.config.gray_on_enter,
            self.config.gray_off_exit,
            self.config.gray_on_is_high,
        )
        self._gray_on[1] = self._update_hysteresis(
            self._gray_on[1],
            filtered[13],
            self.config.gray_on_enter,
            self.config.gray_off_exit,
            self.config.gray_on_is_high,
        )

        self._update_edge(0, frame.digital[0] == 1)
        self._update_edge(1, frame.digital[1] == 1)
        self._update_rear_high(frame.digital[2] == 0)
        self._update_platform_state()

        infrared = filtered[:12]
        clusters = self._build_clusters(infrared, tuple(self._ir_active))
        snapshot = PerceptionSnapshot(
            sequence=frame.sequence,
            received_monotonic=frame.received_monotonic,
            sensor_age=max(0.0, now - frame.received_monotonic),
            raw_analog=frame.analog,
            raw_digital=frame.digital,
            filtered_analog=filtered,
            infrared=infrared,
            infrared_active=tuple(self._ir_active),
            clusters=clusters,
            gray_front=filtered[12],
            gray_rear=filtered[13],
            front_on_platform=self._gray_on[0],
            rear_on_platform=self._gray_on[1],
            platform_state=self._platform_state,
            front_left_edge=self._edge_state[0],
            front_right_edge=self._edge_state[1],
            rear_high_object=self._rear_high_state,
            start_left_hand_near=self._hand_near[0],
            start_right_hand_near=self._hand_near[1],
        )
        self._last_snapshot = snapshot
        return snapshot

    @staticmethod
    def _update_hysteresis(
        current: bool,
        value: int,
        enter: int,
        exit_value: int,
        high_is_true: bool,
    ) -> bool:
        threshold = exit_value if current else enter
        return value >= threshold if high_is_true else value <= threshold

    def _ir_thresholds(self, index: int) -> tuple[int, int]:
        if index == REAR_CENTER_IR_INDEX:
            return (
                self.config.ir_a6_detect_enter,
                self.config.ir_a6_detect_exit,
            )
        return self.config.ir_detect_enter, self.config.ir_detect_exit

    def _update_edge(self, index: int, raw_edge: bool) -> None:
        if raw_edge:
            self._edge_state[index] = True
            self._edge_clear_counts[index] = 0
            return
        if not self._edge_state[index]:
            return
        self._edge_clear_counts[index] += 1
        if self._edge_clear_counts[index] >= self.config.edge_clear_frames:
            self._edge_state[index] = False
            self._edge_clear_counts[index] = 0

    def _update_rear_high(self, detected: bool) -> None:
        required = max(1, self.config.rear_high_confirm_frames)
        if detected:
            self._rear_high_detect_count += 1
            self._rear_high_clear_count = 0
            if self._rear_high_detect_count >= required:
                self._rear_high_state = True
        else:
            self._rear_high_clear_count += 1
            self._rear_high_detect_count = 0
            if self._rear_high_clear_count >= required:
                self._rear_high_state = False

    def _update_platform_state(self) -> None:
        front, rear = self._gray_on
        if front and rear:
            candidate = PlatformState.ON
        elif not front and not rear:
            candidate = PlatformState.OFF
        elif not front and rear:
            candidate = PlatformState.FRONT_TRANSITION
        else:
            candidate = PlatformState.REAR_TRANSITION

        if candidate == self._pending_platform_state:
            self._pending_platform_count += 1
        else:
            self._pending_platform_state = candidate
            self._pending_platform_count = 1

        if self._pending_platform_count >= max(
            1, self.config.platform_confirm_frames
        ):
            self._platform_state = candidate

    def _build_clusters(
        self, values: tuple[int, ...], active: tuple[bool, ...]
    ) -> tuple[IRCluster, ...]:
        if not any(active):
            return ()
        if all(active):
            groups = [tuple(range(12))]
        else:
            start = next(index for index, value in enumerate(active) if not value)
            groups = []
            group = []
            for step in range(1, 13):
                index = (start + step) % 12
                if active[index]:
                    group.append(index)
                elif group:
                    groups.append(tuple(group))
                    group = []
            if group:
                groups.append(tuple(group))

        clusters = []
        for indices in groups:
            weights = [
                self._ir_strength(index, values[index]) for index in indices
            ]
            x = 0.0
            y = 0.0
            for index, weight in zip(indices, weights):
                radians = math.radians(index * 30.0)
                x += math.cos(radians) * weight
                y += math.sin(radians) * weight
            bearing = math.degrees(math.atan2(y, x)) % 360.0
            if bearing >= 180.0:
                bearing -= 360.0
            representative = (
                max(values[index] for index in indices)
                if self.config.ir_near_is_high
                else min(values[index] for index in indices)
            )
            clusters.append(
                IRCluster(
                    indices=indices,
                    bearing_deg=bearing,
                    strength=sum(weights),
                    representative_value=representative,
                )
            )
        return tuple(clusters)

    def _ir_strength(self, index: int, value: int) -> float:
        _, exit_value = self._ir_thresholds(index)
        if self.config.ir_near_is_high:
            return float(max(1, value - exit_value))
        return float(max(1, exit_value - value))


__all__ = [
    "IRCluster",
    "PerceptionEngine",
    "PerceptionSnapshot",
    "PlatformState",
    "bearing_error",
    "normalize_bearing",
]
