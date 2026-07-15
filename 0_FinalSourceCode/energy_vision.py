"""Replaceable forward energy-block vision interface."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from robot_config import DEFAULT_CONFIG, VisionConfig


class EnergyClass(str, Enum):
    GAIN = "GAIN"
    HARMFUL = "HARMFUL"
    NO_BLOCK_MARKER = "NO_BLOCK_MARKER"
    UNKNOWN = "UNKNOWN"
    NONE = "NONE"


@dataclass(frozen=True)
class VisionResult:
    classification: EnergyClass
    confidence: float
    center_x: Optional[float]
    bbox_width: Optional[float]
    tag_id: Optional[int]
    timestamp: float
    frame_width: Optional[int] = None
    error: str = ""

    def is_fresh(self, now: float, stale_after: float) -> bool:
        return now - self.timestamp <= stale_after

    @classmethod
    def none(cls, timestamp: Optional[float] = None) -> "VisionResult":
        return cls(
            classification=EnergyClass.NONE,
            confidence=0.0,
            center_x=None,
            bbox_width=None,
            tag_id=None,
            timestamp=time.monotonic() if timestamp is None else timestamp,
        )


class AprilTagEnergyDetector:
    """Temporary AprilTag backend; final artwork can replace this class."""

    def __init__(
        self,
        config: VisionConfig = DEFAULT_CONFIG.vision,
        clock=time.monotonic,
    ) -> None:
        self.config = config
        self._clock = clock
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._latest = VisionResult.none(self._clock())
        self._healthy = False
        self._last_error = "not started"
        self._active_camera: Optional[int] = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        # Fail before starting the thread if dependencies are absent, so the
        # controller can deliberately select degraded mode.
        self._load_dependencies()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run, name="energy-vision", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout)

    def latest_result(self) -> VisionResult:
        with self._lock:
            return self._latest

    def status(self) -> dict:
        with self._lock:
            return {
                "healthy": self._healthy,
                "camera_index": self._active_camera,
                "last_error": self._last_error,
                "classification": self._latest.classification.value,
                "confidence": self._latest.confidence,
                "tag_id": self._latest.tag_id,
                "timestamp": self._latest.timestamp,
            }

    @staticmethod
    def _load_dependencies():
        try:
            import apriltag
            import cv2
        except ImportError as exc:
            raise RuntimeError(
                "OpenCV and apriltag are required for the temporary vision backend"
            ) from exc
        return cv2, apriltag

    def _run(self) -> None:
        cv2, apriltag = self._load_dependencies()
        options = apriltag.DetectorOptions(families=self.config.tag_family)
        detector = apriltag.Detector(options)

        while not self._stop_event.is_set():
            capture = None
            selected_index = None
            for camera_index in self.config.camera_indices:
                candidate = cv2.VideoCapture(camera_index)
                if candidate.isOpened():
                    capture = candidate
                    selected_index = camera_index
                    break
                candidate.release()

            if capture is None:
                self._set_error("no configured camera could be opened")
                self._stop_event.wait(self.config.reconnect_interval)
                continue

            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.frame_width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.frame_height)
            with self._lock:
                self._active_camera = selected_index
                self._healthy = True
                self._last_error = ""

            try:
                while not self._stop_event.is_set():
                    ok, frame = capture.read()
                    if not ok or frame is None:
                        raise RuntimeError("camera frame read failed")
                    result = self._detect_frame(frame, detector, cv2)
                    with self._lock:
                        self._latest = result
                        self._healthy = True
                        self._last_error = ""

                    if self.config.show_debug_window:
                        cv2.imshow("WheelFight energy vision", frame)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            self._stop_event.set()
                            break
            except Exception as exc:
                self._set_error(str(exc))
            finally:
                capture.release()
                with self._lock:
                    self._active_camera = None
                    self._healthy = False

            self._stop_event.wait(self.config.reconnect_interval)

        if self.config.show_debug_window:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

    def _detect_frame(self, frame, detector, cv2) -> VisionResult:
        now = self._clock()
        height, width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray)
        candidates = []
        for tag in tags:
            corners = tag.corners
            center_x = float(sum(float(corner[0]) for corner in corners) / 4.0)
            normalized_center = center_x / max(1.0, float(width))
            if not (
                self.config.center_region_min
                <= normalized_center
                <= self.config.center_region_max
            ):
                continue
            top_width = math.dist(corners[0], corners[1])
            bottom_width = math.dist(corners[3], corners[2])
            bbox_width = (top_width + bottom_width) / 2.0
            if bbox_width < self.config.min_tag_width_px:
                continue
            center_penalty = abs(normalized_center - 0.5)
            candidates.append((center_penalty, -bbox_width, tag, center_x, bbox_width))

        if not candidates:
            return VisionResult(
                classification=EnergyClass.NO_BLOCK_MARKER,
                confidence=1.0,
                center_x=None,
                bbox_width=None,
                tag_id=None,
                timestamp=now,
                frame_width=width,
            )

        _, _, tag, center_x, bbox_width = min(candidates, key=lambda item: item[:2])
        tag_id = int(tag.tag_id)
        if tag_id in self.config.harmful_tag_ids:
            classification = EnergyClass.HARMFUL
        elif not self.config.gain_tag_ids or tag_id in self.config.gain_tag_ids:
            classification = EnergyClass.GAIN
        else:
            classification = EnergyClass.UNKNOWN

        margin = float(getattr(tag, "decision_margin", 50.0))
        confidence = max(0.0, min(1.0, margin / 100.0))
        return VisionResult(
            classification=classification,
            confidence=confidence,
            center_x=center_x,
            bbox_width=bbox_width,
            tag_id=tag_id,
            timestamp=now,
            frame_width=width,
        )

    def _set_error(self, error: str) -> None:
        now = self._clock()
        with self._lock:
            self._healthy = False
            self._last_error = error
            self._latest = VisionResult(
                classification=EnergyClass.UNKNOWN,
                confidence=0.0,
                center_x=None,
                bbox_width=None,
                tag_id=None,
                timestamp=now,
                error=error,
            )


__all__ = ["AprilTagEnergyDetector", "EnergyClass", "VisionResult"]
