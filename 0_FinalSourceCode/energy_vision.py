"""Forward energy-block recognition using HSV color and a red-X score."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from enum import Enum
from functools import lru_cache
from typing import Any, Optional

from robot_config import DEFAULT_CONFIG, VisionConfig


RED_X_GRID_SIZE = 48
CAMERA_PROBE_READS = 5


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
    # Retained as compatibility placeholders for existing callers. The fixed
    # ROI color backend does not estimate target geometry or an identifier.
    center_x: Optional[float]
    bbox_width: Optional[float]
    tag_id: Optional[int]
    timestamp: float
    frame_width: Optional[int] = None
    error: str = ""
    gain_color_ratio: float = 0.0
    harmful_color_ratio: float = 0.0
    red_x_score: float = 0.0
    red_x_detected: bool = False
    red_x_angle_deg: Optional[float] = None

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


@dataclass(frozen=True)
class RedXEvidence:
    """Normalized red-X geometry evidence for one candidate region."""

    score: float = 0.0
    detected: bool = False
    diag_down_fill: float = 0.0
    diag_up_fill: float = 0.0
    center_fill: float = 0.0
    off_diag_fill: float = 0.0
    arm_fills: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    angle_deg: Optional[float] = None
    candidate_box: Optional[tuple[int, int, int, int]] = None


@dataclass(frozen=True)
class ColorFrameAnalysis:
    """One frame's result plus the exact ROI masks used to produce it."""

    result: VisionResult
    roi_bounds: tuple[int, int, int, int]
    gain_mask: Optional[Any]
    harmful_mask: Optional[Any]
    red_x_evidence: RedXEvidence


def open_first_usable_camera(
    cv2,
    config: VisionConfig,
    probe_reads: int = CAMERA_PROBE_READS,
    stop_event: Optional[threading.Event] = None,
):
    """Open the first configured camera that can also provide a real frame."""

    reads_per_camera = max(1, int(probe_reads))
    for camera_index in config.camera_indices:
        if stop_event is not None and stop_event.is_set():
            break
        capture = None
        selected = False
        try:
            capture = cv2.VideoCapture(camera_index)
            if not capture.isOpened():
                continue
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.frame_width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.frame_height)
            for _ in range(reads_per_camera):
                if stop_event is not None and stop_event.is_set():
                    break
                ok, frame = capture.read()
                if ok and frame is not None:
                    selected = True
                    return capture, camera_index, frame
        except Exception:
            pass
        finally:
            if capture is not None and not selected:
                try:
                    capture.release()
                except Exception:
                    pass

    if stop_event is not None and stop_event.is_set():
        raise RuntimeError("Camera probing stopped")
    indices = ", ".join(str(index) for index in config.camera_indices)
    raise RuntimeError(
        f"No configured camera could provide a frame: {indices}"
    )


class ColorEnergyDetector:
    """Classify yellow-green gain markers and red-X harmful markers."""

    def __init__(
        self,
        config: VisionConfig = DEFAULT_CONFIG.vision,
        clock=time.monotonic,
    ) -> None:
        if not (
            0.0
            <= config.max_red_x_score_for_no_marker
            < config.min_red_x_score
        ):
            raise ValueError(
                "red-X thresholds must satisfy "
                "0 <= max_red_x_score_for_no_marker < min_red_x_score"
            )
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
        # Build every rotation-region map before the match loop starts. The
        # first real red candidate must not pay this pure-Python cache cost.
        self._prewarm_red_x_grid_regions()
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
                "gain_color_ratio": self._latest.gain_color_ratio,
                "harmful_color_ratio": self._latest.harmful_color_ratio,
                "red_x_score": self._latest.red_x_score,
                "red_x_detected": self._latest.red_x_detected,
                "red_x_angle_deg": self._latest.red_x_angle_deg,
                "timestamp": self._latest.timestamp,
            }

    @staticmethod
    def _load_dependencies():
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("OpenCV is required for color vision") from exc
        return cv2

    def _run(self) -> None:
        cv2 = self._load_dependencies()

        while not self._stop_event.is_set():
            try:
                capture, selected_index, first_frame = open_first_usable_camera(
                    cv2,
                    self.config,
                    stop_event=self._stop_event,
                )
            except RuntimeError as exc:
                if self._stop_event.is_set():
                    break
                self._set_error(str(exc))
                self._stop_event.wait(self.config.reconnect_interval)
                continue

            with self._lock:
                self._active_camera = selected_index
                self._healthy = False
                self._last_error = ""

            try:
                while not self._stop_event.is_set():
                    if first_frame is not None:
                        frame = first_frame
                        first_frame = None
                        ok = True
                    else:
                        ok, frame = capture.read()
                    if not ok or frame is None:
                        raise RuntimeError("camera frame read failed")
                    result = self._detect_frame(frame, cv2)
                    with self._lock:
                        self._latest = result
                        self._healthy = True
                        self._last_error = ""

                    if self.config.show_debug_window:
                        cv2.imshow("WheelFight energy color vision", frame)
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

    def analyze_frame(self, frame, cv2) -> ColorFrameAnalysis:
        """Analyze one BGR frame without opening a camera or starting a thread."""

        now = self._clock()
        height, width = frame.shape[:2]
        roi_bounds = self._roi_bounds(width, height)
        x0, x1, y0, y1 = roi_bounds
        if x1 <= x0 or y1 <= y0:
            return ColorFrameAnalysis(
                result=VisionResult(
                    classification=EnergyClass.UNKNOWN,
                    confidence=0.0,
                    center_x=None,
                    bbox_width=None,
                    tag_id=None,
                    timestamp=now,
                    frame_width=width,
                    error="invalid color vision ROI",
                ),
                roi_bounds=roi_bounds,
                gain_mask=None,
                harmful_mask=None,
                red_x_evidence=RedXEvidence(),
            )

        roi = frame[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        config = self.config
        lower_common = (config.min_saturation, config.min_value)

        gain_mask = cv2.inRange(
            hsv,
            (config.yellow_green_h_min, *lower_common),
            (config.yellow_green_h_max, 255, 255),
        )
        red_low_mask = cv2.inRange(
            hsv,
            (config.red_h_low_min, *lower_common),
            (config.red_h_low_max, 255, 255),
        )
        red_high_mask = cv2.inRange(
            hsv,
            (config.red_h_high_min, *lower_common),
            (config.red_h_high_max, 255, 255),
        )
        harmful_mask = cv2.bitwise_or(red_low_mask, red_high_mask)

        roi_pixels = max(1, (x1 - x0) * (y1 - y0))
        gain_ratio = cv2.countNonZero(gain_mask) / float(roi_pixels)
        harmful_ratio = cv2.countNonZero(harmful_mask) / float(roi_pixels)
        red_x_evidence = self._analyze_red_x(harmful_mask, cv2)
        classification, confidence = self._classify_evidence(
            gain_ratio,
            harmful_ratio,
            red_x_evidence.score,
            config.min_color_area_ratio,
            config.min_red_x_score,
            config.max_red_x_score_for_no_marker,
        )

        return ColorFrameAnalysis(
            result=VisionResult(
                classification=classification,
                confidence=confidence,
                center_x=None,
                bbox_width=None,
                tag_id=None,
                timestamp=now,
                frame_width=width,
                gain_color_ratio=gain_ratio,
                harmful_color_ratio=harmful_ratio,
                red_x_score=red_x_evidence.score,
                red_x_detected=red_x_evidence.detected,
                red_x_angle_deg=red_x_evidence.angle_deg,
            ),
            roi_bounds=roi_bounds,
            gain_mask=gain_mask,
            harmful_mask=harmful_mask,
            red_x_evidence=red_x_evidence,
        )

    def _detect_frame(self, frame, cv2) -> VisionResult:
        return self.analyze_frame(frame, cv2).result

    def _roi_bounds(self, width: int, height: int) -> tuple[int, int, int, int]:
        x0 = max(0, min(width, round(width * self.config.roi_x_min)))
        x1 = max(0, min(width, round(width * self.config.roi_x_max)))
        y0 = max(0, min(height, round(height * self.config.roi_y_min)))
        y1 = max(0, min(height, round(height * self.config.roi_y_max)))
        return x0, x1, y0, y1

    def _analyze_red_x(self, harmful_mask, cv2) -> RedXEvidence:
        contour_result = cv2.findContours(
            harmful_mask.copy(),
            cv2.RETR_CCOMP,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        contours = contour_result[-2]
        hierarchy = contour_result[-1]
        if not contours:
            return RedXEvidence()

        mask_height, mask_width = harmful_mask.shape[:2]
        minimum_pixels = (
            mask_width * mask_height * self.config.min_color_area_ratio
        )
        best_evidence = RedXEvidence()
        best_key = (-1.0, -1)

        for contour_index, contour in enumerate(contours):
            # RETR_CCOMP keeps foreground components at the top level and
            # reports holes (for example the white strokes in a red arena
            # marking) as children. Only real red foreground is a candidate.
            if (
                hierarchy is not None
                and int(hierarchy[0][contour_index][3]) >= 0
            ):
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if width <= 0 or height <= 0:
                continue

            # Isolate this contour so a surrounding red frame or a nearby red
            # marking cannot leak into the candidate's bounding rectangle.
            candidate_mask = harmful_mask.copy()
            candidate_mask.fill(0)
            cv2.drawContours(candidate_mask, [contour], -1, 255, -1)
            candidate = candidate_mask[y : y + height, x : x + width]
            candidate_pixels = cv2.countNonZero(candidate)
            if candidate_pixels < minimum_pixels:
                continue

            normalized = cv2.resize(
                candidate,
                (RED_X_GRID_SIZE, RED_X_GRID_SIZE),
                interpolation=cv2.INTER_NEAREST,
            )
            score, angle_deg, fills = self._best_red_x_grid_match(
                normalized.tobytes(),
                RED_X_GRID_SIZE,
                self.config.red_x_diagonal_band_ratio,
                self.config.red_x_center_size_ratio,
                self.config.red_x_angle_step_deg,
            )
            diag_down_fill, diag_up_fill, center_fill, off_diag_fill = fills[:4]
            arm_fills = fills[4:8]
            evidence = RedXEvidence(
                score=score,
                detected=score >= self.config.min_red_x_score,
                diag_down_fill=diag_down_fill,
                diag_up_fill=diag_up_fill,
                center_fill=center_fill,
                off_diag_fill=off_diag_fill,
                arm_fills=arm_fills,
                angle_deg=angle_deg,
                candidate_box=(x, y, width, height),
            )
            candidate_key = (score, candidate_pixels)
            if candidate_key > best_key:
                best_evidence = evidence
                best_key = candidate_key

        return best_evidence

    @staticmethod
    def _red_x_grid_fills(
        grid_bytes: bytes,
        grid_size: int,
        diagonal_band_ratio: float,
        center_size_ratio: float,
        angle_deg: float = 45.0,
    ) -> tuple[float, ...]:
        if grid_size <= 0 or len(grid_bytes) < grid_size * grid_size:
            return (0.0,) * 8

        regions = ColorEnergyDetector._red_x_grid_regions(
            grid_size,
            diagonal_band_ratio,
            center_size_ratio,
            angle_deg % 90.0,
        )
        return tuple(
            (
                sum(1 for pixel_index in region if grid_bytes[pixel_index])
                / len(region)
                if region
                else 0.0
            )
            for region in regions
        )

    @staticmethod
    @lru_cache(maxsize=256)
    def _red_x_grid_regions(
        grid_size: int,
        diagonal_band_ratio: float,
        center_size_ratio: float,
        angle_deg: float,
    ) -> tuple[tuple[int, ...], ...]:
        """Build reusable pixel regions for one normalized cross angle."""

        band = max(0.0, min(0.49, diagonal_band_ratio))
        band_distance = band / math.sqrt(2.0)
        center_half = max(0.0, min(0.5, center_size_ratio / 2.0))
        angle_radians = math.radians(angle_deg % 90.0)
        line_a_x = math.cos(angle_radians)
        line_a_y = math.sin(angle_radians)
        line_b_x = -line_a_y
        line_b_y = line_a_x
        region_indices: list[list[int]] = [[] for _ in range(8)]

        for row in range(grid_size):
            v = (row + 0.5) / grid_size
            for column in range(grid_size):
                u = (column + 0.5) / grid_size
                x = u - 0.5
                y = v - 0.5
                along_a = x * line_a_x + y * line_a_y
                along_b = x * line_b_x + y * line_b_y
                line_a = abs(along_b) <= band_distance
                line_b = abs(along_a) <= band_distance
                center = (
                    abs(x) <= center_half
                    and abs(y) <= center_half
                )
                off_cross = not (line_a or line_b)
                region_membership = (
                    line_a,
                    line_b,
                    center,
                    off_cross,
                    line_a and along_a < -abs(along_b),
                    line_a and along_a >= abs(along_b),
                    line_b and along_b < -abs(along_a),
                    line_b and along_b >= abs(along_a),
                )
                pixel_index = row * grid_size + column
                for region, inside in zip(
                    region_indices, region_membership
                ):
                    if inside:
                        region.append(pixel_index)

        return tuple(tuple(region) for region in region_indices)

    @staticmethod
    def _red_x_search_angles(angle_step_deg: float) -> tuple[float, ...]:
        # Keep this sampling definition shared by prewarming and matching so
        # their floating-point cache keys are exactly identical.
        step = max(1.0, min(15.0, float(angle_step_deg)))
        angles = []
        angle = 0.0
        while angle < 90.0:
            angles.append(angle)
            angle += step
        return tuple(angles)

    def _prewarm_red_x_grid_regions(self) -> None:
        config = self.config
        for angle in self._red_x_search_angles(config.red_x_angle_step_deg):
            self._red_x_grid_regions(
                RED_X_GRID_SIZE,
                config.red_x_diagonal_band_ratio,
                config.red_x_center_size_ratio,
                angle,
            )

    @classmethod
    def _best_red_x_grid_match(
        cls,
        grid_bytes: bytes,
        grid_size: int,
        diagonal_band_ratio: float,
        center_size_ratio: float,
        angle_step_deg: float,
    ) -> tuple[float, float, tuple[float, ...]]:
        """Return the strongest four-arm cross score over one 90° period."""

        # A coarser step can leave a cross too far from every sampled angle
        # and defeat the rotation-independent contract. Keep the tunable
        # search within a range that still has ample score margin.
        best_score = 0.0
        best_angle = 0.0
        best_fills = (0.0,) * 8
        best_key = (-1.0, -1.0, -1.0, -1.0)
        for angle in cls._red_x_search_angles(angle_step_deg):
            fills = cls._red_x_grid_fills(
                grid_bytes,
                grid_size,
                diagonal_band_ratio,
                center_size_ratio,
                angle,
            )
            score = cls._red_x_score_from_fills(fills)
            arm_floor = min(fills[4:8]) if len(fills) >= 8 else 0.0
            key = (score, arm_floor, fills[2], -fills[3])
            if key > best_key:
                best_score = score
                best_angle = angle
                best_fills = fills
                best_key = key
        return best_score, best_angle, best_fills

    @staticmethod
    def _red_x_score_from_fills(fills: tuple[float, ...]) -> float:
        if len(fills) < 8:
            return 0.0
        center_fill = fills[2]
        off_diag_fill = fills[3]
        arm_fills = fills[4:8]
        return max(0.0, min(center_fill, *arm_fills) - off_diag_fill)

    @staticmethod
    def _classify_evidence(
        gain_ratio: float,
        harmful_ratio: float,
        red_x_score: float,
        minimum_color_ratio: float,
        minimum_red_x_score: float,
        maximum_red_x_score_for_no_marker: float,
    ) -> tuple[EnergyClass, float]:
        color_threshold = max(1e-9, minimum_color_ratio)
        x_threshold = max(1e-9, minimum_red_x_score)
        gain_detected = gain_ratio >= color_threshold
        harmful_detected = (
            harmful_ratio >= color_threshold and red_x_score >= x_threshold
        )

        if gain_detected and harmful_detected:
            return EnergyClass.UNKNOWN, 0.0
        if harmful_detected:
            red_confidence = ColorEnergyDetector._color_confidence(
                harmful_ratio, color_threshold
            )
            x_confidence = ColorEnergyDetector._color_confidence(
                red_x_score, x_threshold
            )
            return EnergyClass.HARMFUL, min(red_confidence, x_confidence)
        if gain_detected:
            return EnergyClass.GAIN, ColorEnergyDetector._color_confidence(
                gain_ratio, color_threshold
            )
        if harmful_ratio >= color_threshold:
            if red_x_score <= maximum_red_x_score_for_no_marker:
                no_x_confidence = 1.0 - min(
                    1.0, max(0.0, red_x_score) / x_threshold
                )
                return EnergyClass.NO_BLOCK_MARKER, no_x_confidence

            # A partial cross remains ambiguous: it may be an oblique or
            # incomplete harmful marker, so it must not become enemy evidence.
            return EnergyClass.UNKNOWN, 0.0

        gain_absence = 1.0 - min(
            1.0, max(0.0, gain_ratio) / color_threshold
        )
        harmful_absence = 1.0 - min(
            1.0, max(0.0, harmful_ratio) / color_threshold
        )
        no_marker_confidence = min(gain_absence, harmful_absence)
        return EnergyClass.NO_BLOCK_MARKER, no_marker_confidence

    @staticmethod
    def _color_confidence(color_ratio: float, threshold: float) -> float:
        relative_excess = max(0.0, color_ratio - threshold) / threshold
        return min(1.0, 0.5 + 0.5 * relative_excess)

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


__all__ = [
    "CAMERA_PROBE_READS",
    "ColorEnergyDetector",
    "ColorFrameAnalysis",
    "EnergyClass",
    "RedXEvidence",
    "VisionResult",
    "open_first_usable_camera",
]
