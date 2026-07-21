import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path


sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "0_FinalSourceCode")
)

from energy_vision import (  # noqa: E402
    CAMERA_PROBE_READS,
    ColorEnergyDetector,
    EnergyClass,
    VisionResult,
)
from robot_config import DEFAULT_CONFIG  # noqa: E402


GRID_SIZE = 48


def make_rotated_cross_grid(angle_deg, missing_arms=()):
    """Build a two-line cross in the detector's normalized coordinates."""

    angle_radians = math.radians(angle_deg % 90.0)
    line_a_x = math.cos(angle_radians)
    line_a_y = math.sin(angle_radians)
    line_b_x = -line_a_y
    line_b_y = line_a_x
    values = bytearray()
    for row in range(GRID_SIZE):
        y = (row + 0.5) / GRID_SIZE - 0.5
        for column in range(GRID_SIZE):
            x = (column + 0.5) / GRID_SIZE - 0.5
            along_a = x * line_a_x + y * line_a_y
            along_b = x * line_b_x + y * line_b_y
            line_a = abs(along_b) <= 0.07
            line_b = abs(along_a) <= 0.07
            arms = (
                line_a and along_a < -abs(along_b),
                line_a and along_a >= abs(along_b),
                line_b and along_b < -abs(along_a),
                line_b and along_b >= abs(along_a),
            )
            red = (line_a or line_b) and not any(
                arms[index] for index in missing_arms
            )
            values.append(255 if red else 0)
    return bytes(values)


def make_red_grid(pattern):
    if pattern == "x":
        return make_rotated_cross_grid(45.0)
    if pattern == "plus":
        return make_rotated_cross_grid(0.0)
    if pattern == "missing-arm":
        return make_rotated_cross_grid(45.0, missing_arms=(0,))
    if pattern == "missing-top-arms":
        return make_rotated_cross_grid(45.0, missing_arms=(0, 2))

    values = bytearray()
    for row in range(GRID_SIZE):
        v = (row + 0.5) / GRID_SIZE
        for column in range(GRID_SIZE):
            u = (column + 0.5) / GRID_SIZE
            diag_down = abs(u - v) <= 0.10
            diag_up = abs(u + v - 1.0) <= 0.10
            border = (
                row < 5
                or row >= GRID_SIZE - 5
                or column < 5
                or column >= GRID_SIZE - 5
            )
            triangle = v >= 2.0 * abs(u - 0.5)
            red = {
                "single-diagonal": diag_down,
                "box": border,
                "solid": True,
                "triangle": triangle,
                "none": False,
            }[pattern]
            values.append(255 if red else 0)
    return bytes(values)


class FakeMask:
    def __init__(self, count, grid_bytes=None):
        self.count = count
        self.grid_bytes = grid_bytes or make_red_grid("none")
        self.shape = (70, 120)

    def __getitem__(self, key):
        return self

    def copy(self):
        return FakeMask(self.count, self.grid_bytes)

    def fill(self, value):
        self.count = 0 if value == 0 else self.shape[0] * self.shape[1]
        self.grid_bytes = bytes([value]) * (GRID_SIZE * GRID_SIZE)

    def tobytes(self):
        return self.grid_bytes


class FakeFrame:
    def __init__(self, shape, root=None):
        self.shape = shape
        self.root = root or self
        self.last_slice = None

    def __getitem__(self, key):
        y_slice, x_slice = key
        y0, y1 = y_slice.start, y_slice.stop
        x0, x1 = x_slice.start, x_slice.stop
        self.root.last_slice = (y0, y1, x0, x1)
        return FakeFrame((y1 - y0, x1 - x0, self.shape[2]), root=self.root)


class FakeCameraCapture:
    def __init__(self, index, frames):
        self.index = index
        self.frames = list(frames)
        self.read_calls = 0
        self.released = False
        self.set_calls = []

    def isOpened(self):
        return True

    def set(self, property_id, value):
        self.set_calls.append((property_id, value))
        return True

    def read(self):
        self.read_calls += 1
        if not self.frames:
            return False, None
        frame = self.frames.pop(0)
        return frame is not None, frame

    def release(self):
        self.released = True


class FakeCameraCV2:
    CAP_PROP_FRAME_WIDTH = 3
    CAP_PROP_FRAME_HEIGHT = 4

    def __init__(self, frames_by_index):
        self.frames_by_index = frames_by_index
        self.captures = []

    def VideoCapture(self, index):
        capture = FakeCameraCapture(index, self.frames_by_index.get(index, ()))
        self.captures.append(capture)
        return capture


class FakeCV2:
    COLOR_BGR2HSV = 40
    INTER_NEAREST = 0
    RETR_CCOMP = 2
    CHAIN_APPROX_SIMPLE = 2

    def __init__(
        self,
        *,
        gain=0,
        red_low=0,
        red_high=0,
        red_pattern="x",
        contour_patterns=None,
        contour_parents=None,
    ):
        self.counts = {
            25: gain,
            0: red_low,
            168: red_high,
        }
        self.red_grid = make_red_grid(red_pattern)
        self.contour_patterns = contour_patterns
        self.contour_parents = contour_parents
        self.ranges = []
        self.cvt_calls = 0

    def cvtColor(self, frame, conversion):
        self.cvt_calls += 1
        if conversion != self.COLOR_BGR2HSV:
            raise AssertionError("unexpected color conversion")
        return frame

    def inRange(self, frame, lower, upper):
        self.ranges.append((lower, upper))
        return FakeMask(self.counts[lower[0]])

    def bitwise_or(self, left, right):
        return FakeMask(left.count + right.count, self.red_grid)

    @staticmethod
    def countNonZero(mask):
        return mask.count

    def findContours(self, mask, retrieval_mode, approximation_method):
        if not mask.count:
            return [], None
        if self.contour_patterns is None:
            contours = [mask]
            parents = (-1,)
        else:
            contours = [
                FakeMask(mask.count, make_red_grid(pattern))
                for pattern in self.contour_patterns
            ]
            parents = self.contour_parents or (-1,) * len(contours)
        hierarchy = [
            [(-1, -1, -1, parent) for parent in parents]
        ]
        return contours, hierarchy

    @staticmethod
    def boundingRect(points):
        return 0, 0, GRID_SIZE, GRID_SIZE

    @staticmethod
    def drawContours(image, contours, contour_index, color, thickness):
        contour = contours[0] if contour_index == -1 else contours[contour_index]
        image.count = contour.count
        image.grid_bytes = contour.grid_bytes
        return image

    @staticmethod
    def resize(mask, size, interpolation):
        if size != (GRID_SIZE, GRID_SIZE):
            raise AssertionError("unexpected normalized X grid size")
        return mask


class ColorEnergyDetectorTests(unittest.TestCase):
    def setUp(self):
        self.config = replace(
            DEFAULT_CONFIG.vision,
            min_color_area_ratio=0.015,
        )
        self.detector = ColorEnergyDetector(config=self.config, clock=lambda: 123.0)

    def test_runtime_skips_open_camera_without_frames_and_uses_probe_frame(self):
        first_frame = object()
        cv2 = FakeCameraCV2(
            {
                0: (None,) * CAMERA_PROBE_READS,
                1: (first_frame,),
            }
        )
        detector = ColorEnergyDetector(
            config=replace(
                self.config,
                camera_indices=(0, 1),
                reconnect_interval=0.0,
                show_debug_window=False,
            ),
            clock=lambda: 123.0,
        )
        detector._load_dependencies = lambda: cv2
        analyzed_frames = []

        def detect_frame(frame, actual_cv2):
            analyzed_frames.append(frame)
            self.assertIs(actual_cv2, cv2)
            detector._stop_event.set()
            return VisionResult.none(123.0)

        detector._detect_frame = detect_frame
        detector._run()

        self.assertEqual(analyzed_frames, [first_frame])
        self.assertEqual([capture.index for capture in cv2.captures], [0, 1])
        self.assertEqual(cv2.captures[0].read_calls, CAMERA_PROBE_READS)
        self.assertEqual(cv2.captures[1].read_calls, 1)
        self.assertTrue(all(capture.released for capture in cv2.captures))

    def test_classifier_requires_red_x_but_gain_remains_color_only(self):
        color_threshold = self.config.min_color_area_ratio
        x_threshold = self.config.min_red_x_score
        cases = (
            (0.0, 0.0, 0.0, EnergyClass.NO_BLOCK_MARKER, 1.0),
            (color_threshold, 0.0, 0.0, EnergyClass.GAIN, 0.5),
            (0.0, color_threshold, 0.0, EnergyClass.UNKNOWN, 0.0),
            (
                0.0,
                color_threshold * 0.5,
                0.0,
                EnergyClass.NO_BLOCK_MARKER,
                0.5,
            ),
            (0.0, color_threshold, x_threshold, EnergyClass.HARMFUL, 0.5),
            (
                color_threshold,
                color_threshold,
                0.0,
                EnergyClass.GAIN,
                0.5,
            ),
            (
                color_threshold,
                color_threshold,
                x_threshold,
                EnergyClass.UNKNOWN,
                0.0,
            ),
        )

        for (
            gain,
            harmful,
            x_score,
            expected_class,
            expected_confidence,
        ) in cases:
            with self.subTest(
                gain=gain,
                harmful=harmful,
                x_score=x_score,
            ):
                classification, confidence = self.detector._classify_evidence(
                    gain,
                    harmful,
                    x_score,
                    color_threshold,
                    x_threshold,
                )
                self.assertEqual(classification, expected_class)
                self.assertAlmostEqual(confidence, expected_confidence)

    def test_red_x_grid_match_accepts_cross_at_any_rotation(self):
        for source_angle in (0.0, 7.0, 23.0, 37.0, 45.0, 68.0, 83.0):
            with self.subTest(source_angle=source_angle):
                score, best_angle, _ = self.detector._best_red_x_grid_match(
                    make_rotated_cross_grid(source_angle),
                    GRID_SIZE,
                    self.config.red_x_diagonal_band_ratio,
                    self.config.red_x_center_size_ratio,
                    self.config.red_x_angle_step_deg,
                )
                self.assertGreaterEqual(score, self.config.min_red_x_score)
                self.assertGreaterEqual(best_angle, 0.0)
                self.assertLess(best_angle, 90.0)
                angle_error = abs(best_angle - source_angle) % 90.0
                angle_error = min(angle_error, 90.0 - angle_error)
                self.assertLessEqual(angle_error, 5.0)

    def test_red_x_angle_step_is_clamped_to_safe_rotation_coverage(self):
        score, _, _ = self.detector._best_red_x_grid_match(
            make_rotated_cross_grid(22.5),
            GRID_SIZE,
            self.config.red_x_diagonal_band_ratio,
            self.config.red_x_center_size_ratio,
            45.0,
        )

        self.assertGreaterEqual(score, self.config.min_red_x_score)

    def test_red_x_grid_match_rejects_non_cross_shapes(self):
        patterns = (
            "box",
            "solid",
            "single-diagonal",
            "missing-arm",
            "missing-top-arms",
            "triangle",
        )
        for pattern in patterns:
            score, _, _ = self.detector._best_red_x_grid_match(
                make_red_grid(pattern),
                GRID_SIZE,
                self.config.red_x_diagonal_band_ratio,
                self.config.red_x_center_size_ratio,
                self.config.red_x_angle_step_deg,
            )
            with self.subTest(pattern=pattern):
                self.assertLess(score, self.config.min_red_x_score)

        for missing_arm in range(4):
            with self.subTest(missing_arm=missing_arm):
                score, _, _ = self.detector._best_red_x_grid_match(
                    make_rotated_cross_grid(
                        31.0,
                        missing_arms=(missing_arm,),
                    ),
                    GRID_SIZE,
                    self.config.red_x_diagonal_band_ratio,
                    self.config.red_x_center_size_ratio,
                    self.config.red_x_angle_step_deg,
                )
                self.assertLess(score, self.config.min_red_x_score)

    def test_red_x_contours_skip_holes_but_keep_nested_foreground(self):
        frame = FakeFrame((100, 200, 3))

        field_mark = FakeCV2(
            red_low=200,
            contour_patterns=("solid", "x"),
            contour_parents=(-1, 0),
        )
        field_result = self.detector.analyze_frame(frame, field_mark).result
        self.assertEqual(field_result.classification, EnergyClass.UNKNOWN)
        self.assertFalse(field_result.red_x_detected)

        framed_x = FakeCV2(
            red_low=200,
            contour_patterns=("box", "none", "x"),
            contour_parents=(-1, 0, -1),
        )
        framed_result = self.detector.analyze_frame(frame, framed_x).result
        self.assertEqual(framed_result.classification, EnergyClass.HARMFUL)
        self.assertTrue(framed_result.red_x_detected)

    def test_detect_frame_uses_central_roi_and_configured_hsv_ranges(self):
        frame = FakeFrame((100, 200, 3))
        cv2 = FakeCV2(gain=126)

        analysis = self.detector.analyze_frame(frame, cv2)
        result = analysis.result

        self.assertEqual(frame.last_slice, (15, 85, 40, 160))
        self.assertEqual(analysis.roi_bounds, (40, 160, 15, 85))
        self.assertEqual(analysis.gain_mask.count, 126)
        self.assertEqual(analysis.harmful_mask.count, 0)
        self.assertFalse(analysis.red_x_evidence.detected)
        self.assertEqual(result.classification, EnergyClass.GAIN)
        self.assertAlmostEqual(result.gain_color_ratio, 0.015)
        self.assertEqual(result.harmful_color_ratio, 0.0)
        self.assertGreaterEqual(result.confidence, self.config.min_color_confidence)
        self.assertEqual(result.timestamp, 123.0)
        self.assertEqual(result.frame_width, 200)
        self.assertIsNone(result.tag_id)
        self.assertIsNone(result.center_x)
        self.assertIsNone(result.bbox_width)
        self.assertEqual(
            cv2.ranges,
            [
                ((25, 70, 60), (50, 255, 255)),
                ((0, 70, 60), (12, 255, 255)),
                ((168, 70, 60), (179, 255, 255)),
            ],
        )

    def test_detect_frame_combines_both_red_hue_ranges(self):
        frame = FakeFrame((100, 200, 3))
        cv2 = FakeCV2(red_low=60, red_high=66)

        result = self.detector._detect_frame(frame, cv2)

        self.assertEqual(result.classification, EnergyClass.HARMFUL)
        self.assertAlmostEqual(result.harmful_color_ratio, 0.015)
        self.assertTrue(result.red_x_detected)
        self.assertGreaterEqual(result.red_x_score, self.config.min_red_x_score)
        self.assertIsNotNone(result.red_x_angle_deg)
        self.assertGreaterEqual(result.confidence, self.config.min_color_confidence)

    def test_red_box_without_x_is_not_harmful(self):
        result = self.detector._detect_frame(
            FakeFrame((100, 200, 3)),
            FakeCV2(red_low=126, red_pattern="box"),
        )

        self.assertEqual(result.classification, EnergyClass.UNKNOWN)
        self.assertFalse(result.red_x_detected)
        self.assertEqual(result.confidence, 0.0)

    def test_gain_color_is_not_blocked_by_unshaped_red_area(self):
        result = self.detector._detect_frame(
            FakeFrame((100, 200, 3)),
            FakeCV2(gain=126, red_low=126, red_pattern="box"),
        )

        self.assertEqual(result.classification, EnergyClass.GAIN)

    def test_detect_frame_reports_unknown_when_both_colors_reach_threshold(self):
        frame = FakeFrame((100, 200, 3))
        cv2 = FakeCV2(gain=126, red_low=126)

        result = self.detector._detect_frame(frame, cv2)

        self.assertEqual(result.classification, EnergyClass.UNKNOWN)
        self.assertEqual(result.confidence, 0.0)
        self.assertAlmostEqual(result.gain_color_ratio, 0.015)
        self.assertAlmostEqual(result.harmful_color_ratio, 0.015)

    def test_valid_roi_without_target_colors_is_no_marker(self):
        result = self.detector._detect_frame(
            FakeFrame((100, 200, 3)),
            FakeCV2(),
        )

        self.assertEqual(result.classification, EnergyClass.NO_BLOCK_MARKER)
        self.assertEqual(result.confidence, 1.0)

    def test_invalid_roi_is_unknown_without_running_color_conversion(self):
        detector = ColorEnergyDetector(
            config=replace(self.config, roi_x_min=0.80, roi_x_max=0.20),
            clock=lambda: 456.0,
        )
        cv2 = FakeCV2()

        result = detector._detect_frame(FakeFrame((100, 200, 3)), cv2)

        self.assertEqual(result.classification, EnergyClass.UNKNOWN)
        self.assertEqual(result.error, "invalid color vision ROI")
        self.assertEqual(result.timestamp, 456.0)
        self.assertEqual(cv2.cvt_calls, 0)

    def test_roi_bounds_clamp_to_frame_and_none_ratios_default_to_zero(self):
        detector = ColorEnergyDetector(
            config=replace(
                self.config,
                roi_x_min=-0.50,
                roi_x_max=1.50,
                roi_y_min=-0.50,
                roi_y_max=1.50,
            )
        )

        self.assertEqual(detector._roi_bounds(200, 100), (0, 200, 0, 100))
        result = VisionResult.none(1.0)
        self.assertEqual(result.gain_color_ratio, 0.0)
        self.assertEqual(result.harmful_color_ratio, 0.0)
        self.assertEqual(result.red_x_score, 0.0)
        self.assertFalse(result.red_x_detected)
        self.assertIsNone(result.red_x_angle_deg)


if __name__ == "__main__":
    unittest.main()
