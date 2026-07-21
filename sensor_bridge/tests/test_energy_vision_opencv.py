import sys
import unittest
from dataclasses import replace
from pathlib import Path


sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "0_FinalSourceCode")
)

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from energy_vision import ColorEnergyDetector, EnergyClass  # noqa: E402
from robot_config import DEFAULT_CONFIG  # noqa: E402


@unittest.skipUnless(cv2 is not None and np is not None, "OpenCV is unavailable")
class OpenCVRedXTests(unittest.TestCase):
    def setUp(self):
        config = replace(
            DEFAULT_CONFIG.vision,
            roi_x_min=0.0,
            roi_x_max=1.0,
            roi_y_min=0.0,
            roi_y_max=1.0,
            min_color_area_ratio=0.01,
        )
        self.detector = ColorEnergyDetector(config=config, clock=lambda: 1.0)
        self.red = self._hsv_bgr(0)
        self.gain = self._hsv_bgr(35)

    @staticmethod
    def _hsv_bgr(hue):
        hsv = np.array([[[hue, 255, 255]]], dtype=np.uint8)
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
        return tuple(int(channel) for channel in bgr)

    @staticmethod
    def _frame():
        return np.zeros((240, 320, 3), dtype=np.uint8)

    def _analyze(self, frame):
        return self.detector.analyze_frame(frame, cv2).result

    def _draw_red_x(self, frame):
        cv2.line(frame, (90, 50), (230, 190), self.red, 24, cv2.LINE_8)
        cv2.line(frame, (230, 50), (90, 190), self.red, 24, cv2.LINE_8)

    def test_red_x_is_harmful(self):
        frame = self._frame()
        self._draw_red_x(frame)

        result = self._analyze(frame)

        self.assertEqual(result.classification, EnergyClass.HARMFUL)
        self.assertTrue(result.red_x_detected)

    def test_red_box_solid_patch_single_diagonal_and_plus_are_not_harmful(self):
        patterns = {}

        box = self._frame()
        cv2.rectangle(box, (90, 50), (230, 190), self.red, 16, cv2.LINE_8)
        patterns["box"] = box

        solid = self._frame()
        cv2.rectangle(solid, (90, 50), (230, 190), self.red, -1, cv2.LINE_8)
        patterns["solid"] = solid

        diagonal = self._frame()
        cv2.line(diagonal, (90, 50), (230, 190), self.red, 24, cv2.LINE_8)
        patterns["single-diagonal"] = diagonal

        plus = self._frame()
        cv2.line(plus, (160, 50), (160, 190), self.red, 24, cv2.LINE_8)
        cv2.line(plus, (90, 120), (230, 120), self.red, 24, cv2.LINE_8)
        patterns["plus"] = plus

        arena_mark = self._frame()
        cv2.rectangle(arena_mark, (70, 40), (250, 200), self.red, -1, cv2.LINE_8)
        white = (255, 255, 255)
        cv2.line(arena_mark, (100, 80), (220, 80), white, 15, cv2.LINE_8)
        cv2.line(arena_mark, (160, 65), (160, 175), white, 15, cv2.LINE_8)
        cv2.line(arena_mark, (105, 165), (215, 105), white, 15, cv2.LINE_8)
        patterns["red-field-mark-with-white-strokes"] = arena_mark

        for name, frame in patterns.items():
            with self.subTest(name=name):
                result = self._analyze(frame)
                self.assertEqual(result.classification, EnergyClass.NO_BLOCK_MARKER)
                self.assertFalse(result.red_x_detected)

    def test_gain_remains_color_only_next_to_red_box(self):
        frame = self._frame()
        cv2.rectangle(frame, (10, 10), (80, 80), self.gain, -1, cv2.LINE_8)
        cv2.rectangle(frame, (100, 60), (220, 180), self.red, 14, cv2.LINE_8)

        result = self._analyze(frame)

        self.assertEqual(result.classification, EnergyClass.GAIN)

    def test_gain_and_valid_red_x_are_unknown(self):
        frame = self._frame()
        cv2.rectangle(frame, (10, 10), (80, 80), self.gain, -1, cv2.LINE_8)
        self._draw_red_x(frame)

        result = self._analyze(frame)

        self.assertEqual(result.classification, EnergyClass.UNKNOWN)

    def test_red_x_is_found_separately_from_a_red_arena_box(self):
        frame = self._frame()
        cv2.rectangle(frame, (40, 20), (280, 220), self.red, 12, cv2.LINE_8)
        cv2.line(frame, (100, 55), (220, 185), self.red, 20, cv2.LINE_8)
        cv2.line(frame, (220, 55), (100, 185), self.red, 20, cv2.LINE_8)

        result = self._analyze(frame)

        self.assertEqual(result.classification, EnergyClass.HARMFUL)
        self.assertTrue(result.red_x_detected)


if __name__ == "__main__":
    unittest.main()
