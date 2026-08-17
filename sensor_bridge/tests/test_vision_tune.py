import sys
import unittest
from pathlib import Path


sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "0_FinalSourceCode")
)

from energy_vision import (  # noqa: E402
    ColorFrameAnalysis,
    EnergyClass,
    RedXEvidence,
    VisionResult,
)
from robot_config import DEFAULT_CONFIG  # noqa: E402
from vision_tune import (  # noqa: E402
    _build_status_lines,
    _classification_bgr,
    _format_terminal_status,
    _line_box_endpoints,
    _open_first_camera,
    _stop_requested,
)


class FakeCapture:
    def __init__(self, index, opened, frames):
        self.index = index
        self.opened = opened
        self.frames = list(frames)
        self.released = False
        self.set_calls = []

    def isOpened(self):
        return self.opened

    def set(self, property_id, value):
        self.set_calls.append((property_id, value))
        return True

    def read(self):
        if self.frames:
            return True, self.frames.pop(0)
        return False, None

    def release(self):
        self.released = True


class FakeCV2:
    CAP_PROP_FRAME_WIDTH = 3
    CAP_PROP_FRAME_HEIGHT = 4

    def __init__(self, open_indices, frames_by_index=None):
        self.open_indices = set(open_indices)
        self.frames_by_index = frames_by_index or {}
        self.captures = []

    def VideoCapture(self, index):
        capture = FakeCapture(
            index,
            index in self.open_indices,
            self.frames_by_index.get(index, ()),
        )
        self.captures.append(capture)
        return capture


class VisionTuneTests(unittest.TestCase):
    def test_stop_keys_include_q_enter_and_escape(self):
        for key_code in (ord("q"), ord("Q"), 10, 13, 27):
            with self.subTest(key_code=key_code):
                self.assertTrue(_stop_requested(key_code))
        for key_code in (-1, ord("a"), ord(" ")):
            with self.subTest(key_code=key_code):
                self.assertFalse(_stop_requested(key_code))

    def test_open_first_camera_releases_failed_candidates(self):
        first_frame = object()
        cv2 = FakeCV2(open_indices={1}, frames_by_index={1: (first_frame,)})

        capture, camera_index, frame = _open_first_camera(
            cv2, DEFAULT_CONFIG.vision
        )

        self.assertEqual(camera_index, 1)
        self.assertIs(frame, first_frame)
        self.assertIs(capture, cv2.captures[1])
        self.assertTrue(cv2.captures[0].released)
        self.assertFalse(cv2.captures[1].released)
        self.assertEqual(
            cv2.captures[1].set_calls,
            [
                (cv2.CAP_PROP_FRAME_WIDTH, DEFAULT_CONFIG.vision.frame_width),
                (cv2.CAP_PROP_FRAME_HEIGHT, DEFAULT_CONFIG.vision.frame_height),
            ],
        )

    def test_open_first_camera_skips_open_device_without_frames(self):
        first_frame = object()
        cv2 = FakeCV2(
            open_indices={0, 1},
            frames_by_index={1: (first_frame,)},
        )

        capture, camera_index, frame = _open_first_camera(
            cv2, DEFAULT_CONFIG.vision
        )

        self.assertEqual(camera_index, 1)
        self.assertIs(frame, first_frame)
        self.assertTrue(cv2.captures[0].released)
        self.assertIs(capture, cv2.captures[1])

    def test_open_first_camera_releases_every_unusable_device(self):
        cv2 = FakeCV2(open_indices={0, 1})

        with self.assertRaisesRegex(RuntimeError, "configured camera"):
            _open_first_camera(cv2, DEFAULT_CONFIG.vision)

        self.assertTrue(all(capture.released for capture in cv2.captures))

    def test_status_text_contains_ratios_threshold_and_classification(self):
        result = VisionResult(
            classification=EnergyClass.GAIN,
            confidence=0.75,
            center_x=None,
            bbox_width=None,
            tag_id=None,
            timestamp=1.0,
            frame_width=640,
            gain_color_ratio=0.0234,
            harmful_color_ratio=0.0012,
            red_x_score=0.42,
            red_x_detected=True,
            red_x_angle_deg=30.0,
        )
        analysis = ColorFrameAnalysis(
            result=result,
            roi_bounds=(128, 512, 72, 408),
            gain_mask=None,
            harmful_mask=None,
            red_x_evidence=RedXEvidence(
                score=0.42,
                detected=True,
                diag_down_fill=0.80,
                diag_up_fill=0.75,
                center_fill=0.90,
                off_diag_fill=0.10,
                arm_fills=(0.80, 0.78, 0.76, 0.74),
                angle_deg=30.0,
                candidate_box=(20, 20, 100, 100),
            ),
        )

        lines = _build_status_lines(
            analysis,
            DEFAULT_CONFIG.vision,
            camera_index=0,
            frame_width=640,
            frame_height=480,
            fps=29.8,
        )
        terminal = _format_terminal_status(analysis, camera_index=0)

        self.assertIn("Class: GAIN", lines[1])
        self.assertIn("2.340%", lines[2])
        self.assertIn("0.120%", lines[3])
        self.assertIn("threshold 1.500%", lines[2])
        self.assertIn("Red X: YES", lines[4])
        self.assertIn("score 0.420", lines[4])
        self.assertIn(
            "no-marker <= "
            f"{DEFAULT_CONFIG.vision.max_red_x_score_for_no_marker:.3f}",
            lines[4],
        )
        self.assertIn(
            f"harmful >= {DEFAULT_CONFIG.vision.min_red_x_score:.3f}",
            lines[4],
        )
        self.assertIn("angle 30.0 deg", lines[4])
        self.assertIn("class=GAIN", terminal)
        self.assertIn("gain=0.0234", terminal)
        self.assertIn("red=0.0012", terminal)
        self.assertIn("red_x=0.420/Y", terminal)
        self.assertIn("angle=30.0", terminal)

    def test_cross_guides_map_normalized_angles_into_candidate_box(self):
        box = (10, 20, 110, 70)

        self.assertEqual(
            _line_box_endpoints(*box, 0.0),
            ((10, 45), (110, 45)),
        )
        self.assertEqual(
            _line_box_endpoints(*box, 45.0),
            ((10, 20), (110, 70)),
        )
        self.assertEqual(
            _line_box_endpoints(*box, 90.0),
            ((60, 20), (60, 70)),
        )

    def test_classification_colors_distinguish_gain_and_harmful(self):
        self.assertNotEqual(
            _classification_bgr(EnergyClass.GAIN),
            _classification_bgr(EnergyClass.HARMFUL),
        )


if __name__ == "__main__":
    unittest.main()
