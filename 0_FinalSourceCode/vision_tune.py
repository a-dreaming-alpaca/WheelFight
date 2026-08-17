"""Camera-only color-vision calibration viewer.

This tool deliberately does not import the match controller, Mega reader,
MotionController, or UpTech. It opens only one configured camera and applies
the exact same ColorEnergyDetector analysis used during a match.
"""

from __future__ import annotations

import math
import os
import select
import sys
import time
from typing import Any, Optional

from energy_vision import (
    ColorEnergyDetector,
    ColorFrameAnalysis,
    EnergyClass,
    open_first_usable_camera,
)
from robot_config import DEFAULT_CONFIG, VisionConfig


MAIN_WINDOW = "WheelFight Vision Tune"
GAIN_MASK_WINDOW = "Gain mask - yellow green"
HARMFUL_MASK_WINDOW = "Harmful mask - red"
TERMINAL_UPDATE_INTERVAL = 0.20
MAX_CONSECUTIVE_READ_FAILURES = 5
EXIT_KEYS = {ord("q"), ord("Q"), 10, 13, 27}

CLASSIFICATION_COLORS = {
    EnergyClass.GAIN: (0, 255, 255),
    EnergyClass.HARMFUL: (0, 0, 255),
    EnergyClass.NO_BLOCK_MARKER: (255, 255, 255),
    EnergyClass.UNKNOWN: (0, 165, 255),
    EnergyClass.NONE: (160, 160, 160),
}


def _load_cv2():
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV (cv2) is required for vision calibration"
        ) from exc
    return cv2


def _require_graphical_session() -> None:
    if not sys.platform.startswith("linux"):
        return
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        return
    raise RuntimeError(
        "No graphical desktop was detected. Run vision_tune.py from the RK3588S "
        "desktop, or use SSH with graphical forwarding enabled."
    )


def _open_first_camera(cv2, config: VisionConfig):
    return open_first_usable_camera(cv2, config)


def _stop_requested(key_code: int) -> bool:
    return key_code >= 0 and (key_code & 0xFF) in EXIT_KEYS


def _terminal_enter_pressed() -> bool:
    """Consume one ready terminal line without blocking the camera loop."""

    if not getattr(sys.stdin, "isatty", lambda: False)():
        return False
    try:
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return False
        return sys.stdin.readline() != ""
    except (OSError, ValueError):
        # Windows console handles do not support select(); the OpenCV window's
        # Enter/Q/Esc keys and Ctrl+C remain available there.
        return False


def _classification_bgr(classification: EnergyClass) -> tuple[int, int, int]:
    return CLASSIFICATION_COLORS.get(classification, (160, 160, 160))


def _build_status_lines(
    analysis: ColorFrameAnalysis,
    config: VisionConfig,
    camera_index: int,
    frame_width: int,
    frame_height: int,
    fps: float,
) -> tuple[str, ...]:
    result = analysis.result
    red_x = analysis.red_x_evidence
    threshold_percent = config.min_color_area_ratio * 100.0
    return (
        f"Camera {camera_index}  {frame_width}x{frame_height}  FPS {fps:.1f}",
        f"Class: {result.classification.value}  confidence {result.confidence:.3f}",
        (
            f"Gain yellow-green: {result.gain_color_ratio * 100.0:.3f}%  "
            f"threshold {threshold_percent:.3f}%"
        ),
        (
            f"Harmful red: {result.harmful_color_ratio * 100.0:.3f}%  "
            f"threshold {threshold_percent:.3f}%"
        ),
        (
            f"Red X: {'YES' if red_x.detected else 'NO'}  score {red_x.score:.3f}  "
            f"no-marker <= {config.max_red_x_score_for_no_marker:.3f}  "
            f"harmful >= {config.min_red_x_score:.3f}  "
            f"angle {red_x.angle_deg:.1f} deg"
            if red_x.angle_deg is not None
            else (
                f"Red X: {'YES' if red_x.detected else 'NO'}  "
                f"score {red_x.score:.3f}  "
                f"no-marker <= {config.max_red_x_score_for_no_marker:.3f}  "
                f"harmful >= {config.min_red_x_score:.3f}  "
                "angle --"
            )
        ),
        (
            "X arms="
            f"{red_x.arm_fills[0]:.2f}/{red_x.arm_fills[1]:.2f}/"
            f"{red_x.arm_fills[2]:.2f}/{red_x.arm_fills[3]:.2f} "
            f"center={red_x.center_fill:.2f} off={red_x.off_diag_fill:.2f}"
        ),
        (
            f"HSV gain H={config.yellow_green_h_min}-{config.yellow_green_h_max}; "
            f"red H={config.red_h_low_min}-{config.red_h_low_max}/"
            f"{config.red_h_high_min}-{config.red_h_high_max}; "
            f"S>={config.min_saturation} V>={config.min_value}"
        ),
        "Q / Enter / Esc: quit",
    )


def _put_text(cv2, frame, text: str, origin, color) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    line_type = getattr(cv2, "LINE_AA", 8)
    cv2.putText(frame, text, origin, font, 0.52, (0, 0, 0), 3, line_type)
    cv2.putText(frame, text, origin, font, 0.52, color, 1, line_type)


def _line_box_endpoints(left, top, right, bottom, angle_deg):
    center_x = (left + right) / 2.0
    center_y = (top + bottom) / 2.0
    box_width = max(0.0, right - left)
    box_height = max(0.0, bottom - top)
    radians = math.radians(angle_deg)
    direction_x = math.cos(radians)
    direction_y = math.sin(radians)
    extents = []
    if abs(direction_x) > 1e-9:
        extents.append(0.5 / abs(direction_x))
    if abs(direction_y) > 1e-9:
        extents.append(0.5 / abs(direction_y))
    normalized_extent = min(extents) if extents else 0.0
    offset_x = normalized_extent * direction_x * box_width
    offset_y = normalized_extent * direction_y * box_height
    return (
        (
            int(round(center_x - offset_x)),
            int(round(center_y - offset_y)),
        ),
        (
            int(round(center_x + offset_x)),
            int(round(center_y + offset_y)),
        ),
    )


def _build_display_frame(
    cv2,
    frame,
    analysis: ColorFrameAnalysis,
    config: VisionConfig,
    camera_index: int,
    fps: float,
):
    display = frame.copy()
    height, width = display.shape[:2]
    x0, x1, y0, y1 = analysis.roi_bounds

    if x1 > x0 and y1 > y0:
        cv2.rectangle(display, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 0), 2)
        center_x = (x0 + x1) // 2
        center_y = (y0 + y1) // 2
        cv2.line(display, (center_x, y0), (center_x, y1 - 1), (255, 255, 0), 1)
        cv2.line(
            display,
            (max(x0, center_x - 18), center_y),
            (min(x1 - 1, center_x + 18), center_y),
            (255, 255, 0),
            1,
        )

    red_x = analysis.red_x_evidence
    if red_x.candidate_box is not None:
        box_x, box_y, box_width, box_height = red_x.candidate_box
        left = x0 + box_x
        top = y0 + box_y
        right = left + box_width - 1
        bottom = top + box_height - 1
        guide_color = (0, 255, 0) if red_x.detected else (0, 165, 255)
        cv2.rectangle(display, (left, top), (right, bottom), guide_color, 1)
        angle_deg = red_x.angle_deg if red_x.angle_deg is not None else 45.0
        for guide_angle in (angle_deg, angle_deg + 90.0):
            start, end = _line_box_endpoints(
                left,
                top,
                right,
                bottom,
                guide_angle,
            )
            cv2.line(display, start, end, guide_color, 1)

    color = _classification_bgr(analysis.result.classification)
    lines = _build_status_lines(
        analysis,
        config,
        camera_index,
        width,
        height,
        fps,
    )
    for line_index, line in enumerate(lines):
        _put_text(cv2, display, line, (10, 24 + line_index * 22), color)
    return display


def _format_terminal_status(
    analysis: ColorFrameAnalysis,
    camera_index: int,
) -> str:
    result = analysis.result
    return (
        f"camera={camera_index} "
        f"class={result.classification.value:<16} "
        f"gain={result.gain_color_ratio:.4f} "
        f"red={result.harmful_color_ratio:.4f} "
        f"red_x={result.red_x_score:.3f}/"
        f"{'Y' if result.red_x_detected else 'N'} "
        f"angle={result.red_x_angle_deg if result.red_x_angle_deg is not None else '--'} "
        f"confidence={result.confidence:.3f}"
    )


def _main_window_closed(cv2) -> bool:
    try:
        visible = cv2.getWindowProperty(MAIN_WINDOW, cv2.WND_PROP_VISIBLE)
        return visible < 1.0
    except Exception:
        return False


def run_tuner(
    cv2,
    config: VisionConfig = DEFAULT_CONFIG.vision,
    clock=time.monotonic,
) -> None:
    """Run the synchronous camera viewer until the user requests exit."""

    detector = ColorEnergyDetector(config=config, clock=clock)
    capture: Optional[Any] = None
    camera_index: Optional[int] = None
    first_frame = None
    last_frame_at = clock()
    last_terminal_at = last_frame_at - TERMINAL_UPDATE_INTERVAL
    fps = 0.0

    try:
        capture, camera_index, first_frame = _open_first_camera(cv2, config)
        window_mode = getattr(cv2, "WINDOW_NORMAL", 0)
        cv2.namedWindow(MAIN_WINDOW, window_mode)
        cv2.namedWindow(GAIN_MASK_WINDOW, window_mode)
        cv2.namedWindow(HARMFUL_MASK_WINDOW, window_mode)

        print(
            "Vision tuner started. Only the camera is active; Mega, motors and "
            "servos are not opened."
        )
        print("Press Q, Enter, Esc, or Ctrl+C to stop.")

        consecutive_read_failures = 0
        while True:
            if first_frame is not None:
                frame = first_frame
                first_frame = None
                ok = True
            else:
                ok, frame = capture.read()
            if not ok or frame is None:
                consecutive_read_failures += 1
                if consecutive_read_failures >= MAX_CONSECUTIVE_READ_FAILURES:
                    raise RuntimeError("Camera frame read repeatedly failed")
                time.sleep(0.02)
                continue
            consecutive_read_failures = 0

            now = clock()
            elapsed = now - last_frame_at
            if elapsed > 0.0:
                instant_fps = 1.0 / elapsed
                fps = instant_fps if fps <= 0.0 else 0.90 * fps + 0.10 * instant_fps
            last_frame_at = now

            analysis = detector.analyze_frame(frame, cv2)
            display = _build_display_frame(
                cv2,
                frame,
                analysis,
                config,
                camera_index,
                fps,
            )
            cv2.imshow(MAIN_WINDOW, display)
            if analysis.gain_mask is not None:
                cv2.imshow(GAIN_MASK_WINDOW, analysis.gain_mask)
            if analysis.harmful_mask is not None:
                cv2.imshow(HARMFUL_MASK_WINDOW, analysis.harmful_mask)

            if now - last_terminal_at >= TERMINAL_UPDATE_INTERVAL:
                status = _format_terminal_status(analysis, camera_index)
                print(f"\r{status:<105}", end="", flush=True)
                last_terminal_at = now

            key_code = cv2.waitKey(1)
            if (
                _stop_requested(key_code)
                or _terminal_enter_pressed()
                or _main_window_closed(cv2)
            ):
                break
    finally:
        if capture is not None:
            capture.release()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        print("\nVision tuner stopped.")


def main() -> int:
    try:
        _require_graphical_session()
        cv2 = _load_cv2()
        run_tuner(cv2)
    except KeyboardInterrupt:
        return 0
    except RuntimeError as exc:
        print(f"Vision tuner error: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"Vision tuner OpenCV error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
