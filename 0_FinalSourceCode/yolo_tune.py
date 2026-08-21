"""Camera-only Ultralytics YOLO debugger."""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import replace
from typing import Any, Optional

from energy_vision import EnergyClass, VisionResult, open_first_usable_camera
from robot_config import DEFAULT_CONFIG, VisionConfig


WINDOW_NAME = "WheelFight YOLO Tune"
EXIT_KEYS = {ord("q"), ord("Q"), 10, 13, 27}
MODEL_SIZE = 320
CLASS_NAMES = ("Debuff", "Buff")


class TuneYoloDetector:
    """Run the trained PyTorch model directly through Ultralytics."""

    def __init__(
        self,
        model_path: str,
        confidence_threshold: float,
        iou_threshold: float,
        image_size: int = MODEL_SIZE,
    ):
        if not os.path.isfile(model_path):
            raise RuntimeError(f"YOLO model not found: {model_path}")
        if not 0.0 < confidence_threshold <= 1.0:
            raise ValueError("confidence threshold must be in (0, 1]")
        if not 0.0 < iou_threshold <= 1.0:
            raise ValueError("IoU threshold must be in (0, 1]")
        if image_size < 320 or image_size % 32 != 0:
            raise ValueError("image size must be a multiple of 32 and at least 320")
        self.model_path = os.path.abspath(model_path)
        self.class_names = CLASS_NAMES
        self.confidence_threshold = confidence_threshold
        self.iou_threshold = iou_threshold
        self.image_size = image_size
        self._model = None

    def start(self) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError("ultralytics is required for YOLO tuning") from exc
        self._model = YOLO(self.model_path)

    def stop(self) -> None:
        self._model = None

    def detect(self, frame, cv2) -> tuple[VisionResult, list[dict]]:
        if self._model is None:
            raise RuntimeError("Ultralytics YOLO model is not initialized")
        if frame is None or not hasattr(frame, "shape") or len(frame.shape) < 2:
            raise ValueError("invalid camera frame")

        result = self._model.predict(
            source=frame,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            imgsz=self.image_size,
            max_det=20,
            verbose=False,
        )[0]
        detections = []
        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls.item())
                x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
                detections.append({
                    "box": (x1, y1, x2, y2),
                    "score": float(box.conf.item()),
                    "class_id": class_id,
                })

        timestamp = time.monotonic()
        height, width = frame.shape[:2]
        if not detections:
            return VisionResult(
                classification=EnergyClass.NO_BLOCK_MARKER,
                confidence=1.0,
                center_x=None,
                bbox_width=None,
                tag_id=None,
                timestamp=timestamp,
                frame_width=width,
            ), detections

        best = max(detections, key=lambda item: item["score"])
        class_name = (
            self.class_names[best["class_id"]]
            if 0 <= best["class_id"] < len(self.class_names)
            else ""
        )
        classification = {
            "Buff": EnergyClass.GAIN,
            "Debuff": EnergyClass.HARMFUL,
        }.get(class_name, EnergyClass.UNKNOWN)
        x1, y1, x2, y2 = best["box"]
        return VisionResult(
            classification=classification,
            confidence=best["score"],
            center_x=(x1 + x2) / 2.0,
            bbox_width=max(0.0, x2 - x1),
            tag_id=best["class_id"],
            timestamp=timestamp,
            frame_width=width,
        ), detections


def _load_cv2():
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("OpenCV (cv2) is required for YOLO tuning") from exc
    return cv2


def _require_graphical_session() -> None:
    if sys.platform.startswith("linux") and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        raise RuntimeError("No graphical desktop was detected; run on the desktop.")


def _draw_text(cv2, frame, text: str, origin, color) -> None:
    line_type = getattr(cv2, "LINE_AA", 8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, text, origin, font, 0.55, (0, 0, 0), 3, line_type)
    cv2.putText(frame, text, origin, font, 0.55, color, 1, line_type)


def _class_color(class_name: str):
    if class_name == "Buff":
        return (0, 220, 0)
    if class_name == "Debuff":
        return (0, 0, 255)
    return (0, 165, 255)


def _draw_frame(cv2, frame, result, detections, detector, camera_index, fps):
    display = frame.copy()
    for detection in detections:
        x1, y1, x2, y2 = (int(round(value)) for value in detection["box"])
        class_id = detection["class_id"]
        class_name = (
            detector.class_names[class_id]
            if 0 <= class_id < len(detector.class_names)
            else f"class_{class_id}"
        )
        color = _class_color(class_name)
        cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
        _draw_text(
            cv2, display, f"{class_name} {detection['score']:.3f}",
            (x1, max(20, y1 - 8)), color,
        )

    confidence = f"{result.confidence:.3f}" if detections else "--"
    lines = (
        f"Camera {camera_index}  {frame.shape[1]}x{frame.shape[0]}  FPS {fps:.1f}",
        f"Class: {result.classification.value}  best confidence: {confidence}",
        f"Detections: {len(detections)}  threshold >= {detector.confidence_threshold:.3f}",
        f"Model: {detector.model_path}",
        "Q / Enter / Esc: quit",
    )
    for index, line in enumerate(lines):
        _draw_text(cv2, display, line, (10, 24 + index * 24), (255, 255, 255))
    return display


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tune the trained YOLO model")
    parser.add_argument(
        "--model",
        default=os.path.join(os.path.dirname(__file__), "yolo", "best.pt"),
        help="Ultralytics PyTorch weight path",
    )
    parser.add_argument("--camera", type=int, help="only use this camera index")
    parser.add_argument("--confidence", type=float, default=0.10)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument(
        "--imgsz",
        type=int,
        default=MODEL_SIZE,
        help="inference image size; 320 is fastest, 640 gives maximum detail",
    )
    return parser.parse_args()


def run_tuner(
    cv2,
    model_path: str,
    config: VisionConfig = DEFAULT_CONFIG.vision,
    confidence_threshold: float = 0.10,
    iou_threshold: float = 0.45,
    image_size: int = MODEL_SIZE,
    clock=time.monotonic,
) -> None:
    detector = TuneYoloDetector(
        model_path, confidence_threshold, iou_threshold, image_size
    )
    capture: Optional[Any] = None
    first_frame = None
    last_frame_at = clock()
    fps = 0.0
    try:
        detector.start()
        capture, camera_index, first_frame = open_first_usable_camera(cv2, config)
        cv2.namedWindow(WINDOW_NAME, getattr(cv2, "WINDOW_NORMAL", 0))
        print("YOLO tuner started with Ultralytics best.pt. Mega and motors are not opened.")
        print("Press Q, Enter, Esc, or Ctrl+C to stop.")
        while True:
            if first_frame is not None:
                frame = first_frame
                first_frame = None
            else:
                ok, frame = capture.read()
                if not ok or frame is None:
                    raise RuntimeError("camera frame read failed")
            now = clock()
            elapsed = now - last_frame_at
            if elapsed > 0.0:
                instant_fps = 1.0 / elapsed
                fps = instant_fps if fps <= 0.0 else 0.9 * fps + 0.1 * instant_fps
            last_frame_at = now
            result, detections = detector.detect(frame, cv2)
            display = _draw_frame(
                cv2, frame, result, detections, detector, camera_index, fps
            )
            cv2.imshow(WINDOW_NAME, display)
            key_code = cv2.waitKey(1)
            if key_code >= 0 and (key_code & 0xFF) in EXIT_KEYS:
                break
    finally:
        if capture is not None:
            capture.release()
        detector.stop()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        print("\nYOLO tuner stopped.")


def main() -> int:
    arguments = parse_args()
    config = (
        replace(DEFAULT_CONFIG.vision, camera_indices=(arguments.camera,))
        if arguments.camera is not None
        else DEFAULT_CONFIG.vision
    )
    try:
        _require_graphical_session()
        run_tuner(
            _load_cv2(),
            arguments.model,
            config=config,
            confidence_threshold=arguments.confidence,
            iou_threshold=arguments.iou,
            image_size=arguments.imgsz,
        )
    except KeyboardInterrupt:
        return 0
    except (RuntimeError, ValueError) as exc:
        print(f"YOLO tuner error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
