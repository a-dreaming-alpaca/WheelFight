"""Ultralytics YOLO vision backend for the WheelFight state machine."""

from __future__ import annotations

import os
import threading
import time
from typing import Any, Optional

from energy_vision import (
    EnergyClass,
    VisionResult,
    open_first_usable_camera,
)
from robot_config import DEFAULT_CONFIG, VisionConfig


class YoloEnergyDetector:
    """Run the trained Ultralytics model behind the existing vision API."""

    def __init__(
        self,
        model_path: str,
        class_names: tuple[str, ...] = ("Debuff", "Buff"),
        config: VisionConfig = DEFAULT_CONFIG.vision,
        confidence_threshold: float = 0.10,
        iou_threshold: float = 0.45,
        clock=time.monotonic,
    ) -> None:
        if not model_path:
            raise ValueError("model_path must not be empty")
        if not class_names:
            raise ValueError("class_names must not be empty")
        if not 0.0 < confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be in (0, 1]")
        if not 0.0 < iou_threshold <= 1.0:
            raise ValueError("iou_threshold must be in (0, 1]")

        self.model_path = os.path.abspath(model_path)
        self.class_names = tuple(class_names)
        self.config = config
        self.confidence_threshold = confidence_threshold
        self.iou_threshold = iou_threshold
        self._clock = clock
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._model = None
        self._latest = VisionResult.none(self._clock())
        self._healthy = False
        self._last_error = "not started"
        self._active_camera: Optional[int] = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return

        cv2 = self._load_cv2()
        self._model = self._load_model()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            args=(cv2,),
            name="yolo-energy-vision",
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout)
        self._model = None

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
                "confidence_threshold": self.confidence_threshold,
                "timestamp": self._latest.timestamp,
                "model_path": self.model_path,
            }

    @staticmethod
    def _load_cv2():
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("OpenCV is required for YOLO vision") from exc
        return cv2

    def _load_model(self):
        if not os.path.isfile(self.model_path):
            raise RuntimeError(f"YOLO model not found: {self.model_path}")
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError("ultralytics is required for YOLO vision") from exc
        return YOLO(self.model_path)

    def _run(self, cv2) -> None:
        while not self._stop_event.is_set():
            capture = None
            try:
                capture, camera_index, first_frame = open_first_usable_camera(
                    cv2,
                    self.config,
                    stop_event=self._stop_event,
                )
                with self._lock:
                    self._active_camera = camera_index
                    self._healthy = False
                    self._last_error = ""

                while not self._stop_event.is_set():
                    if first_frame is not None:
                        frame = first_frame
                        first_frame = None
                    else:
                        ok, frame = capture.read()
                        if not ok or frame is None:
                            raise RuntimeError("camera frame read failed")

                    result = self.detect_frame(frame, cv2)
                    with self._lock:
                        self._latest = result
                        self._healthy = True
                        self._last_error = ""
            except RuntimeError as exc:
                if not self._stop_event.is_set():
                    self._set_error(str(exc))
            except Exception as exc:
                if not self._stop_event.is_set():
                    self._set_error(str(exc))
            finally:
                if capture is not None:
                    try:
                        capture.release()
                    except Exception:
                        pass
                with self._lock:
                    self._active_camera = None
                    self._healthy = False

            self._stop_event.wait(self.config.reconnect_interval)

    def detect_frame(self, frame, cv2=None) -> VisionResult:
        """Run one BGR frame through Ultralytics and return a state-machine result."""
        result, _ = self.detect_frame_details(frame, cv2)
        return result

    def detect_frame_details(self, frame, cv2=None) -> tuple[VisionResult, list[dict]]:
        """Run one BGR frame and return the result plus all post-NMS detections."""
        if frame is None or not hasattr(frame, "shape") or len(frame.shape) < 2:
            raise ValueError("invalid camera frame")
        if cv2 is None:
            cv2 = self._load_cv2()
        if self._model is None:
            raise RuntimeError("Ultralytics YOLO model is not initialized")

        height, width = frame.shape[:2]
        prediction = self._model.predict(
            source=frame,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            imgsz=320,
            max_det=20,
            verbose=False,
        )[0]
        detections = []
        if prediction.boxes is not None:
            for box in prediction.boxes:
                class_id = int(box.cls.item())
                x1, y1, x2, y2 = (
                    float(value) for value in box.xyxy[0].tolist()
                )
                detections.append(
                    {
                        "box": (x1, y1, x2, y2),
                        "score": float(box.conf.item()),
                        "class_id": class_id,
                    }
                )

        timestamp = self._clock()
        if not detections:
            result = VisionResult(
                classification=EnergyClass.NO_BLOCK_MARKER,
                confidence=1.0,
                center_x=None,
                bbox_width=None,
                tag_id=None,
                timestamp=timestamp,
                frame_width=width,
            )
            return result, detections

        best = max(detections, key=lambda item: item["score"])
        class_id = best["class_id"]
        class_name = self.class_names[class_id] if class_id < len(self.class_names) else ""
        if class_name == "Buff":
            classification = EnergyClass.GAIN
        elif class_name == "Debuff":
            classification = EnergyClass.HARMFUL
        else:
            classification = EnergyClass.UNKNOWN

        x1, y1, x2, y2 = best["box"]
        result = VisionResult(
            classification=classification,
            confidence=best["score"],
            center_x=(x1 + x2) / 2.0,
            bbox_width=max(0.0, x2 - x1),
            tag_id=class_id,
            timestamp=timestamp,
            frame_width=width,
        )
        return result, detections

    def _postprocess(self, outputs, frame_shape: tuple[int, int], cv2) -> list[dict]:
        import numpy as np

        if not outputs:
            return []
        prediction = np.asarray(outputs[0])
        prediction = np.squeeze(prediction)
        expected_columns = 4 + len(self.class_names)
        if prediction.ndim != 2:
            raise ValueError(f"unexpected YOLO output shape: {prediction.shape}")
        if prediction.shape[0] == expected_columns:
            prediction = prediction.T
        elif prediction.shape[1] != expected_columns:
            raise ValueError(f"unexpected YOLO output shape: {prediction.shape}")

        boxes = prediction[:, :4].astype(np.float32, copy=True)
        class_scores = prediction[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        scores = class_scores[np.arange(len(class_scores)), class_ids]
        keep = scores > self.confidence_threshold
        boxes, scores, class_ids = boxes[keep], scores[keep], class_ids[keep]
        if not len(boxes):
            return []

        boxes[:, 0] -= boxes[:, 2] / 2.0
        boxes[:, 1] -= boxes[:, 3] / 2.0
        boxes[:, 2] += boxes[:, 0]
        boxes[:, 3] += boxes[:, 1]
        frame_height, frame_width = frame_shape
        scale_x = frame_width / 640.0
        scale_y = frame_height / 640.0
        boxes[:, [0, 2]] *= scale_x
        boxes[:, [1, 3]] *= scale_y
        boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, frame_width - 1)
        boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, frame_height - 1)

        detections = []
        for class_id in np.unique(class_ids):
            indices = np.flatnonzero(class_ids == class_id)
            kept = self._nms(boxes[indices], scores[indices])
            for local_index in kept:
                index = int(indices[local_index])
                detections.append(
                    {
                        "box": tuple(float(value) for value in boxes[index]),
                        "score": float(scores[index]),
                        "class_id": int(class_ids[index]),
                    }
                )
        return detections

    def _nms(self, boxes, scores) -> list[int]:
        import numpy as np

        order = scores.argsort()[::-1]
        kept = []
        while order.size:
            current = int(order[0])
            kept.append(current)
            if order.size == 1:
                break
            remaining = order[1:]
            xx1 = np.maximum(boxes[current, 0], boxes[remaining, 0])
            yy1 = np.maximum(boxes[current, 1], boxes[remaining, 1])
            xx2 = np.minimum(boxes[current, 2], boxes[remaining, 2])
            yy2 = np.minimum(boxes[current, 3], boxes[remaining, 3])
            intersection = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            area_current = max(0.0, boxes[current, 2] - boxes[current, 0]) * max(
                0.0, boxes[current, 3] - boxes[current, 1]
            )
            area_remaining = np.maximum(0.0, boxes[remaining, 2] - boxes[remaining, 0]) * np.maximum(
                0.0, boxes[remaining, 3] - boxes[remaining, 1]
            )
            iou = intersection / (area_current + area_remaining - intersection + 1e-9)
            order = remaining[iou <= self.iou_threshold]
        return kept

    def _set_error(self, error: str) -> None:
        with self._lock:
            self._healthy = False
            self._last_error = error
            self._latest = VisionResult(
                classification=EnergyClass.UNKNOWN,
                confidence=0.0,
                center_x=None,
                bbox_width=None,
                tag_id=None,
                timestamp=self._clock(),
                error=error,
            )


__all__ = ["YoloEnergyDetector"]