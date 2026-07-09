import json
import os
import sys
import threading
import time

import apriltag
import cv2

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from match_detection import (
    BD as SENSOR_BD,
    FD as SENSOR_FD,
    LD as SENSOR_LD,
    RD as SENSOR_RD,
    detect_edge,
    detect_enemy,
    detect_fence,
    detect_platform,
    detect_slip,
    read_sensor_snapshot,
)
from match_handlers import MatchHandlersMixin
from motion_controller import MotionController
from uptech import UpTech


class Match_demo(MatchHandlersMixin):
    FD = SENSOR_FD
    RD = SENSOR_RD
    BD = SENSOR_BD
    LD = SENSOR_LD

    na = 0
    nd = 0
    ne = 8

    CONTROL_DT = 0.02
    STATUS_PUBLISH_INTERVAL = 0.1
    STATUS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runtime")
    STATUS_FILE = os.path.join(STATUS_DIR, "match_status.json")
    STATUS_TMP_FILE = STATUS_FILE + ".tmp"

    STATE_INIT = "INIT"
    STATE_CLIMB = "CLIMB"
    STATE_FENCE_ALIGN_LEFT = "FENCE_ALIGN_LEFT"
    STATE_FENCE_ALIGN_RIGHT = "FENCE_ALIGN_RIGHT"
    STATE_FENCE_RECOVER = "FENCE_RECOVER"
    STATE_SEARCH = "SEARCH"
    STATE_TURN_TO_TARGET = "TURN_TO_TARGET"
    STATE_ATTACK = "ATTACK"
    STATE_AVOID_OWN_BLOCK = "AVOID_OWN_BLOCK"
    STATE_EDGE_RECOVER = "EDGE_RECOVER"
    STATE_SLIP_RECOVER = "SLIP_RECOVER"

    def __init__(self):
        self.uptech = UpTech()
        self.uptech.ADC_IO_Open()
        self.motion_controller = MotionController()

        options = apriltag.DetectorOptions(families="tag36h11")
        self.tag_detector = apriltag.Detector(options)

        self.apriltag_width = 0
        self.tag_id = -1
        self.state = self.STATE_INIT
        self._action_sequence = []
        self._action_index = 0
        self._action_deadline = 0
        self._action_label = ""
        self._stun_time = 0
        self._match_running = False
        self._last_stage = None
        self._state_reason = ""
        self._last_status_publish = 0
        self.camera_activate = False

        apriltag_detect = threading.Thread(target=self.apriltag_detect_thread, daemon=True)
        apriltag_detect.start()
        self._publish_status(force=True)

    def apriltag_detect_thread(self):
        print("detect start")
        self.camera_activate = True
        VideoCaptureIndex = 0
        while self.camera_activate:
            cap = None
            try:
                cap = cv2.VideoCapture(VideoCaptureIndex)
                if not cap.isOpened():
                    VideoCaptureIndex = 1
                    raise RuntimeError("attempt to connect camera")

                weight = 320
                height = 240
                cup_w = int((640 - weight) / 2) - 120
                cup_h = int((480 - height) / 2) - 40

                while self.camera_activate:
                    ret, frame = cap.read()
                    if not ret:
                        raise RuntimeError("frame read failed")

                    frame = cv2.resize(frame, (640, 480))
                    frame1 = frame[cup_h:cup_h + 440, cup_w:cup_w + 500]
                    result = frame1.copy()
                    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
                    apriltag_detect_results = self.tag_detector.detect(gray)

                    if len(apriltag_detect_results) == 0:
                        self.tag_id = -1

                    for tag in apriltag_detect_results:
                        self.tag_id = tag.tag_id
                        print("tag_id = {}".format(tag.tag_id))
                        if self.tag_id != 2:
                            print("front target is not own block")
                        else:
                            print("own block detected")
                        cv2.circle(result, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2)
                        cv2.circle(result, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2)
                        cv2.circle(result, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2)
                        cv2.circle(result, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2)

                    cv2.imshow("result", result)
                    if cv2.waitKey(1) & 0xff == ord("q"):
                        self.camera_activate = False
                        break
                cap.release()
            except Exception:
                print("camera error,try to connect camera...")
                if cap is not None:
                    cap.release()
                break

        cv2.destroyAllWindows()

    def _sensor_snapshot(self):
        return read_sensor_snapshot(self.uptech)

    def paltform_detect(self):
        return detect_platform(self._sensor_snapshot())

    def fence_detect(self):
        return detect_fence(self._sensor_snapshot())

    def edge_detect(self):
        return detect_edge(self._sensor_snapshot())

    def enemy_detect(self):
        return detect_enemy(self._sensor_snapshot(), self.tag_id)

    def slip_detect(self):
        return detect_slip(self._sensor_snapshot())

    def _set_state(self, state, reason=""):
        changed = self.state != state or self._state_reason != reason
        if self.state != state:
            if reason:
                print(f"State:{self.state}->{state} {reason}")
            else:
                print(f"State:{self.state}->{state}")
        self.state = state
        self._state_reason = reason
        self._publish_status(force=changed)

    def _status_snapshot(self):
        return {
            "timestamp": time.time(),
            "state": self.state,
            "state_reason": self._state_reason,
            "match_running": self._match_running,
            "last_stage": self._last_stage,
            "action_label": self._action_label,
            "action_index": self._action_index,
            "action_total": len(self._action_sequence),
            "stun_time": self._stun_time,
            "tag_id": self.tag_id,
        }

    def _publish_status(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_status_publish < self.STATUS_PUBLISH_INTERVAL:
            return
        self._last_status_publish = now
        try:
            os.makedirs(self.STATUS_DIR, exist_ok=True)
            with open(self.STATUS_TMP_FILE, "w", encoding="utf-8") as fp:
                json.dump(self._status_snapshot(), fp, ensure_ascii=False, indent=2)
            os.replace(self.STATUS_TMP_FILE, self.STATUS_FILE)
        except Exception as exc:
            print(f"status publish failed: {exc}")

    def _clear_action_sequence(self):
        self._action_sequence = []
        self._action_index = 0
        self._action_deadline = 0
        self._action_label = ""

    def _start_action_sequence(self, state, label, steps, reason=""):
        self._action_sequence = list(steps)
        self._action_index = 0
        self._action_deadline = 0
        self._action_label = label
        self._set_state(state, reason)

    def _run_action_sequence(self):
        if not self._action_sequence:
            return False

        now = time.monotonic()
        if self._action_deadline and now >= self._action_deadline:
            self._action_index += 1
            self._action_deadline = 0

        if self._action_index >= len(self._action_sequence):
            self._clear_action_sequence()
            return False

        left_speed, right_speed, duration = self._action_sequence[self._action_index]
        self.motion_controller.move_cmd(left_speed, right_speed)
        if not self._action_deadline:
            self._action_deadline = now + max(duration, self.CONTROL_DT)
        return True

    def _run_blocking_recovery(self, state, label, action, reason=""):
        self._clear_action_sequence()
        self._action_label = label
        self._set_state(state, reason)
        action()
        self._clear_action_sequence()
        self._set_state(self.STATE_SEARCH, "recovery done")

    def _ensure_sequence(self, state, label, steps, reason=""):
        if self.state != state or self._action_label != label or not self._action_sequence:
            self._start_action_sequence(state, label, steps, reason)
        return self._run_action_sequence()

    def stop_match(self):
        print("Stopping match controller...")
        self._match_running = False
        self.camera_activate = False
        self._clear_action_sequence()
        self._publish_status(force=True)
        try:
            self.motion_controller.move_cmd(0, 0)
        except Exception as exc:
            print(f"motor stop failed: {exc}")
        try:
            cv2.destroyAllWindows()
        except Exception as exc:
            print(f"opencv cleanup failed: {exc}")

    def start_match(self):
        freeSpeed = 600
        enemySpeed = 800
        turn = 0.7
        turn_180 = 1.2

        self._match_running = True
        try:
            self.motion_controller.default_platform()
            self._set_state(self.STATE_SEARCH, "match start")
            time.sleep(1)

            while self._match_running:
                stage = self.paltform_detect()
                print(f"Stage:{stage}")
                if stage != self._last_stage:
                    self._clear_action_sequence()
                    self._last_stage = stage

                if stage == 0:
                    self._handle_off_platform(freeSpeed, turn)
                elif stage == 1:
                    self._handle_on_platform(freeSpeed, enemySpeed, turn, turn_180)
                elif stage == 2:
                    self._handle_slip(freeSpeed)
                else:
                    self._clear_action_sequence()
                    self._set_state(self.STATE_SEARCH, f"unknown stage {stage}")
                    self.motion_controller.move_cmd(0, 0)

                self._publish_status()
                time.sleep(self.CONTROL_DT)
        except KeyboardInterrupt:
            print("KeyboardInterrupt received.")
        finally:
            self.stop_match()


match_demo = Match_demo()


if __name__ == "__main__":
    match_demo.start_match()
