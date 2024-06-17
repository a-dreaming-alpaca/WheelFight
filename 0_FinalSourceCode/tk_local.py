import os
import sys
import time
import tkinter as tk
from tkinter import ttk

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from motion_controller import MotionController


class RemoteControlApp:
    def __init__(self):
        self.controller = MotionController()
        try:
            self.controller.uptech.ADC_IO_Open()
        except Exception as exc:
            print("Warning: ADC_IO_Open failed:", exc)

        self.root = tk.Tk()
        self.root.title("WheelFight Remote Control")
        self.root.geometry("360x340")
        self.root.resizable(False, False)

        self.fence_state_label = None
        self.sensor_raw_label = None
        self.last_fence_code = None

        self._build_ui()
        self._bind_keys()
        self._update_fence_state()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _build_ui(self):
        style = ttk.Style(self.root)
        style.configure("TButton", font=(None, 12), padding=8)
        style.configure("TLabel", font=(None, 11))

        control_frame = ttk.LabelFrame(self.root, text="运动控制")
        control_frame.place(x=20, y=20, width=320, height=170)

        forward_btn = ttk.Button(control_frame, text="前进", command=self.move_forward)
        forward_btn.place(x=120, y=10, width=80)

        left_btn = ttk.Button(control_frame, text="左转", command=self.turn_left)
        left_btn.place(x=20, y=60, width=80)

        stop_btn = ttk.Button(control_frame, text="停止", command=self.stop_motion)
        stop_btn.place(x=120, y=60, width=80)

        right_btn = ttk.Button(control_frame, text="右转", command=self.turn_right)
        right_btn.place(x=220, y=60, width=80)

        backward_btn = ttk.Button(control_frame, text="后退", command=self.move_backward)
        backward_btn.place(x=120, y=110, width=80)

        platform_frame = ttk.LabelFrame(self.root, text="上台模式")
        platform_frame.place(x=20, y=200, width=320, height=100)

        ahead_btn = ttk.Button(platform_frame, text="前上台", command=self.enter_platform_ahead)
        ahead_btn.place(x=30, y=20, width=110)

        behind_btn = ttk.Button(platform_frame, text="后上台", command=self.enter_platform_behind)
        behind_btn.place(x=180, y=20, width=110)

        status_frame = ttk.LabelFrame(self.root, text="围栏状态")
        status_frame.place(x=20, y=310, width=320, height=120)

        self.fence_state_label = ttk.Label(status_frame, text="Fence code: --", anchor="w")
        self.fence_state_label.place(x=10, y=10)

        self.sensor_raw_label = ttk.Label(status_frame, text="Sensor raw: --", anchor="w")
        self.sensor_raw_label.place(x=10, y=40)

    def _bind_keys(self):
        self.root.bind("<Up>", lambda event: self.move_forward())
        self.root.bind("<Down>", lambda event: self.move_backward())
        self.root.bind("<Left>", lambda event: self.turn_left())
        self.root.bind("<Right>", lambda event: self.turn_right())
        self.root.bind("<space>", lambda event: self.stop_motion())

    def move_forward(self):
        print("遥控：前进")
        self.controller.move_cmd(400, 400)

    def move_backward(self):
        print("遥控：后退")
        self.controller.move_cmd(-400, -400)

    def turn_left(self):
        print("遥控：左转")
        self.controller.move_cmd(-400, 400)

    def turn_right(self):
        print("遥控：右转")
        self.controller.move_cmd(400, -400)

    def stop_motion(self):
        print("遥控：停止")
        self.controller.move_cmd(0, 0)

    def enter_platform_ahead(self):
        print("进入前上台模式")
        self.stop_motion()
        self.controller.go_up_ahead_platform()

    def enter_platform_behind(self):
        print("进入后上台模式")
        self.stop_motion()
        self.controller.go_up_behind_platform()

    def _update_fence_state(self):
        fence_code, raw_text = self.fence_detect()
        self.sensor_raw_label.config(text=raw_text)
        if fence_code != self.last_fence_code:
            self.last_fence_code = fence_code
            self.fence_state_label.config(text=f"Fence code: {fence_code}")
            print(f"Fence state changed: {fence_code} | {raw_text}")
        self.root.after(1000, self._update_fence_state)

    def fence_detect(self):
        try:
            io_0 = self.controller.uptech.ADC_IO_GetInputLevel(0)
            io_1 = self.controller.uptech.ADC_IO_GetInputLevel(1)
            io_2 = self.controller.uptech.ADC_IO_GetInputLevel(2)
            io_3 = self.controller.uptech.ADC_IO_GetInputLevel(3)

            ad_0 = self.controller.uptech.ADC_Get_Channel(0)
            ad_1 = self.controller.uptech.ADC_Get_Channel(1)
            ad_2 = self.controller.uptech.ADC_Get_Channel(2)
            ad_3 = self.controller.uptech.ADC_Get_Channel(3)
        except Exception as exc:
            print("Fence detect read failed:", exc)
            return -1, "sensor read failed"

        raw_text = (
            f"IO[0-3]={io_0},{io_1},{io_2},{io_3} "
            f"AD[0-3]={ad_0},{ad_1},{ad_2},{ad_3}"
        )

        FD = 260
        RD = 350
        BD = 450
        LD = 350

        if io_2 == 0 and io_1 == 1 and io_3 == 1 and ad_0 > FD and ad_1 < RD and ad_2 < BD and ad_3 < LD:
            return 1, raw_text
        if io_3 == 0 and io_0 == 1 and io_2 == 1 and ad_0 < FD and ad_1 > RD and ad_2 < BD and ad_3 < LD:
            return 2, raw_text
        if io_0 == 0 and io_1 == 1 and io_3 == 1 and ad_0 < FD and ad_1 < RD and ad_2 > BD and ad_3 < LD:
            return 3, raw_text
        if io_1 == 0 and io_0 == 1 and io_2 == 1 and ad_0 < FD and ad_1 < RD and ad_2 < BD and ad_3 > LD:
            return 4, raw_text
        if io_1 == 1 and io_2 == 1 and ad_0 > FD and ad_1 < RD and ad_2 < BD and ad_3 > LD:
            return 5, raw_text
        if io_2 == 1 and io_3 == 1 and ad_0 > FD and ad_1 > RD and ad_2 < BD and ad_3 < LD:
            return 6, raw_text
        if io_0 == 1 and io_3 == 1 and ad_0 < FD and ad_1 > RD and ad_2 > BD and ad_3 < LD:
            return 7, raw_text
        if io_0 == 1 and io_1 == 1 and ad_0 < FD and ad_1 < RD and ad_2 > BD and ad_3 > LD:
            return 8, raw_text
        if ad_0 > FD and ad_1 < RD and ad_2 > BD and ad_3 < LD:
            return 9, raw_text
        if ad_0 < FD and ad_1 > RD and ad_2 < BD and ad_3 > LD:
            return 10, raw_text
        if ad_0 > FD and ad_1 > RD and ad_2 < BD and ad_3 > LD:
            return 11, raw_text
        if ad_0 > FD and ad_1 > RD and ad_2 > BD and ad_3 < LD:
            return 12, raw_text
        if ad_0 > FD and ad_1 < RD and ad_2 > BD and ad_3 > LD:
            return 13, raw_text
        if ad_0 < FD and ad_1 > RD and ad_2 > BD and ad_3 > LD:
            return 14, raw_text
        if io_0 == 0 and io_1 == 0 and ad_0 < FD and ad_1 < RD:
            return 15, raw_text
        if io_0 == 0 and io_3 == 0 and ad_0 < FD and ad_3 < LD:
            return 16, raw_text
        if io_1 == 0 and io_2 == 0 and ad_1 < RD and ad_2 < BD:
            return 17, raw_text
        if io_2 == 0 and io_3 == 0 and ad_2 < BD and ad_3 < LD:
            return 18, raw_text

        return 101, raw_text

    def _on_close(self):
        print("关闭遥控界面，停止小车")
        try:
            self.stop_motion()
            self.controller.uptech.CDS_Close()
        except Exception as exc:
            print("关闭时发生错误：", exc)
        self.root.destroy()


if __name__ == "__main__":
    RemoteControlApp()
