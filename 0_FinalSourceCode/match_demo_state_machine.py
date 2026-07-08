import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uptech import UpTech
from motion_controller import MotionController

import apriltag
import json
import threading
import time
import cv2

"""
传感器说明
上方四个红外光电检测擂台边缘
中间四个红外测距检测台下朝向
底部四个红外光电检测能量块与对方机器
底部两个灰度传感器用于台上台下判断
传感器安装位置均从机器人上方观察来定位
"""

"""
机器人上方四个红外光电传感器接口说明：
左前方：IO通道4
右前方：IO通道5
右后方：IO通道6
左后方：IO通道7
在擂台上IO为0代表红外光电没有检测到边缘（前方有障碍物，低电平）
在擂台上IO为1代表红外光电检测到了边缘（前方没有障碍物，高电平）

机器人中间的四个红外测距传感器接口说明：
前方：AD通道0
右方：AD通道1
后方：AD通道2
左方：AD通道3
在擂台下AD大于临界值代表红外测距检测到了围栏

机器人底部的四个红外光电传感器接口说明：
前方：IO通道0
右方：IO通道1
后方：IO通道2
左方：IO通道3


机器人底部的两个灰度传感器接口说明：
前方：AD通道4
后方：AD通道5
"""



class Match_demo:


    FD = 350 #前方测距检测围栏临界值
    RD = 250 #右方测距检测围栏临界值
    BD = 280 #后方测距检测围栏临界值
    LD = 200 #左方测距检测围栏临界值
    
    na = 0  # 倾斜计时
    nd = 0  # 前搁浅计时
    ne = 8  # 后搁浅计时

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

        options = apriltag.DetectorOptions(families='tag36h11')
        self.tag_detector = apriltag.Detector(options) 
        
        #开启Tag识别这里用了多线程
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
        apriltag_detect = threading.Thread(target = self.apriltag_detect_thread)
        apriltag_detect.setDaemon(True)
        apriltag_detect.start()
        self._publish_status(force=True)

    def apriltag_detect_thread(self):
        print("detect start")
        self.camera_activate = True
        VideoCaptureIndex = 0
        while self.camera_activate:
            try:
                cap = cv2.VideoCapture(VideoCaptureIndex)
                if not cap.isOpened():
                    VideoCaptureIndex = 1
                    raise RuntimeError (f"attempt to connect camera")
                else:
                    self.camera_activate = True 
                    print("camera connected succesfully")
            except RuntimeError as e:
                print(f"cannot connect camera {VideoCaptureIndex}")

            while self.camera_activate:
                
                weight = 320
                height = 240
                # x坐标为40  
                cup_w = (int)((640 - weight) / 2) -120 
                # y坐标为80
                cup_h = (int)((480 - height) / 2) -40 

                try:             
                    ret, frame = cap.read()
                    if not ret:
                        raise RuntimeError("frame read failed")
                    else:
                        frame= cv2.resize(frame, (640, 480))
                        #进行截取，截取图像左上角坐标为（40,80），视频宽440，高500
                        frame1 = frame[cup_h:cup_h +440,cup_w:cup_w + 500]
                        result = frame1.copy()

                        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY) 
                        
                        # 识别的主函数
                        apriltag_detect_results = self.tag_detector.detect(gray) 
                        
                        # 没识别到tag,id设定为-1
                        if(len(apriltag_detect_results) == 0):
                            self.tag_id = -1
                        
                        # 画出识别结果
                        for tag in apriltag_detect_results:
                            # 将识别到的 tag 的 id 赋值给类做判断
                            self.tag_id = tag.tag_id
                            print("tag_id = {}".format(tag.tag_id))
                            #test_output
                            if self.tag_id != 2 :
                                # 前方有敌人且不是己方物块需要推，返回1
                                print("前方有敌人且不是己方物块需要推，返回1") 
                            else:
                                #自家物块，需要绕
                                print("自家物块，需要绕") 
                            cv2.circle(result, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2) # left-top
                            cv2.circle(result, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2) # right-top
                            cv2.circle(result, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2) # right-bottom
                            cv2.circle(result, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2) # left-bottom
                        cv2.imshow("result", result)
                        #cv2.imshow("frame", frame)   
                        if cv2.waitKey(1) & 0xff == ord('q'):
                            self.camera_activate=False
                            break
                                
                except Exception as e:
                    print("camera error,try to connect camera...")                    
                    cap.release()
                    break  
            
        cap.release()
        cv2.destroyAllWindows()

        # 检测是否在台上-返回状态
    def paltform_detect(self):
        #前方灰度
        ad_4 = self.uptech.ADC_Get_Channel(4)
        #后方灰度
        ad_5 = self.uptech.ADC_Get_Channel(5)
        if ad_4 + ad_5 > 7000 :
            # 灰度值较大在台下
            return 0
        elif (ad_4 <= 3500) != (ad_5 <= 3500):
            # 搁浅状态
            return 2
        else:
            # 灰度值较小在台上
            return 1
      
    # 台下的位置检测判定，底部红外光电可以检测擂台和围栏，上方红外测距可以检测围栏
    # 红外测距读数越小，意味着距离越大    
    def fence_detect(self):
        # 底部前方红外光电
        io_0 = self.uptech.ADC_IO_GetInputLevel(0) 
        # 底部右侧红外光电
        io_1 = self.uptech.ADC_IO_GetInputLevel(1) 
        # 底部后方红外光电
        io_2 = self.uptech.ADC_IO_GetInputLevel(2) 
        # 底部左侧红外光电
        io_3 = self.uptech.ADC_IO_GetInputLevel(3) 

        # 前红外测距传感器
        ad_0 = self.uptech.ADC_Get_Channel(0)
        # 右红外测距传感器
        ad_1 = self.uptech.ADC_Get_Channel(1)
        # 后红外测距传感器
        ad_2 = self.uptech.ADC_Get_Channel(2)
        # 左红外测距传感器
        ad_3 = self.uptech.ADC_Get_Channel(3)


        # ----------------------对擂台，一个测距检测到--------------------
        if io_2 ==0 and io_1 ==1 and io_3 ==1 and ad_0 > self.FD and ad_1 < self.RD and ad_2 < self.BD and ad_3 < self.LD:
            # 在台下，后方对擂台
            return 1
        if io_3 ==0 and io_0 ==1 and io_2 ==1 and ad_0 < self.FD and ad_1 > self.RD and ad_2 < self.BD and ad_3 < self.LD:
            # 在台下，左侧对擂台
            return 2
        if io_0 ==0 and io_1 ==1 and io_3 ==1 and ad_0 < self.FD and ad_1 < self.RD and ad_2 > self.BD and ad_3 < self.LD:
            # 在台下，前方对擂台
            return 3
        if io_1 ==0 and io_0 ==1 and io_2 ==1 and ad_0 < self.FD and ad_1 < self.RD and ad_2 < self.BD and ad_3 > self.LD:
            # 在台下，右侧对擂台
            return 4

        # ------------------------对围栏，两个相邻测距检测到-------------
        if io_1 ==1 and io_2 ==1 and ad_0 > self.FD and ad_1 < self.RD and ad_2 < self.BD and ad_3 > self.LD:
            # 右和后的红外光电都没有检测到擂台，前左的红外测距检测到近度大于阈值，后右检测到近度小于阈值，前和左侧方向对围栏
            return 5

        if io_2 ==1 and io_3 ==1 and ad_0 > self.FD and ad_1 > self.RD and ad_2 < self.BD and ad_3 < self.LD:
            # 左和后的红外光电没有检测到擂台，前右的红外测距检测到近度大于阈值，左后检测到近度小于阈值，前侧和右侧方向对围栏
            return 6
        if io_0 ==1 and io_3 ==1 and ad_0 < self.FD and ad_1 > self.RD and ad_2 > self.BD and ad_3 < self.LD:
            # 后侧和右侧对围栏
            return 7
        if io_0 ==1 and io_1 ==1 and ad_0 < self.FD and ad_1 < self.RD and ad_2 > self.BD and ad_3 > self.LD:
            # 后侧和左侧对围栏
            return 8

        # --------------------------台上有敌人，两个相对测距检测到-----------
        if ad_0 > self.FD and ad_1 < self.RD and ad_2 > self.BD and ad_3 < self.LD:
            # 在台下，前方或后方有台上敌人
            return 9
        if ad_0 < self.FD and ad_1 > self.RD and ad_2 < self.BD and ad_3 > self.LD:
            # 在台下，左侧或右侧由台上敌人
            return 10

        # -------------------------三侧有障碍，三个测距检测到---------------
        if ad_0 > self.FD and ad_1 > self.RD and ad_2 < self.BD and ad_3 > self.LD:
            # 在台下，前方、左侧和右侧检测到围栏
            return 11
        if ad_0 > self.FD and ad_1 > self.RD and ad_2 > self.BD and ad_3 < self.LD:
            # 在台下，前方、右侧和后方检测到围栏
            return 12
        if ad_0 > self.FD and ad_1 < self.RD and ad_2 > self.BD and ad_3 > self.LD:
            # 在台下，前方、左侧和后方检测到围栏
            return 13
        if ad_0 < self.FD and ad_1 > self.RD and ad_2 > self.BD and ad_3 > self.LD:
            # 在台下，右侧、左侧和后方检测到围栏
            return 14

        # -----------------------斜对擂台，两个红外光电检测到----------------
        if io_0 ==0 and io_1 ==0 and ad_0 < self.FD and ad_1 < self.RD:
            # 在台下，前方和右侧对擂台其他传感器没检测到
            return 15
        if io_0 ==0 and io_3 ==0 and ad_0 < self.FD and ad_3 < self.LD:
            # 在台下，在台下，前方和左侧对擂台其他传感器没检测到
            return 16
        if io_1 ==0 and io_2 ==0 and ad_1 < self.FD and ad_2 < self.RD:
            # 在台下，后方和右侧对擂台其他传感器没检测到
            return 17
        if io_2 ==0 and io_3 ==0 and ad_2 < self.FD and ad_3 < self.LD:
            # 在台下，后方和左侧对擂台其他传感器没检测到
            return 18
        else:
            return 101
    # 边缘检测
    def edge_detect(self):
        io_4 =self.uptech.ADC_IO_GetInputLevel(4) #左前
        io_5 =self.uptech.ADC_IO_GetInputLevel(5) #右前
        io_6 =self.uptech.ADC_IO_GetInputLevel(6) #右后
        io_7 =self.uptech.ADC_IO_GetInputLevel(7) #左后

        if io_4 == 0 and io_5 == 0 and io_6 == 0 and io_7 == 0:
            # 四个红外光电都没有检测到边缘,离擂台边缘都很远
            return 0
        elif io_4 == 1 and io_5 == 0 and io_6 == 0 and io_7 == 0:
            # 左前检测到边缘
            return 1
        elif io_4 == 0 and io_5 == 1 and io_6 == 0 and io_7 == 0:
            # 右前检测到边缘
            return 2
        elif io_4 == 0 and io_5 == 0 and io_6 == 1 and io_7 == 0:
            # 右后检测到边缘
            return 3
        elif io_4 == 0 and io_5 == 0 and io_6 == 0 and io_7 == 1:
            # 左后检测到边缘
            return 4
        elif io_4 == 1 and io_5 == 1 and io_6 == 0 and io_7 == 0:
            # 前方两个检测到边缘
            return 5
        elif io_4 == 0 and io_5 == 0 and io_6 == 1 and io_7 == 1:
            # 后方两个检测到边缘
            return 6
        elif io_4 == 1 and io_5 == 0 and io_6 == 0 and io_7 == 1:
            # 左侧两个检测到边缘
            return 7
        elif io_4 == 0 and io_5 == 1 and io_6 == 1 and io_7 == 0:
            # 右侧两个检测到边缘
            return 8  


        else:
            return 102            
    # 敌人检测
    # 反馈前方红外光电是否检测到敌人或物块,检测到敌人或物块，反馈低电平，没检测到敌人或物块反馈高电平
    # 检测到为1，没检测到为0
    def enemy_detect(self):
        # 底部前方红外光电
        io_0 = self.uptech.ADC_IO_GetInputLevel(0)
        # 底部右侧红外光电
        io_1 = self.uptech.ADC_IO_GetInputLevel(1)
        # 底部后方红外光电
        io_2 = self.uptech.ADC_IO_GetInputLevel(2)
        # 底部左侧红外光电
        io_3 = self.uptech.ADC_IO_GetInputLevel(3)
        
        # 前方测距
        ad_0 = self.uptech.ADC_Get_Channel(0)
        
        # 四路红外光电都是空的，没有检测到敌人
        if io_0==1 and io_1==1 and io_2==1 and io_3==1:
            # 无敌人
            return 0
        
        # 前方红外光电感应，其他方向没感应
        elif io_0==0 :
            #默认己方物块信息为2，若为其他物块修改下方tag_id
            if self.tag_id != 2 :                    
                if ad_0 < 1000:
                      # 前方有敌人且不是己方物块需要推，返回1
                    print("前方有敌人且不是己方物块需要推，返回11111")
                    return 1               
                else:
                 # 已经非常接近目标，加速撞击，将对方推下擂台
                    return 11 
            else:
                #自家物块，需要绕
                print("自家物块，需要绕")
                return 5
                           
        elif  io_1==0 :
            # 右侧有敌人或能量块
            return 2
        
        elif  io_2==0 :
            # 后方有敌人或能量块
            return 3
        
        elif  io_3==0:
            # 左侧有敌人或能量块
            return 4
        
        else:
            # 其他情况
            return 103
    
    def slip_detect(self):
        io_4 = self.uptech.ADC_IO_GetInputLevel(4) #左前
        io_5 = self.uptech.ADC_IO_GetInputLevel(5) #右前
        io_6 = self.uptech.ADC_IO_GetInputLevel(6) #右后
        io_7 = self.uptech.ADC_IO_GetInputLevel(7) #左后

        ad_4 = self.uptech.ADC_Get_Channel(4) # 前方灰度
        ad_5 = self.uptech.ADC_Get_Channel(5) # 后方灰度

        # 左侧在台下
        if io_4 == 1 and io_7 == 1 and io_5 == 0 and io_6 == 0:
            return 0
        # 右侧在台下
        elif io_5 == 1 and io_6 == 1 and io_4 == 0 and io_7 == 0:
            return 1
        # 前侧在台下
        elif ad_5 < 3500:
            return 2
        # 后侧在台下
        elif ad_4 < 3500:
            return 3
        else:
            return 105

    def _set_state(self, state, reason=""):
        # 状态变化统一从这里打印，调试时可以直接观察状态跳转链路。
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
        # 清掉尚未完成的分段动作，供高优先级事件抢占当前动作。
        self._action_sequence = []
        self._action_index = 0
        self._action_deadline = 0
        self._action_label = ""

    def _start_action_sequence(self, state, label, steps, reason=""):
        # steps 中每一项为 (left_speed, right_speed, duration)，由主循环逐段执行。
        self._action_sequence = list(steps)
        self._action_index = 0
        self._action_deadline = 0
        self._action_label = label
        self._set_state(state, reason)

    def _run_action_sequence(self):
        # 每轮只执行当前小步，不阻塞主循环，方便边缘/正前目标随时抢占。
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

    def _left_align_ready(self):
        # 左侧对擂台时持续左转，直到前方传感器满足登台朝向。
        io_0 = self.uptech.ADC_IO_GetInputLevel(0)
        io_3 = self.uptech.ADC_IO_GetInputLevel(3)
        ad_1 = self.uptech.ADC_Get_Channel(1)
        return io_0 == 0 and ad_1 < self.RD and io_3 == 1

    def _right_align_ready(self):
        # 右侧对擂台时持续右转，直到前方传感器满足登台朝向。
        io_0 = self.uptech.ADC_IO_GetInputLevel(0)
        io_1 = self.uptech.ADC_IO_GetInputLevel(1)
        ad_3 = self.uptech.ADC_Get_Channel(3)
        return io_0 == 0 and ad_3 < self.LD and io_1 == 1

    def _handle_left_fence_align(self, freeSpeed):
        self._clear_action_sequence()
        self._set_state(self.STATE_FENCE_ALIGN_LEFT, "align platform")
        if self._left_align_ready():
            self._start_action_sequence(
                self.STATE_FENCE_RECOVER,
                "align-left-approach",
                [(freeSpeed, freeSpeed, 0.3)],
                "left align ready",
            )
            self._run_action_sequence()
        else:
            self.motion_controller.move_cmd(-freeSpeed + 50, freeSpeed - 50)

    def _handle_right_fence_align(self, freeSpeed):
        self._clear_action_sequence()
        self._set_state(self.STATE_FENCE_ALIGN_RIGHT, "align platform")
        if self._right_align_ready():
            self._start_action_sequence(
                self.STATE_FENCE_RECOVER,
                "align-right-approach",
                [(freeSpeed, freeSpeed, 0.3)],
                "right align ready",
            )
            self._run_action_sequence()
        else:
            self.motion_controller.move_cmd(freeSpeed - 50, -freeSpeed + 50)

    def _handle_off_platform(self, freeSpeed, turn):
        # 台下优先解决朝向和登台；可分段的调整动作通过 action_sequence 调度。
        fence = self.fence_detect()
        if fence != 101:
            self._stun_time = 0

        if fence == 1:
            self._run_blocking_recovery(
                self.STATE_CLIMB,
                "climb-behind",
                self.motion_controller.go_up_behind_platform,
                "behind platform",
            )
            return
        if fence == 3:
            self._run_blocking_recovery(
                self.STATE_CLIMB,
                "climb-ahead",
                self.motion_controller.go_up_ahead_platform,
                "ahead platform",
            )
            return

        if self.state == self.STATE_FENCE_ALIGN_LEFT:
            self._handle_left_fence_align(freeSpeed)
            return
        if self.state == self.STATE_FENCE_ALIGN_RIGHT:
            self._handle_right_fence_align(freeSpeed)
            return

        if self._run_action_sequence():
            return

        if fence in (2, 16, 18):
            self._handle_left_fence_align(freeSpeed)
        elif fence in (4, 15, 17):
            self._handle_right_fence_align(freeSpeed)
        elif fence in (5, 6):
            self._ensure_sequence(
                self.STATE_FENCE_RECOVER,
                f"fence:{fence}",
                [(-freeSpeed, -freeSpeed, 0.4)],
                "front fence",
            )
        elif fence in (7, 8, 10):
            self._ensure_sequence(
                self.STATE_FENCE_RECOVER,
                f"fence:{fence}",
                [(freeSpeed, freeSpeed, 0.4)],
                "back fence or side enemy",
            )
        elif fence == 9:
            self._ensure_sequence(
                self.STATE_FENCE_RECOVER,
                "fence:9",
                [(freeSpeed, -freeSpeed, turn), (freeSpeed, freeSpeed, 0.4)],
                "front/back target below platform",
            )
        elif fence == 11:
            self._ensure_sequence(
                self.STATE_FENCE_RECOVER,
                "fence:11",
                [(-freeSpeed, -freeSpeed, 0.5), (-freeSpeed, freeSpeed, turn)],
                "three-side fence",
            )
        elif fence == 12:
            self._ensure_sequence(
                self.STATE_FENCE_RECOVER,
                "fence:12",
                [(300, 600, turn)],
                "front-right-back fence",
            )
        elif fence == 13:
            self._ensure_sequence(
                self.STATE_FENCE_RECOVER,
                "fence:13",
                [(600, 300, turn)],
                "front-left-back fence",
            )
        elif fence == 14:
            self._ensure_sequence(
                self.STATE_FENCE_RECOVER,
                "fence:14",
                [(-freeSpeed, freeSpeed, 0.2), (freeSpeed, freeSpeed, 0.3)],
                "side-back fence",
            )
        elif fence == 101:
            self._set_state(self.STATE_FENCE_RECOVER, "search platform")
            self.motion_controller.move_cmd(freeSpeed, -freeSpeed)
            self._stun_time += self.CONTROL_DT
            if self._stun_time > 3.3:
                self._stun_time = 0
                self._start_action_sequence(
                    self.STATE_FENCE_RECOVER,
                    "fence:unstick",
                    [(freeSpeed, freeSpeed, 0.5)],
                    "unstick platform search",
                )
        else:
            self._set_state(self.STATE_FENCE_RECOVER, f"unknown fence {fence}")
            self.motion_controller.move_cmd(freeSpeed, -freeSpeed)

    def _edge_recover_steps(self, edge, freeSpeed, turn):
        # 台上边缘恢复动作表，尽量保留原先实测速度与持续时间。
        edge_steps = {
            1: [(-freeSpeed, -freeSpeed, 0.8), (freeSpeed, -freeSpeed, turn)],
            2: [(-freeSpeed, -freeSpeed, 0.8), (-freeSpeed, freeSpeed, turn)],
            3: [(freeSpeed, freeSpeed, 0.8), (-freeSpeed, freeSpeed, turn)],
            4: [(freeSpeed, freeSpeed, 0.8), (freeSpeed, -freeSpeed, turn)],
            5: [(-freeSpeed, -freeSpeed, 0.8), (freeSpeed, -freeSpeed, turn)],
            6: [(freeSpeed, freeSpeed, 0.5)],
            7: [(freeSpeed + 100, -freeSpeed, 0.5), (freeSpeed, freeSpeed, 0.3)],
            8: [(-freeSpeed, freeSpeed + 100, 0.5), (freeSpeed, freeSpeed, 0.3)],
            102: [(freeSpeed, -freeSpeed, self.CONTROL_DT)],
        }
        return edge_steps.get(edge)

    def _handle_edge_recover(self, edge, freeSpeed, turn):
        # 边缘和搁浅恢复优先级高于攻击，避免机器人为了推目标跌落。
        self._stun_time = 0

        steps = self._edge_recover_steps(edge, freeSpeed, turn)
        if steps is None:
            steps = [(freeSpeed, -freeSpeed, self.CONTROL_DT)]
        self._ensure_sequence(
            self.STATE_EDGE_RECOVER,
            f"edge:{edge}",
            steps,
            f"edge {edge}",
        )

    def _handle_on_platform(self, freeSpeed, enemySpeed, turn, turn_180):
        # 台上优先级：防跌落 -> 正前方可推目标 -> 己方块避让 -> 转向/巡航。
        edge = self.edge_detect()
        if edge != 0:
            self._handle_edge_recover(edge, freeSpeed, turn)
            return

        enemy = self.enemy_detect()
        if enemy in (1, 11):
            self._clear_action_sequence()
            speed = enemySpeed if enemy == 11 else 700
            self._set_state(self.STATE_ATTACK, "front target")
            self.motion_controller.move_cmd(speed, speed)
            return

        if enemy == 5:
            self._ensure_sequence(
                self.STATE_AVOID_OWN_BLOCK,
                "avoid-own-block",
                [
                    (-freeSpeed, -freeSpeed, 0.5),
                    (-freeSpeed, freeSpeed, turn_180),
                    (freeSpeed, freeSpeed, 0.5),
                ],
                "own block",
            )
            return

        if self._run_action_sequence():
            return

        if enemy == 0:
            self._set_state(self.STATE_SEARCH, "patrol")
            self.motion_controller.move_cmd(freeSpeed, freeSpeed)
        elif enemy == 2:
            self._ensure_sequence(
                self.STATE_TURN_TO_TARGET,
                "target-right",
                [(-freeSpeed, -freeSpeed, 0.3), (freeSpeed, -freeSpeed, 0.5)],
                "target right",
            )
        elif enemy == 3:
            self._ensure_sequence(
                self.STATE_TURN_TO_TARGET,
                "target-back",
                [(freeSpeed, -freeSpeed, turn_180)],
                "target back",
            )
        elif enemy == 4:
            self._ensure_sequence(
                self.STATE_TURN_TO_TARGET,
                "target-left",
                [(-freeSpeed, -freeSpeed, 0.3), (-freeSpeed, freeSpeed, 0.3)],
                "target left",
            )
        else:
            self._set_state(self.STATE_SEARCH, f"unknown enemy {enemy}")
            self.motion_controller.move_cmd(freeSpeed, freeSpeed)

    def _handle_slip(self, freeSpeed):
        # 半边在台上/台下时仍调用原有搁浅动作，这些动作包含舵机时序。
        self._clear_action_sequence()
        slip = self.slip_detect()
        if slip == 0:
            self._run_blocking_recovery(
                self.STATE_SLIP_RECOVER,
                "slip-right",
                self.motion_controller.slip_right,
                "left side slipped",
            )
        elif slip == 1:
            self._run_blocking_recovery(
                self.STATE_SLIP_RECOVER,
                "slip-left",
                self.motion_controller.slip_left,
                "right side slipped",
            )
        elif slip == 2:
            self._run_blocking_recovery(
                self.STATE_SLIP_RECOVER,
                "slip-back",
                self.motion_controller.slip_back,
                "front side slipped",
            )
        elif slip == 3:
            self._run_blocking_recovery(
                self.STATE_SLIP_RECOVER,
                "slip-front",
                self.motion_controller.slip_front,
                "back side slipped",
            )
        else:
            self._set_state(self.STATE_SLIP_RECOVER, f"unknown slip {slip}")
            self.motion_controller.move_cmd(freeSpeed, freeSpeed)

    def stop_match(self):
        # Ctrl+C 或异常退出时统一停机，避免电机保持最后一次速度指令。
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
        '''
        State-machine match loop. Long motor-only maneuvers are broken into
        short timed steps so edge and front-target events can preempt turns.
        '''
        freeSpeed = 550
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
                    # 场地状态切换时丢弃旧动作，防止台下动作带到台上继续执行。
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

if __name__ == '__main__':
    match_demo.start_match()

"""
    while True:
        # 底部右侧红外光电
        io_1 = match_demo.uptech.ADC_IO_GetInputLevel(1)
        # 底部左侧红外光电
        io_3 = match_demo.uptech.ADC_IO_GetInputLevel(3)
        #软启动，同时遮挡底盘左右两侧红外光电传感器触发电机运动
        if io_1 ==0 and io_3 ==0 :
            match_demo.motion_controller.go_up_ahead_platform()
            break
        else :
            time.sleep(0.1)
        # match_demo.stop()
    match_demo.start_match()
"""
