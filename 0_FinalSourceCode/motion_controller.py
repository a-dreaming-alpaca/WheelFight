import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


from uptech import UpTech
import time

class MotionController:
    def __init__(self):
        self.uptech = UpTech()

        self.uptech.CDS_Open()
        time.sleep(0.5)
        self.servo_speed = 800
        self.uptech.CDS_SetMode(3, 0)
        self.uptech.CDS_SetMode(4, 0)
        self.uptech.CDS_SetMode(5, 0)
        self.uptech.CDS_SetMode(6, 0)
        self.uptech.CDS_SetMode(7, 0)  
        self.uptech.CDS_SetMode(8, 0)     
        self.uptech.CDS_SetMode(11, 0)     
        self.uptech.CDS_SetMode(12, 0)  

    # 速度指令,参数分别为左速度和右速度，自由控制-开环控制器
    def move_cmd(self, speed = 0):

        if speed == 0:
            self.uptech.CDS_SetSpeed(1, 0)
            self.uptech.CDS_SetSpeed(2, 0)

        elif abs(speed) == 300:
            right_speed = int(speed * 1.207)
            self.uptech.CDS_SetSpeed(1, speed)
            self.uptech.CDS_SetSpeed(2, -right_speed)

        elif abs(speed) == 400:
            right_speed = int(speed * 1.15)
            self.uptech.CDS_SetSpeed(1, speed)
            self.uptech.CDS_SetSpeed(2, -right_speed)




    def move_cmd_300(self, left_speed, right_speed):
        right_speed = int(right_speed * 1.207)
        self.uptech.CDS_SetSpeed(1, left_speed)
        self.uptech.CDS_SetSpeed(2, -right_speed)
    
    def move_cmd_400(self, left_speed, right_speed):
        right_speed = int(right_speed * 1.15)
        self.uptech.CDS_SetSpeed(1, left_speed)
        self.uptech.CDS_SetSpeed(2, -right_speed)

    # 默认为前后爪都收起的状态
    def default_platform(self):
        self.uptech.CDS_SetAngle(5, 424, self.servo_speed)
        self.uptech.CDS_SetAngle(6, 600, self.servo_speed)
        self.uptech.CDS_SetAngle(7, 600, self.servo_speed)
        self.uptech.CDS_SetAngle(8, 424, self.servo_speed)
        time.sleep(0.01)
        self.uptech.CDS_SetAngle(5, 424, self.servo_speed)
        self.uptech.CDS_SetAngle(6, 600, self.servo_speed)
        self.uptech.CDS_SetAngle(7, 600, self.servo_speed)
        self.uptech.CDS_SetAngle(8, 424, self.servo_speed)

    # 支撑前爪,发两遍确认发送成功
    def pack_up_ahead(self):
        self.uptech.CDS_SetAngle(5, 780, self.servo_speed)
        self.uptech.CDS_SetAngle(6, 244, self.servo_speed)
        time.sleep(0.01)
        self.uptech.CDS_SetAngle(5, 780, self.servo_speed)
        self.uptech.CDS_SetAngle(6, 244, self.servo_speed)

    # 支撑后爪，发两遍确认发送成功
    def pack_up_behind(self):
        self.uptech.CDS_SetAngle(7, 244, self.servo_speed)
        self.uptech.CDS_SetAngle(8, 780, self.servo_speed)
        time.sleep(0.01)
        self.uptech.CDS_SetAngle(7, 244, self.servo_speed)
        self.uptech.CDS_SetAngle(8, 780, self.servo_speed)

    # 前上台动作
    def go_up_ahead_platform(self):
        self.move_cmd(0)
        time.sleep(0.1)
        # 前后支撑爪抬起
        self.default_platform()
        time.sleep(0.4)
        #前进0.7s，这时前方已经顶住擂台边缘
        self.move_cmd(400)
        time.sleep(1.2)
        # 支前爪,把前半身撑起来
        self.pack_up_ahead()
        time.sleep(1)
        # 收起前爪
        self.default_platform()
        time.sleep(0.3)
        # 支后爪，把后半身撑起来
        self.pack_up_behind()
        time.sleep(0.8)
        # 恢复成默认上台动作
        self.default_platform()
        time.sleep(1)
        self.move_cmd(0)
        time.sleep(0.5)

    # 后上台动作
    def go_up_behind_platform(self):
        self.move_cmd(0)
        time.sleep(0.1)
        # 前后支撑爪抬起
        self.default_platform()
        time.sleep(0.4)
        # 后退0.7s，这时后方已经顶住擂台边缘
        self.move_cmd(-400)
        time.sleep(1.2)
        # 支后爪，把后半身撑起来
        self.pack_up_behind()
        time.sleep(1)
        # 收起前爪
        self.default_platform()
        # 支前爪，把前半身撑起来
        time.sleep(0.3)
        self.pack_up_ahead()
        time.sleep(0.8)
        # 默认上台
        self.default_platform()
        time.sleep(1)
        self.move_cmd(0)
        time.sleep(0.5)

    # 我跳舞
    def dance_routine(self, loops=3):
        
        dance_speed = 300 
        
        # 定义基准角度
        base_5, base_8 = 424, 424
        base_6, base_7 = 600, 600
        
        # 定义微调幅度
        offset = 80 

        for _ in range(loops):
            # ---------------- 姿态 A：左前右后抬起，右前左后落下 ----------------
            self.uptech.CDS_SetAngle(5, base_5 + offset, dance_speed) # 5号臂微抬
            self.uptech.CDS_SetAngle(6, base_6 + offset, dance_speed) # 6号臂微落
            self.uptech.CDS_SetAngle(7, base_7 - offset, dance_speed) # 7号臂微抬
            self.uptech.CDS_SetAngle(8, base_8 - offset, dance_speed) # 8号臂微落
            time.sleep(0.6)

            # ---------------- 姿态 B：左前右后落下，右前左后抬起 ----------------
            self.uptech.CDS_SetAngle(5, base_5 - offset, dance_speed) # 5号臂微落
            self.uptech.CDS_SetAngle(6, base_6 - offset, dance_speed) # 6号臂微抬
            self.uptech.CDS_SetAngle(7, base_7 + offset, dance_speed) # 7号臂微落
            self.uptech.CDS_SetAngle(8, base_8 + offset, dance_speed) # 8号臂微抬
            time.sleep(0.6)

        # 恢复到默认收起姿态
        self.default_platform()
        time.sleep(0.5)