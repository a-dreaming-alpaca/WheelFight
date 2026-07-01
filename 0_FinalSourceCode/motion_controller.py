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
    def move_cmd(self, left_speed = 0, right_speed = 0):
    # 校正参数表
        CALIB_MAP = {
            300: 1.207,
            400: 1.162,
            500: 1.104,
            600: 1.042,
            700: 1.017,
            800: 0.988
        }
        A, B, C, D = 1.037, -1.9325, 1.9772, -0.0848
        sign = 0
        if right_speed < 0:
            sign = -1
        else:
            sign = 1
        if right_speed == 0:
            pass
        elif abs(right_speed) in CALIB_MAP:
            cal = CALIB_MAP[abs(right_speed)]
            right_speed = int(right_speed * cal)
            right_speed = abs(right_speed)
        else:
            s = abs(right_speed / 1000)
            right_speed = int(1000 * (A * s ** 3 + B * s ** 2 + C * s + D))
            
        self.uptech.CDS_SetSpeed(1, left_speed)
        self.uptech.CDS_SetSpeed(2, - sign * right_speed)

    # 默认为前后爪都收起的状态
    def default_platform(self):
        self.uptech.CDS_SetAngle(5, 224, self.servo_speed)
        self.uptech.CDS_SetAngle(6, 800, self.servo_speed)
        self.uptech.CDS_SetAngle(7, 800, self.servo_speed)
        self.uptech.CDS_SetAngle(8, 224, self.servo_speed)
        time.sleep(0.01)
        self.uptech.CDS_SetAngle(5, 224, self.servo_speed)
        self.uptech.CDS_SetAngle(6, 800, self.servo_speed)
        self.uptech.CDS_SetAngle(7, 800, self.servo_speed)
        self.uptech.CDS_SetAngle(8, 224, self.servo_speed)

    # 支撑前爪,发两遍确认发送成功
    def pack_up_ahead(self):
        self.uptech.CDS_SetAngle(5, 750, self.servo_speed)
        self.uptech.CDS_SetAngle(6, 274, self.servo_speed)
        time.sleep(0.01)
        self.uptech.CDS_SetAngle(5, 750, self.servo_speed)
        self.uptech.CDS_SetAngle(6, 274, self.servo_speed)

    # 支撑后爪，发两遍确认发送成功
    def pack_up_behind(self):
        self.uptech.CDS_SetAngle(7, 274, self.servo_speed)
        self.uptech.CDS_SetAngle(8, 750, self.servo_speed)
        time.sleep(0.01)
        self.uptech.CDS_SetAngle(7, 274, self.servo_speed)
        self.uptech.CDS_SetAngle(8, 750, self.servo_speed)

    # 前上台动作
    def go_up_ahead_platform(self):
        self.move_cmd(0, 0)
        time.sleep(0.1)
        # 前后支撑爪抬起
        self.default_platform()
        time.sleep(0.2)
        #前进0.7s，这时前方已经顶住擂台边缘
        self.move_cmd(400, 400)
        time.sleep(1)
        # 支前爪,把前半身撑起来
        self.pack_up_ahead()
        time.sleep(0.7)
        # 收起前爪
        self.default_platform()
        time.sleep(0.2)
        # 支后爪，把后半身撑起来
        self.pack_up_behind()
        time.sleep(0.7)
        # 恢复成默认上台动作
        self.default_platform()
        time.sleep(0.2)
        self.move_cmd(0, 0)
        time.sleep(0.5)

    # 后上台动作
    def go_up_behind_platform(self):
        self.move_cmd(0, 0)
        time.sleep(0.1)
        # 前后支撑爪抬起
        self.default_platform()
        time.sleep(0.2)
        # 后退0.7s，这时后方已经顶住擂台边缘
        self.move_cmd(-400, -400)
        time.sleep(1)
        # 支后爪，把后半身撑起来
        self.pack_up_behind()
        time.sleep(0.7)
        # 收起前爪
        self.default_platform()
        # 支前爪，把前半身撑起来
        time.sleep(0.2)
        self.pack_up_ahead()
        time.sleep(0.7)
        # 默认上台
        self.default_platform()
        time.sleep(0.2)
        self.move_cmd(0, 0)
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