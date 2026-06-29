import os
import sys
import time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

'''
本程序用于测试硬件连接
'''

from uptech import UpTech
from motion_controller import MotionController

uptech = UpTech()
motion_controller = MotionController()

#测试传感器
uptech.ADC_IO_Open()
for i in range(0, 8):
    uptech.ADC_IO_SetIOMode(i, 0)
    print(uptech.ADC_IO_GetInputLevel(i))
for i in range(0,6):
    print(uptech.ADC_Get_Channel(i))
uptech.ADC_IO_Close()

#测试舵机
motion_controller.default_platform()
time.sleep(0.5)
motion_controller.dance_routine()

#测试电机
motion_controller.move_cmd(300, 300)
input('按Enter停止')
motion_controller.move_cmd(0, 0)









