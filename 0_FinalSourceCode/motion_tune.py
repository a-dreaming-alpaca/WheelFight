import os
import sys
import time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uptech import UpTech
from motion_controller import MotionController

uptech = UpTech()
motion_controller = MotionController()

#测试向前爬坡
motion_controller.go_up_behind_platform()
'''
motion_controller.move_cmd_400(400, 400)
input()
motion_controller.move_cmd_400(0, 0)

'''
