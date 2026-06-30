import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


from uptech import UpTech
from motion_controller import MotionController

uptech = UpTech()
motion_controller = MotionController()

'''
用于急停
'''

motion_controller.move_cmd(0, 0)