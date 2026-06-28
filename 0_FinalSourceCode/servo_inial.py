import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uptech import UpTech
uptech = UpTech()

'''
本程序用于初始化舵机安装位置
'''

uptech.CDS_Open()

for i in range(5, 9):
    uptech.CDS_SetMode(i, 0)

uptech.CDS_SetAngle(5, 424, 300)
uptech.CDS_SetAngle(6, 600, 300)
uptech.CDS_SetAngle(7, 600, 300)
uptech.CDS_SetAngle(8, 424, 300)

uptech.CDS_Close()