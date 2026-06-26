import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uptech import UpTech
uptech = UpTech()

uptech.ADC_IO_Open()
for i in range(0, 8):
    uptech.ADC_IO_SetIOMode(i, 0)
    print(uptech.ADC_IO_GetInputLevel(i))
for i in range(0,6):
    print(uptech.ADC_Get_Channel(i))
uptech.ADC_IO_Close()


uptech.CDS_Open()
for i in range(5, 9):
    if not uptech.CDS_SetMode(i, 0):
         print("Servo not right")
         break


def move_cmd(left_speed, right_speed):
        uptech.CDS_SetSpeed(1, left_speed)
        uptech.CDS_SetSpeed(2, -right_speed)
 
move_cmd(300, 300)
input()
move_cmd(0, 0)

uptech.CDS_Close()



