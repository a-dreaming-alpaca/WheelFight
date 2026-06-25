#!/usr/bin/env python3
import time
from uptech import UpTech

# 初始化 UpTech 实例
bot = UpTech()

def init_motor():
    # 对应原 C 代码中初始化 1-10 号舵机模式
    for i in range(1, 10):
        bot.CDS_SetMode(i, bot.CDS_MODE_SERVO)
    
    # 将 1-9 号舵机归位
    for i in range(1, 10):
        bot.CDS_SetAngle(i, 512, 512)
        
    time.sleep(1.0) # 延迟 1000ms

def move_forward(speed1):
    bot.CDS_SetAngle(1, 640, 512)
    bot.CDS_SetAngle(2, 195, 512)
    bot.CDS_SetAngle(3, 349, 512)
    bot.CDS_SetAngle(4, 196, 1023)
    bot.CDS_SetAngle(5, 559, 512)
    bot.CDS_SetAngle(6, 190, 512)
    bot.CDS_SetAngle(7, 475, 512)
    bot.CDS_SetAngle(8, 189, 512)
    bot.CDS_SetAngle(9, 512, 512)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 340, 908)
    bot.CDS_SetAngle(2, 512, 960)
    bot.CDS_SetAngle(3, 383, 300)
    bot.CDS_SetAngle(4, 196, 300)
    bot.CDS_SetAngle(5, 459, 302)
    bot.CDS_SetAngle(6, 190, 300)
    bot.CDS_SetAngle(7, 375, 302)
    bot.CDS_SetAngle(8, 189, 300)
    bot.CDS_SetAngle(9, 512, 300)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 340, 300)
    bot.CDS_SetAngle(2, 195, 960)
    bot.CDS_SetAngle(3, 449, 300)
    bot.CDS_SetAngle(5, 459, 300)
    bot.CDS_SetAngle(7, 375, 300)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 440, 302)
    bot.CDS_SetAngle(2, 195, 300)
    bot.CDS_SetAngle(3, 549, 302)
    bot.CDS_SetAngle(5, 359, 302)
    bot.CDS_SetAngle(7, 675, 908)
    bot.CDS_SetAngle(8, 512, 978)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 440, 300)
    bot.CDS_SetAngle(3, 549, 300)
    bot.CDS_SetAngle(5, 359, 300)
    bot.CDS_SetAngle(7, 675, 300)
    bot.CDS_SetAngle(8, 189, 978)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 540, 302)
    bot.CDS_SetAngle(3, 649, 302)
    bot.CDS_SetAngle(5, 659, 908)
    bot.CDS_SetAngle(6, 512, 975)
    bot.CDS_SetAngle(7, 623, 300)
    bot.CDS_SetAngle(8, 189, 300)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 540, 300)
    bot.CDS_SetAngle(3, 649, 300)
    bot.CDS_SetAngle(5, 659, 300)
    bot.CDS_SetAngle(6, 190, 975)
    bot.CDS_SetAngle(7, 575, 300)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 640, 302)
    bot.CDS_SetAngle(3, 349, 908)
    bot.CDS_SetAngle(4, 512, 956)
    bot.CDS_SetAngle(5, 559, 302)
    bot.CDS_SetAngle(6, 190, 300)
    bot.CDS_SetAngle(7, 475, 302)
    time.sleep(0.3)

def back_forward(speed1):
    bot.CDS_SetAngle(1, 440, 512)
    bot.CDS_SetAngle(2, 195, 512)
    bot.CDS_SetAngle(3, 649, 512)
    bot.CDS_SetAngle(4, 196, 512)
    bot.CDS_SetAngle(5, 359, 512)
    bot.CDS_SetAngle(6, 190, 1023)
    bot.CDS_SetAngle(7, 675, 512)
    bot.CDS_SetAngle(8, 189, 512)
    bot.CDS_SetAngle(9, 512, 512)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 340, 302)
    bot.CDS_SetAngle(2, 195, 300)
    bot.CDS_SetAngle(3, 580, 300)
    bot.CDS_SetAngle(4, 196, 300)
    bot.CDS_SetAngle(5, 459, 302)
    bot.CDS_SetAngle(6, 190, 300)
    bot.CDS_SetAngle(7, 375, 908)
    bot.CDS_SetAngle(8, 512, 978)
    bot.CDS_SetAngle(9, 512, 300)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 340, 300)
    bot.CDS_SetAngle(5, 459, 300)
    bot.CDS_SetAngle(7, 375, 300)
    bot.CDS_SetAngle(8, 189, 978)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 640, 908)
    bot.CDS_SetAngle(2, 512, 960)
    bot.CDS_SetAngle(3, 449, 396)
    bot.CDS_SetAngle(5, 559, 302)
    bot.CDS_SetAngle(7, 475, 302)
    bot.CDS_SetAngle(8, 189, 300)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 640, 300)
    bot.CDS_SetAngle(2, 195, 960)
    bot.CDS_SetAngle(3, 749, 908)
    bot.CDS_SetAngle(5, 659, 302)
    bot.CDS_SetAngle(7, 522, 300)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(3, 749, 300)
    bot.CDS_SetAngle(4, 196, 956)
    bot.CDS_SetAngle(5, 659, 300)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 440, 490)
    bot.CDS_SetAngle(3, 649, 302)
    bot.CDS_SetAngle(4, 196, 300)
    bot.CDS_SetAngle(5, 359, 908)
    bot.CDS_SetAngle(6, 512, 975)
    bot.CDS_SetAngle(7, 675, 463)
    time.sleep(0.3)

def turn_left(speed1):
    bot.CDS_SetAngle(1, 540, 512)
    bot.CDS_SetAngle(2, 195, 512)
    bot.CDS_SetAngle(3, 749, 512)
    bot.CDS_SetAngle(4, 196, 512)
    bot.CDS_SetAngle(5, 459, 512)
    bot.CDS_SetAngle(6, 190, 512)
    bot.CDS_SetAngle(7, 375, 512)
    bot.CDS_SetAngle(8, 189, 1023)
    bot.CDS_SetAngle(9, 512, 512)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 640, 302)
    bot.CDS_SetAngle(2, 195, 300)
    bot.CDS_SetAngle(3, 449, 908)
    bot.CDS_SetAngle(4, 512, 956)
    bot.CDS_SetAngle(5, 559, 302)
    bot.CDS_SetAngle(6, 190, 300)
    bot.CDS_SetAngle(7, 475, 302)
    bot.CDS_SetAngle(8, 189, 300)
    bot.CDS_SetAngle(9, 512, 300)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 640, 300)
    bot.CDS_SetAngle(3, 449, 300)
    bot.CDS_SetAngle(4, 196, 956)
    bot.CDS_SetAngle(5, 559, 300)
    bot.CDS_SetAngle(7, 475, 300)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 340, 908)
    bot.CDS_SetAngle(2, 512, 960)
    bot.CDS_SetAngle(3, 549, 302)
    bot.CDS_SetAngle(4, 196, 300)
    bot.CDS_SetAngle(7, 675, 605)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 340, 300)
    bot.CDS_SetAngle(2, 195, 960)
    bot.CDS_SetAngle(3, 549, 300)
    bot.CDS_SetAngle(7, 675, 300)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 440, 302)
    bot.CDS_SetAngle(2, 195, 300)
    bot.CDS_SetAngle(3, 649, 302)
    bot.CDS_SetAngle(5, 359, 605)
    bot.CDS_SetAngle(6, 512, 975)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 440, 300)
    bot.CDS_SetAngle(3, 649, 300)
    bot.CDS_SetAngle(5, 359, 300)
    bot.CDS_SetAngle(6, 190, 975)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 540, 302)
    bot.CDS_SetAngle(3, 749, 302)
    bot.CDS_SetAngle(5, 459, 302)
    bot.CDS_SetAngle(6, 190, 300)
    bot.CDS_SetAngle(7, 375, 908)
    bot.CDS_SetAngle(8, 512, 978)
    time.sleep(0.3)

def turn_right(speed1):
    bot.CDS_SetAngle(1, 540, 512)
    bot.CDS_SetAngle(2, 195, 1023)
    bot.CDS_SetAngle(3, 749, 512)
    bot.CDS_SetAngle(4, 196, 512)
    bot.CDS_SetAngle(5, 459, 512)
    bot.CDS_SetAngle(6, 190, 512)
    bot.CDS_SetAngle(7, 375, 512)
    bot.CDS_SetAngle(8, 189, 512)
    bot.CDS_SetAngle(9, 512, 512)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 440, 302)
    bot.CDS_SetAngle(2, 195, 300)
    bot.CDS_SetAngle(3, 649, 302)
    bot.CDS_SetAngle(4, 196, 300)
    bot.CDS_SetAngle(5, 359, 302)
    bot.CDS_SetAngle(6, 190, 300)
    bot.CDS_SetAngle(7, 675, 908)
    bot.CDS_SetAngle(8, 512, 978)
    bot.CDS_SetAngle(9, 512, 300)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 440, 300)
    bot.CDS_SetAngle(3, 649, 300)
    bot.CDS_SetAngle(5, 359, 300)
    bot.CDS_SetAngle(7, 675, 300)
    bot.CDS_SetAngle(8, 189, 978)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 340, 302)
    bot.CDS_SetAngle(3, 549, 302)
    bot.CDS_SetAngle(5, 659, 908)
    bot.CDS_SetAngle(6, 512, 975)
    bot.CDS_SetAngle(7, 575, 302)
    bot.CDS_SetAngle(8, 189, 300)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 340, 300)
    bot.CDS_SetAngle(3, 549, 300)
    bot.CDS_SetAngle(5, 659, 300)
    bot.CDS_SetAngle(6, 190, 975)
    bot.CDS_SetAngle(7, 575, 300)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 640, 908)
    bot.CDS_SetAngle(2, 512, 960)
    bot.CDS_SetAngle(3, 449, 302)
    bot.CDS_SetAngle(5, 559, 302)
    bot.CDS_SetAngle(6, 190, 300)
    bot.CDS_SetAngle(7, 475, 302)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 640, 300)
    bot.CDS_SetAngle(2, 195, 960)
    bot.CDS_SetAngle(3, 449, 300)
    bot.CDS_SetAngle(5, 559, 300)
    bot.CDS_SetAngle(7, 475, 300)
    time.sleep(0.3)
    
    bot.CDS_SetAngle(1, 540, 302)
    bot.CDS_SetAngle(2, 195, 300)
    bot.CDS_SetAngle(3, 749, 908)
    bot.CDS_SetAngle(4, 512, 956)
    bot.CDS_SetAngle(5, 459, 302)
    bot.CDS_SetAngle(7, 375, 302)
    time.sleep(0.3)

def main():
    # 开启模块
    bot.CDS_Open()
    bot.ADC_IO_Open()
    
    # 初始化 IO 端口为输入模式 (0 代表输入模式)
    bot.ADC_IO_SetIOMode(0, 0)
    bot.ADC_IO_SetIOMode(1, 0)
    
    init_motor()
    
    try:
        while True:
            # 获取指定 IO 端口的电平状态
            sl = bot.ADC_IO_GetInputLevel(0)
            sr = bot.ADC_IO_GetInputLevel(1)
            
            # 循迹/避障控制逻辑
            if sl == 1 and sr == 1:
                move_forward(300)
            elif sl == 0 and sr == 0:
                back_forward(300)
            elif sr == 0:
                turn_left(300)
            else:
                turn_right(300)
                
            time.sleep(0.01) # 对应原 C 代码的 UP_delay_ms(10)
            
    except KeyboardInterrupt:
        print("\n正在停止并关闭连接...")
    finally:
        bot.CDS_Close()
        bot.ADC_IO_Close()

if __name__ == '__main__':
    main()