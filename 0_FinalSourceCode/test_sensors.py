import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uptech import UpTech


uptech = UpTech()

uptech.ADC_IO_Open()

# 设置 IO 模式为输入模式 (0)
for i in range(0, 8):
    uptech.ADC_IO_SetIOMode(i, 0)
print("==================================================")
print("            UpTech 传感器数据实时监控程序            ")
print("               ( 按 Ctrl + C 退出 )               ")
print("==================================================")
try:
    while True:
        # 1. 读取 8 个通道的 IO 电平状态
        io_status = []
        for i in range(0, 8):
            level = uptech.ADC_IO_GetInputLevel(i)
            io_status.append(f"IO_{i}: {level}")

        # 2. 读取 6 个通道的 ADC 模拟量数据
        adc_status = []
        for i in range(0, 7):
            val = uptech.ADC_Get_Channel(i)
            adc_status.append(f"CH_{i}: {val:<4}") # <4 用于左对齐对齐格式

        # 3. 拼接输出字符串
        io_line = " | ".join(io_status)
        adc_line = " | ".join(adc_status)

        # 4. 打印到控制台
        # 使用 \r 回到行首，并配合 end='' 从而实现原地刷新，不污染控制台历史
        # \033[K 是终端口令，用于清除从光标到行末的内容，防止长短字符残留
        sys.stdout.write(f"\r[IO 状态] {io_line}\n[ADC数值] {adc_line}\033[K")
        
        # 将光标往回移动一行，以便下一次循环覆盖
        sys.stdout.write("\033[1A") 
        sys.stdout.flush()

        # 刷新频率：0.1秒（可根据需要调整）
        time.sleep(0.1)

except KeyboardInterrupt:
    # 捕获 Ctrl+C，优雅退出
    print("\n\n正在停止监控并关闭设备...")
finally:
    # 确保关闭资源
    uptech.ADC_IO_Close()
    print("设备已安全关闭。")

