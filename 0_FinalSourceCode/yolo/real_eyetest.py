import sys
sys.path.append("..")

from uptech import UpTech

import threading
import time
import cv2
# 导入YOLO检测器
from yolo.yolo import YoloDetector

uptech = UpTech()
uptech.ADC_IO_Open()

class Yolo_demo:

    def __init__(self):
        self.uptech = UpTech()
        self.uptech.ADC_IO_Open()

        # 初始化YOLO检测器
        model_path = sys.path[0] + "/yolo/model/buff-quantized-mmse.rknn"
        self.classes = ("Buff", "Debuff")  # 可根据实际识别目标修改
        self.yolo = YoloDetector(model_path, self.classes)
        
        # 检测结果相关变量
        self.detect_results = []  # 存储YOLO检测结果
        self.target_name = None   # 识别到的目标名称
        self.target_center = None # 识别到的目标中心坐标
        
        # 开启YOLO识别多线程
        self._alive = threading.Event()
        self._alive.set()
        yolo_detect_thread = threading.Thread(target=self.yolo_detect_thread, daemon=True)
        yolo_detect_thread.start()

    def yolo_detect_thread(self):
        """YOLO目标检测线程"""
        print("YOLO detect start")
        cap = cv2.VideoCapture(0)
        
        while self._alive.is_set():
            if not cap.isOpened():
                print("cannot connect camera")
                cap = cv2.VideoCapture(0)
                time.sleep(0.5)
                continue

            print("camera connected successfully")
            
            try:             
                ret, frame = cap.read()
                if not ret:
                    raise RuntimeError("frame read failed")
                
                # 图像预处理（保持和原程序一致的裁剪逻辑）
                frame = cv2.resize(frame, (640, 480))
                cup_w = (int)((640 - 320) / 2) - 120
                cup_h = (int)((480 - 240) / 2) - 40
                frame1 = frame[cup_h:cup_h + 440, cup_w:cup_w + 500]
                
                # YOLO识别核心逻辑
                self.detect_results = self.yolo.recognize(frame1)
                
                # 解析检测结果
                self.target_name = None
                self.target_center = None
                if self.detect_results:
                    for res in self.detect_results:
                        self.target_name = res.get('name')
                        self.target_center = res.get('center')
                        print(f"检测到目标：{self.target_name}，中心坐标：{self.target_center}")
                        
                        # 根据识别结果判断行为（可根据实际需求修改）
                        if self.target_name == "Buff":
                            # 需要推的物块
                            print("前方有目标（Buff），需要推")
                        elif self.target_name == "Debuff":
                            # 需要绕开的物块
                            print("前方有目标（Debuff），需要绕开")
                else:
                    print("0")
                
                # 绘制检测结果并显示
                self.yolo.draw(frame1, self.detect_results)
                cv2.imshow("YOLO Detection", frame1)
                if cv2.waitKey(1) & 0xff == ord('q'):
                    break
                    
            except Exception as e:
                print(f"camera error: {e}, try to connect camera...")
                time.sleep(0.5)
        
        # 释放资源
        cap.release()
        self.yolo.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    y = Yolo_demo()
 
    try:
        while y._alive.is_set():
            time.sleep(1) # 每秒休眠一次，把 CPU 资源让给视觉线程
    except KeyboardInterrupt:
        print("主程序收到 Ctrl+C，正在退出...")
        y._alive.clear()
        time.sleep(0.5)
