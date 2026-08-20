# out.rknn 模型说明文档

## 1. 文件概述

| 项目 | 说明 |
|---|---|
| 文件名 | `out.rknn` |
| 模型类型 | YOLOv8n 目标检测模型（INT8 量化） |
| 目标硬件 | Rockchip RK3588 NPU |
| 推理框架 | rknn-toolkit-lite2（板端）/ rknn-toolkit2 1.6.0（PC端转换） |
| 输入尺寸 | 1 × 3 × 640 × 640（NCHW，RGB，uint8） |
| 输出 | YOLOv8 检测头原始输出（需后处理解码） |

## 2. 模型来源与转换链路

```
best.pt (PyTorch, YOLOv8n 训练权重)
    │  ultralytics export
    ▼
best.onnx (opset=12, simplify=True, imgsz=640)
    │  rknn-toolkit2 1.6.0 build (INT8 量化)
    ▼
out.rknn (RK3588 NPU 可执行模型)
```

### 转换关键参数

- **ONNX 导出**：`opset=12`, `simplify=True`, `imgsz=640`, `dynamic=False`
- **RKNN 构建**：`do_quantization=True`（INT8 量化，需校准数据集）
- **目标平台**：`rk3588`
- **均值/归一化**：mean=`[0,0,0]`, std=`[255,255,255]`（即输入像素值除以 255 归一化到 [0,1]，由 RKNN 内部完成）

## 3. 运行环境依赖

### 板端（RK3588, ARM Linux）

```bash
pip install rknn-toolkit-lite2==1.6.0
pip install opencv-python numpy
```

> `rknn-toolkit-lite2` 版本必须与 PC 端转换用的 `rknn-toolkit2` 版本对齐（均为 1.6.0），否则可能加载失败。

### 硬件要求

- Rockchip RK3588 芯片（含 NPU）
- NPU 驱动已加载（`/dev/dri` 或 `/dev/rknpu` 设备节点存在）

## 4. API 接口说明

### 4.1 初始化与加载

```python
from rknnlite.api import RKNNLite

rknn = RKNNLite()
ret = rknn.load_rknn(path='out.rknn')   # 返回 0 表示成功
ret = rknn.init_runtime()                  # 默认绑定 NPU，返回 0 表示成功
```

### 4.2 推理

```python
outputs = rknn.inference(inputs=[img_data])
```

- **inputs**：`list`，每个元素是一个 numpy array，顺序与模型输入节点对应
- **返回值**：`list`，每个元素对应一个输出节点的 numpy array

### 4.3 释放资源

```python
rknn.release()
```

## 5. 完整推理流程

### 5.1 图像预处理

```python
import cv2
import numpy as np

def preprocess(image_path, img_size=640):
    """
    输入预处理：BGR → RGB → resize → 归一化 → NCHW
    注意：归一化(除以255)由 RKNN 模型内部完成，这里只需传 uint8 RGB
    """
    img = cv2.imread(image_path)                     # BGR, HWC
    img = cv2.resize(img, (img_size, img_size))      # resize 到 640x640
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)       # BGR → RGB
    img = np.expand_dims(img, axis=0)                 # HWC → NHWC (1,640,640,3)
    return img
```

> **关键**：rknn-toolkit-lite2 的输入格式是 **NHWC**（不是 PyTorch 的 NCHW），且数据类型为 `uint8`。归一化（mean/std）在模型转换时已写入 RKNN，推理时由 NPU 硬件完成，不需要在预处理中手动除以 255。

### 5.2 执行推理

```python
img = preprocess('test.jpg')
outputs = rknn.inference(inputs=[img])
```

### 5.3 输出格式与后处理

YOLOv8n 的输出是一个形状为 `(1, 84, 8400)` 的张量（80类COCO模型；若为自定义类别，则为 `(1, 4+num_classes, 8400)`）：

- **维度 1**：batch = 1
- **维度 2**：`4（cx, cy, w, h）+ num_classes（类别概率）`
- **维度 3**：8400 个锚点（640/8 × 640/8 + 640/16 × 640/16 + 640/32 × 640/32 = 6400 + 1600 + 400 = 8400）

后处理需要在 **CPU** 上完成（NPU 不支持动态 NMS），步骤：

1. **转置**：`(1, 84, 8400)` → `(1, 8400, 84)`
2. **提取框坐标**：前 4 列为 `cx, cy, w, h`，需转换为 `x1, y1, x2, y2`
3. **提取类别分数**：后 80 列为各类别概率，取 max 得到类别和置信度
4. **置信度过滤**：去掉 score < conf_threshold 的框
5. **NMS**：非极大值抑制去除重叠框
6. **坐标还原**：将 640×640 上的坐标映射回原图尺寸

### 5.4 后处理参考实现

```python
def postprocess(outputs, img_shape, conf_thres=0.25, iou_thres=0.45, num_classes=80):
    """
    outputs: rknn.inference 返回的 list，outputs[0] shape = (1, 84, 8400)
    img_shape: 原图 (H, W)，用于坐标还原
    返回: list of [x1, y1, x2, y2, score, class_id]
    """
    pred = outputs[0][0]                          # (84, 8400)
    pred = np.transpose(pred, (1, 0))             # (8400, 84)

    boxes = pred[:, :4]                            # (8400, 4) cx,cy,w,h
    scores = pred[:, 4:]                           # (8400, 80) class probs

    # 取每个框的最大类别分数
    class_ids = np.argmax(scores, axis=1)
    class_scores = scores[np.arange(len(scores)), class_ids]

    # 置信度过滤
    mask = class_scores > conf_thres
    boxes = boxes[mask]
    class_scores = class_scores[mask]
    class_ids = class_ids[mask]

    # cx,cy,w,h → x1,y1,x2,y2
    boxes[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    boxes[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    boxes[:, 2] = boxes[:, 0] + boxes[:, 2]
    boxes[:, 3] = boxes[:, 1] + boxes[:, 3]

    # NMS (用 OpenCV 或自定义实现)
    indices = cv2.dnn.NMSBoxes(
        boxes.tolist(), class_scores.tolist(), conf_thres, iou_thres
    )
    if len(indices) > 0:
        indices = indices.flatten()
        boxes = boxes[indices]
        class_scores = class_scores[indices]
        class_ids = class_ids[indices]

    # 坐标还原到原图尺寸（假设是直接 resize，无 letterbox）
    orig_h, orig_w = img_shape
    scale_x = orig_w / 640
    scale_y = orig_h / 640
    boxes[:, 0] *= scale_x
    boxes[:, 1] *= scale_y
    boxes[:, 2] *= scale_x
    boxes[:, 3] *= scale_y

    results = []
    for box, score, cid in zip(boxes, class_scores, class_ids):
        results.append([float(box[0]), float(box[1]), float(box[2]), float(box[3]),
                        float(score), int(cid)])
    return results
```

### 5.5 完整调用示例

```python
from rknnlite.api import RKNNLite
import cv2
import numpy as np

class YOLOv8Detector:
    def __init__(self, model_path='out.rknn', img_size=640,
                 conf_thres=0.25, iou_thres=0.45, num_classes=80):
        self.img_size = img_size
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.num_classes = num_classes

        self.rknn = RKNNLite()
        ret = self.rknn.load_rknn(model_path)
        if ret != 0:
            raise RuntimeError(f"加载 RKNN 模型失败: {ret}")
        ret = self.rknn.init_runtime()
        if ret != 0:
            raise RuntimeError(f"初始化 NPU 运行时失败: {ret}")

    def detect(self, image):
        """
        image: BGR numpy array (cv2.imread 的结果)
        返回: list of [x1, y1, x2, y2, score, class_id]
        """
        orig_h, orig_w = image.shape[:2]

        # 预处理
        img = cv2.resize(image, (self.img_size, self.img_size))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.expand_dims(img, axis=0)   # NHWC, uint8

        # NPU 推理
        outputs = self.rknn.inference(inputs=[img])

        # CPU 后处理
        return self._postprocess(outputs, (orig_h, orig_w))

    def _postprocess(self, outputs, img_shape):
        pred = outputs[0][0]
        pred = np.transpose(pred, (1, 0))
        boxes = pred[:, :4]
        scores = pred[:, 4:]
        class_ids = np.argmax(scores, axis=1)
        class_scores = scores[np.arange(len(scores)), class_ids]

        mask = class_scores > self.conf_thres
        boxes, class_scores, class_ids = boxes[mask], class_scores[mask], class_ids[mask]

        boxes[:, 0] -= boxes[:, 2] / 2
        boxes[:, 1] -= boxes[:, 3] / 2
        boxes[:, 2] += boxes[:, 0]
        boxes[:, 3] += boxes[:, 1]

        indices = cv2.dnn.NMSBoxes(
            boxes.tolist(), class_scores.tolist(), self.conf_thres, self.iou_thres
        )
        if len(indices) > 0:
            indices = indices.flatten()
            boxes, class_scores, class_ids = boxes[indices], class_scores[indices], class_ids[indices]

        orig_h, orig_w = img_shape
        boxes[:, 0] *= orig_w / 640
        boxes[:, 1] *= orig_h / 640
        boxes[:, 2] *= orig_w / 640
        boxes[:, 3] *= orig_h / 640

        return [[float(b[0]), float(b[1]), float(b[2]), float(b[3]),
                 float(s), int(c)] for b, s, c in zip(boxes, class_scores, class_ids)]

    def release(self):
        self.rknn.release()


# 使用示例
if __name__ == '__main__':
    detector = YOLOv8Detector('out.rknn')
    img = cv2.imread('test.jpg')
    results = detector.detect(img)
    for x1, y1, x2, y2, score, cid in results:
        print(f'类别 {cid}, 置信度 {score:.2f}, 框 [{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]')
    detector.release()
```

## 6. 关键注意事项

| 事项 | 说明 |
|---|---|
| **输入格式** | NHWC，uint8，RGB。不是 NCHW，不需要手动归一化 |
| **归一化位置** | mean/std 已在 RKNN 模型内部，NPU 硬件完成，预处理只需传原始 uint8 像素 |
| **后处理位置** | NMS 和 DFL 解码必须在 CPU 上做，NPU 只跑前向卷积 |
| **版本对齐** | 板端 `rknn-toolkit-lite2` 版本必须与 PC 端 `rknn-toolkit2` 一致（1.6.0） |
| **坐标还原** | 若预处理用了 letterbox（保持宽高比填充），后处理需相应还原；上述代码假设直接 resize |
| **类别数** | 自定义训练模型需将 `num_classes` 改为实际类别数，输出维度为 `(1, 4+num_classes, 8400)` |
| **多线程** | `RKNNLite` 对象非线程安全，多线程需每个线程独立初始化或加锁 |
| **资源释放** | 程序结束前调用 `rknn.release()` 释放 NPU 资源 |

## 7. 常见错误排查

| 错误 | 原因 | 解决 |
|---|---|---|
| `Load model failed` | 模型文件路径错误或版本不匹配 | 检查路径；确认 lite2 与 toolkit2 版本一致 |
| `Init runtime failed` | NPU 驱动未加载或设备无权限 | 检查 `/dev/dri`；确认 root 或 video 组权限 |
| 推理结果全是乱框 | 输入格式错误（用了 NCHW 或归一化了） | 改为 NHWC uint8 RGB，去掉手动归一化 |
| 推理速度慢 | 未走 NPU，落到 CPU | 确认 `init_runtime()` 无报错；检查 NPU 驱动 |
| 漏检/误检多 | INT8 量化精度损失 | 增加校准图片数量；或先用 FP16 模型对比 |
