# upai/object_detector/post_process.py
"""YOLOv5 风格的后处理工具（中文注释）

本模块负责将模型多尺度输出解码为检测框、类别与分数，并执行阈值过滤与 NMS。
约定：
 - 输入坐标/尺寸以像素为单位，所有图像尺寸统一到常量 IMG_SIZE（通常等于模型输入尺寸，如 640）。
 - 框的内部表示在不同函数间有两种形式：
     * xywh: [x_center, y_center, width, height]
     * xyxy: [x1, y1, x2, y2]
 - 该模块尽量在异常时容错（记录警告并返回空结果），以避免上层流程崩溃。
"""

import logging
from typing import Iterable, List, Optional, Tuple, Sequence

import numpy as np

# 获取模块级日志器（由上层应用配置级别与 handler）
logger = logging.getLogger(__name__)

# 模型与后处理常量（可根据模型训练/导出时的约定进行调整）
OBJ_THRESH: float = 0.2  # 置信度阈值（同时用于 objectness 与类别置信度的二次筛选）
NMS_THRESH: float = 0.45  # NMS 的 IoU 阈值
IMG_SIZE: int = 640  # 模型输入的短边/正方尺寸，解码时用于缩放网格坐标到像素


def xywh2xyxy(x: np.ndarray) -> np.ndarray:
    """把 [x, y, w, h]（中心点表示）转换为 [x1, y1, x2, y2]（角点表示）。

    参数:
      - x: 形状 (N, 4) 或 (..., 4) 的数组，按最后一维为 [xc, yc, w, h]
    返回:
      - 同形状但最后一维为 [x1, y1, x2, y2]
    """
    y = np.copy(x)
    # 左上角
    y[:, 0] = x[:, 0] - x[:, 2] / 2.0
    y[:, 1] = x[:, 1] - x[:, 3] / 2.0
    # 右下角
    y[:, 2] = x[:, 0] + x[:, 2] / 2.0
    y[:, 3] = x[:, 1] + x[:, 3] / 2.0
    return y


def process(input: np.ndarray, mask: Sequence[int], anchors: Sequence[Sequence[float]]) -> Tuple[
    np.ndarray, np.ndarray, np.ndarray]:
    """把单尺度的模型原始输出解码为 boxes、objectness 与 class scores。

    说明：这是对 YOLOv5 等导出格式的兼容解码，输入通常为 (gh, gw, 3, 5+nc)。

    参数:
      - input: 单尺度输出张量，shape=(gh, gw, 3, 5+num_classes)
      - mask: 本尺度使用的 3 个 anchor 的索引（例如 [0,1,2]）
      - anchors: 全部 anchor 列表（通常为 9 个），单位为像素
    返回：
      - boxes_xywh: (gh, gw, 3, 4) 的 [x_center, y_center, w, h]（像素尺度）
      - box_conf: (gh, gw, 3, 1) 的 objectness
      - box_cls: (gh, gw, 3, num_classes) 的类别置信度（未与 objectness 相乘）
    """
    try:
        # 选择当前尺度对应的三个 anchors
        anchors_sel = np.array([anchors[i] for i in mask], dtype=np.float32)  # (3, 2)
        gh, gw = map(int, input.shape[0:2])

        # objectness（置信度）与类别概率
        box_confidence = np.expand_dims(input[..., 4], axis=-1)  # (gh, gw, 3, 1)
        box_class_probs = input[..., 5:]  # (gh, gw, 3, num_classes)

        # 中心点偏移：sigmoid/其它激活在模型导出端可能已合并，这里采用常见的 x*2-0.5 变换
        box_xy = input[..., :2] * 2.0 - 0.5  # (gh, gw, 3, 2)

        # 构建网格并把相对偏移转换为像素坐标
        col = np.tile(np.arange(gw, dtype=np.float32), (gh, 1))
        row = np.tile(np.arange(gh, dtype=np.float32).reshape(-1, 1), (1, gw))
        col = col.reshape(gh, gw, 1, 1).repeat(3, axis=-2)
        row = row.reshape(gh, gw, 1, 1).repeat(3, axis=-2)
        grid = np.concatenate((col, row), axis=-1)  # (gh, gw, 3, 2)
        # 将网格坐标缩放到模型输入像素尺度
        box_xy = (box_xy + grid) * float(IMG_SIZE / gh)

        # 宽高解码（模型常见为 (pw * (2*pw)^2) 之类的变换，这里使用常见的平方反解码）
        box_wh = np.square(input[..., 2:4] * 2.0) * anchors_sel.reshape(1, 1, 3, 2)

        boxes = np.concatenate((box_xy, box_wh), axis=-1).astype(np.float32)
        return boxes, box_confidence.astype(np.float32), box_class_probs.astype(np.float32)
    except Exception as e:
        logger.warning("process 解码失败：%s", e)
        # 返回空张量以便上游安全判断
        return (
            np.zeros((0, 4), dtype=np.float32),
            np.zeros((0, 1), dtype=np.float32),
            np.zeros((0, 0), dtype=np.float32),
        )


def filter_boxes(boxes: np.ndarray, box_confidences: np.ndarray, box_class_probs: np.ndarray) -> Tuple[
    np.ndarray, np.ndarray, np.ndarray]:
    """依据阈值筛选候选框并返回扁平化的 boxes/classes/scores。

    步骤：
      1. 扁平化所有尺度的格点输出到 (N, 4) 与 (N,)；
      2. 根据 objectness 阈值过滤（OBJ_THRESH）；
      3. 再根据类别置信度过滤并计算最终分数 = objectness * class_score。

    返回：
      - boxes: (N, 4) xywh
      - classes: (N,) int
      - scores: (N,) float
    """
    if boxes.size == 0:
        return boxes, np.array([], dtype=np.int64), np.array([], dtype=np.float32)

    # 扁平化
    boxes = boxes.reshape(-1, 4)
    box_confidences = box_confidences.reshape(-1)
    box_class_probs = box_class_probs.reshape(-1, box_class_probs.shape[-1])

    # 先按 objectness 过滤
    pos = np.where(box_confidences >= OBJ_THRESH)
    boxes = boxes[pos]
    box_confidences = box_confidences[pos]
    box_class_probs = box_class_probs[pos]

    if boxes.size == 0:
        return boxes, np.array([], dtype=np.int64), np.array([], dtype=np.float32)

    # 按类别得分过滤并计算最终分数
    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)
    pos2 = np.where(class_max_score >= OBJ_THRESH)

    boxes = boxes[pos2]
    classes = classes[pos2]
    scores = (class_max_score * box_confidences)[pos2].astype(np.float32)
    return boxes.astype(np.float32), classes.astype(np.int64), scores


def nms_boxes(boxes: np.ndarray, scores: np.ndarray) -> np.ndarray:
    """对给定类别的 boxes/scores 执行 NMS，返回保留索引数组（numpy int64）。

    输入要求：boxes 必须为 xyxy 格式 (N, 4)，scores 为 (N,)。
    """
    if boxes.size == 0:
        return np.array([], dtype=np.int64)

    # 分离坐标
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    # 计算宽高与面积，注意宽/高不能为负
    w = np.maximum(0.0, x2 - x1)
    h = np.maximum(0.0, y2 - y1)
    areas = w * h
    # 按分数从高到低排序
    order = scores.argsort()[::-1]

    keep: List[int] = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)

        # 计算当前最高分框与后续框的交集坐标
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        # 交集宽高
        inter_w = np.maximum(0.0, xx2 - xx1)
        inter_h = np.maximum(0.0, yy2 - yy1)
        inter = inter_w * inter_h

        # IoU = inter / (areaA + areaB - inter)
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        inds = np.where(iou <= NMS_THRESH)[0]
        order = order[inds + 1]
    return np.array(keep, dtype=np.int64)


def yolov5_post_process(input_data: Iterable[np.ndarray]) -> Tuple[
    Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
    """对模型的多尺度输出进行解码、阈值筛选与按类别 NMS，返回最终检测结果。

    参数：
      - input_data: 可迭代的三个尺度输出，每项形如 (gh, gw, 3, 5+nc)
    返回：
      - boxes: (M, 4) xyxy，坐标基于 IMG_SIZE（像素）
      - classes: (M,) int
      - scores: (M,) float
    """
    try:
        masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
        anchors = [
            [10, 13], [16, 30], [33, 23],
            [30, 61], [62, 45], [59, 119],
            [116, 90], [156, 198], [373, 326],
        ]

        boxes_all, classes_all, scores_all = [], [], []
        for out, mask in zip(list(input_data), masks):
            # 解码单尺度输出
            b, c, s = process(out, mask, anchors)
            # 按置信度与类别置信度过滤
            b, c, s = filter_boxes(b, c, s)
            boxes_all.append(b)
            classes_all.append(c)
            scores_all.append(s)

        if not boxes_all:
            logger.warning("yolov5_post_process 输入为空")
            return None, None, None

        # 合并所有尺度的候选框
        boxes = np.concatenate(boxes_all, axis=0) if boxes_all else np.zeros((0, 4), dtype=np.float32)
        # 将 xywh 转为 xyxy
        boxes = xywh2xyxy(boxes)
        classes = np.concatenate(classes_all, axis=0) if classes_all else np.array([], dtype=np.int64)
        scores = np.concatenate(scores_all, axis=0) if scores_all else np.array([], dtype=np.float32)

        if boxes.size == 0:
            return None, None, None

        # 按类别分别执行 NMS
        nboxes, nclasses, nscores = [], [], []
        for c in set(classes.tolist()):
            inds = np.where(classes == c)[0]
            b = boxes[inds]
            s = scores[inds]
            keep = nms_boxes(b, s)
            if keep.size > 0:
                nboxes.append(b[keep])
                nclasses.append(np.full(keep.shape, c, dtype=np.int64))
                nscores.append(s[keep])

        if not nboxes:
            return None, None, None

        boxes_f = np.concatenate(nboxes, axis=0)
        classes_f = np.concatenate(nclasses, axis=0)
        scores_f = np.concatenate(nscores, axis=0)

        logger.info("yolov5_post_process 完成：检测到目标数 %d", int(boxes_f.shape[0]))
        return boxes_f, classes_f, scores_f
    except Exception as e:
        logger.warning("yolov5_post_process 处理异常：%s", e)
        return None, None, None
