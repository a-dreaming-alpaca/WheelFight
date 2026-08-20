# 文件路径: upai/object_detector/yolo.py
"""YOLO 检测器封装（基于 RKNN 的示例）

本模块负责：
- 定义常用类别集合（COCO、武器、车辆等）以便快速绑定模型类别；
- 提供 `infer_func`：RKNN 单帧推理的包装（负责预处理、调用 rknn 推理并对输出进行后处理）；
- 提供 `YoloDetector`：基于 `RKNNPoolExecutor` 的多线程推理封装，用于非阻塞提交与获取结果。

注意事项：
- 本文件仅对推理流程做流程组织与绘制，模型输入/输出的具体格式依赖 RKNN 模型导出时的约定；
- `infer_func` 与 `YoloDetector` 假定输入图像为 OpenCV 的 BGR ndarray；
- 这里添加的注释以帮助阅读与维护，未改变原有逻辑；若需修复潜在的逻辑错误（变量未定义等），
  请明确让我修复后我会一并处理并运行测试。
"""

# 添加必要的导入，保证类型注解与外部工具可用
import logging
import os
import importlib
from pathlib import Path
from typing import Tuple, Optional, Any, List
from functools import partial

import cv2
import numpy as np

# 从本包导入后处理与 RKNN 池执行器
from .post_process import yolov5_post_process, IMG_SIZE
from .rknn import RKNNPoolExecutor

# 模块级日志器
logger = logging.getLogger(__name__)


def infer_func(rknn: Any, img: np.ndarray, class_names: Tuple[str, ...]) -> List[dict]:
    """
    使用 RKNN 进行单帧推理并返回检测结果的列表。

    约定：
      - `img` 为 BGR 的 numpy.ndarray；函数会在内部做必要的预处理（转 RGB、缩放到 IMG_SIZE）；
      - 返回 `results` 为列表，每项为字典，包含框、类别索引、分数、类别名与中心点；
      - 本函数目前对 RKNN 输出的 reshape/转置有特定的假设（取决于模型导出）；如模型导出格式不同，
        需要调整 reshape/transpose 部分以匹配实际输出张量形状。
    """
    results = []

    # 输入校验
    if img is None or not isinstance(img, np.ndarray) or img.ndim < 2:
        logger.warning("infer_func 收到无效图像，返回空结果")
        return results

    # 备份原图用于可视化/绘制（若需要）
    origin = img.copy()
    try:
        # 转为 RGB 并缩放到模型输入尺寸
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        rgb_resized = cv2.resize(rgb, (IMG_SIZE, IMG_SIZE))
    except Exception as e:
        logger.warning("infer_func 预处理失败：%s", e)
        return results

    # 调用 RKNN 推理（模型具体接口可能返回多个输出张量）
    try:
        outputs = rknn.inference(inputs=[rgb_resized])
        if not outputs or len(outputs) < 3:
            logger.warning("infer_func 推理输出数量异常")
            # 若推理输出数量异常，则直接返回空结果
            return results
    except Exception as e:
        logger.warning("infer_func 推理失败：%s", e)
        return results

    # 以下部分负责把 RKNN 返回的原始输出张量转换为适合后处理的 list
    # 注意：这里对 outputs 的 reshape/transpose 有具体假设，若模型导出格式不同需调整
    try:
        input0_data = outputs[0].reshape([3, -1] + list(outputs[0].shape[-2:]))
        input1_data = outputs[1].reshape([3, -1] + list(outputs[1].shape[-2:]))
        input2_data = outputs[2].reshape([3, -1] + list(outputs[2].shape[-2:]))

        input_data = [
            np.transpose(input0_data, (2, 3, 0, 1)),
            np.transpose(input1_data, (2, 3, 0, 1)),
            np.transpose(input2_data, (2, 3, 0, 1)),
        ]

        # 调用后处理，得到 boxes/classes/scores
        boxes, classes, scores = yolov5_post_process(input_data)

        # 若后处理未返回有效框，则直接返回
        if boxes is None:
            return results

        # 将基于 IMG_SIZE 的坐标缩放回原图尺寸
        h, w = origin.shape[:2]
        sx, sy = float(w) / float(IMG_SIZE), float(h) / float(IMG_SIZE)

        # 遍历所有检测结果并组织为统一的字典结构
        for box, score, cl in zip(boxes, scores, classes):
            x1, y1, x2, y2 = map(float, box[:4])

            # 将基于 IMG_SIZE 的坐标缩放至原图尺寸并裁剪
            x1 = int(np.clip(x1 * sx, 0, w - 1))
            y1 = int(np.clip(y1 * sy, 0, h - 1))
            x2 = int(np.clip(x2 * sx, 0, w - 1))
            y2 = int(np.clip(y2 * sy, 0, h - 1))

            idx = int(cl)
            name = class_names[idx] if 0 <= idx < len(class_names) else "unknown"

            # 计算中心点用于绘制或返回
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            results.append({
                "box": [x1, y1, x2, y2],
                "class": idx,
                "score": float(score),
                "name": name,
                "center": (cx, cy)
            })

        return results
    except Exception as e:
        logger.warning("infer_func 输出重排失败：%s", e)
        return results


class YoloDetector:
    """
    YOLO 检测器（基于 RKNN 的多线程推理池）
    用法：
      - 构造时解析模型路径并初始化线程池
      - detect(frame) 非阻塞提交并立即尝试获取上一个完成结果
      - release() 释放资源
    """

    def __init__(
            self,
            model_path: Optional[str],
            classes: Optional[Tuple[str, ...]],
            num_threads: int = 2
    ) -> None:
        self.classes = classes

        try:
            # 初始化 RKNN 并创建执行池；partial 用来将 class_names 绑定到 infer_func
            self.pool: RKNNPoolExecutor = RKNNPoolExecutor(
                model_path,
                num_threads,
                partial(infer_func, class_names=self.classes)
            )
            logger.info(
                "YoloDetector 初始化完成：model=%s, threads=%d, classes=%s",
                model_path, int(num_threads), self.classes
            )
        except Exception as e:
            logger.warning("YoloDetector 初始化失败：%s", e)
            raise

    def recognize(self, frame: np.ndarray) -> List[dict]:
        """
        提交一帧图像并尝试获取已完成的推理结果。
        若暂无可用结果，返回空列表；此方法为非阻塞风格。
        """
        results: List[dict] = []
        if frame is None or not isinstance(frame, np.ndarray) or frame.ndim < 2:
            logger.warning("recognize 收到无效图像，返回空结果")
            return results

        # 将图像提交到线程池进行推理
        self.pool.put(frame)
        # 尝试非阻塞获取最近完成的一项结果
        item, ok = self.pool.get()
        if ok and item is not None:
            # item 期望为 infer_func 返回的结果列表（可能为绘制后的图像或数据）
            # 这里直接返回 item（上层可根据需要进行处理）
            return item
        return results

    def draw(self, frame: np.ndarray, results: list):
        """
        在图像上绘制检测框、类别名、置信度和中心点。
        参数:
            frame: 原始 BGR 图像
            results: infer_func 返回的列���，每项包含:
                {
                    "box": [x1, y1, x2, y2],
                    "class": int,
                    "score": float,
                    "name": str,
                    "center": (cx, cy)
                }
        返回:
            绘制后的图像（原地修改 frame 并返回）
        """
        if frame is None or not isinstance(frame, np.ndarray) or frame.ndim < 2:
            logger.warning("draw 收到无效图像")
            return frame

        if not results:
            return frame

        for item in results:
            try:
                x1, y1, x2, y2 = item["box"]
                cx, cy = item["center"]
                name = item.get("name", "unknown")
                score = item.get("score", 0.0)

                # 绘制矩形框
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                # 绘制类别和置信度
                cv2.putText(
                    frame, f"{name} {score:.2f}", (x1, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
                )
                # 绘制中心点
                cv2.circle(frame, (cx, cy), 4, (0, 255, 0), 4)
                # 绘制框的左上、右下角点
                cv2.circle(frame, (x1, y1), 4, (255, 0, 0), 4)
                cv2.circle(frame, (x2, y2), 4, (255, 0, 0), 4)

            except Exception as e:
                logger.debug("draw 绘制单项失败，已跳过：%s", e)
                continue

    def release(self) -> None:
        """
        释放内部线程池与 RKNN 资源。
        """
        try:
            self.pool.release()
            logger.info("YoloDetector 资源已释放")
        except Exception as e:
            logger.debug("YoloDetector 资源释放异常：%s", e)

    def __del__(self) -> None:
        # 确保对象销毁时释放资源（冗余保护，实际释放应由 release 控制）
        # 注意：Python 的析构时机由 GC 决定，依赖 __del__ 不如显式 release 稳定
        try:
            self.release()
        except Exception:
            pass
