# 文件路径: upai/object_detector/rknn.py
"""RKNN 管理与多线程推理池（中文注释）

本模块包含两个主要接口：
- init_rknn(model_path, core_id)：初始化并返回单个 RKNNLite 实例（绑定到指定 NPU 核心）；
- RKNNPoolExecutor：基于线程池的并行推理封装，每个线程持有独立的 RKNN 实例，
  适用于需要在多线程场景下并行调用 RKNN 推理的工程。

设计要点：
- 每个线程使用独立的 RKNN 实例以避免并发使用同一实例导致的线程安全问题；
- 结果通过内部队列返回，`put()` 提交任务为异步，`get()` 非阻塞地获取已完成结果；
- release() 与上下文管理支持幂等释放，确保资源可正常回收。
"""

import logging
import os
from concurrent.futures import ThreadPoolExecutor, Future
from queue import Queue, Empty
from typing import Any, Callable, List, Optional, Tuple
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    # Import for type checking only (requires rknnlite in the environment)
    from rknnlite.api import RKNNLite  # type: ignore
else:
    # Runtime: try to import RKNNLite, but tolerate missing dependency so module can be imported
    try:
        from rknnlite.api import RKNNLite  # type: ignore
    except Exception:
        RKNNLite = Any  # type: ignore

# 获取模块级日志器（由上层应用配置级别与 handler）
logger = logging.getLogger(__name__)

__all__ = ["init_rknn", "RKNNPoolExecutor"]


def init_rknn(model_path: str, core_id: int = 0) -> RKNNLite:
    """初始化单个 RKNN 实例并绑定到指定 NPU 核心。

    参数说明：
      - model_path：RKNN 模型文件路径（字符串）
      - core_id：目标 NPU 核心（0/1/2），-1 表示使用多核 (0_1_2)

    返回：
      - 已初始化的 RKNNLite 实例；若加载或初始化失败会抛出异常。
    """
    if not isinstance(model_path, str) or not model_path:
        logger.warning("init_rknn 收到无效模型路径")
        raise ValueError("模型路径不能为空")

    if not os.path.exists(model_path):
        logger.warning("init_rknn 模型文件不存在：%s", model_path)
        raise FileNotFoundError(f"模型文件不存在: {model_path}")

    rknn = RKNNLite()
    ret = rknn.load_rknn(model_path)
    if ret != 0:
        try:
            rknn.release()
        except Exception:
            pass
        logger.warning("init_rknn 模型加载失败，ret=%s，路径=%s", ret, model_path)
        raise RuntimeError(f"模型加载失败: {model_path}")

    core_map = {
        0: RKNNLite.NPU_CORE_0,
        1: RKNNLite.NPU_CORE_1,
        2: RKNNLite.NPU_CORE_2,
        -1: RKNNLite.NPU_CORE_0_1_2,
    }
    core_mask = core_map.get(int(core_id), RKNNLite.NPU_CORE_0)
    ret = rknn.init_runtime(core_mask=core_mask)
    if ret != 0:
        try:
            rknn.release()
        except Exception:
            pass
        logger.warning("init_rknn 运行时初始化失败，ret=%s，core_id=%s，路径=%s", ret, core_id, model_path)
        raise RuntimeError(f"运行时初始化失败: {model_path} (core={core_id})")

    logger.info("init_rknn 成功：%s (core=%s)", model_path, core_id)
    return rknn


class RKNNPoolExecutor:
    """RKNN 多线程推理池封装（加强注释版）。

    使用方法示例：
      executor = RKNNPoolExecutor(model_path, num_threads, infer_func)
      executor.put(frame)
      res, ok = executor.get()
      executor.release()

    说明：
      - infer_func: 用户提供的推理函数，签名应为 (RKNNLite, frame) -> result；
      - put() 将异步提交推理任务，任务完成后通过内部队列返回结果；
      - get() 为非阻塞获取接口，若无结果立即返回 (None, False)。
    """

    def __init__(self, model_path: str, num_threads: int, infer_func: Callable[[RKNNLite, Any], Any]) -> None:
        # 参数校验
        if not callable(infer_func):
            logger.warning("RKNNPoolExecutor 初始化失败：infer_func 非可调用对象")
            raise ValueError("infer_func 不能为空且必须可调用")
        if not isinstance(num_threads, int) or num_threads <= 0:
            logger.warning("RKNNPoolExecutor 初始化失败：num_threads 无效：%s", num_threads)
            raise ValueError("num_threads 必须为正整数")

        # 初始化线程池与内部队列
        self.num_threads: int = num_threads
        self.pool: ThreadPoolExecutor = ThreadPoolExecutor(max_workers=num_threads, thread_name_prefix="rknn")
        self.queue: "Queue[Any]" = Queue()
        self.infer_func: Callable[[RKNNLite, Any], Any] = infer_func
        self.models: List[RKNNLite] = []
        self.idx: int = 0
        self._closed: bool = False

        try:
            # 为每个线程初始化独立的 RKNN 实例，使用循环绑定不同核心以分散负载
            self.models = [init_rknn(model_path, i % 3) for i in range(num_threads)]
            logger.info("RKNNPoolExecutor 初始化完成：线程数=%d，模型=%s", num_threads, model_path)
        except Exception as e:
            logger.warning("RKNNPoolExecutor 初始化模型失败：%s，开始清理", e)
            self.release()
            raise

    def put(self, frame: Any) -> None:
        """提交一帧进行异步推理。

        行为说明：
          - 采用轮询（round-robin）方式选择 RKNN 实例以均衡负载；
          - 使用 ThreadPoolExecutor 异步执行 infer_func；
          - 在 Future 完成后通过回调将结果放入内部队列（非阻塞）。
        """
        if self._closed:
            logger.warning("put 调用于已关闭的执行器，忽略请求")
            return
        if not self.models:
            logger.warning("put 无可用 RKNN 实例，忽略请求")
            return

        # 轮询选择模型实例
        model = self.models[self.idx % self.num_threads]
        self.idx += 1
        try:
            # 提交异步任务
            future: Future = self.pool.submit(self.infer_func, model, frame)

            # 完成回调：将计算结果放入队列
            def _on_done(f: Future) -> None:
                try:
                    res = f.result()
                    self.queue.put(res)
                    logger.debug("推理任务完成，结果已入队")
                except Exception as ex:
                    # 推理或回调中发生异常时记录日志并忽略该结果
                    logger.debug("推理任务异常，已跳过：%s", ex)

            future.add_done_callback(_on_done)
            logger.debug("推理任务已提交（idx=%d）", self.idx - 1)
        except Exception as e:
            logger.debug("推理任务提交失败：%s", e)

    def get(self) -> Tuple[Optional[Any], bool]:
        """非阻塞获取已完成的推理结果。

        返回：(result, available)
          - result: 推理结果或 None
          - available: 布尔，表示是否成功取到结果
        """
        try:
            item = self.queue.get_nowait()
            return item, True
        except Empty:
            return None, False
        except Exception as e:
            logger.debug("get 获取结果异常：%s", e)
            return None, False

    def release(self) -> None:
        """释放线程池与所有 RKNN 资源（幂等）。

        说明：
          - 先关闭线程池再释���每个 RKNN 实例，最后清空结果队列；
          - 本方法可安全重复调用（幂等）。
        """
        if self._closed:
            return
        self._closed = True

        try:
            self.pool.shutdown(wait=True)
        except Exception as e:
            logger.debug("线程池关闭异常：%s", e)

        for m in self.models:
            try:
                m.release()
            except Exception as e:
                logger.debug("模型释放异常：%s", e)
        self.models.clear()

        # 清空结果队列，忽略异常
        try:
            while True:
                self.queue.get_nowait()
        except Empty:
            pass
        except Exception:
            pass

        logger.info("RKNNPoolExecutor 资源已释放")

    def __enter__(self) -> "RKNNPoolExecutor":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.release()

    def __del__(self) -> None:
        try:
            self.release()
        except Exception:
            pass
