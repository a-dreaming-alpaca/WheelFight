# WheelFight 2026 控制程序

本目录是 RK3588S 上运行的比赛控制程序。新的控制链路为：

```text
15 路模拟量 + 2 路数字量
        ↓
Mega 2560（50 Hz 采样、CRC 帧）
        ↓ USB 串口
RK3588S（感知层 → 可抢占状态机 → 电机/舵机）
```

主状态机每 20 ms 刷新一次。普通搜索、对准、识别、进攻和冲台动作都不使用阻塞式
`sleep`；传感器断流、掉台、前方边缘和后方围栏可以在动作进行中抢占当前状态。
台上没有目标时，`ARENA_SEARCH` 以低速直线主动巡台；目标簇一出现便当周期停车并进入
对准，抵达边缘后由 `EDGE_RECOVER` 完成退让和改向，再返回巡台。

## 主要文件

- `match_demo_state_machine.py`：比赛入口、状态机、安全仲裁和状态输出。
- `robot_config.py`：所有传感器阈值与通道组、速度、持续时间、通信时间、设备 ID 和舵机角度。
- `mega_sensor_reader.py`：Mega 串口自动发现、CRC 校验、丢帧统计和断线重连。
- `perception.py`：滤波、迟滞、台面状态、边缘语义和 12 方向目标聚类。
- `energy_vision.py`：摄像头视觉接口；黄绿色面积识别增益块，红色面积加 X 形状识别减益块。
- `yolo_vision.py`：`yolo/out.rknn` 的 YOLOv8 RKNN 后端；`Buff` 映射为增益块，`Debuff` 映射为减益块，无检测结果按敌车证据处理。
- `vision_tune.py`：只打开摄像头的视觉标定工具，显示 ROI、分类、颜色占比和掩膜。
- `motion_controller.py`：唯一的电机/舵机写入边界；左侧 ID 2，右侧 ID 1，
  铲子左 ID 5、右 ID 6。电机直接使用 `CDS_SetSpeed`，只有两个舵机需要设置模式0。
- `tk_monitor.py`：只读网页监控，不打开串口或 UpTech ADC。
- `sensor_monitor.py`：独立传感器接线/采样测试工具，不参与比赛控制。
- `BEHAVIOR_DESIGN.md`：完整行为、优先级和标定依据。

## RK3588S 启动

建议使用独立虚拟环境。Mega 链路至少需要 Python 3.9 和 `pyserial`：

```bash
cd 0_FinalSourceCode
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r sensor_requirements.txt
```

视觉模块只使用项目原有的 `cv2` Python 模块。启动前可检查：

```bash
python3 -c "import cv2; print('vision ok')"
```

正式控制器默认加载 `yolo/out.rknn`。板端还需要与模型转换版本匹配的
`rknn-toolkit-lite2`，并确认类别顺序为 `0=Buff`、`1=Debuff`。模型输入为
`640x640` 的 RGB、NHWC、`uint8` 图像；YOLOv8 输出由 `yolo_vision.py` 在 CPU
上完成解码和 NMS。目标未检测到时输出 `NO_BLOCK_MARKER`，状态机在连续帧和
红外近距条件满足后将其作为敌车处理。

## 摄像头视觉标定

在 RK3588S 的图形桌面终端中运行：

```bash
python3 vision_tune.py
```

该工具只打开 `robot_config.py` 中配置的摄像头，不导入 Mega、UpTech、状态机或运动控制。
标定工具和正式视觉线程都会跳过“能够打开但读取不到有效首帧”的设备编号，并继续尝试下一个摄像头。
主窗口会显示中央 ROI、中心线、实时分类、置信度、两种颜色占比、红色 X 分数和最佳
交叉角度；另外两个窗口分别显示黄绿色和红色掩膜。红色候选框中的两条辅助线会随最佳
角度旋转。判定会在 0°～85° 间搜索，要求四条交叉臂及中心都存在，并排除交叉线外
大面积铺满的红色，因此正放的 X 和斜视后呈十字的 X 使用同一规则。
按 `Q`、回车、`Esc` 或 `Ctrl+C` 退出。纯 SSH 终端没有图形显示环境，
不能直接显示这些 OpenCV 窗口；标定工具也不能与比赛控制器同时占用同一摄像头。

标定时先在最终照明和安装角度下调整 HSV 范围，再调整 `min_color_area_ratio`，最后用正式
减益图案和场地红框分别标定 `min_red_x_score` 与
`max_red_x_score_for_no_marker`。画面上的颜色比例以百分数显示，例如 `1.500%` 对应配置值
`0.015`；场地红框必须保持 `Red X: NO`。红色达到面积门槛后，X 分数达到前一门槛才输出
`HARMFUL`，不高于后一门槛则输出 `NO_BLOCK_MARKER`，落在两门槛之间的疑似不完整 X 输出
`UNKNOWN`。若黄绿色本身已达到门槛，旁边无效的红色形状不会覆盖 `GAIN`，只有有效红色
交叉会与其冲突。`NO_BLOCK_MARKER` 仍须通过状态机的连续帧、居中和近距红外确认才会作为
敌车证据。

如果视觉依赖或摄像头暂时不可用，控制器会进入保守降级模式：继续安全移动搜索，
但不会把一次未识别到完整能量块标识当成敌人，也不会推动无法确认的能量块。

`MotionController` 创建真实硬件对象时会根据源码位置自动加入仓库根目录，以便加载
根目录下的 `uptech.py`；注入测试替身时不会改变模块搜索路径。RK3588S 系统仍须能够
从动态库搜索路径加载厂商提供的 `libuptech.so`。

启动比赛控制器：

```bash
python3 match_demo_state_machine.py
```

默认会自动寻找 Mega。正式部署建议使用稳定设备名：

```bash
python3 match_demo_state_machine.py --mega-port /dev/serial/by-id/你的Mega设备名
```

另开一个终端启动只读网页监控：

```bash
python3 tk_monitor.py
```

同一局域网内访问 `http://RK3588S的IP:8001`。状态由
`runtime/match_status.json` 原子更新，网页监控不会与控制器争抢 Mega 串口。

## 串口占用规则

`sensor_monitor.py` 是单独测试 Mega 和传感器的工具。它与比赛控制器都会打开 Mega
串口，因此两者不能同时运行。调试传感器时先停止比赛控制器；开始整车测试前再退出
`sensor_monitor.py`。

## 上电前必须标定

`robot_config.py` 里的 ADC 阈值、速度、计时和舵机角度目前都是临时起始值，不能直接
视为比赛参数。尤其注意：

1. `shovel_motion_enabled` 默认是 `False`。先卸载舵机负载并确认左右安全角度，之后才可
   改为 `True`。
2. 第一次电机测试必须架空车轮，使用低速短时命令确认 ID 2/ID 1 与前进方向。
3. 分别记录台上、台下、平台、围栏、敌车和能量块的原始传感器值，再修改迟滞阈值；
   HSV 范围、最小颜色面积占比、红色 X 分数以及“无完整标识可判为敌人”的红外近距门槛
   也必须实测。
4. 依次验证边缘停车、部分掉台恢复、平台/围栏判别、低速冲台，最后才能提高速度。

调参时只修改 `robot_config.py` 中的数值，并在 `motion_tune.py` 的
`run_selected_action()` 中保留当前要测试的一条 `MotionController` 调用。例如：

```bash
python3 motion_tune.py
```

动作运行后按回车停止；`Ctrl+C` 或普通Python异常也会通过 `finally` 尝试停车并关闭
CDS总线。运行该工具时不要同时启动比赛控制器。

## 离线测试

在仓库根目录运行：

```bash
python3 -m unittest discover -s sensor_bridge/tests -v
```

这些测试不需要 UpTech 动态库、Mega、摄像头或电机。目前覆盖通信协议、传感器语义、
电机映射、启动手势、安全抢占、平台/围栏判断、冲台、掉台恢复和比赛终止。
