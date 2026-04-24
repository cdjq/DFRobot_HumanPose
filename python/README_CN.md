# DFRobot_HumanPose

* [English](./README.md)

HumanPose 模组（如 **SEN0670**）在端侧完成推理；本目录为树莓派等主机上的 **Python** 驱动，通过 I2C 或 UART 与模组通信，实现人体姿态、手部检测及已学习目标识别。

## 产品链接（https://www.dfrobot.com）

    人体姿态传感器

## 目录

* [摘要](#摘要)
* [安装](#安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [致谢](#致谢)

## 摘要

本库为树莓派上的 Human Pose 模组提供 Python 驱动，与 Arduino 库共用同一套二进制协议思路。模型与协议在模组侧运行；主机侧仅做配置与取数。

**关键点约定：** 人体 **17 点** 顺序与 **COCO 人体 17 关键点** 拓扑一致；手部 **21 点** 与 **MediaPipe Hands** 的 21 点骨架命名一致，便于与常见可视化与算法示例对接。支持 I2C / UART、姿态与手部检测、固定手势及学习类相似度；默认二进制传输（`TPROTO=1`），并可开关关键点输出（`TKPTS`）。

## 安装

使用本库前，请先将库下载到树莓派并安装 pinpong：

```
pip install pinpong
pip3 install pinpong
```

然后进入用例目录，在命令行中执行 `python xxx.py` 运行对应用例。例如运行 set_baud 波特率设置用例：

```
cd python/raspberrypi/examples
python set_baud.py
```

运行 pose_detect_blink 姿态检测 LED 指示用例：

```
python pose_detect_blink.py
```

运行 get_pose_result 获取姿态结果并显示用例：

```
python get_pose_result.py
```

运行 get_hand_result 获取手部结果并显示用例：

```
python get_hand_result.py
```

## 方法

```python

    def begin(self):
        '''
          @brief 初始化传感器并校验设备名称
          @return True: 初始化成功, False: 初始化失败
        '''

    def get_result(self):
        '''
          @brief 触发一次检测并等待结果；结果通过 available_result/pop_result 读取
          @return CODE_OK: 成功, CODE_TIMEOUT: 超时
        '''

    def set_confidence(self, confidence):
        '''
          @brief 设置检测置信度阈值 (0-100)，默认约 60
          @param confidence: 置信度阈值
          @return CODE_OK: 成功, CODE_TIMEOUT: 超时
        '''

    def set_iou(self, iou):
        '''
          @brief 设置 IOU 阈值 (0-100)，用于非极大值抑制，默认约 45
          @param iou: IOU 阈值
          @return CODE_OK: 成功, CODE_TIMEOUT: 超时
        '''

    def set_learn_similarity(self, similarity):
        '''
          @brief 设置已学习目标匹配相似度阈值 (0-100)，默认约 60
          @param similarity: 相似度阈值
          @return CODE_OK: 成功, CODE_TIMEOUT: 超时
        '''

    def set_model_type(self, model):
        '''
          @brief 设置检测模型类型
          @param model: MODEL_HAND (1) 手部检测, MODEL_POSE (3) 人体姿态检测
          @return CODE_OK: 成功, CODE_TIMEOUT: 超时
        '''

    def get_confidence(self):
        '''
          @brief 获取当前置信度阈值
          @return 成功返回当前值，超时返回 None
        '''

    def get_iou(self):
        '''
          @brief 获取当前 IOU 阈值
          @return 成功返回当前值，超时返回 None
        '''

    def get_learn_similarity(self):
        '''
          @brief 获取当前学习相似度阈值
          @return 成功返回当前值，超时返回 None
        '''

    def get_learn_list(self, model):
        '''
          @brief 获取指定模型的已学习目标名称列表
          @param model: MODEL_POSE / MODEL_HAND / MODEL_GES
          @n     MODEL_GES 使用固定类别名（id 0..13），因此始终返回 []
          @return 返回名称列表；HAND/POSE 返回当前缓存列表
        '''

    def set_keypoint_output(self, enable):
        '''
          @brief 配置 INVOKE 输出是否包含关键点
          @param enable: True 输出关键点, False 仅输出框
          @return CODE_OK: 成功, CODE_TIMEOUT: 超时
        '''

    def get_keypoint_output(self):
        '''
          @brief 获取 INVOKE 输出是否包含关键点
          @return 成功返回 1/0，超时或返回值异常（非 0/1）时返回 None
        '''

    def available_result(self):
        '''
          @brief 检查是否还有未读的检测结果
          @return True: 至少有一个结果, False: 无结果
        '''

    def pop_result(self):
        '''
          @brief 弹出一个未读结果（根据模型为 PoseResult 或 HandResult），并标记为已读
          @return 返回一个 Result 实例，无未读结果时返回 None
        '''

    # 仅 UART：
    def set_baud(self, baudrate):
        '''
          @brief 设置传感器 UART 波特率
          @param baudrate: 波特率（如 BAUD_9600, BAUD_115200）。支持: BAUD_9600, BAUD_14400, BAUD_19200, BAUD_38400, BAUD_57600, BAUD_115200, BAUD_230400, BAUD_460800, BAUD_921600
          @n     注意：设置后需用新波特率重新初始化串口和传感器
          @return True: 设置成功, False: 设置失败
        '''
```

## 兼容性

| MCU          | 正常 | 异常 | 未测 | 备注 |
| ------------ | :--: | :--: | :--: | ---- |
| Raspberry Pi |  √   |      |      |      |
| UNIHIKER     |  √   |      |      |      |

* Python 版本

| Python  | 正常 | 异常 | 未测 | 备注 |
| ------- | :--: | :--: | :--: | ---- |
| Python2 |      |      |  √   |      |
| Python3 |  √   |      |      |      |

## 历史

- 2026-04-13 - 发布版本 1.0.0。

## 致谢

Written by thdyyl(yuanlong.yu@dfrobot.com), 2026-04-13 (欢迎访问 [官网](https://www.dfrobot.com/))
