# DFRobot_HumanPose

* [English](./README.md)

HumanPose 是一款可通过 I2C 或 UART 进行人体姿态检测、手部检测及已学习目标识别的传感器。

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

本库为树莓派上的 Human Pose 传感器提供 Python 驱动。支持 I2C 与 UART 通信、人体姿态检测（17 个关键点）、手部检测（21 个关键点）以及已学习目标/手势识别。

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
          @param model: MODEL_POSE 或 MODEL_HAND
          @return 成功返回名称列表，超时返回 None
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

- 2026/02/04 - 发布版本 1.0.0。

## 致谢

Written by thdyyl(yuanlong.yu@dfrobot.com), 2026.02.04 (欢迎访问 [官网](https://www.dfrobot.com/))
