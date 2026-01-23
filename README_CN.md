# DFRobot_HumanPose

* [English Version](./README.md)

HumanPose是一款可以检测人体姿态和手势的传感器库。

## 目录

* [简介](#简介)
* [安装](#安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [贡献者](#贡献者)

## 简介

提供用于控制HumanPose传感器的Arduino库。这是一个可以通过 I2C/UART 端口控制的人体姿态检测传感器。具有人体姿态检测、手势检测等功能，支持实时获取检测结果，包括关键点坐标、边界框信息等。

## 安装

1. 要使用此库，首先下载库文件，将其粘贴到 `\Arduino\libraries` 目录中，然后打开示例文件夹并运行其中的示例。
2. 使用该库还需下载依赖：https://github.com/bblanchon/ArduinoJson

## 方法
```c++
    /**
     * @fn begin
     * @brief 初始化传感器
     * @return 如果初始化成功返回 true，否则返回 false
     */
    bool begin();

    /**
     * @fn getResult
     * @brief 从传感器获取检测结果
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     * @note 调用此函数后，检测结果将存储在内部结果数组中。
     *       您可以使用 availableResult() 和 popResult() 方法访问结果。
     */
    eCmdCode_t getResult();

    /**
     * @fn setConfidence
     * @brief 设置检测置信度阈值
     * 
     * 设置检测被视为有效所需的最小置信度分数。
     * 较高的值会导致更少但更可靠的检测。较低的值允许更多
     * 检测但可能包含误报。
     *
     * @param confidence 置信度阈值值（0-100）。默认值通常为 60。
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     */
    eCmdCode_t setConfidence(uint8_t confidence);

    /**
     * @fn setIOU
     * @brief 设置交并比（IOU）阈值
     * 
     * 设置用于目标检测过程中非极大值抑制的 IOU 阈值。
     * 此参数有助于过滤重叠的检测。
     *
     * @param iou IOU 阈值值（0-100）。默认值通常为 45。
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     */
    eCmdCode_t setIOU(uint8_t iou);

    /**
     * @fn setModelType
     * @brief 设置检测模型
     * 
     * 选择要使用的检测模型。传感器支持手势检测和人体姿态检测。
     *
     * @param model 类型为 `eModel_t` 的模型类型，可能的值包括：
     *              - `eHand` - 手势检测模型
     *              - `ePose` - 人体姿态检测模型
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     */
    eCmdCode_t setModelType(eModel_t model);

    /**
     * @fn setLearnSimilarity
     * @brief 设置学习目标的相似度阈值
     * 
     * 设置用于将检测到的目标与学习目标匹配的相似度阈值。
     * 此参数用于手势识别和学习姿态匹配。
     *
     * @param Similarity 相似度阈值值（0-100）。默认值通常为 60。
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     */
    eCmdCode_t setLearnSimilarity(uint8_t Similarity);

    /**
     * @fn getConfidence
     * @brief 获取当前检测置信度阈值
     * 
     * 检索当前配置的检测置信度阈值。
     *
     * @param confidence 用于存储置信度阈值值的指针（0-100）
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     */
    eCmdCode_t getConfidence(uint8_t* confidence);

    /**
     * @fn getIOU
     * @brief 获取当前 IOU 阈值
     * 
     * 检索当前配置的 IOU 阈值值。
     *
     * @param iou 用于存储 IOU 阈值值的指针（0-100）
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     */
    eCmdCode_t getIOU(uint8_t* iou);

    /**
     * @fn getLearnSimilarity
     * @brief 获取当前相似度阈值
     * 
     * 检索当前配置的相似度阈值值。
     *
     * @param Similarity 用于存储相似度阈值值的指针（0-100）
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     */
    eCmdCode_t getLearnSimilarity(uint8_t* Similarity);

    /**
     * @fn getLearnList
     * @brief 获取指定模型的学习目标列表
     * 
     * 检索指定检测模型的所有学习目标名称列表。
     * 此函数可用于手势识别和姿态学习应用。
     *
     * @param model 类型为 `eModel_t` 的模型类型：
     *              - `eHand` - 获取学习手势列表
     *              - `ePose` - 获取学习姿态列表
     * @return 包含学习目标名称的字符串向量。出错时返回空向量。
     */
    LearnList getLearnList(eModel_t model);

    /**
     * @fn availableResult
     * @brief 检查是否有可用的检测结果
     * @return 如果有可用的检测结果返回 true，否则返回 false
     */
    bool availableResult();

    /**
     * @fn popResult
     * @brief 获取并弹出下一个检测结果
     * @return 指向 Result 对象的指针。如果没有可用结果则返回 NULL。
     * @note Result 对象可能是 PoseResult 或 HandResult 类型，取决于当前设置的模型类型。
     *       使用完后，结果会被标记为已使用。
     */
    Result *popResult();

    /**
     * @fn setBaud
     * @brief 设置 UART 波特率
     * 
     * 配置 UART 通信波特率。用户可以根据需要选择合适的
     * 波特率，以确保与设备的稳定和有效通信。
     *
     * @param baud 类型为 `eBaudConfig_t` 的波特率配置，可能的值包括：
     *             - `eBaud_9600`  - 9600 波特
     *             - `eBaud_14400` - 14400 波特
     *             - `eBaud_19200` - 19200 波特
     *             - `eBaud_38400` - 38400 波特
     *             - `eBaud_57600` - 57600 波特
     *             - `eBaud_115200`- 115200 波特
     *             - `eBaud_230400`- 230400 波特
     *             - `eBaud_460800`- 460800 波特
     *             - `eBaud_921600`- 921600 波特（默认）
     * @return 如果成功设置波特率返回 true，否则返回 false
     */
    bool setBaud(eBaudConfig_t baud);

    // Result 基类包含以下公共成员：
    uint8_t id;           // 检测目标ID
    uint16_t xLeft;        // 边界框左上角X坐标
    uint16_t yTop;         // 边界框左上角Y坐标
    uint16_t width;        // 边界框宽度
    uint16_t height;       // 边界框高度
    uint8_t confidence;    // 置信度（0-100）
    String name;           // 检测目标名称
    bool used;             // 是否已使用标记

    // PoseResult 类（人体姿态检测结果）额外包含以下关键点：
    PointU16 nose;         // 鼻子
    PointU16 leye;         // 左眼
    PointU16 reye;         // 右眼
    PointU16 lear;         // 左耳
    PointU16 rear;         // 右耳
    PointU16 lshoulder;    // 左肩
    PointU16 rshoulder;    // 右肩
    PointU16 lelbow;       // 左肘
    PointU16 relbow;       // 右肘
    PointU16 lwrist;       // 左手腕
    PointU16 rwrist;       // 右手腕
    PointU16 lhip;         // 左髋
    PointU16 rhip;         // 右髋
    PointU16 lknee;        // 左膝
    PointU16 rknee;        // 右膝
    PointU16 lankle;       // 左踝
    PointU16 rankle;        // 右踝

    // HandResult 类（手势检测结果）额外包含以下关键点：
    PointU16 wrist;        // 手腕
    PointU16 thumbCmc;     // 拇指CMC
    PointU16 thumbMcp;     // 拇指MCP
    PointU16 thumbIp;      // 拇指IP
    PointU16 thumbTip;     // 拇指指尖
    PointU16 indexFingerMcp;   // 食指MCP
    PointU16 indexFingerPip;   // 食指PIP
    PointU16 indexFingerDip;   // 食指DIP
    PointU16 indexFingerTip;   // 食指指尖
    PointU16 middleFingerMcp; // 中指MCP
    PointU16 middleFingerPip; // 中指PIP
    PointU16 middleFingerDip; // 中指DIP
    PointU16 middleFingerTip; // 中指指尖
    PointU16 ringFingerMcp;   // 无名指MCP
    PointU16 ringFingerPip;   // 无名指PIP
    PointU16 ringFingerDip;   // 无名指DIP
    PointU16 ringFingerTip;   // 无名指指尖
    PointU16 pinkyFingerMcp; // 小指MCP
    PointU16 pinkyFingerPip; // 小指PIP
    PointU16 pinkyFingerDip; // 小指DIP
    PointU16 pinkyFingerTip; // 小指指尖
```

## 兼容性

| MCU                | 表现良好 | 表现异常 | 未测试 | 备注 |
| ------------------ | :------: | :------: | :-----: | ---- |
| Arduino Uno        |    √     |          |         | 需要 SoftwareSerial |
| Arduino Leonardo   |    √     |          |         |                     |
| Arduino MEGA2560   |    √     |          |         |                     |
| FireBeetle-ESP32-E |    √     |          |         |                     |
| ESP8266            |    √     |          |         | 需要 SoftwareSerial |
| FireBeetle-M0      |    √     |          |         |                     |
| Micro:bit          |    √     |          |         |                     |
| Raspberry Pi       |    √     |          |         |                     |

## 历史

- Date 2026-01-09
- Version V1.0.0

## 贡献者

Written by DFRobot, 2026.01.09 (Welcome to our [website](https://www.dfrobot.com/))
