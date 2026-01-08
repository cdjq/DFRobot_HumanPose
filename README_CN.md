# DFRobot_HumanPose

* [English Version](./README.md)

HumanPose是一款可以检测人体姿态和手势的传感器

## 目录

* [简介](#简介)
* [安装](#安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [贡献者](#贡献者)

## 简介

提供用于控制HumanPose传感器的Arduino库。这是一个可以通过 I2C/UART 端口控制的人体姿态检测传感器。EPII_CM55M_APP_S 具有人体姿态检测、手势检测等功能。

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
     * @note 调用此函数后，检测结果将存储在 keypoints 向量中。
     *       您可以使用 keypoints() 方法访问结果。
     */
    eCmdCode_t getResult();

    /**
     * @fn setTScore
     * @brief 设置检测阈值分数
     * 
     * 设置检测被视为有效所需的最小置信度分数。
     * 较高的值会导致更少但更可靠的检测。较低的值允许更多
     * 检测但可能包含误报。
     *
     * @param tscore 阈值分数值（0-100）。默认值通常为 60。
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     */
    eCmdCode_t setTScore(uint8_t tscore);

    /**
     * @fn setTIOU
     * @brief 设置交并比（IOU）阈值
     * 
     * 设置用于目标检测过程中非极大值抑制的 IOU 阈值。
     * 此参数有助于过滤重叠的检测。
     *
     * @param tious IOU 阈值值（0-100）。默认值通常为 45。
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     */
    eCmdCode_t setTIOU(uint8_t tious);

    /**
     * @fn setModel
     * @brief 设置检测模型
     * 
     * 选择要使用的检测模型。传感器支持手势检测和人体姿态检测。
     *
     * @param model 类型为 `eModel_t` 的模型类型，可能的值包括：
     *              - `eHand` - 手势检测模型
     *              - `ePose` - 人体姿态检测模型
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     */
    eCmdCode_t setModel(eModel_t model);

    /**
     * @fn setSimilarity
     * @brief 设置学习目标的相似度阈值
     * 
     * 设置用于将检测到的目标与学习目标匹配的相似度阈值。
     * 此参数用于手势识别和学习姿态匹配。
     *
     * @param Similarity 相似度阈值值（0-100）。默认值通常为 60。
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     */
    eCmdCode_t setSimilarity(uint8_t Similarity);

    /**
     * @fn getTScore
     * @brief 获取当前检测阈值分数
     * 
     * 检索当前配置的检测阈值分数。
     *
     * @param score 用于存储阈值分数值的指针（0-100）
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     */
    eCmdCode_t getTScore(uint8_t* score);

    /**
     * @fn getTIOU
     * @brief 获取当前 IOU 阈值
     * 
     * 检索当前配置的 IOU 阈值值。
     *
     * @param iou 用于存储 IOU 阈值值的指针（0-100）
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     */
    eCmdCode_t getTIOU(uint8_t* iou);

    /**
     * @fn getModel
     * @brief 获取当前检测模型
     * 
     * 检索当前活动的检测模型类型。
     *
     * @param model 用于存储模型名称（"HAND" 或 "POSE"）的字符缓冲区指针
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     * @note 缓冲区应足够大以存储模型名称字符串。
     */
    eCmdCode_t getModel(char* model);

    /**
     * @fn getSimilarity
     * @brief 获取当前相似度阈值
     * 
     * 检索当前配置的相似度阈值值。
     *
     * @param Similarity 用于存储相似度阈值值的指针（0-100）
     * @return 类型为 `eCmdCode_t` 的状态代码。成功返回 `eOK`，否则返回错误代码。
     */
    eCmdCode_t getSimilarity(uint8_t* Similarity);

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
    std::vector<std::string> getLearnList(eModel_t model);

    /**
     * @fn boxes
     * @brief 获取边界框向量的引用
     * @return 包含检测到的边界框的向量引用
     */
    std::vector<sBox_t>& boxes();

    /**
     * @fn classes
     * @brief 获取分类结果向量的引用
     * @return 包含分类结果的向量引用
     */
    std::vector<sClass_t>& classes();

    /**
     * @fn points
     * @brief 获取关键点向量的引用
     * @return 包含检测到的关键点的向量引用
     */
    std::vector<sPoint_t>& points();

    /**
     * @fn keypoints
     * @brief 获取关键点结构向量的引用
     * @return 包含关键点结构的向量引用（每个包括边界框和关键点数组）
     * @note 这是在调用 getResult() 后访问检测结果的主要方法
     */
    std::vector<sKeypoints_t>& keypoints();

    /**
     * @fn DFRobot_HumanPose_I2C
     * @brief DFRobot_HumanPose_I2C 类的构造函数
     * @param wire TwoWire 对象指针（通常为 &Wire）
     * @param address I2C 设备地址（默认为 0x3A）
     */
    DFRobot_HumanPose_I2C(TwoWire *wire, uint8_t address);

    /**
     * @fn DFRobot_HumanPose_I2C::begin
     * @brief 初始化 I2C 通信和传感器
     * @return 如果初始化成功返回 true，否则返回 false
     */
    bool begin(void);

    /**
     * @fn DFRobot_HumanPose_UART
     * @brief DFRobot_HumanPose_UART 类的构造函数（ESP32/其他）
     * @param hSerial HardwareSerial 对象指针（通常为 &Serial1）
     * @param baud 波特率值（默认为 921600）
     * @param rxpin RX 引脚号（默认为 0，ESP32 需要）
     * @param txpin TX 引脚号（默认为 0，ESP32 需要）
     */
    DFRobot_HumanPose_UART(HardwareSerial *hSerial, uint32_t baud, uint8_t rxpin = 0, uint8_t txpin = 0);

    /**
     * @fn DFRobot_HumanPose_UART
     * @brief DFRobot_HumanPose_UART 类的构造函数（UNO/ESP8266）
     * @param sSerial SoftwareSerial 对象指针
     * @param baud 波特率值（例如 921600）
     */
    DFRobot_HumanPose_UART(SoftwareSerial *sSerial, uint32_t baud);

    /**
     * @fn DFRobot_HumanPose_UART::begin
     * @brief 初始化 UART 通信和传感器
     * @return 如果初始化成功返回 true，否则返回 false
     */
    bool begin(void);

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
```

## 兼容性

MCU                | 表现良好	|表现异常	|未测试	|备注 |
------------------ | :----------: | :----------: | :---------: | -----
Arduino Uno        |      √       |              |             | 需要 SoftwareSerial
Arduino Leonardo   |      √       |              |             | 
Arduino MEGA2560   |      √       |              |             | 
FireBeetle-ESP32-E |      √       |              |             | 
ESP8266            |      √       |              |             | 需要 SoftwareSerial
FireBeetle-M0      |      √       |              |             | 
Micro:bit          |      √       |              |             | 
Raspberry Pi       |      √       |              |             | 

## 历史

- Date 2025-01-01
- Version V1.0.0

## 贡献者

Written by DFRobot, 2025.01.01 (Welcome to our [website](https://www.dfrobot.com/))
