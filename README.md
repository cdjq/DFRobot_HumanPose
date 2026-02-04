# DFRobot_HumanPose

* [Chinese Version](./README_CN.md)

HumanPose is a sensor library that can detect human poses and gestures.

## Table of Contents

* [Description](#description)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Description

Arduino library for controlling HumanPose sensor. This is a human pose detection sensor that can be controlled through I2C/UART ports. such as human pose detection, hand detection, etc. It supports real-time detection results including keypoint coordinates, bounding box information, and more.

## Installation

1. To use this library, first download the library file, paste it into the `\Arduino\libraries` directory, then open the example folder and run the examples in it.
2. To use this library, you also need to download the dependency: https://github.com/bblanchon/ArduinoJson

## Methods
```c++
    /**
     * @fn begin
     * @brief Initialize the sensor
     * @return True if initialization is successful, otherwise false
     */
    bool begin();

    /**
     * @fn getResult
     * @brief Get detection results from the sensor
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     * @note After calling this function, the detection results will be stored in the internal result array.
     *       You can access the results using availableResult() and popResult() methods.
     */
    eCmdCode_t getResult();

    /**
     * @fn setConfidence
     * @brief Set the detection confidence threshold
     *
     * Sets the minimum confidence score required for a detection to be considered valid.
     * Higher values result in fewer but more reliable detections. Lower values allow more
     * detections but may include false positives.
     *
     * @param confidence Threshold score value (0-100). Default is typically 60.
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     */
    eCmdCode_t setConfidence(uint8_t confidence);

    /**
     * @fn setIOU
     * @brief Set the Intersection over Union (IOU) threshold
     *
     * Sets the IOU threshold used for non-maximum suppression during object detection.
     * This parameter helps filter out overlapping detections.
     *
     * @param iou IOU threshold value (0-100). Default is typically 45.
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     */
    eCmdCode_t setIOU(uint8_t iou);

    /**
     * @fn setModelType
     * @brief Set the detection model
     *
     * Selects which detection model to use. The sensor supports hand detection and human pose detection.
     *
     * @param model Model type of type `eModel_t`, with possible values including:
     *              - `eHand` - Hand detection model
     *              - `ePose` - Human pose detection model
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     */
    eCmdCode_t setModelType(eModel_t model);

    /**
     * @fn setLearnSimilarity
     * @brief Set the similarity threshold for learned targets
     *
     * Sets the similarity threshold used when matching detected objects against learned targets.
     * This parameter is used for gesture recognition and learned pose matching.
     *
     * @param Similarity Similarity threshold value (0-100). Default is typically 60.
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     */
    eCmdCode_t setLearnSimilarity(uint8_t Similarity);

    /**
     * @fn getConfidence
     * @brief Get the current detection confidence threshold
     *
     * Retrieves the currently configured detection confidence threshold.
     *
     * @param confidence Pointer to store the confidence threshold value (0-100)
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     */
    eCmdCode_t getConfidence(uint8_t* confidence);

    /**
     * @fn getIOU
     * @brief Get the current IOU threshold
     *
     * Retrieves the currently configured IOU threshold value.
     *
     * @param iou Pointer to store the IOU threshold value (0-100)
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     */
    eCmdCode_t getIOU(uint8_t* iou);

    /**
     * @fn getLearnSimilarity
     * @brief Get the current similarity threshold
     *
     * Retrieves the currently configured similarity threshold value.
     *
     * @param Similarity Pointer to store the similarity threshold value (0-100)
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     */
    eCmdCode_t getLearnSimilarity(uint8_t* Similarity);

    /**
     * @fn getLearnList
     * @brief Get the list of learned targets for the specified model
     *
     * Retrieves a list of all learned target names for the specified detection model.
     * This function is useful for gesture recognition and pose learning applications.
     *
     * @param model Model type of type `eModel_t`:
     *              - `eHand` - Get list of learned hand gestures
     *              - `ePose` - Get list of learned poses
     * @return Vector of strings containing the names of learned targets. Returns empty vector on error.
     */
    LearnList getLearnList(eModel_t model);

    /**
     * @fn availableResult
     * @brief Check if detection results are available
     * @return True if detection results are available, otherwise false
     */
    bool availableResult();

    /**
     * @fn popResult
     * @brief Get and pop the next detection result
     * @return Pointer to Result object. Returns NULL if no results are available.
     * @note The Result object may be of type PoseResult or HandResult, depending on the currently set model type.
     *       After use, the result will be marked as used.
     */
    Result *popResult();

    /**
     * @fn setBaud
     * @brief Set the UART baud rate
     *
     * Configures the UART communication baud rate. Users can choose the appropriate
     * baud rate based on their needs to ensure stable and effective communication
     * with the device.
     *
     * @param baud Baud rate configuration of type `eBaudConfig_t`, with possible values including:
     *             - `eBaud_9600`  - 9600 baud
     *             - `eBaud_14400` - 14400 baud
     *             - `eBaud_19200` - 19200 baud
     *             - `eBaud_38400` - 38400 baud
     *             - `eBaud_57600` - 57600 baud
     *             - `eBaud_115200`- 115200 baud
     *             - `eBaud_230400`- 230400 baud
     *             - `eBaud_460800`- 460800 baud
     *             - `eBaud_921600`- 921600 baud (Default)
     * @return True if the baud rate is set successfully, otherwise false
     */
    bool setBaud(eBaudConfig_t baud);

    // Result base class contains the following public members:
    uint8_t id;           // Detected target ID
    uint16_t xLeft;        // Bounding box top-left X coordinate
    uint16_t yTop;         // Bounding box top-left Y coordinate
    uint16_t width;        // Bounding box width
    uint16_t height;       // Bounding box height
    /**
     * @brief Score of the result (0–100).
     *
     * Meaning depends on `id`:
     * - if `id == 0`: `score` is the detection confidence (probability/quality of detection).
     * - if `id != 0`: `score` is the similarity score (match degree to a learned class/gesture/pose).
     */
    uint8_t score;         // score (0-100)
    String name;           // Detected target name
    bool used;             // Used flag

    // PoseResult class (human pose detection result) additionally contains the following keypoints:
    PointU16 nose;         // Nose
    PointU16 leye;         // Left eye
    PointU16 reye;         // Right eye
    PointU16 lear;         // Left ear
    PointU16 rear;         // Right ear
    PointU16 lshoulder;    // Left shoulder
    PointU16 rshoulder;    // Right shoulder
    PointU16 lelbow;       // Left elbow
    PointU16 relbow;       // Right elbow
    PointU16 lwrist;       // Left wrist
    PointU16 rwrist;       // Right wrist
    PointU16 lhip;         // Left hip
    PointU16 rhip;         // Right hip
    PointU16 lknee;        // Left knee
    PointU16 rknee;        // Right knee
    PointU16 lankle;       // Left ankle
    PointU16 rankle;        // Right ankle

    // HandResult class (hand detection result) additionally contains the following keypoints:
    PointU16 wrist;        // Wrist
    PointU16 thumbCmc;     // Thumb CMC
    PointU16 thumbMcp;     // Thumb MCP
    PointU16 thumbIp;      // Thumb IP
    PointU16 thumbTip;     // Thumb tip
    PointU16 indexFingerMcp;   // Index finger MCP
    PointU16 indexFingerPip;   // Index finger PIP
    PointU16 indexFingerDip;   // Index finger DIP
    PointU16 indexFingerTip;   // Index finger tip
    PointU16 middleFingerMcp; // Middle finger MCP
    PointU16 middleFingerPip; // Middle finger PIP
    PointU16 middleFingerDip; // Middle finger DIP
    PointU16 middleFingerTip; // Middle finger tip
    PointU16 ringFingerMcp;   // Ring finger MCP
    PointU16 ringFingerPip;   // Ring finger PIP
    PointU16 ringFingerDip;   // Ring finger DIP
    PointU16 ringFingerTip;   // Ring finger tip
    PointU16 pinkyFingerMcp; // Pinky finger MCP
    PointU16 pinkyFingerPip; // Pinky finger PIP
    PointU16 pinkyFingerDip; // Pinky finger DIP
    PointU16 pinkyFingerTip; // Pinky finger tip
```

## Compatibility

| MCU                | Work Well | Work Wrong | Untested | Remarks |
| ------------------ | :--------: | :--------: | :------: | ------- |
| Arduino Uno        |            |     √      |          |                         |
| Arduino Leonardo   |            |     √      |          |                         |
| Arduino MEGA2560   |            |     √      |          |                         |
| FireBeetle-ESP32-E |     √      |            |          |                         |
| ESP8266            |     √      |            |          | Requires SoftwareSerial |
| FireBeetle-M0      |     √      |            |          |                         |
| Micro:bit          |     √      |            |          |                         |
| Raspberry Pi       |     √      |            |          |                         |

## History

- Date 2026-01-09
- Version V1.0.0

## Credits

Written by DFRobot, 2026.01.09 (Welcome to our [website](https://www.dfrobot.com/))
