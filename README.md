# DFRobot_HumanPose

* [Chinese Version](./README_CN.md)

HumanPose is a sensor that can detect human poses and gestures.

## Table of Contents

* [Description](#description)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Description

Arduino library for controlling HumanPose sensor. This is a human pose detection sensor that can be controlled through I2C/UART ports. EPII_CM55M_APP_S has functions such as human pose detection, hand detection, etc.

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
     * @note After calling this function, the detection results will be stored in the keypoints vector.
     *       You can access the results using the keypoints() method.
     */
    eCmdCode_t getResult();

    /**
     * @fn setTScore
     * @brief Set the detection threshold score
     * 
     * Sets the minimum confidence score required for a detection to be considered valid.
     * Higher values result in fewer but more reliable detections. Lower values allow more
     * detections but may include false positives.
     *
     * @param tscore Threshold score value (0-100). Default is typically 60.
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     */
    eCmdCode_t setTScore(uint8_t tscore);

    /**
     * @fn setTIOU
     * @brief Set the Intersection over Union (IOU) threshold
     * 
     * Sets the IOU threshold used for non-maximum suppression during object detection.
     * This parameter helps filter out overlapping detections.
     *
     * @param tious IOU threshold value (0-100). Default is typically 45.
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     */
    eCmdCode_t setTIOU(uint8_t tious);

    /**
     * @fn setModel
     * @brief Set the detection model
     * 
     * Selects which detection model to use. The sensor supports hand detection and human pose detection.
     *
     * @param model Model type of type `eModel_t`, with possible values including:
     *              - `eHand` - Hand detection model
     *              - `ePose` - Human pose detection model
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     */
    eCmdCode_t setModel(eModel_t model);

    /**
     * @fn setSimilarity
     * @brief Set the similarity threshold for learned targets
     * 
     * Sets the similarity threshold used when matching detected objects against learned targets.
     * This parameter is used for gesture recognition and learned pose matching.
     *
     * @param Similarity Similarity threshold value (0-100). Default is typically 60.
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     */
    eCmdCode_t setSimilarity(uint8_t Similarity);

    /**
     * @fn getTScore
     * @brief Get the current detection threshold score
     * 
     * Retrieves the currently configured detection threshold score.
     *
     * @param score Pointer to store the threshold score value (0-100)
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     */
    eCmdCode_t getTScore(uint8_t* score);

    /**
     * @fn getTIOU
     * @brief Get the current IOU threshold
     * 
     * Retrieves the currently configured IOU threshold value.
     *
     * @param iou Pointer to store the IOU threshold value (0-100)
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     */
    eCmdCode_t getTIOU(uint8_t* iou);

    /**
     * @fn getModel
     * @brief Get the current detection model
     * 
     * Retrieves the currently active detection model type.
     *
     * @param model Pointer to a character buffer to store the model name ("HAND" or "POSE")
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     * @note The buffer should be large enough to store the model name string.
     */
    eCmdCode_t getModel(char* model);

    /**
     * @fn getSimilarity
     * @brief Get the current similarity threshold
     * 
     * Retrieves the currently configured similarity threshold value.
     *
     * @param Similarity Pointer to store the similarity threshold value (0-100)
     * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
     */
    eCmdCode_t getSimilarity(uint8_t* Similarity);

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
    std::vector<std::string> getLearnList(eModel_t model);

    /**
     * @fn boxes
     * @brief Get reference to the bounding boxes vector
     * @return Reference to vector containing detected bounding boxes
     */
    std::vector<sBox_t>& boxes();

    /**
     * @fn classes
     * @brief Get reference to the classification results vector
     * @return Reference to vector containing classification results
     */
    std::vector<sClass_t>& classes();

    /**
     * @fn points
     * @brief Get reference to the keypoints vector
     * @return Reference to vector containing detected keypoints
     */
    std::vector<sPoint_t>& points();

    /**
     * @fn keypoints
     * @brief Get reference to the keypoints structure vector
     * @return Reference to vector containing keypoints structures (each includes a bounding box and keypoint array)
     * @note This is the primary method to access detection results after calling getResult()
     */
    std::vector<sKeypoints_t>& keypoints();

    /**
     * @fn DFRobot_HumanPose_I2C
     * @brief Constructor of DFRobot_HumanPose_I2C class
     * @param wire Pointer to TwoWire object (typically &Wire)
     * @param address I2C device address (default is 0x3A)
     */
    DFRobot_HumanPose_I2C(TwoWire *wire, uint8_t address);

    /**
     * @fn DFRobot_HumanPose_I2C::begin
     * @brief Initialize the I2C communication and sensor
     * @return True if initialization is successful, otherwise false
     */
    bool begin(void);

    /**
     * @fn DFRobot_HumanPose_UART
     * @brief Constructor of DFRobot_HumanPose_UART class
     * @param hSerial Pointer to HardwareSerial object (typically &Serial1)
     * @param baud Baud rate value (default is 921600)
     * @param rxpin RX pin number (default is 0, required for ESP32)
     * @param txpin TX pin number (default is 0, required for ESP32)
     */
    DFRobot_HumanPose_UART(HardwareSerial *hSerial, uint32_t baud, uint8_t rxpin = 0, uint8_t txpin = 0);

    /**
     * @fn DFRobot_HumanPose_UART
     * @brief Constructor of DFRobot_HumanPose_UART class (for UNO/ESP8266)
     * @param sSerial Pointer to SoftwareSerial object
     * @param baud Baud rate value (e.g., 921600)
     */
    DFRobot_HumanPose_UART(SoftwareSerial *sSerial, uint32_t baud);

    /**
     * @fn DFRobot_HumanPose_UART::begin
     * @brief Initialize the UART communication and sensor
     * @return True if initialization is successful, otherwise false
     */
    bool begin(void);

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
```

## Compatibility

| MCU                | Work Well | Work Wrong | Untested | Remarks |
| ------------------ | :----------: | :----------: | :---------: | ----- |
| Arduino Uno        |      √       |              |             | Requires SoftwareSerial |
| Arduino Leonardo   |      √       |              |             |         |
| Arduino MEGA2560   |      √       |              |             |         |
| FireBeetle-ESP32-E |      √       |              |             |         |
| ESP8266            |      √       |              |             | Requires SoftwareSerial |
| FireBeetle-M0      |      √       |              |             |         |
| Micro:bit          |      √       |              |             |         |
| Raspberry Pi       |      √       |              |             |         |

## History

- Date 2026-01-09
- Version V1.0.0

## Credits

Written by DFRobot, 2026.01.09 (Welcome to our [website](https://www.dfrobot.com/))
