# DFRobot_HumanPose

* [中文版](./README_CN.md)

HumanPose is a sensor capable of human pose detection, hand detection, and learned target recognition via I2C or UART.

## Product Link（https://www.dfrobot.com）

    Human Pose Sensor

## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary

Provide a Python library to control the Human Pose sensor on Raspberry Pi. Supports I2C and UART communication, human pose detection (17 keypoints), hand detection (21 keypoints), and learned target / gesture recognition.

## Installation

To use this library, first download it to Raspberry Pi and install pinpong:

```
pip install pinpong
pip3 install pinpong
```

Then open the use case folder. To execute a use case, enter `python xxx.py` in the command line. For example, to run the set_baud use case:

```
cd python/raspberrypi/examples
python set_baud.py
```

To run the pose detect blink use case:

```
python pose_detect_blink.py
```

To run the get pose result use case:

```
python get_pose_result.py
```

To run the get hand result use case:

```
python get_hand_result.py
```

## Methods

```python

    def begin(self):
        '''
          @brief Initialize the sensor and verify device name
          @return True: Initialization succeeded, False: Initialization failed
        '''

    def get_result(self):
        '''
          @brief Trigger one detection and wait for result; results are stored and read via available_result/pop_result
          @return CODE_OK: Success, CODE_TIMEOUT: Timeout
        '''

    def set_confidence(self, confidence):
        '''
          @brief Set detection confidence threshold (0-100), default typically 60
          @param confidence: Confidence threshold
          @return CODE_OK: Success, CODE_TIMEOUT: Timeout
        '''

    def set_iou(self, iou):
        '''
          @brief Set IOU threshold (0-100) for non-maximum suppression, default typically 45
          @param iou: IOU threshold
          @return CODE_OK: Success, CODE_TIMEOUT: Timeout
        '''

    def set_learn_similarity(self, similarity):
        '''
          @brief Set similarity threshold (0-100) for matching learned targets, default typically 60
          @param similarity: Similarity threshold
          @return CODE_OK: Success, CODE_TIMEOUT: Timeout
        '''

    def set_model_type(self, model):
        '''
          @brief Set detection model type
          @param model: MODEL_HAND (1) for hand detection, MODEL_POSE (3) for human pose detection
          @return CODE_OK: Success, CODE_TIMEOUT: Timeout
        '''

    def get_confidence(self):
        '''
          @brief Get current confidence threshold
          @return Current value on success, None on timeout
        '''

    def get_iou(self):
        '''
          @brief Get current IOU threshold
          @return Current value on success, None on timeout
        '''

    def get_learn_similarity(self):
        '''
          @brief Get current learn similarity threshold
          @return Current value on success, None on timeout
        '''

    def get_learn_list(self, model):
        '''
          @brief Get the list of learned target names for the given model
          @param model: MODEL_POSE or MODEL_HAND
          @return List of names on success, None on timeout
        '''

    def available_result(self):
        '''
          @brief Check if there is at least one unread detection result
          @return True: At least one result available, False: No result
        '''

    def pop_result(self):
        '''
          @brief Pop one unread result (PoseResult or HandResult depending on model); marks it as used
          @return One Result instance, or None if no unread result
        '''

    # UART only:
    def set_baud(self, baudrate):
        '''
          @brief Set the UART baud rate of the sensor
          @param baudrate: Baud rate (e.g. BAUD_9600, BAUD_115200). Supported: BAUD_9600, BAUD_14400, BAUD_19200, BAUD_38400, BAUD_57600, BAUD_115200, BAUD_230400, BAUD_460800, BAUD_921600
          @n     Note: After setting, re-initialize the serial port and sensor with the new baud rate
          @return True: Set baud rate succeeded, False: Set baud rate failed
        '''
```

## Compatibility

| MCU            | Work Well | Work Wrong | Untested | Remarks |
| -------------- | :-------: | :--------: | :------: | ------- |
| Raspberry Pi   |     √     |            |          |         |
| UNIHIKER       |     √     |            |          |         |

* Python version

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |           |            |     √    |         |
| Python3 |     √     |            |          |         |

## History

- 2026/02/04 - Version 1.0.0 released.

## Credits

Written by thdyyl(yuanlong.yu@dfrobot.com), 2026.02.04 (Welcome to our [website](https://www.dfrobot.com/))
