/*!
 * @file DFRobot_HumanPose.h
 * @brief Define the basic structure of class DFRobot_HumanPose, the implementation of basic methods.
 * @details This is a human pose detection sensor that can be controlled through I2C/UART ports. Supports human pose detection, hand detection, and learned target recognition.
 * @copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @License     The MIT License (MIT)
 * @author [thdyyl](yuanlong.yu@dfrobot.com)
 * @version  V1.0
 * @date  2026-02-04
 * @url         https://github.com/DFRobot/DFRobot_HumanPose
 */
#ifndef DFROBOT_HUMANPOSE_H
#define DFROBOT_HUMANPOSE_H

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Avoid min/max macro conflicts on nRF5/microbit and other platforms. */
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

/* Detect small-memory boards (microbit, nRF5, AVR) for reduced buffers. */
#if defined(ARDUINO_BBC_MICROBIT) || defined(ARDUINO_BBC_MICROBIT_V2) || defined(ARDUINO_AVR_LEONARDO) || defined(ARDUINO_AVR_UNO) || defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_ARCH_NRF5)
#define USE_SIMPLE_CONTAINERS 1
#else
#define USE_SIMPLE_CONTAINERS 0
#endif

/* Extra-tight profile for UNO-class RAM (ATmega328p family). */
#ifndef DFR_HUMANPOSE_TINY_RAM
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MINI)
#define DFR_HUMANPOSE_TINY_RAM 1
#else
#define DFR_HUMANPOSE_TINY_RAM 0
#endif
#endif

/* Low-memory policy for runtime defaults:
 * - small-memory boards: binary + box-only
 * - others: binary + keypoints
 */
#if USE_SIMPLE_CONTAINERS
#define DFR_HUMANPOSE_LOW_MEMORY 1
#else
#define DFR_HUMANPOSE_LOW_MEMORY 0
#endif

#if DFR_HUMANPOSE_LOW_MEMORY
#define DFR_HUMANPOSE_SMALL_RAM_PROFILE 1
#else
#define DFR_HUMANPOSE_SMALL_RAM_PROFILE 0
#endif

#if DFR_HUMANPOSE_LOW_MEMORY
#define DFR_HUMANPOSE_LARGE_MEMORY 0
#else
#define DFR_HUMANPOSE_LARGE_MEMORY 1
#endif

/* Backward compatibility for older code. */
#ifndef DFR_HUMANPOSE_AVR_BINARY_ONLY
#define DFR_HUMANPOSE_AVR_BINARY_ONLY DFR_HUMANPOSE_SMALL_RAM_PROFILE
#endif

#include "Result.h"

#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include <SoftwareSerial.h>
#endif

#ifdef LDBG
#undef LDBG
#endif

#ifndef DFR_HUMANPOSE_DEBUG
#define DFR_HUMANPOSE_DEBUG 0
#endif

#if DFR_HUMANPOSE_DEBUG
#define LDBG(v)                                                                        \
  do {                                                                                 \
    Serial.print(F("[HPDBG] "));                                                       \
    Serial.print(__FUNCTION__);                                                        \
    Serial.print(F(":"));                                                              \
    Serial.print(__LINE__);                                                            \
    Serial.print(F(" -> "));                                                           \
    Serial.println(v);                                                                 \
  } while (0)
#else
#define LDBG(...)
#endif

/** Fixed-capacity list for learned target names (avoids std::vector for AVR/min-max compatibility). */
#if USE_SIMPLE_CONTAINERS
#if DFR_HUMANPOSE_SMALL_RAM_PROFILE
#if DFR_HUMANPOSE_TINY_RAM
#define LEARN_LIST_CAP 4
#else
#define LEARN_LIST_CAP 8
#endif
#else
#define LEARN_LIST_CAP 16
#endif
#else
#define LEARN_LIST_CAP 24
#endif
class LearnList {
public:
  static const size_t CAP = LEARN_LIST_CAP;
  void                clear()
  {
    _count = 0;
  }
  void push_back(const String &s)
  {
    if (_count < CAP)
      _items[_count++] = s;
  }
  size_t size() const
  {
    return _count;
  }
  String &operator[](size_t i)
  {
    return _items[i];
  }
  const String &operator[](size_t i) const
  {
    return _items[i];
  }

private:
  String _items[CAP];
  size_t _count = 0;
};

class DFRobot_HumanPose {
protected:
#define I2C_ADDRESS    (0x3A)
#define HUMANPOSE_NAME "DFRobot Human Pose"

#define HEADER_LEN     (uint8_t)4
#if DFR_HUMANPOSE_SMALL_RAM_PROFILE
// Small-memory boards often have tight transport buffers. Keep payload conservative.
#define MAX_PL_LEN     (uint8_t)24
#else
#define MAX_PL_LEN     (uint8_t)50
#endif
#define MAX_SPI_PL_LEN (uint16_t)4095
#define CHECKSUM_LEN   (uint8_t)2

#define PACKET_SIZE (uint16_t)(HEADER_LEN + MAX_PL_LEN + CHECKSUM_LEN)

#define FEATURE_TRANSPORT               0x10
#define FEATURE_TRANSPORT_CMD_READ      0x01
#define FEATURE_TRANSPORT_CMD_WRITE     0x02
#define FEATURE_TRANSPORT_CMD_AVAILABLE 0x03
#define FEATURE_TRANSPORT_CMD_RESET     0x06

#ifndef RX_MAX_SIZE
#if USE_SIMPLE_CONTAINERS
#if DFR_HUMANPOSE_SMALL_RAM_PROFILE
#if DFR_HUMANPOSE_TINY_RAM
#define RX_MAX_SIZE 192
#else
#define RX_MAX_SIZE 256
#endif
#else
#define RX_MAX_SIZE 512
#endif
#elif defined(ARDUINO_ARCH_ESP32)
#define RX_MAX_SIZE 32 * 1024
#elif defined(ESP8266)
#define RX_MAX_SIZE 2 * 1024
#else
#define RX_MAX_SIZE 4 * 1024
#endif
#endif

#ifndef AT_PAYLOAD_MAX_SIZE
#if USE_SIMPLE_CONTAINERS
#if DFR_HUMANPOSE_SMALL_RAM_PROFILE
#if DFR_HUMANPOSE_TINY_RAM
#define AT_PAYLOAD_MAX_SIZE 192
#else
#define AT_PAYLOAD_MAX_SIZE 192
#endif
#else
#define AT_PAYLOAD_MAX_SIZE 384
#endif
#else
#define AT_PAYLOAD_MAX_SIZE RX_MAX_SIZE
#endif
#endif

#ifndef TX_MAX_SIZE
#if USE_SIMPLE_CONTAINERS
#define TX_MAX_SIZE 32
#elif defined(ESP8266)
#define TX_MAX_SIZE 512
#else
#define TX_MAX_SIZE 4 * 1024
#endif
#endif

#ifndef I2C_CLOCK
#define I2C_CLOCK 400000
#endif

#ifndef UART_BAUD
#define UART_BAUD 9600
#endif

#define CMD_PRE "AT+"
#define CMD_SUF "\r\n"

#define CMD_TYPE_RESPONSE 0
#define CMD_TYPE_EVENT    1
#define CMD_TYPE_LOG      2

#if DFR_HUMANPOSE_SMALL_RAM_PROFILE
#if DFR_HUMANPOSE_TINY_RAM
#define MAX_RESULT_NUM 1
#else
#define MAX_RESULT_NUM 2
#endif
#else
#define MAX_RESULT_NUM 4
#endif

#if DFR_HUMANPOSE_TINY_RAM
#define DEVICE_NAME_BUF_SIZE 20
#define AT_RSP_NAME_BUF_SIZE 12
#else
#define DEVICE_NAME_BUF_SIZE 24
#define AT_RSP_NAME_BUF_SIZE 20
#endif

#define AT_NAME        "NAME"
#define AT_INVOKE      "INVOKE"
#define AT_TSCORE      "TSCORE"
#define AT_TIOU        "TIOU"
#define AT_MODELS      "MODELS?"
#define AT_MODEL       "MODEL"
#define EVENT_INVOKE   "INVOKE"
#define AT_TSIMILARITY "TSIMILARITY"
#define AT_BAUD        "BAUDRATE"
#define AT_POSELIST    "POSELIST?"
#define AT_HANDLIST    "HANDLIST?"
#define AT_TPROTO      "TPROTO"
#define AT_TPKTSZ      "TPKTSZ"
#define AT_TKPTS       "TKPTS"

public:
  /**
   * @enum eCmdCode_t
   * @brief Command execution status code
   */
  typedef enum {
    eOK       = 0,     ///< Operation successful
    eAgain    = 1,     ///< Operation needs to be retried
    eLog      = 2,     ///< Log message
    eTimedOut = 3,     ///< Operation timed out
    eIO       = 4,     ///< Input/output error
    eINVAL    = 5,     ///< Invalid argument
    eNOMEM    = 6,     ///< Out of memory
    eBUSY     = 7,     ///< Device busy
    eNOTSUP   = 8,     ///< Operation not supported
    ePERM     = 9,     ///< Permission denied
    eUnknown  = 10,    ///< Unknown error
  } eCmdCode_t;

  /**
   * @enum eModel_t
   * @brief Detection model type
   */
  typedef enum {
    eHand = 1,    ///< Hand detection model
    ePose = 3,    ///< Human pose detection model
    eGes  = 4,    ///< GES: fixed gesture classification (MODEL 4); no user learn list on device
  } eModel_t;

protected:
  Result  *_result[MAX_RESULT_NUM];
#if !DFR_HUMANPOSE_LOW_MEMORY
  bool     _result_is_pose[MAX_RESULT_NUM];
  bool     _result_is_ges[MAX_RESULT_NUM];
#endif
  int      _wait_delay;
  uint32_t rx_end;
  char    *tx_buf;
  uint32_t tx_len;
  char    *rx_buf;
  uint32_t rx_len;

  uint8_t   _ret_data;
  char      _name[DEVICE_NAME_BUF_SIZE] = { 0 };
#if !DFR_HUMANPOSE_LOW_MEMORY
  LearnList _learn_list;
#endif
  LearnList _pose_class_list;
  LearnList _hand_class_list;
  bool      _binary_mode = false;
  eModel_t  _current_model = eHand;

  bool   _at_rsp_ready = false;
  int8_t _at_rsp_type = CMD_TYPE_RESPONSE;
  int16_t _at_rsp_code = 0;
  uint16_t _at_rsp_cmd_id = 0;
  char _at_rsp_name[AT_RSP_NAME_BUF_SIZE] = { 0 };

  bool    _invoke_event_ready = false;
  int16_t _invoke_event_code = 0;
  uint16_t _invoke_model_id = 0;
  uint8_t *_at_payload_buf = NULL;
  uint32_t _at_payload_len = 0;
  uint16_t _at_payload_cap = 0;

#if DFR_HUMANPOSE_SMALL_RAM_PROFILE
  // Small-memory profile uses static buffers to avoid heap allocation failure/fragmentation.
  char    _rx_buf_static[RX_MAX_SIZE + 1];
  char    _tx_buf_static[TX_MAX_SIZE + 1];
  uint8_t _at_payload_static[AT_PAYLOAD_MAX_SIZE];
#endif

  typedef struct {
    bool     used;
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    uint8_t  score;
    uint16_t target;
#if !DFR_HUMANPOSE_LOW_MEMORY
    bool     is_pose;
    PointU16 points[21];
    uint8_t  point_valid[21];
#endif
  } binary_kp_result_t;

  binary_kp_result_t _bin_results[MAX_RESULT_NUM];
  uint16_t _bin_result_count = 0;

  // Command processing helper functions
  /**
   * @fn wait
   * @brief Wait for command response from the sensor
   * @details Internal helper function to wait for response matching the specified command type and name
   * @param type Command type (CMD_TYPE_RESPONSE, CMD_TYPE_EVENT, or CMD_TYPE_LOG)
   * @param cmd Command name string to match
   * @param timeout Timeout value in milliseconds (default is 1000)
   * @return Status code of type `eCmdCode_t`. Returns `eOK` if response received successfully, `eTimedOut` if timeout occurs
   */
  int wait(int type, const char *cmd, uint32_t timeout = 1000);

  bool process_binary_frames();
  bool process_binary_at_response(uint8_t flags, const uint8_t *payload, uint16_t payload_len);
  bool process_binary_invoke(uint8_t msg_type, uint8_t flags, const uint8_t *payload, uint16_t payload_len);
  bool command_matches(uint16_t cmd_id, const char *cmd) const;
  String resolve_class_name(uint16_t id) const;
  void clear_binary_results();
  void finalize_binary_results();

  /**
   * @fn getName
   * @brief Get the device name
   * @details Internal helper function to retrieve the device name from the sensor
   * @param name Pointer to character buffer to store the device name
   * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
   */
  eCmdCode_t getName(char *name);
  /**
   * @fn available
   * @brief Check if data is available for reading (pure virtual function)
   * @return Number of bytes available for reading
   */
  virtual int available() = 0;

  /**
   * @fn read
   * @brief Read data from the sensor (pure virtual function)
   * @param data Buffer to store the read data
   * @param len Maximum number of bytes to read
   * @return Number of bytes actually read
   */
  virtual int read(char *data, int len) = 0;

  /**
   * @fn write
   * @brief Write data to the sensor (pure virtual function)
   * @param data Data to write
   * @param len Number of bytes to write
   * @return Number of bytes actually written
   */
  virtual int write(const char *data, int len) = 0;

public:
  /**
   * @fn DFRobot_HumanPose
   * @brief Constructor of DFRobot_HumanPose class
   */
  DFRobot_HumanPose();

  /**
   * @fn ~DFRobot_HumanPose
   * @brief Destructor of DFRobot_HumanPose class
   */
  ~DFRobot_HumanPose();

  /**
   * @fn begin
   * @brief Initialize the sensor (binary AT mode, packet size, keypoints policy, verify name)
   * @return True if initialization is successful, otherwise false
   */
  bool begin();

  /**
   * @fn getResult
   * @brief Get detection results from the sensor
   * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
   * @note After calling this function, the detection results will be stored in the internal result array.
   *       Use availableResult() and popResult(). `ePose` -> PoseResult, `eHand` -> HandResult, `eGes` -> Result (bbox + class name).
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
   * @param confidence Confidence threshold value (0-100). Default is typically 60.
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
   * Selects which detection model to use: hand, human pose, or GES fixed gesture classification.
   *
   * @param model Model type of type `eModel_t`, with possible values including:
   *              - `eHand` - Hand detection model
   *              - `ePose` - Human pose detection model
   *              - `eGes`  - GES fixed gesture classification (MODEL 4)
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
  eCmdCode_t getConfidence(uint8_t *confidence);

  /**
   * @fn getIOU
   * @brief Get the current IOU threshold
   *
   * Retrieves the currently configured IOU threshold value.
   *
   * @param iou Pointer to store the IOU threshold value (0-100)
   * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
   */
  eCmdCode_t getIOU(uint8_t *iou);

  /**
   * @fn getLearnSimilarity
   * @brief Get the current similarity threshold
   *
   * Retrieves the currently configured similarity threshold value.
   *
   * @param Similarity Pointer to store the similarity threshold value (0-100)
   * @return Status code of type `eCmdCode_t`. Returns `eOK` if successful, otherwise returns an error code.
   */
  eCmdCode_t getLearnSimilarity(uint8_t *Similarity);

  /**
   * @fn setKeypointOutput
   * @brief Configure whether INVOKE output includes keypoints.
   * @param enable 1: include keypoints, 0: boxes only
   * @return Status code of type `eCmdCode_t`.
   */
  eCmdCode_t setKeypointOutput(bool enable);

  /**
   * @fn getKeypointOutput
   * @brief Get current keypoint output mode.
   * @param enable Pointer to store 1(include keypoints)/0(boxes only)
   * @return Status code of type `eCmdCode_t`.
   */
  eCmdCode_t getKeypointOutput(uint8_t *enable);

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
   *              - `eGes`  - Not applicable (returns empty list; fixed class names only, id 0..14)
   * @return Vector of learned names. Empty for `eGes` or on error.
   */
  LearnList getLearnList(eModel_t model);

  /**
   * @fn availableResult
   * @brief Returns true if at least one detection result from the last successful getResult() is unread.
   * @return true if an unread Result exists, false otherwise
   */
  bool availableResult();

  /**
   * @fn popResult
   * @brief Take the next unread detection result (marks it used). Cast by current model: PoseResult / HandResult / Result.
   * @return Pointer to Result, or NULL if none available
   */
  Result *popResult();
};

/**
 * @class DFRobot_HumanPose_I2C
 * @brief I2C communication class for DFRobot_HumanPose sensor
 */
class DFRobot_HumanPose_I2C : public DFRobot_HumanPose {
protected:
  TwoWire *_wire;        ///< I2C communication interface
  uint8_t  __address;    ///< I2C device address

public:
  /**
   * @fn DFRobot_HumanPose_I2C
   * @brief Constructor of DFRobot_HumanPose_I2C class
   * @param wire Pointer to TwoWire object (typically &Wire)
   * @param address I2C device address (default is 0x3A)
   */
  DFRobot_HumanPose_I2C(TwoWire *wire, uint8_t address);

  /**
   * @fn ~DFRobot_HumanPose_I2C
   * @brief Destructor of DFRobot_HumanPose_I2C class
   */
  ~DFRobot_HumanPose_I2C();

  /**
   * @fn begin
   * @brief Initialize the I2C communication and sensor
   * @return True if initialization is successful, otherwise false
   */
  bool begin(void);

protected:
  /**
   * @fn available
   * @brief Check if data is available for reading via I2C
   * @return Number of bytes available for reading
   */
  int available();

  /**
   * @fn read
   * @brief Read data from the sensor via I2C
   * @param data Buffer to store the read data
   * @param len Maximum number of bytes to read
   * @return Number of bytes actually read
   */
  int read(char *data, int len);

  /**
   * @fn write
   * @brief Write data to the sensor via I2C
   * @param data Data to write
   * @param len Number of bytes to write
   * @return Number of bytes actually written
   */
  int write(const char *data, int len);
};

/**
 * @class DFRobot_HumanPose_UART
 * @brief UART communication class for DFRobot_HumanPose sensor
 */
class DFRobot_HumanPose_UART : public DFRobot_HumanPose {
public:
  /**
   * @enum eBaudConfig_t
   * @brief Baud rate configuration options
   */
  /* Use uint32_t on AVR: int is 16-bit; values >32767 (e.g. 115200) would overflow. */
  typedef enum : uint32_t {
    eBaud_9600   = 9600,      ///< Baud rate 9600 (default UART link; see UART_BAUD)
    eBaud_14400  = 14400,     ///< Baud rate 14400
    eBaud_19200  = 19200,     ///< Baud rate 19200
    eBaud_38400  = 38400,     ///< Baud rate 38400
    eBaud_57600  = 57600,     ///< Baud rate 57600
    eBaud_115200 = 115200,    ///< Baud rate 115200
    eBaud_230400 = 230400,    ///< Baud rate 230400
    eBaud_460800 = 460800,    ///< Baud rate 460800
    eBaud_921600 = 921600,    ///< Baud rate 921600 (high speed)
  } eBaudConfig_t;

protected:
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
  SoftwareSerial *_serial;    ///< SoftwareSerial object for UNO/ESP8266
#else
  HardwareSerial *_serial;    ///< HardwareSerial object
  uint8_t         __rxpin;    ///< RX pin number (for ESP32)
  uint8_t         __txpin;    ///< TX pin number (for ESP32)
#endif
  uint32_t __baud;    ///< Baud rate value

public:
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
  /**
   * @fn DFRobot_HumanPose_UART
   * @brief Constructor of DFRobot_HumanPose_UART class (for UNO/ESP8266)
   * @param sSerial Pointer to SoftwareSerial object
   * @param baud Baud rate value (default UART_BAUD, 9600)
   */
  DFRobot_HumanPose_UART(SoftwareSerial *sSerial, uint32_t baud = UART_BAUD);
#else
  /**
   * @fn DFRobot_HumanPose_UART
   * @brief Constructor of DFRobot_HumanPose_UART class
   * @param hSerial Pointer to HardwareSerial object (typically &Serial1)
   * @param baud Baud rate value (default UART_BAUD, 9600)
   * @param rxpin RX pin number (default is 0, required for ESP32)
   * @param txpin TX pin number (default is 0, required for ESP32)
   */
  DFRobot_HumanPose_UART(HardwareSerial *hSerial, uint32_t baud = UART_BAUD, uint8_t rxpin = 0, uint8_t txpin = 0);
#endif

  /**
   * @fn ~DFRobot_HumanPose_UART
   * @brief Destructor of DFRobot_HumanPose_UART class
   */
  ~DFRobot_HumanPose_UART();

  /**
   * @fn begin
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
   *             - `eBaud_9600`  - 9600 baud (default UART link; UART_BAUD)
   *             - `eBaud_14400` - 14400 baud
   *             - `eBaud_19200` - 19200 baud
   *             - `eBaud_38400` - 38400 baud
   *             - `eBaud_57600` - 57600 baud
   *             - `eBaud_115200`- 115200 baud
   *             - `eBaud_230400`- 230400 baud
   *             - `eBaud_460800`- 460800 baud
   *             - `eBaud_921600`- 921600 baud (high speed)
   * @return True if the baud rate is set successfully, otherwise false
   */
  bool setBaud(eBaudConfig_t baud);

protected:
  /**
   * @fn available
   * @brief Check if data is available for reading via UART
   * @return Number of bytes available for reading
   */
  int available();

  /**
   * @fn read
   * @brief Read data from the sensor via UART
   * @param data Buffer to store the read data
   * @param len Maximum number of bytes to read
   * @return Number of bytes actually read
   */
  int read(char *data, int len);

  /**
   * @fn write
   * @brief Write data to the sensor via UART
   * @param data Data to write
   * @param len Number of bytes to write
   * @return Number of bytes actually written
   */
  int write(const char *data, int len);
};

#endif
