/*!
 *@file DFRobot_HumanPose.h
 *@brief Define the basic structure of class DFRobot_HumanPose, the implementation of basic methods.
 *@details This is a human pose detection sensor that can be controlled through I2C/UART ports. Supports human pose detection, hand detection, and learned target recognition.
 *@copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 *@License     The MIT License (MIT)
 *@author [thdyyl](yuanlong.yu@dfrobot.com)
 *@version  V1.0
 *@date  2026-02-04
 *@url         https://github.com/DFRobot/DFRobot_HumanPose
*/
#ifndef DFROBOT_HUMANPOSE_H
#define DFROBOT_HUMANPOSE_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <Wire.h>

/* Avoid min/max macro conflicts on nRF5/microbit and other platforms. */
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

/* Detect small-memory boards (microbit, nRF5, AVR) for reduced buffers. Must be before ArduinoJson. */
#if defined(ARDUINO_BBC_MICROBIT) || defined(ARDUINO_BBC_MICROBIT_V2) || defined(ARDUINO_AVR_LEONARDO) || defined(ARDUINO_AVR_UNO) || \
    (defined(ARDUINO_ARCH_AVR) && !defined(ARDUINO_AVR_MEGA2560) && !defined(ARDUINO_AVR_MEGA)) || \
    defined(ARDUINO_ARCH_NRF5)
#define USE_SIMPLE_CONTAINERS 1
#else
#define USE_SIMPLE_CONTAINERS 0
#endif

/* ArduinoJson: reduce memory only for small-memory boards. Large boards (ESP32 etc) need Arduino String. */
#if USE_SIMPLE_CONTAINERS
#define ARDUINOJSON_ENABLE_STD_STRING      0
#define ARDUINOJSON_ENABLE_ARDUINO_STRING  0
#define ARDUINOJSON_ENABLE_ARDUINO_STREAM  0
#define ARDUINOJSON_ENABLE_STD_STREAM      0
#define ARDUINOJSON_DECODE_UNICODE         0
#define ARDUINOJSON_ENABLE_COMMENTS        0
#define ARDUINOJSON_ENABLE_NAN             0
#define ARDUINOJSON_ENABLE_INFINITY        0
#define ARDUINOJSON_USE_LONG_LONG          0
#define ARDUINOJSON_USE_DOUBLE             0
#endif

#include <ArduinoJson.h>

#include "Result.h"

#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include <SoftwareSerial.h>
#endif

// #define ENABLE_DBG
// #ifdef ENABLE_DBG
// #define LDBG(...)  {Serial.print("["); Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
// #else
// #define LDBG(...)
// #endif

/** Fixed-capacity list for learned target names (avoids std::vector for AVR/min-max compatibility). */
#if USE_SIMPLE_CONTAINERS
#define LEARN_LIST_CAP 16
#else
#define LEARN_LIST_CAP 24
#endif
class LearnList {
public:
  static const size_t CAP = LEARN_LIST_CAP;
  void clear() { _count = 0; }
  void push_back(const String &s) {
    if (_count < CAP) _items[_count++] = s;
  }
  size_t size() const { return _count; }
  String       &operator[](size_t i) { return _items[i]; }
  const String &operator[](size_t i) const { return _items[i]; }

private:
  String _items[CAP];
  size_t _count = 0;
};

class DFRobot_HumanPose {
protected:
#define I2C_ADDRESS    (0x3A)
#define HUMANPOSE_NAME "DFRobot Human Pose"

#define HEADER_LEN     (uint8_t)4
#define MAX_PL_LEN     (uint8_t)50
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
#define RX_MAX_SIZE 1024
#elif defined(ARDUINO_ARCH_ESP32)
#define RX_MAX_SIZE 32 * 1024
#elif defined(ESP8266)
#define RX_MAX_SIZE 2 * 1024
#else
#define RX_MAX_SIZE 4 * 1024
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
#define UART_BAUD 921600
#endif

#define RES_PRE "\r{"
#define RES_SUF "}\n"

#define RES_PRE_LEN (sizeof(RES_PRE) - 1)
#define RES_SUF_LEN (sizeof(RES_SUF) - 1)

#define CMD_PRE "AT+"
#define CMD_SUF "\r\n"

#define CMD_TYPE_RESPONSE 0
#define CMD_TYPE_EVENT    1
#define CMD_TYPE_LOG      2

#define MAX_RESULT_NUM 4

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
  } eModel_t;

protected:
  Result  *_result[MAX_RESULT_NUM];
  int      _wait_delay;
  uint32_t rx_end;

#if ARDUINOJSON_VERSION_MAJOR == 7
  JsonDocument response;    // for json response
#else
  #if USE_SIMPLE_CONTAINERS
  StaticJsonDocument<1024> response;    // smaller for microbit/nRF5/AVR
  #else
  /* INVOKE event (hand/pose keypoints) can be large; 8KB avoids deserializeJson NoMemory. */
  StaticJsonDocument<8192> response;    // for json response
  #endif
#endif
  char    *tx_buf;
  uint32_t tx_len;
  char    *rx_buf;
  uint32_t rx_len;
  char    *payload;

  uint8_t   _ret_data;
  char      _name[24] = "";
  LearnList _learn_list;
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

  /**
     * @fn parser_event
     * @brief Parse event data from the sensor
     * @details Internal helper function to parse JSON event data and extract pose_keypoints or hand_keypoints information,
     *          populating the internal keypoints vector with detection results
     */
  void parser_event();

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

  bool availableResult();

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
  typedef enum {
    eBaud_9600   = 9600,      ///< Baud rate 9600
    eBaud_14400  = 14400,     ///< Baud rate 14400
    eBaud_19200  = 19200,     ///< Baud rate 19200
    eBaud_38400  = 38400,     ///< Baud rate 38400
    eBaud_57600  = 57600,     ///< Baud rate 57600
    eBaud_115200 = 115200,    ///< Baud rate 115200
    eBaud_230400 = 230400,    ///< Baud rate 230400
    eBaud_460800 = 460800,    ///< Baud rate 460800
    eBaud_921600 = 921600,    ///< Baud rate 921600 (Default)
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
     * @param baud Baud rate value (e.g., 921600)
     */
  DFRobot_HumanPose_UART(SoftwareSerial *sSerial, uint32_t baud);
#else
  /**
     * @fn DFRobot_HumanPose_UART
     * @brief Constructor of DFRobot_HumanPose_UART class
     * @param hSerial Pointer to HardwareSerial object (typically &Serial1)
     * @param baud Baud rate value (default is 921600)
     * @param rxpin RX pin number (default is 0, required for ESP32)
     * @param txpin TX pin number (default is 0, required for ESP32)
     */
  DFRobot_HumanPose_UART(HardwareSerial *hSerial, uint32_t baud, uint8_t rxpin = 0, uint8_t txpin = 0);
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
