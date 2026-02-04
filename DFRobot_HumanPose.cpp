/*!
 *@file DFRobot_HumanPose.cpp
 *@brief Implementation of class DFRobot_HumanPose (I2C/UART communication and detection logic).
 *@details This module implements the Human Pose sensor driver: AT command protocol, JSON parsing, pose/hand result handling.
 *@copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 *@License     The MIT License (MIT)
 *@author [thdyyl](yuanlong.yu@dfrobot.com)
 *@version  V1.0
 *@date  2026-02-04
 *@url         https://github.com/DFRobot/DFRobot_HumanPose
*/

#include "DFRobot_HumanPose.h"

#ifdef ARDUINO_ARCH_RENESAS
char *strnstr(const char *haystack, const char *needle, size_t n)
{
  if (!needle || n == 0) {
    return NULL;
  }

  size_t needle_len = 0;
  while (needle[needle_len] != '\0') {
    needle_len++;
  }

  if (needle_len == 0) {
    return (char *)haystack;
  }

  for (size_t i = 0; i < n && haystack[i] != '\0'; i++) {
    if (i + needle_len <= n && haystack[i] == needle[0]) {
      size_t j = 1;
      while (j < needle_len && haystack[i + j] == needle[j]) {
        j++;
      }
      if (j == needle_len) {
        return (char *)&haystack[i];
      }
    }
  }

  return NULL;
}
#endif

// ============ Base Class: DFRobot_HumanPose ============

/**
 * @fn DFRobot_HumanPose::DFRobot_HumanPose
 * @brief Constructor of DFRobot_HumanPose class
 * @details Initializes internal variables and sets default values
 */
DFRobot_HumanPose::DFRobot_HumanPose()
{
  _wait_delay = 2;
  rx_end      = 0;
  tx_len      = 0;
  rx_len      = 0;
  tx_buf      = NULL;
  rx_buf      = NULL;
  payload     = NULL;
}

/**
 * @fn DFRobot_HumanPose::~DFRobot_HumanPose
 * @brief Destructor of DFRobot_HumanPose class
 * @details Frees allocated memory for transmit and receive buffers
 */
DFRobot_HumanPose::~DFRobot_HumanPose()
{
  if (tx_buf) {
    free(tx_buf);
    tx_buf = NULL;
  }
  if (rx_buf) {
    free(rx_buf);
    rx_buf = NULL;
  }
}

/**
 * @fn DFRobot_HumanPose::begin
 * @brief Initialize the sensor
 * @details Allocates memory for transmit and receive buffers, and initializes the JSON response parser
 * @return True if initialization is successful, otherwise false
 */
bool DFRobot_HumanPose::begin()
{
  // Allocate buffers
  if (!rx_buf || !tx_buf) {
    rx_buf = (char *)malloc(RX_MAX_SIZE);
    tx_buf = (char *)malloc(TX_MAX_SIZE);
    rx_len = RX_MAX_SIZE;
    tx_len = TX_MAX_SIZE;
  }

  if (!rx_buf || !tx_buf) {
    return false;
  }

  rx_end = 0;
  response.clear();

  char name[24] = "";
  if (getName(name) != eOK || !strstr(name, HUMANPOSE_NAME)) {
    LDBG(name);
    return false;
  }

  return true;
}

/**
 * @fn DFRobot_HumanPose::wait
 * @brief Wait for command response from the sensor
 * @details Reads data from the sensor and waits for a response matching the specified command type and name
 * @param type Command type (CMD_TYPE_RESPONSE, CMD_TYPE_EVENT, or CMD_TYPE_LOG)
 * @param cmd Command name string to match
 * @param timeout Timeout value in milliseconds (default is 1000)
 * @return Status code of type `eCmdCode_t`. Returns `eOK` if response received successfully, `eTimedOut` if timeout occurs
 */
int DFRobot_HumanPose::wait(int type, const char *cmd, uint32_t timeout)
{
  int           ret       = eOK;
  unsigned long startTime = millis();

  while (millis() - startTime <= timeout) {
    int len = available();
    if (len == 0) {
      delay(1);
      continue;
    }

    if (len + rx_end > rx_len) {
      len = rx_len - rx_end;
      if (len <= 0) {
        rx_end = 0;
        continue;
      }
    }

    rx_end += read(rx_buf + rx_end, len);
    rx_buf[rx_end] = '\0';

    while (char *suffix = strnstr(rx_buf, RES_SUF, rx_end)) {
      if (char *prefix = strnstr(rx_buf, RES_PRE, suffix - rx_buf)) {
        // Extract JSON payload
        len     = suffix - prefix + RES_SUF_LEN;
        payload = (char *)malloc(len);

        if (!payload) {
          continue;
        }

        memcpy(payload, prefix + 1, len - 1);    // remove "\r" and "\n"
        memmove(rx_buf, suffix + RES_SUF_LEN, rx_end - (suffix - rx_buf) - RES_SUF_LEN);
        rx_end -= (suffix - rx_buf) + RES_SUF_LEN;
        payload[len - 1] = '\0';

        response.clear();
        DeserializationError error = deserializeJson(response, payload);
        free(payload);
        payload = NULL;

        if (error) {
          continue;
        }
        LDBG(response["type"].as<uint8_t>())
        if (response["type"] == CMD_TYPE_RESPONSE) {
          const char *event_name = response["name"];
          LDBG("RESPONSE");
          if (event_name && strstr(event_name, cmd)) {
            LDBG("NAME");
            if (strstr(event_name, AT_HANDLIST) || strstr(event_name, AT_POSELIST)) {
              LDBG("LEARN LIST");
              JsonArray learn_data = response["data"].as<JsonArray>();
              for (size_t i = 0; i < learn_data.size(); i++) {
                _learn_list.push_back(learn_data[i].as<std::string>());
              }
            } else if (strstr(event_name, AT_TSCORE) || strstr(event_name, AT_TIOU) || strstr(event_name, AT_TSIMILARITY)) {
              LDBG("CONFIG")
              _ret_data = response["data"];
            } else if (strstr(event_name, AT_MODEL)) {
              JsonObject data = response["data"].as<JsonObject>();
              _ret_data       = data["id"];
            } else if (strstr(event_name, AT_NAME)) {
              const char *s = response["data"] | "";
              snprintf(_name, sizeof(_name), "%s", s);
            }
          }
        }
        if (response["type"] == CMD_TYPE_EVENT) {
          parser_event();
        }

        ret = response["code"];

        // Match command name
        const char *resp_name = response["name"];
        if (resp_name && response["type"] == type) {
          size_t cmd_len  = strlen(cmd);
          size_t resp_len = strlen(resp_name);
          if ((cmd_len == resp_len && strcmp(resp_name, cmd) == 0) || (cmd_len > 0 && resp_len > 0 && strncmp(resp_name, cmd, cmd_len) == 0)) {
            return ret;
          }
        }
      } else {
        // discard this reply (no prefix found before suffix)
        memmove(rx_buf, suffix + RES_SUF_LEN, rx_end - (suffix - rx_buf) - RES_SUF_LEN);
        rx_end -= (suffix - rx_buf) + RES_SUF_LEN;
        rx_buf[rx_end] = '\0';
      }
    }
  }

  return eTimedOut;
}

/**
 * @fn DFRobot_HumanPose::parser_event
 * @brief Parse event data from the sensor
 * @details Parses JSON event data and extracts pose_keypoints or hand_keypoints information,
 *          populating the internal keypoints vector with detection results
 */
void DFRobot_HumanPose::parser_event()
{
  const char *event_name = response["name"];
  if (event_name && strstr(event_name, EVENT_INVOKE)) {
    JsonObject data = response["data"].as<JsonObject>();

    // ---- pose_keypoints ----
    if (data["pose_keypoints"].is<JsonArray>()) {
      // _keypoints.clear();
      JsonArray      keypoints = data["pose_keypoints"].as<JsonArray>();
      JsonArrayConst arr       = data["pose_class"]["available_classes"].as<JsonArrayConst>();
      for (size_t i = 0; i < keypoints.size(); i++) {
        if (_result[i] != NULL)
          delete _result[i];

        _result[i] = new PoseResult(keypoints[i], arr);
      }
    }

    // ---- hand_keypoints ----
    else if (data["hand_keypoints"].is<JsonArray>()) {
      JsonArray      keypoints = data["hand_keypoints"].as<JsonArray>();
      JsonArrayConst arr       = data["hand_class"]["available_classes"].as<JsonArrayConst>();
      for (size_t i = 0; i < keypoints.size(); i++) {
        if (_result[i] != NULL)
          delete _result[i];

        _result[i] = new HandResult(keypoints[i], arr);
      }
    }
  }
}

/**
 * @fn DFRobot_HumanPose::getResult
 * @brief Get detection results from the sensor
 * @details Sends an INVOKE command to trigger detection and waits for the response and event data
 * @return Status code of type `eCmdCode_t`. Returns `eOK` if results are successfully retrieved,
 *         otherwise returns an error code (typically `eTimedOut`)
 * @note After successful execution, detection results are available via the keypoints() method
 */
DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::getResult()
{
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s=1,0,1" CMD_SUF, AT_INVOKE);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_INVOKE) == eOK) {
    if (wait(CMD_TYPE_EVENT, EVENT_INVOKE) == eOK) {
      return eOK;
    }
  }

  return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::setConfidence(uint8_t confidence)
{
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s=%d" CMD_SUF, AT_TSCORE, confidence);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_TSCORE) == eOK) {
    LDBG("SCORE OK");
    return eOK;
  }
  return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::setIOU(uint8_t iou)
{
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s=%d" CMD_SUF, AT_TIOU, iou);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_TIOU) == eOK) {
    return eOK;
  }
  return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::setModelType(eModel_t model)
{
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s=%d" CMD_SUF, AT_MODEL, (int)model);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_MODEL) == eOK) {
    return eOK;
  }
  return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::setLearnSimilarity(uint8_t Similarity)
{
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s=%d" CMD_SUF, AT_TSIMILARITY, (int)Similarity);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_TSIMILARITY) == eOK) {
    return eOK;
  }
  return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::getConfidence(uint8_t *confidence)
{
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s?" CMD_SUF, AT_TSCORE);
  write(cmd, strlen(cmd));
  *confidence = 0;
  if (wait(CMD_TYPE_RESPONSE, AT_TSCORE) == eOK) {
    *confidence = _ret_data;
    return eOK;
  }
  return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::getIOU(uint8_t *iou)
{
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s?" CMD_SUF, AT_TIOU);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_TIOU) == eOK) {
    *iou = _ret_data;
    return eOK;
  }
  return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::getLearnSimilarity(uint8_t *Similarity)
{
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s?" CMD_SUF, AT_TSIMILARITY);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_TSIMILARITY) == eOK) {
    *Similarity = _ret_data;
    return eOK;
  }
  return eTimedOut;
}

LearnList DFRobot_HumanPose::getLearnList(eModel_t model)
{
  char cmd[64] = { 0 };
  _learn_list.clear();
  if (model == ePose) {
    snprintf(cmd, sizeof(cmd), CMD_PRE "%s" CMD_SUF, AT_POSELIST);
    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, AT_POSELIST) == eOK) {
      LDBG("POSE OK");
      return _learn_list;
    }
  } else {
    snprintf(cmd, sizeof(cmd), CMD_PRE "%s" CMD_SUF, AT_HANDLIST);
    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, AT_HANDLIST) == eOK) {
      LDBG("HAND OK");
      return _learn_list;
    }
  }
  return _learn_list;
}

bool DFRobot_HumanPose::availableResult()
{
  bool ret = false;
  for (uint8_t i = 0; i < MAX_RESULT_NUM; ++i) {
    if (_result[i] != NULL) {
      if (!_result[i]->used) {
        ret = true;
        break;
      }
    }
  }
  return ret;
}

Result *DFRobot_HumanPose::popResult()
{
  for (uint8_t i = 0; i < MAX_RESULT_NUM; ++i) {
    if (_result[i] != NULL) {
      if (_result[i]->used) {
        continue;
      }

      _result[i]->used = true;
      return _result[i];
    }
  }
  return NULL;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::getName(char *name)
{
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s?" CMD_SUF, AT_NAME);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_NAME) == eOK) {
    strcpy(name, _name);
    return eOK;
  }
  return eTimedOut;
}

// ============ Derived Class: DFRobot_HumanPose_I2C ============

/**
 * @fn DFRobot_HumanPose_I2C::DFRobot_HumanPose_I2C
 * @brief Constructor of DFRobot_HumanPose_I2C class
 * @param wire Pointer to TwoWire object (typically &Wire)
 * @param address I2C device address (default is 0x3A)
 */
DFRobot_HumanPose_I2C::DFRobot_HumanPose_I2C(TwoWire *wire, uint8_t address)
{
  _wire     = wire;
  __address = address;
}

/**
 * @fn DFRobot_HumanPose_I2C::~DFRobot_HumanPose_I2C
 * @brief Destructor of DFRobot_HumanPose_I2C class
 */
DFRobot_HumanPose_I2C::~DFRobot_HumanPose_I2C() {}

/**
 * @fn DFRobot_HumanPose_I2C::begin
 * @brief Initialize the I2C communication and sensor
 * @details Initializes the I2C bus, sets the I2C clock speed, and calls the base class begin() method
 * @return True if initialization is successful, otherwise false
 */
bool DFRobot_HumanPose_I2C::begin(void)
{
  _wire->begin();
  _wire->setClock(I2C_CLOCK);
  _wait_delay = 2;

  return DFRobot_HumanPose::begin();
}

int DFRobot_HumanPose_I2C::available()
{
  uint8_t buf[2] = { 0 };
  delay(_wait_delay);
  _wire->beginTransmission(__address);
  _wire->write(FEATURE_TRANSPORT);
  _wire->write(FEATURE_TRANSPORT_CMD_AVAILABLE);
  _wire->write(0);
  _wire->write(0);
  // TODO checksum
  _wire->write(0);
  _wire->write(0);
  if (_wire->endTransmission() == 0) {
    delay(_wait_delay);
    _wire->requestFrom(__address, (uint8_t)2);
    _wire->readBytes(buf, (uint8_t)2);
  }

  return (buf[0] << 8) | buf[1];
}

int DFRobot_HumanPose_I2C::read(char *data, int len)
{
  uint16_t packets = len / MAX_PL_LEN;
  uint8_t  remain  = len % MAX_PL_LEN;

  for (uint16_t i = 0; i < packets; i++) {
    delay(_wait_delay);
    _wire->beginTransmission(__address);
    _wire->write(FEATURE_TRANSPORT);
    _wire->write(FEATURE_TRANSPORT_CMD_READ);
    _wire->write(MAX_PL_LEN >> 8);
    _wire->write(MAX_PL_LEN & 0xFF);
    // TODO checksum
    _wire->write(0);
    _wire->write(0);
    if (_wire->endTransmission() == 0) {
      delay(_wait_delay);
      _wire->requestFrom(__address, MAX_PL_LEN);
      _wire->readBytes(data + i * MAX_PL_LEN, MAX_PL_LEN);
    }
  }

  if (remain) {
    delay(_wait_delay);
    _wire->beginTransmission(__address);
    _wire->write(FEATURE_TRANSPORT);
    _wire->write(FEATURE_TRANSPORT_CMD_READ);
    _wire->write(remain >> 8);
    _wire->write(remain & 0xFF);
    // TODO checksum
    _wire->write(0);
    _wire->write(0);
    if (_wire->endTransmission() == 0) {
      delay(_wait_delay);
      _wire->requestFrom(__address, remain);
      _wire->readBytes(data + packets * MAX_PL_LEN, remain);
    }
  }

  return len;
}

int DFRobot_HumanPose_I2C::write(const char *data, int len)
{
  uint16_t packets = len / MAX_PL_LEN;
  uint16_t remain  = len % MAX_PL_LEN;

  for (uint16_t i = 0; i < packets; i++) {
    delay(_wait_delay);
    _wire->beginTransmission(__address);
    _wire->write(FEATURE_TRANSPORT);
    _wire->write(FEATURE_TRANSPORT_CMD_WRITE);
    _wire->write(MAX_PL_LEN >> 8);
    _wire->write(MAX_PL_LEN & 0xFF);
    _wire->write((const uint8_t *)(data + i * MAX_PL_LEN), MAX_PL_LEN);
    // TODO checksum
    _wire->write(0);
    _wire->write(0);
    _wire->endTransmission();
  }

  if (remain) {
    delay(_wait_delay);
    _wire->beginTransmission(__address);
    _wire->write(FEATURE_TRANSPORT);
    _wire->write(FEATURE_TRANSPORT_CMD_WRITE);
    _wire->write(remain >> 8);
    _wire->write(remain & 0xFF);
    _wire->write((const uint8_t *)(data + packets * MAX_PL_LEN), remain);
    _wire->write(0);
    _wire->write(0);
    _wire->endTransmission();
  }

  return len;
}

// ============ Derived Class: DFRobot_HumanPose_UART ============

#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
/**
 * @fn DFRobot_HumanPose_UART::DFRobot_HumanPose_UART
 * @brief Constructor of DFRobot_HumanPose_UART class (for UNO/ESP8266)
 * @param sSerial Pointer to SoftwareSerial object
 * @param baud Baud rate value (e.g., 921600)
 */
DFRobot_HumanPose_UART::DFRobot_HumanPose_UART(SoftwareSerial *sSerial, uint32_t baud)
{
  _serial = sSerial;
  __baud  = baud;
}
#else
/**
 * @fn DFRobot_HumanPose_UART::DFRobot_HumanPose_UART
 * @brief Constructor of DFRobot_HumanPose_UART class
 * @param hSerial Pointer to HardwareSerial object (typically &Serial1)
 * @param baud Baud rate value (default is 921600)
 * @param rxpin RX pin number (default is 0, required for ESP32)
 * @param txpin TX pin number (default is 0, required for ESP32)
 */
DFRobot_HumanPose_UART::DFRobot_HumanPose_UART(HardwareSerial *hSerial, uint32_t baud = UART_BAUD, uint8_t rxpin, uint8_t txpin)
{
  _serial = hSerial;
  __baud  = baud;
  __rxpin = rxpin;
  __txpin = txpin;
}
#endif

/**
 * @fn DFRobot_HumanPose_UART::~DFRobot_HumanPose_UART
 * @brief Destructor of DFRobot_HumanPose_UART class
 */
DFRobot_HumanPose_UART::~DFRobot_HumanPose_UART() {}

/**
 * @fn DFRobot_HumanPose_UART::begin
 * @brief Initialize the UART communication and sensor
 * @details Initializes the serial port with the configured baud rate, sets timeout, and calls the base class begin() method
 * @return True if initialization is successful, otherwise false
 */
bool DFRobot_HumanPose_UART::begin(void)
{
  _wait_delay = 2;
#ifdef ESP32
  _serial->begin(__baud, SERIAL_8N1, __rxpin, __txpin);
#elif defined(ARDUINO_AVR_UNO) || defined(ESP8266)
  _serial->begin(__baud);
#else
  _serial->begin(__baud);
#endif
  _serial->setTimeout(1000);
  _serial->flush();

  return DFRobot_HumanPose::begin();
}

bool DFRobot_HumanPose_UART::setBaud(eBaudConfig_t baud)
{
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s=%d" CMD_SUF, AT_BAUD, baud);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_BAUD) == eOK) {
    return true;
  }
  return false;
}

int DFRobot_HumanPose_UART::available()
{
  return _serial->available();
}

int DFRobot_HumanPose_UART::read(char *data, int len)
{
  return _serial->readBytes(data, len);
}

int DFRobot_HumanPose_UART::write(const char *data, int len)
{
  return _serial->write(data, len);
}
