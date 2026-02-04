/*!
 *@file Result.h
 *@brief Define the basic structure of Result, PoseResult, HandResult and PointU16.
 *@details This module defines detection result data: bounding box, score, id, name, and keypoints (pose 17 points, hand 21 points).
 *@copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 *@License     The MIT License (MIT)
 *@author [thdyyl](yuanlong.yu@dfrobot.com)
 *@version  V1.0
 *@date  2026-02-04
 *@url         https://github.com/DFRobot/DFRobot_HumanPose
*/
#ifndef DFROBOT_HUMAN_RESULT
#define DFROBOT_HUMAN_RESULT
#include <Arduino.h>

/* Must match DFRobot_HumanPose.h - same ArduinoJson config for all TUs to avoid ABI mismatch. */
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#if defined(ARDUINO_BBC_MICROBIT) || defined(ARDUINO_BBC_MICROBIT_V2) || defined(ARDUINO_AVR_LEONARDO) || defined(ARDUINO_AVR_UNO) || (defined(ARDUINO_ARCH_AVR) && !defined(ARDUINO_AVR_MEGA2560) && !defined(ARDUINO_AVR_MEGA)) || defined(ARDUINO_ARCH_NRF5)
#define ARDUINOJSON_ENABLE_STD_STRING     0
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 0
#define ARDUINOJSON_ENABLE_ARDUINO_STREAM 0
#define ARDUINOJSON_ENABLE_STD_STREAM     0
#define ARDUINOJSON_DECODE_UNICODE        0
#define ARDUINOJSON_ENABLE_COMMENTS       0
#define ARDUINOJSON_ENABLE_NAN            0
#define ARDUINOJSON_ENABLE_INFINITY       0
#define ARDUINOJSON_USE_LONG_LONG         0
#define ARDUINOJSON_USE_DOUBLE            0
#endif

#include <ArduinoJson.h>

struct PointU16 {
  uint16_t x{}, y{};
};
// #define ENABLE_DBG
#ifdef ENABLE_DBG
#define LDBG(...)                \
  {                              \
    Serial.print("[");           \
    Serial.print(__FUNCTION__);  \
    Serial.print("(): ");        \
    Serial.print(__LINE__);      \
    Serial.print(" ] ");         \
    Serial.println(__VA_ARGS__); \
  }
#else
#define LDBG(...)
#endif
class Result {
public:
  Result(JsonArray data, JsonArrayConst names);
  ~Result();

public:
  uint8_t  id;
  uint16_t xLeft;
  uint16_t yTop;
  uint16_t width;
  uint16_t height;
  uint8_t  score;
  String   name;
  bool     used = false;
};

class PoseResult : public Result {
public:
  PoseResult(JsonArray data, JsonArrayConst names);

public:
  PointU16 nose;
  PointU16 leye;
  PointU16 reye;
  PointU16 lear;
  PointU16 rear;

  PointU16 lshoulder;
  PointU16 rshoulder;
  PointU16 lelbow;
  PointU16 relbow;

  PointU16 lwrist;
  PointU16 rwrist;
  PointU16 lhip;
  PointU16 rhip;

  PointU16 lknee;
  PointU16 rknee;

  PointU16 lankle;
  PointU16 rankle;
};

class HandResult : public Result {
public:
  HandResult(JsonArray data, JsonArrayConst names);

public:
  PointU16 wrist;
  PointU16 thumbCmc;
  PointU16 thumbMcp;
  PointU16 thumbIp;
  PointU16 thumbTip;

  PointU16 indexFingerMcp;
  PointU16 indexFingerPip;
  PointU16 indexFingerDip;
  PointU16 indexFingerTip;

  PointU16 middleFingerMcp;
  PointU16 middleFingerPip;
  PointU16 middleFingerDip;
  PointU16 middleFingerTip;

  PointU16 ringFingerMcp;
  PointU16 ringFingerPip;
  PointU16 ringFingerDip;
  PointU16 ringFingerTip;

  PointU16 pinkyFingerMcp;
  PointU16 pinkyFingerPip;
  PointU16 pinkyFingerDip;
  PointU16 pinkyFingerTip;
};

#endif
