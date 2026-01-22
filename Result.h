#ifndef DFROBOT_HUMAN_RESULT
#define DFROBOT_HUMAN_RESULT
#include <Arduino.h>
#include <ArduinoJson.h>

struct PointU16 {uint16_t x{}, y{};};

class Result {
public:
  Result(JsonArray data, JsonArrayConst names);
  ~Result();

public:
  uint8_t id;
  uint16_t xLeft;
  uint16_t yTop;
  uint16_t width;
  uint16_t height;
  uint8_t confidence;
  String name;
  bool used = false;
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