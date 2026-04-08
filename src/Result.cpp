/*!
 * @file Result.cpp
 * @brief Implementation of Result, PoseResult, HandResult (binary result mapping).
 * @details This module maps binary protocol output into Result/PoseResult/HandResult structures.
 * @copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @License     The MIT License (MIT)
 * @author [thdyyl](yuanlong.yu@dfrobot.com)
 * @version  V1.0
 * @date  2026-02-04
 * @url         https://github.com/DFRobot/DFRobot_HumanPose
*/

#include "Result.h"

Result::Result(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t scoreValue, uint8_t targetId, const String &targetName)
{
  used   = false;
  xLeft  = x;
  yTop   = y;
  width  = w;
  height = h;
  score  = scoreValue;
  id     = targetId;
  name   = targetName;
}

Result::~Result() {}

PoseResult::PoseResult(uint16_t x,
                       uint16_t y,
                       uint16_t w,
                       uint16_t h,
                       uint8_t  scoreValue,
                       uint8_t  targetId,
                       const PointU16 *points,
                       size_t pointCount,
                       const String &targetName) :
  Result(x, y, w, h, scoreValue, targetId, targetName)
{
  const PointU16 zero = PointU16{};
  nose = leye = reye = lear = rear = zero;
  lshoulder = rshoulder = lelbow = relbow = zero;
  lwrist = rwrist = lhip = rhip = zero;
  lknee = rknee = lankle = rankle = zero;

  if (!points) {
    return;
  }

  if (pointCount > 0)
    nose = points[0];
  if (pointCount > 1)
    leye = points[1];
  if (pointCount > 2)
    reye = points[2];
  if (pointCount > 3)
    lear = points[3];
  if (pointCount > 4)
    rear = points[4];
  if (pointCount > 5)
    lshoulder = points[5];
  if (pointCount > 6)
    rshoulder = points[6];
  if (pointCount > 7)
    lelbow = points[7];
  if (pointCount > 8)
    relbow = points[8];
  if (pointCount > 9)
    lwrist = points[9];
  if (pointCount > 10)
    rwrist = points[10];
  if (pointCount > 11)
    lhip = points[11];
  if (pointCount > 12)
    rhip = points[12];
  if (pointCount > 13)
    lknee = points[13];
  if (pointCount > 14)
    rknee = points[14];
  if (pointCount > 15)
    lankle = points[15];
  if (pointCount > 16)
    rankle = points[16];
}

HandResult::HandResult(uint16_t x,
                       uint16_t y,
                       uint16_t w,
                       uint16_t h,
                       uint8_t  scoreValue,
                       uint8_t  targetId,
                       const PointU16 *points,
                       size_t pointCount,
                       const String &targetName) :
  Result(x, y, w, h, scoreValue, targetId, targetName)
{
  const PointU16 zero = PointU16{};
  wrist = thumbCmc = thumbMcp = thumbIp = thumbTip = zero;
  indexFingerMcp = indexFingerPip = indexFingerDip = indexFingerTip = zero;
  middleFingerMcp = middleFingerPip = middleFingerDip = middleFingerTip = zero;
  ringFingerMcp = ringFingerPip = ringFingerDip = ringFingerTip = zero;
  pinkyFingerMcp = pinkyFingerPip = pinkyFingerDip = pinkyFingerTip = zero;

  if (!points) {
    return;
  }

  if (pointCount > 0)
    wrist = points[0];
  if (pointCount > 1)
    thumbCmc = points[1];
  if (pointCount > 2)
    thumbMcp = points[2];
  if (pointCount > 3)
    thumbIp = points[3];
  if (pointCount > 4)
    thumbTip = points[4];
  if (pointCount > 5)
    indexFingerMcp = points[5];
  if (pointCount > 6)
    indexFingerPip = points[6];
  if (pointCount > 7)
    indexFingerDip = points[7];
  if (pointCount > 8)
    indexFingerTip = points[8];
  if (pointCount > 9)
    middleFingerMcp = points[9];
  if (pointCount > 10)
    middleFingerPip = points[10];
  if (pointCount > 11)
    middleFingerDip = points[11];
  if (pointCount > 12)
    middleFingerTip = points[12];
  if (pointCount > 13)
    ringFingerMcp = points[13];
  if (pointCount > 14)
    ringFingerPip = points[14];
  if (pointCount > 15)
    ringFingerDip = points[15];
  if (pointCount > 16)
    ringFingerTip = points[16];
  if (pointCount > 17)
    pinkyFingerMcp = points[17];
  if (pointCount > 18)
    pinkyFingerPip = points[18];
  if (pointCount > 19)
    pinkyFingerDip = points[19];
  if (pointCount > 20)
    pinkyFingerTip = points[20];
}

