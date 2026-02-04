/*!
 *@file Result.cpp
 *@brief Implementation of Result, PoseResult, HandResult (JSON parsing and keypoint mapping).
 *@details This module parses sensor JSON output into Result/PoseResult/HandResult structures.
 *@copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 *@License     The MIT License (MIT)
 *@author [thdyyl](yuanlong.yu@dfrobot.com)
 *@version  V1.0
 *@date  2026-02-04
 *@url         https://github.com/DFRobot/DFRobot_HumanPose
*/

#include "Result.h"

static bool readPointU16(const JsonArray& points, size_t i, PointU16& out)
{
  if (i >= points.size())
    return false;

  JsonArray p = points[i].as<JsonArray>();
  if (p.size() < 2)
    return false;

  out.x = p[0].as<uint16_t>();
  out.y = p[1].as<uint16_t>();
  return true;
}

Result::Result(JsonArray data, JsonArrayConst names)
{
  used          = false;
  JsonArray box = data[0].as<JsonArray>();
  xLeft         = box[0] | 0;
  yTop          = box[1] | 0;
  width         = box[2] | 0;
  height        = box[3] | 0;
  score         = box[4] | 0;
  id            = box[5] | 0;
  name          = "unknown";
  if (id != 0) {
    JsonObjectConst ob = names[id - 1];
    name = name = ob["name"] | "unknown";
  }
}

Result::~Result() {}

PoseResult::PoseResult(JsonArray data, JsonArrayConst names) : Result(data, names)
{
  JsonArray points = data[1].as<JsonArray>();
  readPointU16(points, 0, nose);
  readPointU16(points, 1, leye);
  readPointU16(points, 2, reye);
  readPointU16(points, 3, lear);
  readPointU16(points, 4, rear);
  readPointU16(points, 5, lshoulder);
  readPointU16(points, 6, rshoulder);
  readPointU16(points, 7, lelbow);
  readPointU16(points, 8, relbow);
  readPointU16(points, 9, lwrist);
  readPointU16(points, 10, rwrist);
  readPointU16(points, 11, lhip);
  readPointU16(points, 12, rhip);
  readPointU16(points, 13, lknee);
  readPointU16(points, 14, rknee);
  readPointU16(points, 15, lankle);
  readPointU16(points, 16, rankle);
}

HandResult::HandResult(JsonArray data, JsonArrayConst names) : Result(data, names)
{
  JsonArray points = data[1].as<JsonArray>();

  LDBG(points.size());
  readPointU16(points, 0, wrist);
  readPointU16(points, 1, thumbCmc);
  readPointU16(points, 2, thumbMcp);
  readPointU16(points, 3, thumbIp);
  readPointU16(points, 4, thumbTip);
  readPointU16(points, 5, indexFingerMcp);
  readPointU16(points, 6, indexFingerPip);
  readPointU16(points, 7, indexFingerDip);
  readPointU16(points, 8, indexFingerTip);
  readPointU16(points, 9, middleFingerMcp);
  readPointU16(points, 10, middleFingerPip);
  readPointU16(points, 11, middleFingerDip);
  readPointU16(points, 12, middleFingerTip);
  readPointU16(points, 13, ringFingerMcp);
  readPointU16(points, 14, ringFingerPip);
  readPointU16(points, 15, ringFingerDip);
  readPointU16(points, 16, ringFingerTip);
  readPointU16(points, 17, pinkyFingerMcp);
  readPointU16(points, 18, pinkyFingerPip);
  readPointU16(points, 19, pinkyFingerDip);
  readPointU16(points, 20, pinkyFingerTip);
}
