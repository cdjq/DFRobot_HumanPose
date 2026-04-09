#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file   get_ges_result.py
@brief  Get GES (fixed gesture classification) result example.
@details This example runs GES detection over I2C/UART and prints bounding box and class name.
@n      Uses MODEL_GES (4); no keypoints and no user learn list on the device.
@copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
@license     The MIT License (MIT)
@author [thdyyl](yuanlong.yu@dfrobot.com)
@version V1.0
@date   2026-04-08
@url    https://github.com/DFRobot/DFRobot_HumanPose
"""

import sys
import time

from pinpong.board import Board

sys.path.append("../")
from DFRobot_HumanPose import DFRobot_HumanPose_I2C, DFRobot_HumanPose_UART


NO_TARGET_PRINT_INTERVAL = 20
TIMEOUT_PRINT_INTERVAL = 20
POLL_INTERVAL_SEC = 0.1


def format_detection_line(index, result):
  return (
    f"  [{index:02d}] {result.name:<11} (id={result.id:>2}, score={result.score:>3}) "
    f"bbox=({result.xLeft:>4},{result.yTop:>4},{result.width:>4},{result.height:>4})"
  )


def read_results(sensor):
  results = []
  while sensor.available_result():
    result = sensor.pop_result()
    if result is not None:
      results.append(result)
  return results


def build_signature(results):
  """Create a compact signature so repeated unchanged results do not spam output."""
  return tuple(
    (r.id, r.score, r.xLeft, r.yTop, r.width, r.height, r.name) for r in results
  )


def print_detection_block(results):
  print(f"[GES] detected {len(results)} target(s)")
  for idx, result in enumerate(results, start=1):
    print(format_detection_line(idx, result))
  print("-" * 72)


def main():
  Board("RPI").begin()

  sensor = DFRobot_HumanPose_I2C(bus_num=1)
  # sensor = DFRobot_HumanPose_UART(tty_name="/dev/ttyAMA0")

  if not sensor.begin():
    print("Sensor init failed")
    return

  sensor.set_model_type(sensor.MODEL_GES)
  learn_list = sensor.get_learn_list(sensor.MODEL_GES)

  print("Sensor init success!")
  print("Model: GES (fixed gesture classification)")
  print("Learn list:", learn_list if learn_list else "[]")
  print("Output: event-driven (changed result / periodic no target / periodic timeout)")
  print("-" * 72)
  print("Press Ctrl+C to stop.\n")

  last_signature = None
  timeout_count = 0
  no_target_count = 0

  try:
    while True:
      if sensor.get_result() == sensor.CODE_OK:
        if timeout_count:
          print(f"[GES] recovered from timeout (x{timeout_count})")
          timeout_count = 0

        results = read_results(sensor)
        if results:
          no_target_count = 0
          signature = build_signature(results)
          if signature != last_signature:
            print_detection_block(results)
            last_signature = signature
        else:
          last_signature = ()
          no_target_count += 1
          if no_target_count == 1 or no_target_count % NO_TARGET_PRINT_INTERVAL == 0:
            print(f"[GES] no target ({no_target_count} consecutive frame(s))")
      else:
        timeout_count += 1
        if timeout_count == 1 or timeout_count % TIMEOUT_PRINT_INTERVAL == 0:
          print(f"[GES] get_result timeout ({timeout_count} consecutive time(s))")

      time.sleep(POLL_INTERVAL_SEC)
  except KeyboardInterrupt:
    print("\nStopped by user.")


if __name__ == "__main__":
  main()
