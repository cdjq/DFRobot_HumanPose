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


def main():
  Board("RPI").begin()

  sensor = DFRobot_HumanPose_I2C(bus_num=1)
  # sensor = DFRobot_HumanPose_UART(tty_name="/dev/ttyAMA0")

  if not sensor.begin():
    print("Sensor init failed")
    return

  # MODEL_GES (4): fixed gesture classification; names are GES_CLASS_NAMES (id 0..14), no learn list
  sensor.set_model_type(sensor.MODEL_GES)
  print("GES model active; learned list is empty by design:", sensor.get_learn_list(sensor.MODEL_GES))

  while True:
    if sensor.get_result() == sensor.CODE_OK:
      while sensor.available_result():
        r = sensor.pop_result()
        if r is None:
          continue
        print(
          f"name={r.name} id={r.id} score={r.score} "
          f"box=({r.xLeft},{r.yTop})+{r.width}x{r.height}"
        )
    else:
      print("get_result timeout")
    time.sleep(0.1)


if __name__ == "__main__":
  main()
