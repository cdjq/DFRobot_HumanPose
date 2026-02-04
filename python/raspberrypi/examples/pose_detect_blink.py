#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file   pose_detect_blink.py
@brief  Pose detect blink example.
@details This example detects human pose/hand and turns on LED when a learned target (id != 0) is detected.
@n      Supports I2C or UART; option MODEL_HAND (hand) or MODEL_POSE (pose). LED on when learned target detected, off otherwise.
@copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
@license     The MIT License (MIT)
@author [thdyyl](yuanlong.yu@dfrobot.com)
@version V1.0
@date   2026-02-04
@url    https://github.com/DFRobot/DFRobot_HumanPose
"""

import time
import os
import sys
sys.path.append("../")
# Add parent path so DFRobot_HumanPose can be imported when running example standalone
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinpong.board import Board, Pin
from DFRobot_HumanPose import (
  DFRobot_HumanPose_I2C,
  DFRobot_HumanPose_UART,
  HandResult,
  PoseResult,
)

# ------------ Configuration: choose communication ------------
USE_I2C = True  # True: I2C  /  False: UART

# I2C config (when USE_I2C=True)
I2C_BUS = 1  # I2C bus number on Raspberry Pi, usually 1

# UART config (when USE_I2C=False)
UART_TTY = "/dev/ttyS0"
UART_BAUD = 921600

# LED: GPIO (BCM) for indicator, e.g. external LED on GPIO17; set None to only print "LED ON/OFF"
LED_PIN_NUM = 17


def main():
  # Initialize board (Raspberry Pi)
  Board("RPI").begin()

  # Init LED (or only print state if not configured)
  led = None
  if LED_PIN_NUM is not None:
    try:
      led = Pin(Board("RPI"), LED_PIN_NUM, Pin.OUT)
      led.write_digital(0)
    except Exception as e:
      print("LED init warning:", e, "- will only print LED state")

  # Initialize sensor
  if USE_I2C:
    human_pose = DFRobot_HumanPose_I2C(bus_num=I2C_BUS)
  else:
    human_pose = DFRobot_HumanPose_UART(tty_name=UART_TTY, baudrate=UART_BAUD)

  if not human_pose.begin():
    print("Sensor init fail!")
    return
  print("Sensor init success!")

  # Detection model: hand (MODEL_HAND) or pose (MODEL_POSE); same as Arduino example uses hand
  human_pose.set_model_type(human_pose.MODEL_HAND)

  # Detection thresholds
  human_pose.set_iou(45)  # IOU 0-100, NMS, default ~45
  human_pose.set_confidence(80)  # Confidence 0-100, default ~60
  human_pose.set_learn_similarity(60)  # Learn similarity 0-100, default ~60

  # Optional: read and print current params
  iou = human_pose.get_iou()
  score = human_pose.get_confidence()
  similarity = human_pose.get_learn_similarity()
  print("iou: {}, score: {}, similarity: {}".format(iou, score, similarity))

  # Get and print learn list
  learn_list = human_pose.get_learn_list(human_pose.MODEL_HAND)
  print("Learn list:")
  if learn_list is not None and isinstance(learn_list, list):
    for i, name in enumerate(learn_list):
      print("  id: {}, name: {}".format(i, name))
  else:
    print("  (none or not available)")

  print("Start detecting. When a learned target is detected (id!=0), LED will turn on.")
  print("Press Ctrl+C to exit.\n")

  try:
    while True:
      led_val = 0  # Default LED off

      if human_pose.get_result() == human_pose.CODE_OK:
        while human_pose.available_result():
          result = human_pose.pop_result()
          if result is None:
            break
          # id != 0 means learned target detected (same as Arduino)
          if result.id != 0:
            print("ID: {}  Name: {}".format(result.id, result.name))
            led_val = 1

      if led is not None:
        led.write_digital(led_val)
      else:
        print("LED {}".format("ON" if led_val else "OFF"))

      time.sleep(0.05)

  except KeyboardInterrupt:
    if led is not None:
      led.write_digital(0)
    print("\nBye.")


if __name__ == "__main__":
  main()
