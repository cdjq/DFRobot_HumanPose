#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file   pose_detect_blink.py
@brief  Hand detect + LED blink example (formerly pose-oriented; hand is easier to reproduce).
@details Uses MODEL_HAND. LED lights when conditions below are met (configurable).
@n      Supports I2C or UART on Raspberry Pi + PinPong.
@copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
@license     The MIT License (MIT)
@author [thdyyl](yuanlong.yu@dfrobot.com)
@version V1.0
@date   2026-04-13
@url    https://github.com/DFRobot/DFRobot_HumanPose
"""

import os
import sys
import time

sys.path.append("../")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinpong.board import Board, Pin
from DFRobot_HumanPose import DFRobot_HumanPose_I2C, DFRobot_HumanPose_UART, HandResult

# ------------ Configuration: choose communication ------------
USE_I2C = True  # True: I2C  /  False: UART

# I2C config (when USE_I2C=True)
I2C_BUS = 1  # Raspberry Pi: usually 1

# UART config (when USE_I2C=False)
UART_TTY = "/dev/ttyAMA0"
UART_BAUD = 9600

# LED: GPIO (BCM), e.g. 17; None = only print LED state
LED_PIN_NUM = 17

# True: LED on for any detected hand (id==0 detection or id!=0 learned). Easiest to demo.
# False: LED only when a learned class matches (id != 0), same as original pose demo intent.
LED_ON_ANY_HAND = True

TAG = "HAND"
SEP = "-" * 64
NO_HIT_PRINT_INTERVAL = 20


def print_hit_block(frame, results):
  print(SEP)
  print(f"[{TAG}][frame {frame}] hit(s):")
  for idx, r in enumerate(results, start=1):
    print(f"  #{idx} id={r.id} name={r.name}")
    print(f"    score: {r.score}")
    print(f"    box: ({r.xLeft},{r.yTop},{r.width},{r.height})")
    if isinstance(r, HandResult):
      print(f"    wrist: ({r.wrist.x}, {r.wrist.y})")
  print(SEP)


def main():
  Board("RPI").begin()

  led = None
  if LED_PIN_NUM is not None:
    try:
      led = Pin(Board("RPI"), LED_PIN_NUM, Pin.OUT)
      led.write_digital(0)
    except Exception as e:
      print("LED init warning:", e, "- will only print LED state")

  if USE_I2C:
    human_pose = DFRobot_HumanPose_I2C(bus_num=I2C_BUS)
    print(f"[COMM] I2C bus={I2C_BUS}")
  else:
    human_pose = DFRobot_HumanPose_UART(tty_name=UART_TTY, baudrate=UART_BAUD)
    print(f"[COMM] UART tty={UART_TTY}, baud={UART_BAUD}")

  if not human_pose.begin():
    print("Sensor init fail!")
    return
  print("Sensor init success!")

  human_pose.set_model_type(human_pose.MODEL_HAND)

  human_pose.set_iou(45)
  human_pose.set_confidence(60)
  human_pose.set_learn_similarity(80)

  iou = human_pose.get_iou()
  score = human_pose.get_confidence()
  similarity = human_pose.get_learn_similarity()
  print("iou: {}, score: {}, similarity: {}".format(iou, score, similarity))

  learn_list = human_pose.get_learn_list(human_pose.MODEL_HAND)
  print("Hand learn list (id 1..N maps to name):")
  if learn_list:
    for i, name in enumerate(learn_list):
      print("  id: {}, name: {}".format(i + 1, name))
  else:
    print("  (empty — only id=0 / unknown until you teach on device)")

  if LED_ON_ANY_HAND:
    print("Mode: LED ON when any hand is detected (including id=0).")
  else:
    print("Mode: LED ON only when learned target id != 0.")
  print("Press Ctrl+C to exit.\n")

  frame = 0
  no_hit_streak = 0

  try:
    while True:
      frame += 1
      led_val = 0
      hits = []

      if human_pose.get_result() == human_pose.CODE_OK:
        while human_pose.available_result():
          result = human_pose.pop_result()
          if result is None:
            break
          want = (result.id != 0) if not LED_ON_ANY_HAND else True
          if want:
            hits.append(result)
            led_val = 1
        if hits:
          no_hit_streak = 0
          print_hit_block(frame, hits)
        else:
          no_hit_streak += 1
          if no_hit_streak == 1 or no_hit_streak % NO_HIT_PRINT_INTERVAL == 0:
            print("[{}] no hit, streak={}".format(TAG, no_hit_streak))
      else:
        print("[{}] get_result timeout".format(TAG))

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
