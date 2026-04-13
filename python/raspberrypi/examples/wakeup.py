#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file   wakeup.py
@brief  Control the HumanPose sensor EN / wakeup pin on Raspberry Pi (BCM GPIO).
@details
  **User scenario (same idea as wakeup.ino)**\n
  Use EN to **disable** the sensor when idle (save power, night mode) and **enable** it before
  running `get_pose_result.py` / `get_hand_result.py` / `get_ges_result.py`. This script performs
  one OFF → ON cycle, then exits (GPIO cleaned up). Run your detection script **after** this, or
  keep EN wired high from another process if you need the sensor always on.
@copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
@license     The MIT License (MIT)
@author [thdyyl](yuanlong.yu@dfrobot.com)
@version V1.0.1
@date   2026-04-13
@url    https://github.com/DFRobot/DFRobot_HumanPose
"""

import time
import RPi.GPIO as GPIO

# BCM pin connected to sensor EN / wakeup (change to match your wiring)
WAKEUP_PIN = 17

POWER_CYCLE_DELAY_SEC = 1.0

# After OFF->ON, keep this process running with EN=HIGH so you can open **another terminal**
# and run get_pose_result / get_hand_result / get_ges_result. Press Ctrl+C here when done.
HOLD_EN_HIGH_UNTIL_CTRL_C = True


def print_scenario():
  print()
  print("========== Scenario (when to use EN / wakeup) ==========")
  print("Example: kiosk / classroom / battery use — power off the sensor when idle,")
  print("power on before you need pose/hand/GES. EN=LOW: sensor off; EN=HIGH: ready for I2C/UART.")
  print("==========================================================")
  print()
  print("[Step 1] EN as OUTPUT, start LOW (sensor off).")
  print("[Step 2] Wait for power-down settle.")
  print("[Step 3] Drive EN HIGH (sensor on).")
  print()
  print("Next: run get_pose_result.py / get_hand_result.py / get_ges_result.py")
  print("(same GND; keep EN HIGH while those scripts run).")
  print()


def sensor_power_off():
  GPIO.output(WAKEUP_PIN, GPIO.LOW)
  print("Sensor OFF (wakeup=LOW, sensor disconnected)")


def sensor_power_on():
  GPIO.output(WAKEUP_PIN, GPIO.HIGH)
  print("Sensor ON (wakeup=HIGH, sensor enabled)")


def main():
  print_scenario()

  GPIO.setmode(GPIO.BCM)
  GPIO.setup(WAKEUP_PIN, GPIO.OUT, initial=GPIO.LOW)
  print("BCM wakeup pin =", WAKEUP_PIN)
  print()

  sensor_power_off()
  time.sleep(POWER_CYCLE_DELAY_SEC)

  sensor_power_on()
  print()
  print("--- EN is now HIGH ---")

  if HOLD_EN_HIGH_UNTIL_CTRL_C:
    print("Keeping EN=HIGH. In another terminal, run e.g. get_pose_result.py.")
    print("Press Ctrl+C in this window when finished (GPIO will be released).")
    try:
      while True:
        time.sleep(1)
    except KeyboardInterrupt:
      print("\nExiting.")
      print(
        "Note: after GPIO.cleanup(), the pin is released; sensor may power down unless EN "
        "has a pull-up or you run detection before unplugging."
      )
  else:
    print("Exiting immediately; EN was HIGH only briefly — use HOLD_EN_HIGH_UNTIL_CTRL_C=True for demos.")


if __name__ == "__main__":
  try:
    main()
  finally:
    GPIO.cleanup()
