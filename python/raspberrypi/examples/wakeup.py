#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file   wakeup.py
@brief  Wakeup example (placeholder).
@details This example is for waking or resetting the Human Pose sensor; implement as needed for your hardware (e.g. GPIO enable pin or wake command over I2C/UART).
@copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
@license     The MIT License (MIT)
@author [thdyyl](yuanlong.yu@dfrobot.com)
@version V1.0
@date   2026-02-04
@url    https://github.com/DFRobot/DFRobot_HumanPose
"""

import time
import RPi.GPIO as GPIO

# Use BCM numbering
WAKEUP_PIN = 17  # Change to the BCM pin you actually use


def sensor_power_off():
  """Power off: LOW level = disconnect / disable sensor"""
  GPIO.output(WAKEUP_PIN, GPIO.LOW)
  print("Sensor OFF (wakeup=LOW, sensor disconnected)")


def sensor_power_on():
  """Power on: HIGH level = enable / start sensor"""
  GPIO.output(WAKEUP_PIN, GPIO.HIGH)
  print("Sensor ON (wakeup=HIGH, sensor enabled)")


def main():
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(WAKEUP_PIN, GPIO.OUT, initial=GPIO.LOW)
  print("Use BCM numbering, wakeup pin =", WAKEUP_PIN)

  # Turn sensor off first
  sensor_power_off()
  time.sleep(1)

  # Then turn sensor on
  sensor_power_on()
  # After this, you can run get_pose_result / get_hand_result examples if needed


if __name__ == "__main__":
  try:
    main()
  finally:
    GPIO.cleanup()
