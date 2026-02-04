#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file   set_baud.py
@brief  Set UART baud rate example.
@details This example configures and modifies the sensor's UART communication baud rate.
@n      Connect and init sensor at current baud rate, send set command to change to target baud rate.
@n      After change, re-open serial port at new baud rate to continue communication.
@copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
@license     The MIT License (MIT)
@author [thdyyl](yuanlong.yu@dfrobot.com)
@version V1.0
@date   2026-02-04
@url    https://github.com/DFRobot/DFRobot_HumanPose
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinpong.board import Board
from DFRobot_HumanPose import DFRobot_HumanPose_UART

# ------------ Configuration (use BAUD_* constants from library) ------------
# Serial port where sensor is connected (e.g. /dev/ttyS0 or /dev/ttyAMA0 on Raspberry Pi)
UART_TTY = "/dev/ttyS0"

# Current baud rate the sensor is using (often BAUD_9600 when first used or never changed)
INITIAL_BAUD = DFRobot_HumanPose_UART.BAUD_9600

# Target baud rate to set (after success, use this baud rate for next connection)
TARGET_BAUD = DFRobot_HumanPose_UART.BAUD_115200


def main():
  Board("RPI").begin()

  print("========== DFRobot HumanPose Baud Rate Setting Example ==========")

  # Initialize sensor at current baud rate
  print("Initializing sensor with initial baud rate {}...".format(INITIAL_BAUD))
  human_pose = DFRobot_HumanPose_UART(tty_name=UART_TTY, baudrate=INITIAL_BAUD)

  if not human_pose.begin():
    print("Sensor init fail! Check wiring and that current baud rate is {}.".format(INITIAL_BAUD))
    return
  print("Sensor init success!")

  if TARGET_BAUD not in DFRobot_HumanPose_UART.BAUD_OPTIONS:
    print("Error: target baud {} not in supported list: {}".format(TARGET_BAUD, DFRobot_HumanPose_UART.BAUD_OPTIONS))
    return

  if TARGET_BAUD == INITIAL_BAUD:
    print("Already {}; no change needed.".format(TARGET_BAUD))
    return

  # Set target baud rate
  print("\nSetting baud rate to {}...".format(TARGET_BAUD))
  if human_pose.set_baud(TARGET_BAUD):
    print("Baud rate set success!")
    print("Note: After change, re-open serial port and init sensor at new baud rate {}.".format(TARGET_BAUD))
    print("Next run: set INITIAL_BAUD to {}.".format(TARGET_BAUD))
  else:
    print("Baud rate set failed!")

  print("\nExample completed.")


if __name__ == "__main__":
  main()
