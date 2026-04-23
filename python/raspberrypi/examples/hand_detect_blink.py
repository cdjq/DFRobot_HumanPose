#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file   hand_detect_blink.py
@brief  Same as pose_detect_blink.py — hand detection + LED (easier to demo than full body pose).
@details Wrapper so you can run ``python hand_detect_blink.py`` without the old filename.
"""

from pose_detect_blink import main

if __name__ == "__main__":
  main()
