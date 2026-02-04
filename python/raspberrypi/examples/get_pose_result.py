#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file   get_pose_result.py
@brief  Get pose result example.
@details This example runs human pose detection and displays results (bounding box and keypoints) on a tkinter canvas.
@n      Uses MODEL_POSE; detection runs in a background thread and updates the GUI.
@copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
@license     The MIT License (MIT)
@author [thdyyl](yuanlong.yu@dfrobot.com)
@version V1.0
@date   2026-02-04
@url    https://github.com/DFRobot/DFRobot_HumanPose
"""

import time
import threading
import tkinter as tk
from pinpong.board import Board
import sys
sys.path.append("../")
from DFRobot_HumanPose import DFRobot_HumanPose_I2C, DFRobot_HumanPose_UART, HandResult, PoseResult

Board("RPI").begin()
# Board("UNIHIKER").begin()

# Canvas size (adjust to sensor output resolution if needed)
W, H = 320, 240

root = tk.Tk()
root.title("DFRobot HumanPose")
root.geometry(f"{W}x{H}")

canvas = tk.Canvas(root, width=W, height=H, bg="white")
canvas.pack()

COLORS = ["red", "blue", "green", "orange", "purple", "brown"]


def draw_point(x, y, color="black", label=None):
  """Draw a single keypoint on the canvas."""
  r = 3
  if x == 0 and y == 0:
    return
  canvas.create_oval(x - r, y - r, x + r, y + r, fill=color, outline=color)
  if label:
    canvas.create_text(x, y - 10, text=label, fill=color)


def draw_line(p1, p2, color="blue"):
  """Draw a line between two keypoints."""
  if (p1.x == 0 and p1.y == 0) or (p2.x == 0 and p2.y == 0):
    return
  canvas.create_line(p1.x, p1.y, p2.x, p2.y, fill=color, width=2)


def draw_results(results):
  """Redraw canvas with all detection results (bbox, keypoints, skeleton)."""
  canvas.delete("all")
  if not results:
    return

  for idx, r in enumerate(results):
    if r is None:
      continue
    color = COLORS[(getattr(r, "id", idx) or idx) % len(COLORS)]

    # Draw bounding box
    x1 = getattr(r, "xLeft", 0)
    y1 = getattr(r, "yTop", 0)
    w = getattr(r, "width", 0)
    h = getattr(r, "height", 0)
    if w and h:
      canvas.create_rectangle(x1, y1, x1 + w, y1 + h, outline=color, width=2)

    # Title (id, score, name)
    title = f"id:{getattr(r, 'id', 0)} score:{getattr(r, 'score', 0)} {getattr(r, 'name', '')}"
    canvas.create_text(x1 + 5, y1 - 5, text=title, fill=color, anchor="sw")

    # HandResult: 21 keypoints + skeleton
    if isinstance(r, HandResult):
      pts = [
        ("wrist", r.wrist),
        ("thumbCmc", r.thumbCmc),
        ("thumbMcp", r.thumbMcp),
        ("thumbIp", r.thumbIp),
        ("thumbTip", r.thumbTip),
        ("indexMcp", r.indexFingerMcp),
        ("indexPip", r.indexFingerPip),
        ("indexDip", r.indexFingerDip),
        ("indexTip", r.indexFingerTip),
        ("middleMcp", r.middleFingerMcp),
        ("middlePip", r.middleFingerPip),
        ("middleDip", r.middleFingerDip),
        ("middleTip", r.middleFingerTip),
        ("ringMcp", r.ringFingerMcp),
        ("ringPip", r.ringFingerPip),
        ("ringDip", r.ringFingerDip),
        ("ringTip", r.ringFingerTip),
        ("pinkyMcp", r.pinkyFingerMcp),
        ("pinkyPip", r.pinkyFingerPip),
        ("pinkyDip", r.pinkyFingerDip),
        ("pinkyTip", r.pinkyFingerTip),
      ]
      for name, p in pts:
        draw_point(p.x, p.y, color=color)

      draw_line(r.wrist, r.thumbCmc, color)
      draw_line(r.thumbCmc, r.thumbMcp, color)
      draw_line(r.thumbMcp, r.thumbIp, color)
      draw_line(r.thumbIp, r.thumbTip, color)

      draw_line(r.wrist, r.indexFingerMcp, color)
      draw_line(r.indexFingerMcp, r.indexFingerPip, color)
      draw_line(r.indexFingerPip, r.indexFingerDip, color)
      draw_line(r.indexFingerDip, r.indexFingerTip, color)

      draw_line(r.wrist, r.middleFingerMcp, color)
      draw_line(r.middleFingerMcp, r.middleFingerPip, color)
      draw_line(r.middleFingerPip, r.middleFingerDip, color)
      draw_line(r.middleFingerDip, r.middleFingerTip, color)

      draw_line(r.wrist, r.ringFingerMcp, color)
      draw_line(r.ringFingerMcp, r.ringFingerPip, color)
      draw_line(r.ringFingerPip, r.ringFingerDip, color)
      draw_line(r.ringFingerDip, r.ringFingerTip, color)

      draw_line(r.wrist, r.pinkyFingerMcp, color)
      draw_line(r.pinkyFingerMcp, r.pinkyFingerPip, color)
      draw_line(r.pinkyFingerPip, r.pinkyFingerDip, color)
      draw_line(r.pinkyFingerDip, r.pinkyFingerTip, color)

    # PoseResult: 17 keypoints + skeleton
    elif isinstance(r, PoseResult):
      pose_pts = [r.nose, r.leye, r.reye, r.lear, r.rear, r.lshoulder, r.rshoulder, r.lelbow, r.relbow, r.lwrist, r.rwrist, r.lhip, r.rhip, r.lknee, r.rknee, r.lankle, r.rankle]
      for p in pose_pts:
        draw_point(p.x, p.y, color=color)

      draw_line(r.lshoulder, r.rshoulder, color)
      draw_line(r.lshoulder, r.lelbow, color)
      draw_line(r.lelbow, r.lwrist, color)
      draw_line(r.rshoulder, r.relbow, color)
      draw_line(r.relbow, r.rwrist, color)

      draw_line(r.lshoulder, r.lhip, color)
      draw_line(r.rshoulder, r.rhip, color)
      draw_line(r.lhip, r.rhip, color)

      draw_line(r.lhip, r.lknee, color)
      draw_line(r.lknee, r.lankle, color)
      draw_line(r.rhip, r.rknee, color)
      draw_line(r.rknee, r.rankle, color)


class ProtocolThread(threading.Thread):
  """Background thread: poll get_result and push results to GUI for drawing."""

  def __init__(self, hp):
    super().__init__(daemon=True)
    self.hp = hp

  def run(self):
    while True:
      self.hp.get_result()
      results = []
      while self.hp.available_result():
        r = self.hp.pop_result()
        if r:
          results.append(r)
          print(r)
      root.after(0, draw_results, results)
      time.sleep(0.02)


if __name__ == "__main__":
  humanpose = DFRobot_HumanPose_I2C(bus_num=1)
  # humanpose = DFRobot_HumanPose_UART(tty_name="/dev/ttyAMA0")

  if not humanpose.begin():
    print("begin failed")
  else:
    humanpose.set_model_type(humanpose.MODEL_POSE)
    ProtocolThread(humanpose).start()
    root.mainloop()
