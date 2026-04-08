"""
@file   DFRobot_HumanPose.py
@brief  Raspberry Pi Python driver for DFRobot Human Pose sensor (I2C/UART).
@n      Human pose detection, hand detection, and learned target recognition.
@copyright Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
@license The MIT License (MIT)
@author [thdyyl](yuanlong.yu@dfrobot.com)
@version V1.0
@date   2026-02-04
@url    https://github.com/DFRobot/DFRobot_HumanPose
"""

from pinpong.board import I2C, gboard, UART
import time
import logging
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any, Union
from abc import ABC, abstractmethod

logging.basicConfig(
  level=logging.INFO,
  format="%(asctime)s [%(levelname)s] %(message)s",
  datefmt="%Y-%m-%d %H:%M:%S",
)

ByteList = List[int]
BytesLike = Union[str, bytes, bytearray, ByteList]


@dataclass
class PointU16:
  """2D point with 16-bit unsigned x, y coordinates (e.g. keypoint position)."""

  x: int = 0
  y: int = 0


def read_point_u16(points: list, i: int) -> Optional[PointU16]:
  """
  @fn    read_point_u16
  @brief Read a 16-bit point from a list of points at index i.
  @param points: List of point data (each element [x, y] or compatible).
  @param i: Index of the point.
  @return PointU16 if valid, else None.
  """
  if not isinstance(points, list) or i >= len(points):
    return None
  p = points[i]
  if not (isinstance(p, list) and len(p) >= 2):
    return None
  return PointU16(int(p[0]) & 0xFFFF, int(p[1]) & 0xFFFF)


def _id_to_name(names: list, _id: int) -> str:
  """Resolve class id to name from available_classes list; returns 'unknown' if id is 0 or out of range."""
  if _id and isinstance(names, list) and 1 <= _id <= len(names):
    ob = names[_id - 1]
    if isinstance(ob, dict):
      return ob.get("name", "unknown") or "unknown"
  return "unknown"


@dataclass
class Result:
  """
  @brief Base detection result: bounding box, score, and class id/name.
  @n     Used for both pose and hand detection; subclass adds keypoints.
  """

  id: int = 0
  xLeft: int = 0
  yTop: int = 0
  width: int = 0
  height: int = 0
  score: int = 0
  name: str = "unknown"
  used: bool = False

  @classmethod
  def from_json(cls, data: list, names: list) -> "Result":
    """
    @fn    from_json
    @brief Parse a single target from JSON. data format: [box, points], box: [xLeft, yTop, width, height, score, id].
    @param data: Single target data [box, points].
    @param names: available_classes list for id-to-name mapping.
    @return Result instance.
    """
    r = cls()
    r.used = False

    box = data[0] if isinstance(data, list) and len(data) > 0 else None
    if not (isinstance(box, list) and len(box) >= 6):
      return r  # invalid structure, return default

    r.xLeft = int(box[0]) & 0xFFFF
    r.yTop = int(box[1]) & 0xFFFF
    r.width = int(box[2]) & 0xFFFF
    r.height = int(box[3]) & 0xFFFF
    r.score = int(box[4]) & 0xFF
    r.id = int(box[5]) & 0xFF
    r.name = _id_to_name(names, r.id)
    return r


@dataclass
class PoseResult(Result):
  """
  @brief Pose detection result with 17 body keypoints (nose, eyes, ears, shoulders, elbows, wrists, hips, knees, ankles).
  """

  nose: PointU16 = field(default_factory=PointU16)
  leye: PointU16 = field(default_factory=PointU16)
  reye: PointU16 = field(default_factory=PointU16)
  lear: PointU16 = field(default_factory=PointU16)
  rear: PointU16 = field(default_factory=PointU16)

  lshoulder: PointU16 = field(default_factory=PointU16)
  rshoulder: PointU16 = field(default_factory=PointU16)
  lelbow: PointU16 = field(default_factory=PointU16)
  relbow: PointU16 = field(default_factory=PointU16)

  lwrist: PointU16 = field(default_factory=PointU16)
  rwrist: PointU16 = field(default_factory=PointU16)
  lhip: PointU16 = field(default_factory=PointU16)
  rhip: PointU16 = field(default_factory=PointU16)

  lknee: PointU16 = field(default_factory=PointU16)
  rknee: PointU16 = field(default_factory=PointU16)

  lankle: PointU16 = field(default_factory=PointU16)
  rankle: PointU16 = field(default_factory=PointU16)

  @classmethod
  def from_json(cls, data: list, names: list) -> "PoseResult":
    """
    @fn    from_json
    @brief Parse pose result from JSON; keypoint order 0~16: nose, leye, reye, lear, rear, shoulders, elbows, wrists, hips, knees, ankles.
    @param data: [box, points].
    @param names: available_classes.
    @return PoseResult instance.
    """
    base = Result.from_json(data, names)
    pr = cls(**base.__dict__)

    points = data[1] if isinstance(data, list) and len(data) > 1 else []
    mapping = [
      ("nose", 0),
      ("leye", 1),
      ("reye", 2),
      ("lear", 3),
      ("rear", 4),
      ("lshoulder", 5),
      ("rshoulder", 6),
      ("lelbow", 7),
      ("relbow", 8),
      ("lwrist", 9),
      ("rwrist", 10),
      ("lhip", 11),
      ("rhip", 12),
      ("lknee", 13),
      ("rknee", 14),
      ("lankle", 15),
      ("rankle", 16),
    ]
    for attr, i in mapping:
      p = read_point_u16(points, i)
      if p is not None:
        setattr(pr, attr, p)
    return pr


@dataclass
class HandResult(Result):
  """
  @brief Hand detection result with 21 keypoints (wrist, thumb, index, middle, ring, pinky joints and tips).
  """

  wrist: PointU16 = field(default_factory=PointU16)
  thumbCmc: PointU16 = field(default_factory=PointU16)
  thumbMcp: PointU16 = field(default_factory=PointU16)
  thumbIp: PointU16 = field(default_factory=PointU16)
  thumbTip: PointU16 = field(default_factory=PointU16)

  indexFingerMcp: PointU16 = field(default_factory=PointU16)
  indexFingerPip: PointU16 = field(default_factory=PointU16)
  indexFingerDip: PointU16 = field(default_factory=PointU16)
  indexFingerTip: PointU16 = field(default_factory=PointU16)

  middleFingerMcp: PointU16 = field(default_factory=PointU16)
  middleFingerPip: PointU16 = field(default_factory=PointU16)
  middleFingerDip: PointU16 = field(default_factory=PointU16)
  middleFingerTip: PointU16 = field(default_factory=PointU16)

  ringFingerMcp: PointU16 = field(default_factory=PointU16)
  ringFingerPip: PointU16 = field(default_factory=PointU16)
  ringFingerDip: PointU16 = field(default_factory=PointU16)
  ringFingerTip: PointU16 = field(default_factory=PointU16)

  pinkyFingerMcp: PointU16 = field(default_factory=PointU16)
  pinkyFingerPip: PointU16 = field(default_factory=PointU16)
  pinkyFingerDip: PointU16 = field(default_factory=PointU16)
  pinkyFingerTip: PointU16 = field(default_factory=PointU16)

  @classmethod
  def from_json(cls, data: list, names: list) -> "HandResult":
    """
    @fn    from_json
    @brief Parse hand result from JSON; 21 keypoints: wrist, thumb (4), index/middle/ring/pinky (4 each).
    @param data: [box, points].
    @param names: available_classes.
    @return HandResult instance.
    """
    base = Result.from_json(data, names)
    hr = cls(**base.__dict__)

    points = data[1] if isinstance(data, list) and len(data) > 1 else []
    mapping = [
      ("wrist", 0),
      ("thumbCmc", 1),
      ("thumbMcp", 2),
      ("thumbIp", 3),
      ("thumbTip", 4),
      ("indexFingerMcp", 5),
      ("indexFingerPip", 6),
      ("indexFingerDip", 7),
      ("indexFingerTip", 8),
      ("middleFingerMcp", 9),
      ("middleFingerPip", 10),
      ("middleFingerDip", 11),
      ("middleFingerTip", 12),
      ("ringFingerMcp", 13),
      ("ringFingerPip", 14),
      ("ringFingerDip", 15),
      ("ringFingerTip", 16),
      ("pinkyFingerMcp", 17),
      ("pinkyFingerPip", 18),
      ("pinkyFingerDip", 19),
      ("pinkyFingerTip", 20),
    ]
    for attr, i in mapping:
      p = read_point_u16(points, i)
      if p is not None:
        setattr(hr, attr, p)
    return hr


class DFRobot_HumanPose(object):
  """
  @brief Base class for DFRobot Human Pose sensor (I2C/UART).
  @n     Use DFRobot_HumanPose_I2C or DFRobot_HumanPose_UART to communicate.
  """

  I2C_ADDRESS = 0x3A
  MAX_PL_LEN = 250
  MAX_RESULT_NUM = 16
  HUMANPOSE_NAME = "DFRobot Human Pose"

  TRANSPORT = 0x10
  TRANSPORT_CMD_READ = 0x01
  TRANSPORT_CMD_WRITE = 0x02
  TRANSPORT_CMD_AVAILABLE = 0x03
  TRANSPORT_CMD_RESET = 0x06

  CMD_TYPE_RESPONSE = 0
  CMD_TYPE_EVENT = 1
  CMD_TYPE_LOG = 2

  AT_NAME = "NAME"
  AT_INVOKE = "INVOKE"
  AT_TSCORE = "TSCORE"
  AT_TIOU = "TIOU"
  AT_MODELS = "MODELS?"
  AT_MODEL = "MODEL"
  EVENT_INVOKE = "INVOKE"
  AT_TSIMILARITY = "TSIMILARITY"
  AT_BAUD = "BAUDRATE"
  AT_POSELIST = "POSELIST?"
  AT_HANDLIST = "HANDLIST?"
  AT_TPROTO = "TPROTO"
  AT_TPKTSZ = "TPKTSZ"
  AT_TKPTS = "TKPTS"

  # Binary protocol constants
  HP_BIN_SOF0 = 0x55
  HP_BIN_SOF1 = 0xAA
  HP_BIN_MSG_AT_RSP = 0x11
  HP_BIN_MSG_INVOKE_META = 0x00
  HP_BIN_MSG_INVOKE_BEGIN = 0x01
  HP_BIN_MSG_DET_CHUNK = 0x03
  HP_BIN_MSG_HAND_KP_CHUNK = 0x04
  HP_BIN_MSG_POSE_KP_CHUNK = 0x05
  HP_BIN_MSG_INVOKE_END = 0x07
  HP_BIN_HDR_LEN = 11

  # Binary command id
  CMD_ID_NAME = 0x0003
  CMD_ID_INVOKE = 0x0017
  CMD_ID_MODEL_SET = 0x000B
  CMD_ID_MODEL_GET = 0x000C
  CMD_ID_POSELIST = 0x0029
  CMD_ID_HANDLIST = 0x0030
  CMD_ID_TPROTO_SET = 0x001F
  CMD_ID_TPROTO_GET = 0x0020
  CMD_ID_TPKTSZ_SET = 0x0021
  CMD_ID_TPKTSZ_GET = 0x0022
  CMD_ID_TKPTS_SET = 0x0037
  CMD_ID_TKPTS_GET = 0x0038
  CMD_ID_TSCORE_SET = 0x0101
  CMD_ID_TSCORE_GET = 0x0102
  CMD_ID_TIOU_SET = 0x0103
  CMD_ID_TIOU_GET = 0x0104
  CMD_ID_TSIM_SET = 0x0105
  CMD_ID_TSIM_GET = 0x0106
  CMD_ID_BAUD_SET = 0x001D
  CMD_ID_BAUD_GET = 0x001E

  # Binary value types
  HP_BIN_NULL = 0x00
  HP_BIN_BOOL = 0x01
  HP_BIN_I64 = 0x02
  HP_BIN_U64 = 0x03
  HP_BIN_F64 = 0x04
  HP_BIN_STRING = 0x05
  HP_BIN_OBJECT = 0x06
  HP_BIN_ARRAY = 0x07
  HP_BIN_BYTES = 0x08

  CODE_OK = 0
  CODE_AGAIN = 1
  CODE_LOG = 2
  CODE_TIMEOUT = 3
  CODE_IO = 4
  CODE_INVAL = 5
  CODE_NOMEM = 6
  CODE_BUSY = 7
  CODE_NOTSUP = 8
  CODE_PERM = 9
  CODE_UNKNOWN = 10

  MODEL_HAND = 1
  MODEL_POSE = 3

  _cmd_available = [0x10, 0x03, 0, 0, 0, 0]

  def __init__(self):
    self._wait_delay = 2
    self._rx_buf = bytearray()
    self._at_payload_buf = bytearray()
    self._results = []
    self._learn_list = []
    self._pose_class_list = []
    self._hand_class_list = []
    self._name = ""
    self._ret_data = None
    self._binary_mode = False
    self._current_model = self.MODEL_HAND

    self._at_rsp_ready = False
    self._at_rsp_type = self.CMD_TYPE_RESPONSE
    self._at_rsp_code = 0
    self._at_rsp_cmd_id = 0
    self._at_rsp_name = ""

    self._invoke_event_ready = False
    self._invoke_event_code = 0
    self._invoke_model_id = 0

    self._bin_result_count = 0
    self._bin_results = [self._new_bin_result() for _ in range(self.MAX_RESULT_NUM)]

  @staticmethod
  def _read_u16_le(buf: bytes, off: int = 0) -> int:
    return int(buf[off]) | (int(buf[off + 1]) << 8)

  @staticmethod
  def _read_i16_le(buf: bytes, off: int = 0) -> int:
    return int.from_bytes(buf[off:off + 2], byteorder="little", signed=True)

  @staticmethod
  def _read_u32_le(buf: bytes, off: int = 0) -> int:
    return int.from_bytes(buf[off:off + 4], byteorder="little", signed=False)

  @staticmethod
  def _crc16_ccitt_update(crc: int, data: bytes) -> int:
    for b in data:
      crc ^= (b << 8)
      for _ in range(8):
        if crc & 0x8000:
          crc = ((crc << 1) ^ 0x1021) & 0xFFFF
        else:
          crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF

  @staticmethod
  def _new_bin_result() -> Dict[str, Any]:
    return {
      "used": False,
      "is_pose": False,
      "x": 0,
      "y": 0,
      "w": 0,
      "h": 0,
      "score": 0,
      "target": 0,
      "points": [PointU16() for _ in range(21)],
      "point_valid": [0] * 21,
    }

  def _clear_binary_results(self):
    self._bin_result_count = 0
    self._invoke_model_id = 0
    for i in range(self.MAX_RESULT_NUM):
      self._bin_results[i] = self._new_bin_result()

  def _resolve_class_name(self, cls_id: int) -> str:
    if cls_id == 0:
      return "unknown"
    names = self._pose_class_list if self._current_model == self.MODEL_POSE else self._hand_class_list
    if 1 <= cls_id <= len(names):
      return str(names[cls_id - 1])
    return f"class_{cls_id}"

  def _command_matches(self, cmd_id: int, cmd: str) -> bool:
    if not cmd:
      return False

    if cmd_id != 0:
      c = cmd
      if self.AT_INVOKE in c:
        return cmd_id == self.CMD_ID_INVOKE
      if self.AT_NAME in c or self.AT_NAME + "?" in c:
        return cmd_id == self.CMD_ID_NAME
      if self.AT_MODEL in c:
        return cmd_id in (self.CMD_ID_MODEL_SET, self.CMD_ID_MODEL_GET)
      if self.AT_TSCORE in c:
        return cmd_id in (self.CMD_ID_TSCORE_SET, self.CMD_ID_TSCORE_GET)
      if self.AT_TIOU in c:
        return cmd_id in (self.CMD_ID_TIOU_SET, self.CMD_ID_TIOU_GET)
      if self.AT_TSIMILARITY in c:
        return cmd_id in (self.CMD_ID_TSIM_SET, self.CMD_ID_TSIM_GET)
      if self.AT_POSELIST in c:
        return cmd_id == self.CMD_ID_POSELIST
      if self.AT_HANDLIST in c:
        return cmd_id == self.CMD_ID_HANDLIST
      if self.AT_BAUD in c:
        return cmd_id in (self.CMD_ID_BAUD_SET, self.CMD_ID_BAUD_GET)
      if self.AT_TPROTO in c:
        return cmd_id in (self.CMD_ID_TPROTO_SET, self.CMD_ID_TPROTO_GET)
      if self.AT_TPKTSZ in c:
        return cmd_id in (self.CMD_ID_TPKTSZ_SET, self.CMD_ID_TPKTSZ_GET)
      if self.AT_TKPTS in c:
        return cmd_id in (self.CMD_ID_TKPTS_SET, self.CMD_ID_TKPTS_GET)
      return False

    if not self._at_rsp_name:
      return False
    return self._at_rsp_name == cmd or self._at_rsp_name.startswith(cmd)

  def _parse_bin_value(self, data: bytes, offset: int):
    if offset + 8 > len(data):
      return None, offset
    v_type = int(data[offset + 0])
    v_flags = int(data[offset + 1])
    v_len = self._read_u32_le(data, offset + 4)
    offset += 8
    if offset + v_len > len(data):
      return None, offset
    body = data[offset:offset + v_len]
    offset += v_len
    return {"type": v_type, "flags": v_flags, "length": v_len, "body": body}, offset

  def _bin_to_uint8(self, v) -> Optional[int]:
    if not v:
      return None
    t = v["type"]
    body = v["body"]
    length = v["length"]
    if t == self.HP_BIN_U64 and length >= 8:
      n = int.from_bytes(body[:8], byteorder="little", signed=False)
      return min(n, 255)
    if t == self.HP_BIN_I64 and length >= 8:
      n = int.from_bytes(body[:8], byteorder="little", signed=True)
      if n < 0:
        n = 0
      if n > 255:
        n = 255
      return n
    if t == self.HP_BIN_BOOL:
      return 1 if v["flags"] else 0
    return None

  def _bin_to_string(self, v) -> Optional[str]:
    if not v or v["type"] != self.HP_BIN_STRING:
      return None
    return bytes(v["body"]).decode("utf-8", errors="ignore")

  def _bin_object_get(self, obj_v, key: str):
    if not obj_v or obj_v["type"] != self.HP_BIN_OBJECT:
      return None
    body = obj_v["body"]
    if len(body) < 2:
      return None
    count = self._read_u16_le(body, 0)
    cursor = 2
    key_bytes = key.encode("utf-8")
    for _ in range(count):
      if cursor + 2 > len(body):
        return None
      key_len = self._read_u16_le(body, cursor)
      cursor += 2
      if cursor + key_len > len(body):
        return None
      cur_key = body[cursor:cursor + key_len]
      cursor += key_len
      value, new_cursor = self._parse_bin_value(body, cursor)
      if value is None:
        return None
      cursor = new_cursor
      if cur_key == key_bytes:
        return value
    return None

  def _bin_array_to_string_list(self, arr_v) -> List[str]:
    out: List[str] = []
    if not arr_v or arr_v["type"] != self.HP_BIN_ARRAY:
      return out
    body = arr_v["body"]
    if len(body) < 2:
      return out
    count = self._read_u16_le(body, 0)
    cursor = 2
    for _ in range(count):
      v, cursor = self._parse_bin_value(body, cursor)
      if not v:
        break
      if v["type"] == self.HP_BIN_STRING:
        out.append(bytes(v["body"]).decode("utf-8", errors="ignore"))
    return out

  def _process_binary_at_response(self, flags: int, payload: bytes):
    self._at_payload_buf.extend(payload)
    if (flags & 0x01) == 0:
      return True

    p = bytes(self._at_payload_buf)
    self._at_payload_buf.clear()
    if len(p) < 12:
      return False

    rsp_type = int.from_bytes(p[1:2], byteorder="little", signed=True)
    rsp_code = self._read_i16_le(p, 2)
    rsp_cmd_id = self._read_u16_le(p, 4)
    data_len = self._read_u32_le(p, 8)

    if 12 + data_len <= len(p) and data_len >= 8:
      data_ptr = p[12:12 + data_len]
      root, _ = self._parse_bin_value(data_ptr, 0)
      if root:
        if rsp_cmd_id == self.CMD_ID_NAME:
          s = self._bin_to_string(root)
          if s is not None:
            self._name = s
        elif rsp_cmd_id in (
          self.CMD_ID_TSCORE_GET, self.CMD_ID_TSCORE_SET,
          self.CMD_ID_TIOU_GET, self.CMD_ID_TIOU_SET,
          self.CMD_ID_TSIM_GET, self.CMD_ID_TSIM_SET,
          self.CMD_ID_TKPTS_GET, self.CMD_ID_TKPTS_SET,
        ):
          v = self._bin_to_uint8(root)
          if v is not None:
            self._ret_data = v
        elif rsp_cmd_id in (self.CMD_ID_MODEL_GET, self.CMD_ID_MODEL_SET):
          v = None
          if root["type"] == self.HP_BIN_OBJECT:
            id_value = self._bin_object_get(root, "id")
            v = self._bin_to_uint8(id_value)
          if v is None:
            v = self._bin_to_uint8(root)
          if v is not None:
            self._ret_data = v
            if v in (self.MODEL_HAND, self.MODEL_POSE):
              self._current_model = v
        elif rsp_cmd_id in (self.CMD_ID_HANDLIST, self.CMD_ID_POSELIST):
          parsed = self._bin_array_to_string_list(root)
          self._learn_list = parsed
          if rsp_cmd_id == self.CMD_ID_HANDLIST:
            self._hand_class_list = parsed
          else:
            self._pose_class_list = parsed

    if rsp_type == self.CMD_TYPE_RESPONSE:
      if not self._at_rsp_ready:
        self._at_rsp_type = rsp_type
        self._at_rsp_code = rsp_code
        self._at_rsp_cmd_id = rsp_cmd_id
        self._at_rsp_name = ""
        self._at_rsp_ready = True
    elif rsp_type == self.CMD_TYPE_EVENT and rsp_cmd_id == self.CMD_ID_INVOKE:
      self._invoke_event_code = rsp_code
      self._invoke_event_ready = True
    return True

  def _finalize_binary_results(self):
    new_results = []
    count = min(self._bin_result_count, self.MAX_RESULT_NUM)
    for i in range(count):
      b = self._bin_results[i]
      if not b["used"]:
        continue

      name = self._resolve_class_name(int(b["target"]))
      target_id = int(b["target"]) & 0xFF

      if b["is_pose"]:
        r = PoseResult()
        r.nose = b["points"][0]
        r.leye = b["points"][1]
        r.reye = b["points"][2]
        r.lear = b["points"][3]
        r.rear = b["points"][4]
        r.lshoulder = b["points"][5]
        r.rshoulder = b["points"][6]
        r.lelbow = b["points"][7]
        r.relbow = b["points"][8]
        r.lwrist = b["points"][9]
        r.rwrist = b["points"][10]
        r.lhip = b["points"][11]
        r.rhip = b["points"][12]
        r.lknee = b["points"][13]
        r.rknee = b["points"][14]
        r.lankle = b["points"][15]
        r.rankle = b["points"][16]
      else:
        r = HandResult()
        r.wrist = b["points"][0]
        r.thumbCmc = b["points"][1]
        r.thumbMcp = b["points"][2]
        r.thumbIp = b["points"][3]
        r.thumbTip = b["points"][4]
        r.indexFingerMcp = b["points"][5]
        r.indexFingerPip = b["points"][6]
        r.indexFingerDip = b["points"][7]
        r.indexFingerTip = b["points"][8]
        r.middleFingerMcp = b["points"][9]
        r.middleFingerPip = b["points"][10]
        r.middleFingerDip = b["points"][11]
        r.middleFingerTip = b["points"][12]
        r.ringFingerMcp = b["points"][13]
        r.ringFingerPip = b["points"][14]
        r.ringFingerDip = b["points"][15]
        r.ringFingerTip = b["points"][16]
        r.pinkyFingerMcp = b["points"][17]
        r.pinkyFingerPip = b["points"][18]
        r.pinkyFingerDip = b["points"][19]
        r.pinkyFingerTip = b["points"][20]

      r.xLeft = int(b["x"]) & 0xFFFF
      r.yTop = int(b["y"]) & 0xFFFF
      r.width = int(b["w"]) & 0xFFFF
      r.height = int(b["h"]) & 0xFFFF
      r.score = int(b["score"]) & 0xFF
      r.id = target_id
      r.name = name
      r.used = False
      new_results.append(r)
    self._results = new_results

  def _process_binary_invoke(self, msg_type: int, flags: int, payload: bytes):
    _ = flags
    if msg_type == self.HP_BIN_MSG_INVOKE_META:
      ret_code = self._read_i16_le(payload, 0) if len(payload) >= 2 else 0
      if len(payload) >= 4:
        model_id = self._read_u16_le(payload, 2)
        if model_id in (self.MODEL_HAND, self.MODEL_POSE):
          self._current_model = model_id

      if not self._at_rsp_ready:
        self._at_rsp_type = self.CMD_TYPE_RESPONSE
        self._at_rsp_cmd_id = self.CMD_ID_INVOKE
        self._at_rsp_code = ret_code
        self._at_rsp_name = ""
        self._at_rsp_ready = True
      return True

    if msg_type == self.HP_BIN_MSG_INVOKE_BEGIN:
      self._clear_binary_results()
      if len(payload) >= 12:
        self._invoke_model_id = self._read_u16_le(payload, 10)
        if self._invoke_model_id in (self.MODEL_HAND, self.MODEL_POSE):
          self._current_model = self._invoke_model_id
      return True

    if msg_type == self.HP_BIN_MSG_DET_CHUNK:
      if len(payload) < 6:
        return False
      total_boxes = self._read_u16_le(payload, 0)
      offset = self._read_u16_le(payload, 2)
      item_count = payload[4]
      item_size = payload[5]
      if item_size != 11:
        return False
      self._bin_result_count = total_boxes
      cursor = 6
      for i in range(item_count):
        if cursor + 11 > len(payload):
          break
        idx = offset + i
        if idx < self.MAX_RESULT_NUM:
          b = self._bin_results[idx]
          b["used"] = True
          b["is_pose"] = (self._current_model == self.MODEL_POSE)
          b["x"] = self._read_u16_le(payload, cursor + 0)
          b["y"] = self._read_u16_le(payload, cursor + 2)
          b["w"] = self._read_u16_le(payload, cursor + 4)
          b["h"] = self._read_u16_le(payload, cursor + 6)
          b["score"] = payload[cursor + 8]
          b["target"] = self._read_u16_le(payload, cursor + 9)
        cursor += 11
      return True

    if msg_type in (self.HP_BIN_MSG_HAND_KP_CHUNK, self.HP_BIN_MSG_POSE_KP_CHUNK):
      if len(payload) < 10:
        return False
      total_kps = self._read_u16_le(payload, 0)
      kp_index = self._read_u16_le(payload, 2)
      total_points = payload[4]
      point_offset = payload[5]
      point_count = payload[6]
      point_format = payload[7]  # 0 hand[x,y], 1 pose[x,y,score,target]
      has_box = payload[8]
      self._bin_result_count = total_kps
      if kp_index >= self.MAX_RESULT_NUM:
        return True

      b = self._bin_results[kp_index]
      b["used"] = True
      b["is_pose"] = (point_format == 1)

      cursor = 10
      if has_box:
        if cursor + 11 > len(payload):
          return False
        b["x"] = self._read_u16_le(payload, cursor + 0)
        b["y"] = self._read_u16_le(payload, cursor + 2)
        b["w"] = self._read_u16_le(payload, cursor + 4)
        b["h"] = self._read_u16_le(payload, cursor + 6)
        b["score"] = payload[cursor + 8]
        b["target"] = self._read_u16_le(payload, cursor + 9)
        cursor += 11

      if point_offset > total_points:
        return False
      if point_count > (total_points - point_offset):
        point_count = total_points - point_offset

      for i in range(point_count):
        point_index = point_offset + i
        if point_format == 1:
          if cursor + 6 > len(payload):
            break
          x = self._read_u16_le(payload, cursor + 0)
          y = self._read_u16_le(payload, cursor + 2)
          target = payload[cursor + 5]
          if target < 17:
            point_index = target
          cursor += 6
        else:
          if cursor + 4 > len(payload):
            break
          x = self._read_u16_le(payload, cursor + 0)
          y = self._read_u16_le(payload, cursor + 2)
          cursor += 4

        if point_index < 21:
          b["points"][point_index] = PointU16(int(x), int(y))
          b["point_valid"][point_index] = 1
      return True

    if msg_type == self.HP_BIN_MSG_INVOKE_END:
      self._invoke_event_code = self._read_i16_le(payload, 0) if len(payload) >= 2 else 0
      self._finalize_binary_results()
      self._invoke_event_ready = True
      return True

    return False

  def _process_binary_frames(self) -> bool:
    if len(self._rx_buf) < 2:
      return False

    sof_pos = -1
    for i in range(len(self._rx_buf) - 1):
      if self._rx_buf[i] == self.HP_BIN_SOF0 and self._rx_buf[i + 1] == self.HP_BIN_SOF1:
        sof_pos = i
        break

    if sof_pos < 0:
      return False
    if sof_pos > 0:
      del self._rx_buf[:sof_pos]
      return True

    if len(self._rx_buf) < self.HP_BIN_HDR_LEN:
      return False

    frame = bytes(self._rx_buf)
    payload_len = self._read_u16_le(frame, 7)
    frame_len = self.HP_BIN_HDR_LEN + payload_len
    if frame_len > len(self._rx_buf):
      return False
    if frame_len > 32768:
      del self._rx_buf[:1]
      return True

    frame = bytes(self._rx_buf[:frame_len])
    crc_rx = self._read_u16_le(frame, 9)
    crc = self._crc16_ccitt_update(0xFFFF, frame[:9])
    crc = self._crc16_ccitt_update(crc, frame[self.HP_BIN_HDR_LEN:])
    if crc != crc_rx:
      del self._rx_buf[:1]
      return True

    msg_type = frame[3]
    flags = frame[4]
    payload = frame[self.HP_BIN_HDR_LEN:]
    if msg_type == self.HP_BIN_MSG_AT_RSP:
      self._process_binary_at_response(flags, payload)
    else:
      self._process_binary_invoke(msg_type, flags, payload)

    del self._rx_buf[:frame_len]
    return True

  def _get_name(self) -> int:
    self._write(f"AT+{self.AT_NAME}?\r\n")
    self._name = ""
    if self._wait(self.CMD_TYPE_RESPONSE, self.AT_NAME) == self.CODE_OK:
      return self.CODE_OK
    return self.CODE_TIMEOUT

  def begin(self):
    """
    @fn    begin
    @brief Initialize the sensor in binary mode and verify device name.
    @return True: Initialization succeeded, False: Initialization failed.
    """
    self._rx_buf.clear()
    self._at_payload_buf.clear()
    self._results = []
    self._clear_binary_results()
    self._at_rsp_ready = False
    self._invoke_event_ready = False
    self._binary_mode = False

    # Force binary mode
    self._write(f"AT+{self.AT_TPROTO}=1\r\n")
    tproto_ret = self._wait(self.CMD_TYPE_RESPONSE, self.AT_TPROTO, timeout_ms=1200)
    if tproto_ret != self.CODE_OK:
      return False
    self._binary_mode = True

    # Tune packet size for stable transfer on Python side
    self._write(f"AT+{self.AT_TPKTSZ}=192\r\n")
    self._wait(self.CMD_TYPE_RESPONSE, self.AT_TPKTSZ, timeout_ms=1200)

    # Python runs on large-memory boards, default to keypoints enabled
    if self.set_keypoint_output(True) != self.CODE_OK:
      return False

    if (self._get_name() != self.CODE_OK) or (self.HUMANPOSE_NAME not in self._name):
      return False
    return True

  def _wait(self, expected_type: int, cmd: str, timeout_ms=3000) -> int:
    start = time.monotonic()

    while (time.monotonic() - start) * 1000 <= timeout_ms:
      if expected_type == self.CMD_TYPE_RESPONSE and self._at_rsp_ready and self._at_rsp_type == self.CMD_TYPE_RESPONSE:
        if self._command_matches(self._at_rsp_cmd_id, cmd):
          ret = int(self._at_rsp_code)
          self._at_rsp_ready = False
          return ret
        self._at_rsp_ready = False

      if expected_type == self.CMD_TYPE_EVENT and cmd and self.EVENT_INVOKE in cmd and self._invoke_event_ready:
        ret = int(self._invoke_event_code)
        self._invoke_event_ready = False
        return ret

      n = self._available()
      if n <= 0:
        time.sleep(0.001)
        continue

      chunk = self._read(n)
      if not chunk:
        continue

      data = bytes(chunk)
      self._rx_buf.extend(data)
      progressed = True
      while progressed:
        progressed = self._process_binary_frames()

    return self.CODE_TIMEOUT

  def get_result(self):
    """
    @fn    get_result
    @brief Trigger one detection and wait for INVOKE response/event; results are stored and read via available_result/pop_result.
    @return CODE_OK: Success, CODE_TIMEOUT: Timeout.
    """
    self._invoke_event_ready = False
    self._invoke_event_code = 0
    for r in self._results:
      r.used = True
    self._clear_binary_results()
    self._write(f"AT+{self.AT_INVOKE}=1,0,1\r\n")
    if self._wait(self.CMD_TYPE_RESPONSE, self.AT_INVOKE, timeout_ms=1000) == self.CODE_OK:
      if self._wait(self.CMD_TYPE_EVENT, self.EVENT_INVOKE, timeout_ms=1000) == self.CODE_OK:
        return self.CODE_OK
    return self.CODE_TIMEOUT

  def set_confidence(self, confidence):
    """
    @fn    set_confidence
    @brief Set detection confidence threshold (0-100), default typically 60.
    @param confidence: Confidence threshold.
    @return CODE_OK: Success, CODE_TIMEOUT: Timeout.
    """
    self._write(f"AT+{self.AT_TSCORE}={confidence}\r\n")
    if self._wait(self.CMD_TYPE_RESPONSE, self.AT_TSCORE) == self.CODE_OK:
      return self.CODE_OK
    return self.CODE_TIMEOUT

  def set_iou(self, iou):
    """
    @fn    set_iou
    @brief Set IOU threshold (0-100) for non-maximum suppression, default typically 45.
    @param iou: IOU threshold.
    @return CODE_OK: Success, CODE_TIMEOUT: Timeout.
    """
    self._write(f"AT+{self.AT_TIOU}={iou}\r\n")
    if self._wait(self.CMD_TYPE_RESPONSE, self.AT_TIOU) == self.CODE_OK:
      return self.CODE_OK
    return self.CODE_TIMEOUT

  def set_learn_similarity(self, similarity):
    """
    @fn    set_learn_similarity
    @brief Set similarity threshold (0-100) for matching learned targets, default typically 60.
    @param similarity: Similarity threshold.
    @return CODE_OK: Success, CODE_TIMEOUT: Timeout.
    """
    self._write(f"AT+{self.AT_TSIMILARITY}={similarity}\r\n")
    if self._wait(self.CMD_TYPE_RESPONSE, self.AT_TSIMILARITY) == self.CODE_OK:
      return self.CODE_OK
    return self.CODE_TIMEOUT

  def set_model_type(self, model):
    """
    @fn    set_model_type
    @brief Set detection model type.
    @param model: MODEL_HAND (1) for hand detection, MODEL_POSE (3) for human pose detection.
    @return CODE_OK: Success, CODE_TIMEOUT: Timeout.
    """
    self._write(f"AT+{self.AT_MODEL}={model}\r\n")
    if self._wait(self.CMD_TYPE_RESPONSE, self.AT_MODEL) == self.CODE_OK:
      self._current_model = model
      return self.CODE_OK
    return self.CODE_TIMEOUT

  def get_confidence(self):
    """
    @fn    get_confidence
    @brief Get current confidence threshold.
    @return Current value on success, None on timeout.
    """
    self._write(f"AT+{self.AT_TSCORE}?\r\n")
    if self._wait(self.CMD_TYPE_RESPONSE, self.AT_TSCORE) == self.CODE_OK:
      return self._ret_data
    return None

  def get_iou(self):
    """
    @fn    get_iou
    @brief Get current IOU threshold.
    @return Current value on success, None on timeout.
    """
    self._write(f"AT+{self.AT_TIOU}?\r\n")
    if self._wait(self.CMD_TYPE_RESPONSE, self.AT_TIOU) == self.CODE_OK:
      return self._ret_data
    return None

  def get_learn_similarity(self):
    """
    @fn    get_learn_similarity
    @brief Get current learn similarity threshold.
    @return Current value on success, None on timeout.
    """
    self._write(f"AT+{self.AT_TSIMILARITY}?\r\n")
    if self._wait(self.CMD_TYPE_RESPONSE, self.AT_TSIMILARITY) == self.CODE_OK:
      return self._ret_data
    return None

  def get_learn_list(self, model):
    """
    @fn    get_learn_list
    @brief Get the list of learned target names for the given model.
    @param model: MODEL_POSE or MODEL_HAND.
    @return List of names. Returns empty list on timeout.
    """
    self._learn_list = []
    model_list = self.AT_POSELIST if model == self.MODEL_POSE else self.AT_HANDLIST
    self._write(f"AT+{model_list}\r\n")
    if self._wait(self.CMD_TYPE_RESPONSE, model_list) == self.CODE_OK:
      return list(self._learn_list)
    return []

  def set_keypoint_output(self, enable: bool):
    """
    @fn    set_keypoint_output
    @brief Configure whether INVOKE output includes keypoints.
    @param enable: True include keypoints, False boxes only.
    @return CODE_OK: Success, CODE_TIMEOUT: Timeout.
    """
    self._write(f"AT+{self.AT_TKPTS}={1 if enable else 0}\r\n")
    if self._wait(self.CMD_TYPE_RESPONSE, self.AT_TKPTS) == self.CODE_OK:
      return self.CODE_OK
    return self.CODE_TIMEOUT

  def get_keypoint_output(self):
    """
    @fn    get_keypoint_output
    @brief Get whether INVOKE output includes keypoints.
    @return 1/0 on success, None on timeout.
    """
    self._write(f"AT+{self.AT_TKPTS}?\r\n")
    if self._wait(self.CMD_TYPE_RESPONSE, self.AT_TKPTS) == self.CODE_OK:
      return 1 if self._ret_data else 0
    return None

  def available_result(self):
    """
    @fn    available_result
    @brief Check if there is at least one unread detection result.
    @return True: At least one result available, False: No result.
    """
    for res in self._results:
      if not res.used:
        return True
    return False

  def pop_result(self):
    """
    @fn    pop_result
    @brief Pop one unread result (PoseResult or HandResult depending on model); marks it as used.
    @return One Result instance, or None if no unread result.
    """
    for res in self._results:
      if not res.used:
        res.used = True
        return res
    return None

  @abstractmethod
  def _write(self, data: BytesLike) -> bool:
    """Transport write. Implemented by I2C/UART."""
    raise NotImplementedError

  @abstractmethod
  def _available(self) -> int:
    """How many bytes can be read right now."""
    raise NotImplementedError

  @abstractmethod
  def _read(self, length: int) -> ByteList:
    """Transport read exactly length bytes (best-effort)."""
    raise NotImplementedError


class DFRobot_HumanPose_I2C(DFRobot_HumanPose):
  """
  @brief Human Pose sensor driver over I2C. Default I2C address 0x3A.
  """

  def __init__(self, board=None, bus_num=0):
    """
    @fn    __init__
    @brief Initialize I2C communication.
    @param board: Board instance (optional); uses gboard if None.
    @param bus_num: I2C bus number, e.g. 0 or 1 on Raspberry Pi.
    """
    if isinstance(board, int) or board is None:
      board = gboard
    self.board = board
    self.i2c = I2C(bus_num)
    self.i2c_addr = self.I2C_ADDRESS
    super().__init__()

  def _write(self, data: BytesLike) -> bool:
    if isinstance(data, str):
      data = data.encode("ascii")
    if isinstance(data, (bytes, bytearray)):
      data = list(data)
    length = len(data)
    logging.debug(length)
    packets = length // self.MAX_PL_LEN
    remain = length % self.MAX_PL_LEN
    try:
      for i in range(packets):
        start = i * self.MAX_PL_LEN
        end = start + self.MAX_PL_LEN
        buf = bytearray([0x10, 0x02, self.MAX_PL_LEN >> 8, self.MAX_PL_LEN & 0xFF])
        buf.extend(data[start:end])
        self.i2c.writeto(self.i2c_addr, buf)
      if remain:
        start = packets * self.MAX_PL_LEN
        end = start + remain
        buf = bytearray([0x10, 0x02, remain >> 8, remain & 0xFF])
        buf.extend(data[start:end])
        self.i2c.writeto(self.i2c_addr, buf)
      return True
    except Exception as e:
      logging.debug("write error: %s", e)
      return False

  def _read(self, length: int) -> List[int]:
    packets = length // self.MAX_PL_LEN
    remain = length % self.MAX_PL_LEN
    ret_data = []
    try:
      for _ in range(packets):
        self.i2c.writeto(self.i2c_addr, bytes([0x10, 0x01, self.MAX_PL_LEN >> 8, self.MAX_PL_LEN & 0xFF]))
        time.sleep(0.01)
        ret_data.extend(self.i2c.readfrom(self.i2c_addr, self.MAX_PL_LEN))
      if remain:
        self.i2c.writeto(self.i2c_addr, bytes([0x10, 0x01, remain >> 8, remain & 0xFF]))
        time.sleep(0.01)
        ret_data.extend(self.i2c.readfrom(self.i2c_addr, remain))
      return ret_data
    except Exception as e:
      logging.debug("read error: %s", e)
      return []

  def _available(self) -> int:
    try:
      self.i2c.writeto(self.i2c_addr, self._cmd_available)
      len_data = self.i2c.readfrom(self.i2c_addr, 2)
      return (len_data[0] << 8) | len_data[1]
    except Exception as e:
      logging.debug("available error: %s", e)
      return 0


class DFRobot_HumanPose_UART(DFRobot_HumanPose):
  """
  @brief Human Pose sensor driver over UART. Baud rate can be set via BAUD_* constants or integer.
  @n     Baud rate constants: BAUD_9600, BAUD_14400, BAUD_19200, BAUD_38400, BAUD_57600,
  @n     BAUD_115200, BAUD_230400, BAUD_460800, BAUD_921600.
  """

  BAUD_9600 = 9600
  BAUD_14400 = 14400
  BAUD_19200 = 19200
  BAUD_38400 = 38400
  BAUD_57600 = 57600
  BAUD_115200 = 115200
  BAUD_230400 = 230400
  BAUD_460800 = 460800
  BAUD_921600 = 921600

  BAUD_OPTIONS = (BAUD_9600, BAUD_14400, BAUD_19200, BAUD_38400, BAUD_57600, BAUD_115200, BAUD_230400, BAUD_460800, BAUD_921600)

  def __init__(self, board=None, tty_name="/dev/ttyS0", baudrate=921600):
    """
    @fn    __init__
    @brief Initialize UART communication.
    @param board: Board instance or tty path string (optional).
    @param tty_name: Serial port name, e.g. "/dev/ttyS0" or "/dev/ttyAMA0" on Raspberry Pi.
    @param baudrate: Baud rate (use BAUD_* constants or integer); default 921600.
    """
    if isinstance(board, str):
      tty_name = board
      board = gboard
    elif board is None:
      board = gboard

    self.uart = UART(tty_name=tty_name)
    self.uart.init(baud_rate=baudrate, bits=8, parity=0, stop=1)
    super().__init__()

  def _write(self, data: BytesLike) -> bool:
    if isinstance(data, str):
      data = data.encode("ascii")
    if isinstance(data, (bytes, bytearray)):
      data = list(data)
    length = len(data)
    logging.debug(length)
    try:
      self.uart.write(data)
      return True
    except Exception as e:
      logging.debug("uart error: %s", e)
      return False

  def _read(self, length: int) -> List[int]:
    try:
      ret_data = self.uart.read(length)
      return ret_data
    except Exception as e:
      logging.debug("uart _read error: %s", e)
      return []

  def _available(self) -> int:
    try:
      len_data = self.uart.any()
      logging.debug(len_data)
      return len_data
    except Exception as e:
      logging.debug("_available error: %s", e)
      return 0

  def set_baud(self, baudrate: int) -> bool:
    """
    @fn    set_baud
    @brief Set the UART baud rate of the sensor.
    @param baudrate: Baud rate (e.g. BAUD_9600, BAUD_115200, or 9600, 115200, etc.).
    @n     Supported: 9600, 14400, 19200, 38400, 57600, 115200, 230400, 460800, 921600.
    @n     Note: After setting, re-initialize the serial port and sensor with the new baud rate.
    @return True: Set baud rate succeeded, False: Set baud rate failed.
    """
    self._write(f"AT+{self.AT_BAUD}={baudrate}\r\n")
    if self._wait(self.CMD_TYPE_RESPONSE, self.AT_BAUD) == self.CODE_OK:
      return True
    return False
