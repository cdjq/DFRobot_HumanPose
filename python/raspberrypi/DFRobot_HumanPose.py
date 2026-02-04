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
import json
import logging
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any, Union
from __future__ import annotations
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
  HUMANPOSE_NAME = "DFRobot Human Pose"

  TRANSPORT = 0x10
  TRANSPORT_CMD_READ = 0x01
  TRANSPORT_CMD_WRITE = 0x02
  TRANSPORT_CMD_AVAILABLE = 0x03
  TRANSPORT_CMD_RESET = 0x06

  CMD_TYPE_RESPONSE = 0
  CMD_TYPE_EVENT = 1
  CMD_TYPE_LOG = 2

  AT_NAME = "NAME?"
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

  CODE_OK = 0
  CODE_AGAIN = 1
  CODE_LGO = 2
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
    _wait_delay = 2
    self._rx_buf = bytearray()
    self._results = []
    self._name = ""
    self._ret_data = None

    # super.__init__()

  def _get_name(self) -> int:
    self._write(f"AT+{self.AT_NAME}\r\n")
    self._name = ""
    if self._wait(self.CMD_TYPE_RESPONSE, self.AT_NAME) == self.CODE_OK:
      return self.CODE_OK
    return self.CODE_TIMEOUT

  def begin(self):
    """
    @fn    begin
    @brief Initialize the sensor and verify device name.
    @return True: Initialization succeeded, False: Initialization failed.
    """
    if (self._get_name() != self.CODE_OK) or (self._name != self.HUMANPOSE_NAME):
      return False
    return True

  def _wait(self, expected_type: int, cmd: str, timeout_ms=3000) -> int:
    start = time.monotonic()
    dec = json.JSONDecoder()

    while (time.monotonic() - start) * 1000 <= timeout_ms:
      n = self._available()
      if n <= 0:
        time.sleep(0.001)
        continue

      chunk = self._read(n)
      if not chunk:
        continue

      # _read returns list[int]; normalize to bytes for _rx_buf.extend
      data = bytes(chunk)
      self._rx_buf.extend(data)

      # Decode buffer to string (ignore avoids partial utf-8)
      text = self._rx_buf.decode("utf-8", errors="ignore")

      # Parse multiple JSON objects from text (allows leading non-JSON bytes)
      idx = 0
      consumed_upto = 0

      while idx < len(text):
        # Find next JSON start
        p1 = text.find("{", idx)
        p2 = text.find("[", idx)
        if p1 == -1 and p2 == -1:
          break
        start_pos = p1 if p2 == -1 else (p2 if p1 == -1 else min(p1, p2))

        try:
          obj, end = dec.raw_decode(text, start_pos)
        except json.JSONDecodeError:
          # JSON may be incomplete, wait for more data
          break

        # Parsed one complete JSON, record consumed position
        consumed_upto = end
        idx = end

        if not isinstance(obj, dict):
          continue

        r_type = obj.get("type")
        r_name = obj.get("name") or ""
        r_code = obj.get("code", 0)

        if r_type == self.CMD_TYPE_EVENT:
          self._parser_event(obj)

        if r_type == expected_type and r_name:
          if r_name == cmd or r_name.startswith(cmd):
            self._parser_response(obj)
            # Consume processed bytes (simple: clear buffer)
            self._rx_buf.clear()
            return int(r_code)

      # Remove parsed portion from rx_buf
      # Simple strategy: clear decoded buffer if at least one JSON was parsed
      # Precise byte alignment is trickier; for ASCII/JSON serial data this is usually stable enough
      if consumed_upto > 0:
        # Re-encode to get consumed byte length (safe for ASCII/UTF-8)
        consumed_bytes = len(text[:consumed_upto].encode("utf-8", errors="ignore"))
        del self._rx_buf[:consumed_bytes]

    return self.CODE_TIMEOUT

  def _parse_json_objects_from_text(self, text: str):
    """Parse multiple JSON objects from text (allows leading non-JSON bytes)."""
    dec = json.JSONDecoder()
    idx = 0
    n = len(text)

    while idx < n:
      # Find next JSON start symbol
      p1 = text.find("{", idx)
      p2 = text.find("[", idx)
      if p1 == -1 and p2 == -1:
        break
      start = p1 if p2 == -1 else (p2 if p1 == -1 else min(p1, p2))
      idx = start

      try:
        obj, end = dec.raw_decode(text, idx)
        yield obj
        idx = end
      except json.JSONDecodeError:
        # May be partial/garbage, advance and continue
        idx += 1

  def _parser_event(self, obj: dict):
    """Handle INVOKE event: parse pose_keypoints or hand_keypoints into self._results."""
    event_name = obj.get("name", "")
    if not event_name or "INVOKE" not in event_name:
      return

    data = obj.get("data") or {}
    # Compat: if data is not dict return
    if not isinstance(data, dict):
      return

    # ---- pose_keypoints ----
    if isinstance(data.get("pose_keypoints"), list):
      keypoints = data["pose_keypoints"]
      classes = ((data.get("pose_class") or {}).get("available_classes")) or []
      self._results = [PoseResult.from_json(kp, classes) for kp in keypoints]
      logging.debug("Parsed %d pose results", len(self._results))
      return

    # ---- hand_keypoints ----
    if isinstance(data.get("hand_keypoints"), list):
      keypoints = data["hand_keypoints"]
      classes = ((data.get("hand_class") or {}).get("available_classes")) or []
      self._results = [HandResult.from_json(kp, classes) for kp in keypoints]
      logging.debug("Parsed %d hand results", len(self._results))
      return

    logging.debug(f"EVENT: {json.dumps(obj, ensure_ascii=False)}")

  def _parser_response(self, obj: dict):
    """
    Handle response frames and update internal cached values.

    Mirrors the C++ driver behaviour:
    - NAME      -> update self._name
    - TSCORE    -> numeric config value in self._ret_data
    - TIOU      -> numeric config value in self._ret_data
    - TSIMILARITY -> numeric config value in self._ret_data
    - MODEL     -> current model id in self._ret_data
    - POSELIST/HANDLIST -> learned target list in self._ret_data
    """
    r_name = obj.get("name", "") or ""
    data = obj.get("data")

    # Device name
    if r_name == self.AT_NAME:
      # Expect a simple string for name
      self._name = str(data) if data is not None else ""
      logging.debug("Device name: %s", self._name)

    # Learned list (pose/hand)
    elif self.AT_POSELIST in r_name or self.AT_HANDLIST in r_name:
      # C++ stores into an internal LearnList; Python returns a plain list
      if isinstance(data, list):
        self._ret_data = data
      else:
        # Fallback: wrap single value
        self._ret_data = [data] if data is not None else []
      logging.debug("Learn list size: %d", len(self._ret_data))

    # Threshold-like numeric configs (TSCORE/TIOU/TSIMILARITY)
    elif self.AT_TSCORE in r_name or self.AT_TIOU in r_name or self.AT_TSIMILARITY in r_name:
      try:
        self._ret_data = int(data)
      except (TypeError, ValueError):
        self._ret_data = None
      logging.debug("Config value (%s): %s", r_name, self._ret_data)

    # Model info: {"id": <int>, ...}
    elif self.AT_MODEL in r_name and isinstance(data, dict):
      try:
        self._ret_data = int(data.get("id", 0))
      except (TypeError, ValueError):
        self._ret_data = None
      logging.debug("Model id: %s", self._ret_data)

    logging.debug(f"RESPONSE: {json.dumps(obj, ensure_ascii=False)}")

  def get_result(self):
    """
    @fn    get_result
    @brief Trigger one detection and wait for INVOKE event; results are stored and read via available_result/pop_result.
    @return CODE_OK: Success, CODE_TIMEOUT: Timeout.
    """
    self._write(f"AT+{self.AT_INVOKE}=1,0,1\r\n")
    if self._wait(self.CMD_TYPE_EVENT, self.AT_INVOKE) == self.CODE_OK:
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
    @return List of names on success, None on timeout.
    """
    model_list = self.AT_POSELIST if model == self.MODEL_POSE else self.AT_HANDLIST
    self._write(f"AT+{model_list}\r\n")
    if self._wait(self.CMD_TYPE_RESPONSE, model_list) == self.CODE_OK:
      return self._ret_data
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
