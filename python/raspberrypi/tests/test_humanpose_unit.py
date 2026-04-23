"""
Unit tests for DFRobot_HumanPose.py (no real sensor; mock transport + binary frame injection).

Run from repository:
  cd python/raspberrypi
  python -m unittest discover -s tests -p "test_*.py" -v

Or:
  python tests/test_humanpose_unit.py
"""

from __future__ import annotations

import os
import sys
import types
import unittest

# Allow "python tests/test_humanpose_unit.py" from raspberrypi/
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _ROOT not in sys.path:
  sys.path.insert(0, _ROOT)


def _install_pinpong_stubs() -> None:
  """DFRobot_HumanPose imports pinpong at module load; stub for CI / dev PCs without PinPong."""
  if "pinpong.board" in sys.modules:
    return
  board = types.ModuleType("pinpong.board")

  class _Gboard:
    pass

  board.gboard = _Gboard()

  class I2C:  # noqa: D401
    def __init__(self, *_a, **_k):
      pass

  class UART:  # noqa: D401
    def __init__(self, *_a, **_k):
      pass

    def init(self, **_k):
      pass

    def write(self, *_a, **_k):
      pass

    def read(self, *_a, **_k):
      return []

    def any(self):
      return 0

  board.I2C = I2C
  board.UART = UART
  pkg = types.ModuleType("pinpong")
  pkg.board = board
  sys.modules["pinpong"] = pkg
  sys.modules["pinpong.board"] = board


_install_pinpong_stubs()

from DFRobot_HumanPose import (  # noqa: E402
  DFRobot_HumanPose,
  PointU16,
  PoseResult,
  Result,
  read_point_u16,
)


def build_hp_binary_frame(hp: DFRobot_HumanPose, msg_type: int, flags: int, payload: bytes) -> bytes:
  """
  Build one 0x55AA binary frame with correct CRC (uses driver static CRC).
  Header: [0]=0x55 [1]=0xAA [2]=0 [3]=msg [4]=flags [5]=0 [6]=0 [7:8]=payload_len LE [9:10]=crc LE
  """
  plen = len(payload)
  hdr = bytearray(11)
  hdr[0] = 0x55
  hdr[1] = 0xAA
  hdr[2] = 0
  hdr[3] = msg_type & 0xFF
  hdr[4] = flags & 0xFF
  hdr[5] = 0
  hdr[6] = 0
  hdr[7] = plen & 0xFF
  hdr[8] = (plen >> 8) & 0xFF
  crc = hp._crc16_ccitt_update(0xFFFF, bytes(hdr[:9]))
  crc = hp._crc16_ccitt_update(crc, payload)
  hdr[9] = crc & 0xFF
  hdr[10] = (crc >> 8) & 0xFF
  return bytes(hdr) + payload


def encode_bin_string(s: str) -> bytes:
  """One HP_BIN_STRING value (8-byte header + UTF-8 body; matches _parse_bin_value layout)."""
  body = s.encode("utf-8")
  out = bytearray()
  out.append(0x05)  # HP_BIN_STRING
  out.append(0)
  out.extend(b"\x00\x00")  # reserved (bytes 2–3); length at offset 4
  out.extend(len(body).to_bytes(4, "little"))
  out.extend(body)
  return bytes(out)


def encode_bin_u64_u8(val: int) -> bytes:
  """One HP_BIN_U64 value holding a small integer (used for TSCORE etc.)."""
  out = bytearray()
  out.append(0x03)  # HP_BIN_U64
  out.append(0)
  out.extend(b"\x00\x00")
  out.extend((8).to_bytes(4, "little"))
  out.extend(int(val).to_bytes(8, "little", signed=False))
  return bytes(out)


def build_at_rsp_assembled(
  *,
  rsp_type: int = 0,
  rsp_code: int = 0,
  rsp_cmd_id: int,
  data_body: bytes,
) -> bytes:
  """
  Assembled AT binary response body (what _process_binary_at_response parses when flags&1).
  Layout matches Python driver: p[0] format, p[1] type, p[2:4] code, p[4:6] cmd_id, p[6:8] pad, p[8:12] data_len.
  """
  inner = bytearray()
  inner.append(1)  # format_ver
  inner.append(rsp_type & 0xFF)
  inner.extend(int(rsp_code).to_bytes(2, "little", signed=True))
  inner.extend(int(rsp_cmd_id).to_bytes(2, "little"))
  inner.extend(b"\x00\x00")
  inner.extend(len(data_body).to_bytes(4, "little"))
  inner.extend(data_body)
  return bytes(inner)


class MockHumanPose(DFRobot_HumanPose):
  """Captures writes and returns scripted binary RX for _wait()."""

  def __init__(self):
    super().__init__()
    self.writes: list[bytes] = []
    self._rx_queue: bytearray = bytearray()

  def inject(self, data: bytes) -> None:
    self._rx_queue.extend(data)

  def _write(self, data) -> bool:
    if isinstance(data, str):
      data = data.encode("ascii")
    if isinstance(data, (bytes, bytearray)):
      self.writes.append(bytes(data))
    else:
      self.writes.append(bytes(data))
    return True

  def _available(self) -> int:
    return len(self._rx_queue)

  def _read(self, length: int) -> list:
    n = min(length, len(self._rx_queue))
    out = list(self._rx_queue[:n])
    del self._rx_queue[:n]
    return out


class TestDataHelpers(unittest.TestCase):
  def test_read_point_u16(self):
    pts = [[10, 20], [0, 0]]
    p = read_point_u16(pts, 0)
    self.assertIsNotNone(p)
    self.assertEqual(p.x, 10)
    self.assertEqual(p.y, 20)
    self.assertIsNone(read_point_u16(pts, 9))
    self.assertIsNone(read_point_u16("bad", 0))

  def test_result_from_json(self):
    names = [{"name": "a"}, {"name": "b"}]
    data = [[5, 6, 7, 8, 90, 2], []]
    r = Result.from_json(data, names)
    self.assertEqual(r.xLeft, 5)
    self.assertEqual(r.id, 2)
    self.assertEqual(r.name, "b")

  def test_pose_result_from_json(self):
    names = []
    pts = [[0, 0]] * 17
    pts[0] = [11, 22]
    data = [[0, 0, 10, 10, 50, 0], pts]
    pr = PoseResult.from_json(data, names)
    self.assertEqual(pr.nose.x, 11)
    self.assertEqual(pr.nose.y, 22)


class TestValidators(unittest.TestCase):
  def test_invalid_percent(self):
    m = MockHumanPose()
    self.assertEqual(m.set_confidence(-1), m.CODE_INVAL)
    self.assertEqual(m.set_confidence(101), m.CODE_INVAL)
    self.assertEqual(m.set_iou(200), m.CODE_INVAL)
    self.assertEqual(m.set_learn_similarity(101), m.CODE_INVAL)

  def test_invalid_model(self):
    m = MockHumanPose()
    self.assertEqual(m.set_model_type(2), m.CODE_INVAL)
    self.assertEqual(m.set_model_type(99), m.CODE_INVAL)


class TestCommandMatches(unittest.TestCase):
  def test_cmd_id_invoke(self):
    m = MockHumanPose()
    self.assertTrue(m._command_matches(m.CMD_ID_INVOKE, m.AT_INVOKE))

  def test_cmd_id_name(self):
    m = MockHumanPose()
    self.assertTrue(m._command_matches(m.CMD_ID_NAME, m.AT_NAME))

  def test_cmd_id_fallback_name(self):
    m = MockHumanPose()
    m._at_rsp_name = "TKPTS"
    self.assertTrue(m._command_matches(0, "TKPTS"))


class TestBinaryFrames(unittest.TestCase):
  def test_crc_frame_parse_at_rsp_sets_name(self):
    m = MockHumanPose()
    inner = build_at_rsp_assembled(
      rsp_type=m.CMD_TYPE_RESPONSE,
      rsp_code=0,
      rsp_cmd_id=m.CMD_ID_NAME,
      data_body=encode_bin_string("DFRobot Human Pose v1"),
    )
    frame = build_hp_binary_frame(m, m.HP_BIN_MSG_AT_RSP, 0x01, inner)
    m._rx_buf.extend(frame)
    self.assertTrue(m._process_binary_frames())
    self.assertTrue(m._at_rsp_ready)
    self.assertIn("DFRobot Human Pose", m._name)

  def test_invoke_end_finalize_pose_result(self):
    m = MockHumanPose()
    m._current_model = m.MODEL_POSE
    # One detection slot
    m._bin_results[0]["used"] = True
    m._bin_results[0]["is_pose"] = True
    m._bin_results[0]["x"] = 10
    m._bin_results[0]["y"] = 20
    m._bin_results[0]["w"] = 100
    m._bin_results[0]["h"] = 200
    m._bin_results[0]["score"] = 88
    m._bin_results[0]["target"] = 0
    m._bin_results[0]["points"][0] = PointU16(50, 60)
    m._bin_result_count = 1

    end_payload = int(0).to_bytes(2, "little", signed=True)
    frame = build_hp_binary_frame(m, m.HP_BIN_MSG_INVOKE_END, 0, end_payload)
    m._rx_buf.extend(frame)
    self.assertTrue(m._process_binary_frames())
    self.assertEqual(len(m._results), 1)
    self.assertIsInstance(m._results[0], PoseResult)
    self.assertEqual(m._results[0].nose.x, 50)


class AutoTscoreMock(MockHumanPose):
  """On AT+TSCORE=..., inject a matching binary AT response so _wait() succeeds."""

  def _write(self, data) -> bool:
    if isinstance(data, str):
      b = data.encode("ascii")
    elif isinstance(data, (bytes, bytearray)):
      b = bytes(data)
    else:
      b = bytes(data)
    self.writes.append(b)
    if b"TSCORE=" in b:
      body = encode_bin_u64_u8(60)
      inner = build_at_rsp_assembled(
        rsp_type=self.CMD_TYPE_RESPONSE,
        rsp_code=0,
        rsp_cmd_id=self.CMD_ID_TSCORE_SET,
        data_body=body,
      )
      self.inject(build_hp_binary_frame(self, self.HP_BIN_MSG_AT_RSP, 0x01, inner))
    return True


class TestWaitWithMock(unittest.TestCase):
  def test_set_confidence_ok_path(self):
    m = AutoTscoreMock()
    self.assertEqual(m.set_confidence(60), m.CODE_OK)

  def test_get_learn_list_ges_empty(self):
    m = MockHumanPose()
    self.assertEqual(m.get_learn_list(m.MODEL_GES), [])


def main():
  loader = unittest.TestLoader()
  suite = loader.loadTestsFromModule(sys.modules[__name__])
  runner = unittest.TextTestRunner(verbosity=2)
  result = runner.run(suite)
  raise SystemExit(0 if result.wasSuccessful() else 1)


if __name__ == "__main__":
  main()
