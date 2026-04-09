/*!
 * @file DFRobot_HumanPose.cpp
 * @brief Implementation of class DFRobot_HumanPose (I2C/UART communication and detection logic).
 * @details This module implements the Human Pose sensor driver with binary AT protocol and pose/hand result handling.
 * @copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @License     The MIT License (MIT)
 * @author [thdyyl](yuanlong.yu@dfrobot.com)
 * @version  V1.0
 * @date  2026-02-04
 * @url         https://github.com/DFRobot/DFRobot_HumanPose
 */

#include "DFRobot_HumanPose.h"

// ---------------- Binary protocol helpers ----------------
static const uint8_t HP_BIN_SOF0 = 0x55;
static const uint8_t HP_BIN_SOF1 = 0xAA;
static const uint8_t HP_BIN_MSG_AT_RSP = 0x11;
static const uint8_t HP_BIN_MSG_INVOKE_META = 0x00;
static const uint8_t HP_BIN_MSG_INVOKE_BEGIN = 0x01;
static const uint8_t HP_BIN_MSG_DET_CHUNK = 0x03;
static const uint8_t HP_BIN_MSG_HAND_KP_CHUNK = 0x04;
static const uint8_t HP_BIN_MSG_POSE_KP_CHUNK = 0x05;
static const uint8_t HP_BIN_MSG_INVOKE_END = 0x07;
static const uint16_t HP_BIN_HDR_LEN = 11;

static const uint16_t CMD_ID_NAME = 0x0003;
static const uint16_t CMD_ID_INVOKE = 0x0017;
static const uint16_t CMD_ID_MODEL_SET = 0x000B;
static const uint16_t CMD_ID_MODEL_GET = 0x000C;
static const uint16_t CMD_ID_POSELIST = 0x0029;
static const uint16_t CMD_ID_HANDLIST = 0x0030;
static const uint16_t CMD_ID_TPROTO_SET = 0x001F;
static const uint16_t CMD_ID_TPROTO_GET = 0x0020;
static const uint16_t CMD_ID_TPKTSZ_SET = 0x0021;
static const uint16_t CMD_ID_TPKTSZ_GET = 0x0022;
static const uint16_t CMD_ID_TKPTS_SET = 0x0037;
static const uint16_t CMD_ID_TKPTS_GET = 0x0038;
static const uint16_t CMD_ID_TSCORE_SET = 0x0101;
static const uint16_t CMD_ID_TSCORE_GET = 0x0102;
static const uint16_t CMD_ID_TIOU_SET = 0x0103;
static const uint16_t CMD_ID_TIOU_GET = 0x0104;
static const uint16_t CMD_ID_TSIM_SET = 0x0105;
static const uint16_t CMD_ID_TSIM_GET = 0x0106;
static const uint16_t CMD_ID_BAUD_SET = 0x001D;
static const uint16_t CMD_ID_BAUD_GET = 0x001E;

/** Fixed GES class names (class id 0..14). Same mapping as firmware / Himax host tools. */
static const char *const GES_CLASS_NAMES[] = {
  "zero",    "one",     "two",     "three",   "four",
  "five",    "six",     "dislike", "like",    "ok",
  "stop",    "rock",    "three2",  "two_up",  "no_gesture",
};
static const size_t GES_CLASS_COUNT = sizeof(GES_CLASS_NAMES) / sizeof(GES_CLASS_NAMES[0]);

enum HumanPoseBinType {
  HP_BIN_NULL = 0x00,
  HP_BIN_BOOL = 0x01,
  HP_BIN_I64 = 0x02,
  HP_BIN_U64 = 0x03,
  HP_BIN_F64 = 0x04,
  HP_BIN_STRING = 0x05,
  HP_BIN_OBJECT = 0x06,
  HP_BIN_ARRAY = 0x07,
  HP_BIN_BYTES = 0x08,
};

typedef struct {
  uint8_t type;
  uint8_t flags;
  const uint8_t *body;
  uint32_t length;
} bin_value_view_t;

static uint16_t hp_read_u16_le(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t hp_read_i16_le(const uint8_t *p)
{
  return (int16_t)hp_read_u16_le(p);
}

static uint32_t hp_read_u32_le(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t hp_crc16_ccitt_update(uint16_t crc, const uint8_t *data, size_t len)
{
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

static bool hp_parse_bin_value(const uint8_t *data, size_t len, size_t &offset, bin_value_view_t &view)
{
  if (!data || offset + 8 > len) {
    return false;
  }
  const uint8_t *hdr = data + offset;
  view.type = hdr[0];
  view.flags = hdr[1];
  view.length = hp_read_u32_le(hdr + 4);
  offset += 8;
  if (offset + view.length > len) {
    return false;
  }
  view.body = data + offset;
  offset += view.length;
  return true;
}

static bool hp_bin_to_uint8(const bin_value_view_t &v, uint8_t &out)
{
  if (v.type == HP_BIN_U64 && v.length >= 8) {
    uint64_t tmp = 0;
    memcpy(&tmp, v.body, sizeof(tmp));
    out = (uint8_t)((tmp > 255) ? 255 : tmp);
    return true;
  }
  if (v.type == HP_BIN_I64 && v.length >= 8) {
    int64_t tmp = 0;
    memcpy(&tmp, v.body, sizeof(tmp));
    if (tmp < 0)
      tmp = 0;
    if (tmp > 255)
      tmp = 255;
    out = (uint8_t)tmp;
    return true;
  }
  if (v.type == HP_BIN_BOOL) {
    out = v.flags ? 1 : 0;
    return true;
  }
  return false;
}

static bool hp_bin_to_string(const bin_value_view_t &v, String &out)
{
  if (v.type != HP_BIN_STRING) {
    return false;
  }
  out = "";
  for (uint32_t i = 0; i < v.length; ++i) {
    out += (char)v.body[i];
  }
  return true;
}

static bool hp_bin_object_get(const bin_value_view_t &obj, const char *key, bin_value_view_t &out)
{
  if (obj.type != HP_BIN_OBJECT || !obj.body || obj.length < 2 || !key) {
    return false;
  }

  const uint8_t *body = obj.body;
  const size_t body_len = (size_t)obj.length;
  size_t cursor = 2;
  uint16_t count = hp_read_u16_le(body);

  for (uint16_t i = 0; i < count; ++i) {
    if (cursor + 2 > body_len) {
      return false;
    }
    const uint16_t key_len = hp_read_u16_le(body + cursor);
    cursor += 2;
    if (cursor + key_len > body_len) {
      return false;
    }

    bool key_match = false;
    if (strlen(key) == key_len) {
      key_match = (memcmp(body + cursor, key, key_len) == 0);
    }
    cursor += key_len;

    size_t value_off = cursor;
    if (!hp_parse_bin_value(body, body_len, value_off, out)) {
      return false;
    }
    cursor = value_off;
    if (key_match) {
      return true;
    }
  }

  return false;
}

static bool hp_bin_array_to_string_list(const bin_value_view_t &arr, LearnList &out)
{
  out.clear();
  if (arr.type != HP_BIN_ARRAY || !arr.body || arr.length < 2) {
    return false;
  }

  const uint8_t *body = arr.body;
  const size_t body_len = (size_t)arr.length;
  uint16_t count = hp_read_u16_le(body);
  size_t cursor = 2;

  for (uint16_t i = 0; i < count; ++i) {
    bin_value_view_t v = { 0, 0, nullptr, 0 };
    if (!hp_parse_bin_value(body, body_len, cursor, v)) {
      return false;
    }
    if (v.type == HP_BIN_STRING) {
      String item = "";
      for (uint32_t k = 0; k < v.length; ++k) {
        item += (char)v.body[k];
      }
      out.push_back(item);
    }
  }
  return true;
}

// ============ Base Class: DFRobot_HumanPose ============

/**
 * @fn DFRobot_HumanPose::DFRobot_HumanPose
 * @brief Constructor of DFRobot_HumanPose class
 * @details Initializes internal variables and sets default values
 */
DFRobot_HumanPose::DFRobot_HumanPose()
{
  _wait_delay = 2;
  rx_end      = 0;
  tx_len      = 0;
  rx_len      = 0;
  tx_buf      = NULL;
  rx_buf      = NULL;
  _at_payload_buf = NULL;
  _at_payload_len = 0;
  _at_payload_cap = 0;
  _binary_mode = false;
  _current_model = eHand;

  for (uint8_t i = 0; i < MAX_RESULT_NUM; ++i) {
    _result[i] = NULL;
#if !DFR_HUMANPOSE_LOW_MEMORY
    _result_is_pose[i] = false;
    _result_is_ges[i] = false;
#endif
  }
  clear_binary_results();
}

/**
 * @fn DFRobot_HumanPose::~DFRobot_HumanPose
 * @brief Destructor of DFRobot_HumanPose class
 * @details Frees allocated memory for transmit and receive buffers
 */
DFRobot_HumanPose::~DFRobot_HumanPose()
{
  for (uint8_t i = 0; i < MAX_RESULT_NUM; ++i) {
    if (_result[i] != NULL) {
      delete _result[i];
      _result[i] = NULL;
    }
  }
#if DFR_HUMANPOSE_SMALL_RAM_PROFILE
  tx_buf = NULL;
  rx_buf = NULL;
  _at_payload_buf = NULL;
#else
  if (tx_buf) {
    free(tx_buf);
    tx_buf = NULL;
  }
  if (rx_buf) {
    free(rx_buf);
    rx_buf = NULL;
  }
  if (_at_payload_buf) {
    free(_at_payload_buf);
    _at_payload_buf = NULL;
  }
#endif
  _at_payload_cap = 0;
}

/**
 * @fn DFRobot_HumanPose::begin
 * @brief Initialize the sensor
 * @details Allocates memory for transmit and receive buffers, then initializes binary protocol communication
 * @return True if initialization is successful, otherwise false
 */
bool DFRobot_HumanPose::begin()
{
  // Allocate/attach buffers
#if DFR_HUMANPOSE_SMALL_RAM_PROFILE
  rx_buf = _rx_buf_static;
  tx_buf = _tx_buf_static;
  _at_payload_buf = _at_payload_static;
  _at_payload_cap = AT_PAYLOAD_MAX_SIZE;
  rx_len = RX_MAX_SIZE;
  tx_len = TX_MAX_SIZE;
#else
  if (!rx_buf || !tx_buf || !_at_payload_buf) {
    rx_buf = (char *)malloc(RX_MAX_SIZE);
    tx_buf = (char *)malloc(TX_MAX_SIZE);
    _at_payload_buf = (uint8_t *)malloc(AT_PAYLOAD_MAX_SIZE);
    _at_payload_cap = _at_payload_buf ? AT_PAYLOAD_MAX_SIZE : 0;
    rx_len = RX_MAX_SIZE;
    tx_len = TX_MAX_SIZE;
  }
#endif

  if (!rx_buf || !tx_buf || !_at_payload_buf) {
    LDBG("malloc fail");
    return false;
  }

  rx_end = 0;
  _at_payload_len = 0;
  _at_rsp_ready = false;
  _invoke_event_ready = false;
  _binary_mode = false;
  clear_binary_results();

  // Force binary response mode on all platforms.
  char cmd[32] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s=1" CMD_SUF, AT_TPROTO);
  write(cmd, strlen(cmd));
  const int tproto_ret = wait(CMD_TYPE_RESPONSE, AT_TPROTO, 1200);
  if (tproto_ret == eOK) {
    _binary_mode = true;
  }
#if DFR_HUMANPOSE_DEBUG
  Serial.print(F("[HPDBG] begin tproto ret="));
  Serial.print(tproto_ret);
  Serial.print(F(", binary_mode="));
  Serial.println(_binary_mode ? 1 : 0);
#endif
  if (!_binary_mode) {
    LDBG("TPROTO force failed");
    return false;
  }

  // Tune packet size by memory class.
#if DFR_HUMANPOSE_LOW_MEMORY
  const uint16_t packet_size = 64;
#else
  const uint16_t packet_size = 192;
#endif
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s=%u" CMD_SUF, AT_TPKTSZ, (unsigned)packet_size);
  write(cmd, strlen(cmd));
  const int tpktsz_ret = wait(CMD_TYPE_RESPONSE, AT_TPKTSZ, 1200);
#if DFR_HUMANPOSE_DEBUG
  Serial.print(F("[HPDBG] begin tpktsz ret="));
  Serial.println(tpktsz_ret);
#else
  (void)tpktsz_ret;
#endif

  // Runtime output policy:
  // - low-memory boards: boxes only
  // - others: boxes + keypoints
#if DFR_HUMANPOSE_LOW_MEMORY
  const bool keypoints_enable = false;
#else
  const bool keypoints_enable = true;
#endif
  const int tkpts_ret = setKeypointOutput(keypoints_enable);
#if DFR_HUMANPOSE_DEBUG
  Serial.print(F("[HPDBG] begin tkpts ret="));
  Serial.print(tkpts_ret);
  Serial.print(F(", mode="));
  Serial.println(keypoints_enable ? 1 : 0);
#endif
  if (tkpts_ret != eOK) {
    LDBG("TKPTS force failed");
    return false;
  }

  char name[DEVICE_NAME_BUF_SIZE] = { 0 };
  const eCmdCode_t name_ret = getName(name);
#if DFR_HUMANPOSE_DEBUG
  Serial.print(F("[HPDBG] begin getName ret="));
  Serial.print((int)name_ret);
  Serial.print(F(", name='"));
  Serial.print(name);
  Serial.print(F("', expected contains '"));
  Serial.print(F(HUMANPOSE_NAME));
  Serial.println(F("'"));
#endif
  if (name_ret != eOK || !strstr(name, HUMANPOSE_NAME)) {
    LDBG(name);
    return false;
  }

  return true;
}

/**
 * @fn DFRobot_HumanPose::wait
 * @brief Wait for command response from the sensor
 * @details Reads data from the sensor and waits for a response matching the specified command type and name
 * @param type Command type (CMD_TYPE_RESPONSE, CMD_TYPE_EVENT, or CMD_TYPE_LOG)
 * @param cmd Command name string to match
 * @param timeout Timeout value in milliseconds (default is 1000)
 * @return Status code of type `eCmdCode_t`. Returns `eOK` if response received successfully, `eTimedOut` if timeout occurs
 */
int DFRobot_HumanPose::wait(int type, const char *cmd, uint32_t timeout)
{
  unsigned long startTime = millis();
#if DFR_HUMANPOSE_DEBUG
  Serial.print(F("[HPDBG] wait begin type="));
  Serial.print(type);
  Serial.print(F(", cmd="));
  if (cmd) {
    Serial.print(cmd);
  } else {
    Serial.print(F(""));
  }
  Serial.print(F(", timeout="));
  Serial.println(timeout);
#endif

  while (millis() - startTime <= timeout) {
    if (type == CMD_TYPE_RESPONSE && _at_rsp_ready && _at_rsp_type == CMD_TYPE_RESPONSE) {
      if (command_matches(_at_rsp_cmd_id, cmd)) {
        int ret = _at_rsp_code;
        _at_rsp_ready = false;
#if DFR_HUMANPOSE_DEBUG
        Serial.print(F("[HPDBG] wait hit response cmd_id=0x"));
        Serial.print(_at_rsp_cmd_id, HEX);
        Serial.print(F(", code="));
        Serial.println(ret);
#endif
        return ret;
      }

      // Drop stale/unrelated responses so they don't block subsequent valid replies.
#if DFR_HUMANPOSE_DEBUG
      Serial.print(F("[HPDBG] wait drop stale response cmd_id=0x"));
      Serial.print(_at_rsp_cmd_id, HEX);
      Serial.print(F(", expect="));
      if (cmd) {
        Serial.println(cmd);
      } else {
        Serial.println(F(""));
      }
#endif
      _at_rsp_ready = false;
    }
    if (type == CMD_TYPE_EVENT && cmd && strstr(cmd, EVENT_INVOKE) && _invoke_event_ready) {
      int ret = _invoke_event_code;
      _invoke_event_ready = false;
#if DFR_HUMANPOSE_DEBUG
      Serial.print(F("[HPDBG] wait hit event INVOKE code="));
      Serial.println(ret);
#endif
      return ret;
    }

    int len = available();
    if (len <= 0) {
      delay(1);
      continue;
    }

#if DFR_HUMANPOSE_SMALL_RAM_PROFILE
    // Keep small-memory profile reads small so we can parse/drain continuously under burst frames.
    if (len > 64) {
      len = 64;
    }
#endif

    if ((uint32_t)len + rx_end > rx_len) {
      len = (int)(rx_len - rx_end);
      if (len <= 0) {
        rx_end = 0;
        continue;
      }
    }

    int rlen = read(rx_buf + rx_end, len);
    if (rlen <= 0) {
      delay(1);
      continue;
    }
    rx_end += (uint32_t)rlen;
    rx_buf[rx_end] = '\0';

    bool progressed = false;
    do {
      progressed = false;
      if (process_binary_frames()) {
        progressed = true;
      }
    } while (progressed && rx_end > 0);
  }

#if DFR_HUMANPOSE_DEBUG
  Serial.print(F("[HPDBG] wait timeout type="));
  Serial.print(type);
  Serial.print(F(", cmd="));
  if (cmd) {
    Serial.println(cmd);
  } else {
    Serial.println(F(""));
  }
#endif
  return eTimedOut;
}

bool DFRobot_HumanPose::command_matches(uint16_t cmd_id, const char *cmd) const
{
  if (!cmd) {
    return false;
  }

  if (cmd_id != 0) {
    if (strstr(cmd, AT_INVOKE)) {
      return (cmd_id == CMD_ID_INVOKE);
    }
    if (strstr(cmd, AT_NAME)) {
      return (cmd_id == CMD_ID_NAME);
    }
    if (strstr(cmd, AT_MODEL)) {
      return (cmd_id == CMD_ID_MODEL_SET || cmd_id == CMD_ID_MODEL_GET);
    }
    if (strstr(cmd, AT_TSCORE)) {
      return (cmd_id == CMD_ID_TSCORE_SET || cmd_id == CMD_ID_TSCORE_GET);
    }
    if (strstr(cmd, AT_TIOU)) {
      return (cmd_id == CMD_ID_TIOU_SET || cmd_id == CMD_ID_TIOU_GET);
    }
    if (strstr(cmd, AT_TSIMILARITY)) {
      return (cmd_id == CMD_ID_TSIM_SET || cmd_id == CMD_ID_TSIM_GET);
    }
    if (strstr(cmd, AT_POSELIST)) {
      return (cmd_id == CMD_ID_POSELIST);
    }
    if (strstr(cmd, AT_HANDLIST)) {
      return (cmd_id == CMD_ID_HANDLIST);
    }
    if (strstr(cmd, AT_BAUD)) {
      return (cmd_id == CMD_ID_BAUD_SET || cmd_id == CMD_ID_BAUD_GET);
    }
    if (strstr(cmd, AT_TPROTO)) {
      return (cmd_id == CMD_ID_TPROTO_SET || cmd_id == CMD_ID_TPROTO_GET);
    }
    if (strstr(cmd, AT_TPKTSZ)) {
      return (cmd_id == CMD_ID_TPKTSZ_SET || cmd_id == CMD_ID_TPKTSZ_GET);
    }
    if (strstr(cmd, AT_TKPTS)) {
      return (cmd_id == CMD_ID_TKPTS_SET || cmd_id == CMD_ID_TKPTS_GET);
    }
    return false;
  }

  if (_at_rsp_name[0] == '\0') {
    return false;
  }
  size_t cmd_len = strlen(cmd);
  size_t rsp_len = strlen(_at_rsp_name);
  if (cmd_len == rsp_len && strcmp(_at_rsp_name, cmd) == 0) {
    return true;
  }
  return (cmd_len > 0 && rsp_len > 0 && strncmp(_at_rsp_name, cmd, cmd_len) == 0);
}

bool DFRobot_HumanPose::process_binary_frames()
{
  if (!rx_buf || rx_end < 2) {
    return false;
  }

  uint32_t sof_pos = 0xFFFFFFFFu;
  for (uint32_t i = 0; i + 1 < rx_end; ++i) {
    if ((uint8_t)rx_buf[i] == HP_BIN_SOF0 && (uint8_t)rx_buf[i + 1] == HP_BIN_SOF1) {
      sof_pos = i;
      break;
    }
  }

  if (sof_pos == 0xFFFFFFFFu) {
    return false;
  }

  if (sof_pos > 0) {
    memmove(rx_buf, rx_buf + sof_pos, rx_end - sof_pos);
    rx_end -= sof_pos;
    rx_buf[rx_end] = '\0';
    return true;
  }

  if (rx_end < HP_BIN_HDR_LEN) {
    return false;
  }

  const uint8_t *frame = (const uint8_t *)rx_buf;
  const uint16_t payload_len = hp_read_u16_le(frame + 7);
  const uint32_t frame_len = (uint32_t)HP_BIN_HDR_LEN + payload_len;
  if (frame_len > rx_len) {
    memmove(rx_buf, rx_buf + 1, rx_end - 1);
    rx_end -= 1;
    rx_buf[rx_end] = '\0';
    return true;
  }
  if (rx_end < frame_len) {
    return false;
  }

  const uint16_t crc_rx = hp_read_u16_le(frame + 9);
  uint16_t crc = hp_crc16_ccitt_update(0xFFFFu, frame, 9);
  crc = hp_crc16_ccitt_update(crc, frame + HP_BIN_HDR_LEN, payload_len);
  if (crc != crc_rx) {
#if DFR_HUMANPOSE_DEBUG
    Serial.print(F("[HPDBG] binary crc mismatch rx=0x"));
    Serial.print(crc_rx, HEX);
    Serial.print(F(", calc=0x"));
    Serial.println(crc, HEX);
#endif
    memmove(rx_buf, rx_buf + 1, rx_end - 1);
    rx_end -= 1;
    rx_buf[rx_end] = '\0';
    return true;
  }

  const uint8_t msg_type = frame[3];
  const uint8_t flags = frame[4];
  const uint8_t *payload_ptr = frame + HP_BIN_HDR_LEN;
#if DFR_HUMANPOSE_DEBUG
  Serial.print(F("[HPDBG] binary frame msg=0x"));
  Serial.print(msg_type, HEX);
  Serial.print(F(", flags=0x"));
  Serial.print(flags, HEX);
  Serial.print(F(", len="));
  Serial.println(payload_len);
#endif

  if (msg_type == HP_BIN_MSG_AT_RSP) {
    process_binary_at_response(flags, payload_ptr, payload_len);
  } else {
    process_binary_invoke(msg_type, flags, payload_ptr, payload_len);
  }

  memmove(rx_buf, rx_buf + frame_len, rx_end - frame_len);
  rx_end -= frame_len;
  rx_buf[rx_end] = '\0';
  return true;
}

bool DFRobot_HumanPose::process_binary_at_response(uint8_t flags, const uint8_t *payload_ptr, uint16_t payload_len)
{
  if (!_at_payload_buf || _at_payload_cap == 0) {
    return false;
  }
  if (_at_payload_len + payload_len > _at_payload_cap) {
    _at_payload_len = 0;
  }
  if (_at_payload_len + payload_len > _at_payload_cap) {
    return false;
  }

  memcpy(_at_payload_buf + _at_payload_len, payload_ptr, payload_len);
  _at_payload_len += payload_len;

  if ((flags & 0x01u) == 0) {
    return true;
  }

  if (_at_payload_len < 12) {
    _at_payload_len = 0;
    return false;
  }

  const uint8_t *p = _at_payload_buf;
  const uint8_t format_ver = p[0];
  (void)format_ver;
  const int8_t rsp_type = (int8_t)p[1];
  const int16_t rsp_code = hp_read_i16_le(p + 2);
  const uint16_t rsp_cmd_id = hp_read_u16_le(p + 4);

  const uint32_t data_len = hp_read_u32_le(p + 8);
#if DFR_HUMANPOSE_DEBUG
  Serial.print(F("[HPDBG] at_rsp fmt="));
  Serial.print(format_ver);
  Serial.print(F(", type="));
  Serial.print((int)rsp_type);
  Serial.print(F(", code="));
  Serial.print((int)rsp_code);
  Serial.print(F(", cmd_id=0x"));
  Serial.print(rsp_cmd_id, HEX);
  Serial.print(F(", data_len="));
  Serial.println((unsigned long)data_len);
#endif
  if (12 + data_len <= _at_payload_len && data_len >= 8) {
    const uint8_t *data_ptr = p + 12;
    size_t off = 0;
    bin_value_view_t root = { 0, 0, nullptr, 0 };
    if (hp_parse_bin_value(data_ptr, data_len, off, root)) {
      if (rsp_cmd_id == CMD_ID_NAME) {
        String v;
        if (hp_bin_to_string(root, v)) {
          snprintf(_name, sizeof(_name), "%s", v.c_str());
        }
      } else if (rsp_cmd_id == CMD_ID_TSCORE_GET || rsp_cmd_id == CMD_ID_TSCORE_SET ||
                 rsp_cmd_id == CMD_ID_TIOU_GET || rsp_cmd_id == CMD_ID_TIOU_SET ||
                 rsp_cmd_id == CMD_ID_TSIM_GET || rsp_cmd_id == CMD_ID_TSIM_SET ||
                 rsp_cmd_id == CMD_ID_TKPTS_GET || rsp_cmd_id == CMD_ID_TKPTS_SET) {
        uint8_t v = 0;
        if (hp_bin_to_uint8(root, v)) {
          _ret_data = v;
        }
      } else if (rsp_cmd_id == CMD_ID_MODEL_GET || rsp_cmd_id == CMD_ID_MODEL_SET) {
        uint8_t v = 0;
        if (root.type == HP_BIN_OBJECT) {
          bin_value_view_t id_value = { 0, 0, nullptr, 0 };
          if (hp_bin_object_get(root, "id", id_value) && hp_bin_to_uint8(id_value, v)) {
            _ret_data = v;
          }
        } else if (hp_bin_to_uint8(root, v)) {
          _ret_data = v;
        }
        if (_ret_data == (uint8_t)eHand || _ret_data == (uint8_t)ePose || _ret_data == (uint8_t)eGes) {
          _current_model = (eModel_t)_ret_data;
        }
      } else if (rsp_cmd_id == CMD_ID_HANDLIST || rsp_cmd_id == CMD_ID_POSELIST) {
        LearnList parsed;
        if (hp_bin_array_to_string_list(root, parsed)) {
#if !DFR_HUMANPOSE_LOW_MEMORY
          _learn_list = parsed;
#endif
          if (rsp_cmd_id == CMD_ID_HANDLIST) {
            _hand_class_list = parsed;
          } else {
            _pose_class_list = parsed;
          }
        }
      }
    }
  }

  /* Always record the latest complete AT response. A stale _at_rsp_ready guard would drop
   * a second frame (e.g. BAUD) if any prior response was not consumed yet — breaks setBaud(). */
  if (rsp_type == CMD_TYPE_RESPONSE) {
    _at_rsp_type = rsp_type;
    _at_rsp_code = rsp_code;
    _at_rsp_cmd_id = rsp_cmd_id;
    _at_rsp_name[0] = '\0';
    _at_rsp_ready = true;
  } else if (rsp_type == CMD_TYPE_EVENT && rsp_cmd_id == CMD_ID_INVOKE) {
    _invoke_event_code = rsp_code;
    _invoke_event_ready = true;
  }
  _at_payload_len = 0;
  return true;
}

void DFRobot_HumanPose::clear_binary_results()
{
  _bin_result_count = 0;
  _invoke_model_id = 0;
  for (uint8_t i = 0; i < MAX_RESULT_NUM; ++i) {
    _bin_results[i].used = false;
#if !DFR_HUMANPOSE_LOW_MEMORY
    _bin_results[i].is_pose = false;
#endif
    _bin_results[i].x = 0;
    _bin_results[i].y = 0;
    _bin_results[i].w = 0;
    _bin_results[i].h = 0;
    _bin_results[i].score = 0;
    _bin_results[i].target = 0;
#if !DFR_HUMANPOSE_LOW_MEMORY
    for (size_t j = 0; j < sizeof(_bin_results[i].points) / sizeof(_bin_results[i].points[0]); ++j) {
      _bin_results[i].points[j].x = 0;
      _bin_results[i].points[j].y = 0;
    }
    memset(_bin_results[i].point_valid, 0, sizeof(_bin_results[i].point_valid));
#endif
  }
}

String DFRobot_HumanPose::resolve_class_name(uint16_t id) const
{
  if (_current_model == eGes) {
    if (id < GES_CLASS_COUNT) {
      return String(GES_CLASS_NAMES[id]);
    }
    String s = "class_";
    s += String((unsigned long)id);
    return s;
  }

  if (id == 0) {
    return "unknown";
  }

  const LearnList *list = (_current_model == ePose) ? &_pose_class_list : &_hand_class_list;
  if (list && id > 0 && id <= list->size()) {
    return (*list)[id - 1];
  }

  String name = "class_";
  name += String(id);
  return name;
}

static void hp_set_result_base(Result *dst, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t score, uint8_t id, const String &name)
{
  if (!dst) {
    return;
  }
  dst->xLeft = x;
  dst->yTop = y;
  dst->width = w;
  dst->height = h;
  dst->score = score;
  dst->id = id;
  dst->name = name;
  dst->used = false;
}

#if !DFR_HUMANPOSE_LOW_MEMORY
static void hp_set_pose_points(PoseResult *dst, const PointU16 *points)
{
  if (!dst || !points) {
    return;
  }
  dst->nose = points[0];
  dst->leye = points[1];
  dst->reye = points[2];
  dst->lear = points[3];
  dst->rear = points[4];
  dst->lshoulder = points[5];
  dst->rshoulder = points[6];
  dst->lelbow = points[7];
  dst->relbow = points[8];
  dst->lwrist = points[9];
  dst->rwrist = points[10];
  dst->lhip = points[11];
  dst->rhip = points[12];
  dst->lknee = points[13];
  dst->rknee = points[14];
  dst->lankle = points[15];
  dst->rankle = points[16];
}

static void hp_set_hand_points(HandResult *dst, const PointU16 *points)
{
  if (!dst || !points) {
    return;
  }
  dst->wrist = points[0];
  dst->thumbCmc = points[1];
  dst->thumbMcp = points[2];
  dst->thumbIp = points[3];
  dst->thumbTip = points[4];
  dst->indexFingerMcp = points[5];
  dst->indexFingerPip = points[6];
  dst->indexFingerDip = points[7];
  dst->indexFingerTip = points[8];
  dst->middleFingerMcp = points[9];
  dst->middleFingerPip = points[10];
  dst->middleFingerDip = points[11];
  dst->middleFingerTip = points[12];
  dst->ringFingerMcp = points[13];
  dst->ringFingerPip = points[14];
  dst->ringFingerDip = points[15];
  dst->ringFingerTip = points[16];
  dst->pinkyFingerMcp = points[17];
  dst->pinkyFingerPip = points[18];
  dst->pinkyFingerDip = points[19];
  dst->pinkyFingerTip = points[20];
}
#endif

void DFRobot_HumanPose::finalize_binary_results()
{
  for (uint8_t i = 0; i < MAX_RESULT_NUM; ++i) {
    if (_result[i]) {
      // Mark all cached objects consumed first; only slots updated by this frame become available.
      _result[i]->used = true;
    }
  }

  const uint8_t count = (uint8_t)((_bin_result_count > MAX_RESULT_NUM) ? MAX_RESULT_NUM : _bin_result_count);
  for (uint8_t i = 0; i < count; ++i) {
    if (!_bin_results[i].used) {
      continue;
    }
#if DFR_HUMANPOSE_LOW_MEMORY
    if (_result[i] == NULL) {
      _result[i] = new Result(0, 0, 0, 0, 0, 0, "unknown");
      if (_result[i] == NULL) {
        continue;
      }
    }

    const uint8_t target8 = (uint8_t)((_bin_results[i].target > 0xFFu) ? 0xFFu : _bin_results[i].target);
    const String name = resolve_class_name(_bin_results[i].target);
    hp_set_result_base(_result[i],
                       _bin_results[i].x,
                       _bin_results[i].y,
                       _bin_results[i].w,
                       _bin_results[i].h,
                       _bin_results[i].score,
                       target8,
                       name);
#else
    const bool use_ges = (_current_model == eGes);
    const bool use_pose = _bin_results[i].is_pose;
    if (use_ges) {
      if (_result[i] == NULL || !_result_is_ges[i]) {
        if (_result[i] != NULL) {
          delete _result[i];
          _result[i] = NULL;
        }
        _result[i] = new Result(0, 0, 0, 0, 0, 0, "unknown");
        if (_result[i] == NULL) {
          continue;
        }
        _result_is_ges[i] = true;
        _result_is_pose[i] = false;
      }
    } else if (_result[i] == NULL || _result_is_pose[i] != use_pose || _result_is_ges[i]) {
      if (_result[i] != NULL) {
        delete _result[i];
        _result[i] = NULL;
      }
      if (use_pose) {
        _result[i] = new PoseResult(0, 0, 0, 0, 0, 0, NULL, 0, "unknown");
      } else {
        _result[i] = new HandResult(0, 0, 0, 0, 0, 0, NULL, 0, "unknown");
      }
      if (_result[i] == NULL) {
        continue;
      }
      _result_is_pose[i] = use_pose;
      _result_is_ges[i] = false;
    }

    const uint8_t target8 = (uint8_t)((_bin_results[i].target > 0xFFu) ? 0xFFu : _bin_results[i].target);
    const String name = resolve_class_name(_bin_results[i].target);
    if (use_ges) {
      hp_set_result_base(_result[i],
                         _bin_results[i].x,
                         _bin_results[i].y,
                         _bin_results[i].w,
                         _bin_results[i].h,
                         _bin_results[i].score,
                         target8,
                         name);
    } else if (use_pose) {
      PoseResult *pose = (PoseResult *)_result[i];
      hp_set_result_base(pose,
                         _bin_results[i].x,
                         _bin_results[i].y,
                         _bin_results[i].w,
                         _bin_results[i].h,
                         _bin_results[i].score,
                         target8,
                         name);
      hp_set_pose_points(pose, _bin_results[i].points);
    } else {
      HandResult *hand = (HandResult *)_result[i];
      hp_set_result_base(hand,
                         _bin_results[i].x,
                         _bin_results[i].y,
                         _bin_results[i].w,
                         _bin_results[i].h,
                         _bin_results[i].score,
                         target8,
                         name);
      hp_set_hand_points(hand, _bin_results[i].points);
    }
#endif
  }
}

bool DFRobot_HumanPose::process_binary_invoke(uint8_t msg_type, uint8_t flags, const uint8_t *payload_ptr, uint16_t payload_len)
{
  (void)flags;
  if (!payload_ptr) {
    return false;
  }

  if (msg_type == HP_BIN_MSG_INVOKE_META) {
    // Binary INVOKE flow uses META as the command-level response equivalent.
    int16_t ret_code = 0;
    if (payload_len >= 2) {
      ret_code = hp_read_i16_le(payload_ptr);
    }
    if (payload_len >= 4) {
      const uint16_t model_id = hp_read_u16_le(payload_ptr + 2);
      if (model_id == (uint16_t)ePose) {
        _current_model = ePose;
      } else if (model_id == (uint16_t)eHand) {
        _current_model = eHand;
      } else if (model_id == (uint16_t)eGes) {
        _current_model = eGes;
      }
    }

    if (!_at_rsp_ready) {
      _at_rsp_type = CMD_TYPE_RESPONSE;
      _at_rsp_cmd_id = CMD_ID_INVOKE;
      _at_rsp_code = ret_code;
      _at_rsp_name[0] = '\0';
      _at_rsp_ready = true;
    }
#if DFR_HUMANPOSE_DEBUG
    Serial.print(F("[HPDBG] invoke meta as response, code="));
    Serial.println((int)ret_code);
#endif
    return true;
  }

  if (msg_type == HP_BIN_MSG_INVOKE_BEGIN) {
    clear_binary_results();
    if (payload_len >= 12) {
      _invoke_model_id = hp_read_u16_le(payload_ptr + 10);
      if (_invoke_model_id == (uint16_t)ePose) {
        _current_model = ePose;
      } else if (_invoke_model_id == (uint16_t)eHand) {
        _current_model = eHand;
      } else if (_invoke_model_id == (uint16_t)eGes) {
        _current_model = eGes;
      }
    }
    return true;
  }

  if (msg_type == HP_BIN_MSG_DET_CHUNK) {
    if (payload_len < 6) {
      return false;
    }

    const uint16_t total_boxes = hp_read_u16_le(payload_ptr + 0);
    const uint16_t offset = hp_read_u16_le(payload_ptr + 2);
    const uint8_t item_count = payload_ptr[4];
    const uint8_t item_size = payload_ptr[5];
    if (item_size != 11) {
      return false;
    }

#if DFR_HUMANPOSE_DEBUG
    Serial.print(F("[HPDBG] det chunk total="));
    Serial.print(total_boxes);
    Serial.print(F(", off="));
    Serial.print(offset);
    Serial.print(F(", cnt="));
    Serial.println(item_count);
#endif

    _bin_result_count = total_boxes;
    size_t cursor = 6;
    for (uint8_t i = 0; i < item_count; ++i) {
      if (cursor + 11 > payload_len) {
        break;
      }
      const uint16_t idx = (uint16_t)(offset + i);
      if (idx < MAX_RESULT_NUM) {
        binary_kp_result_t &dst = _bin_results[idx];
        dst.used = true;
#if !DFR_HUMANPOSE_LOW_MEMORY
        dst.is_pose = (_current_model == ePose);
#endif
        dst.x = hp_read_u16_le(payload_ptr + cursor + 0);
        dst.y = hp_read_u16_le(payload_ptr + cursor + 2);
        dst.w = hp_read_u16_le(payload_ptr + cursor + 4);
        dst.h = hp_read_u16_le(payload_ptr + cursor + 6);
        dst.score = payload_ptr[cursor + 8];
        dst.target = hp_read_u16_le(payload_ptr + cursor + 9);
      }
      cursor += 11;
    }
    return true;
  }

  if (msg_type == HP_BIN_MSG_HAND_KP_CHUNK || msg_type == HP_BIN_MSG_POSE_KP_CHUNK) {
    if (payload_len < 10) {
      return false;
    }

    const uint16_t total_keypoints = hp_read_u16_le(payload_ptr + 0);
    const uint16_t keypoint_index = hp_read_u16_le(payload_ptr + 2);
    const uint8_t total_points = payload_ptr[4];
    const uint8_t point_offset = payload_ptr[5];
    uint8_t point_count = payload_ptr[6];
    const uint8_t point_format = payload_ptr[7];
    const uint8_t has_box = payload_ptr[8];

#if DFR_HUMANPOSE_DEBUG
    Serial.print(F("[HPDBG] kp chunk type=0x"));
    Serial.print(msg_type, HEX);
    Serial.print(F(", total="));
    Serial.print(total_keypoints);
    Serial.print(F(", idx="));
    Serial.print(keypoint_index);
    Serial.print(F(", tpts="));
    Serial.print(total_points);
    Serial.print(F(", off="));
    Serial.print(point_offset);
    Serial.print(F(", cnt="));
    Serial.print(point_count);
    Serial.print(F(", fmt="));
    Serial.print(point_format);
    Serial.print(F(", box="));
    Serial.println(has_box);
#endif

    _bin_result_count = total_keypoints;
    if (keypoint_index >= MAX_RESULT_NUM) {
      return true;
    }

    binary_kp_result_t &dst = _bin_results[keypoint_index];
    dst.used = true;
#if !DFR_HUMANPOSE_LOW_MEMORY
    // Use point_format as the source of truth:
    // 0 => hand points [x,y], 1 => pose points [x,y,score,target].
    dst.is_pose = (point_format == 1);
#endif

    size_t cursor = 10;
    if (has_box) {
      if (cursor + 11 > payload_len) {
        return false;
      }
      dst.x = hp_read_u16_le(payload_ptr + cursor + 0);
      dst.y = hp_read_u16_le(payload_ptr + cursor + 2);
      dst.w = hp_read_u16_le(payload_ptr + cursor + 4);
      dst.h = hp_read_u16_le(payload_ptr + cursor + 6);
      dst.score = payload_ptr[cursor + 8];
      dst.target = hp_read_u16_le(payload_ptr + cursor + 9);
      cursor += 11;
    }

    if (point_offset > total_points) {
      return false;
    }
    if (point_count > (uint8_t)(total_points - point_offset)) {
      point_count = (uint8_t)(total_points - point_offset);
    }

#if DFR_HUMANPOSE_LOW_MEMORY
    // Low-memory mode keeps only bounding boxes; keypoints are parsed as stream bytes and discarded.
    for (uint8_t i = 0; i < point_count; ++i) {
      const uint8_t point_step = (point_format == 1) ? 6 : 4;
      if (cursor + point_step > payload_len) {
        break;
      }
      cursor += point_step;
    }
#else
    for (uint8_t i = 0; i < point_count; ++i) {
      uint8_t point_index = (uint8_t)(point_offset + i);
      uint16_t x = 0;
      uint16_t y = 0;

      if (point_format == 1) {
        if (cursor + 6 > payload_len) {
          break;
        }
        x = hp_read_u16_le(payload_ptr + cursor + 0);
        y = hp_read_u16_le(payload_ptr + cursor + 2);
        const uint8_t target = payload_ptr[cursor + 5];
        if (target < 17) {
          point_index = target;
        }
        cursor += 6;
      } else {
        if (cursor + 4 > payload_len) {
          break;
        }
        x = hp_read_u16_le(payload_ptr + cursor + 0);
        y = hp_read_u16_le(payload_ptr + cursor + 2);
        cursor += 4;
      }

      if (point_index < 21) {
        dst.points[point_index].x = x;
        dst.points[point_index].y = y;
        dst.point_valid[point_index] = 1;
      }
    }
#endif
    return true;
  }

  if (msg_type == HP_BIN_MSG_INVOKE_END) {
    _invoke_event_code = 0;
    if (payload_len >= 2) {
      _invoke_event_code = hp_read_i16_le(payload_ptr);
    }
    finalize_binary_results();
#if DFR_HUMANPOSE_DEBUG
    uint8_t used_cnt = 0;
    for (uint8_t i = 0; i < MAX_RESULT_NUM; ++i) {
      if (_bin_results[i].used) {
        ++used_cnt;
      }
    }
    Serial.print(F("[HPDBG] invoke end code="));
    Serial.print((int)_invoke_event_code);
    Serial.print(F(", total_kps="));
    Serial.print(_bin_result_count);
    Serial.print(F(", used_slots="));
    Serial.println(used_cnt);
#endif
    _invoke_event_ready = true;
    return true;
  }

  return false;
}

/**
 * @fn DFRobot_HumanPose::getResult
 * @brief Get detection results from the sensor
 * @details Sends an INVOKE command to trigger detection and waits for the response and event data
 * @return Status code of type `eCmdCode_t`. Returns `eOK` if results are successfully retrieved,
 *         otherwise returns an error code (typically `eTimedOut`)
 * @note After successful execution, detection results are available via the keypoints() method
 */
DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::getResult()
{
  char cmd[64] = { 0 };
  _invoke_event_ready = false;
  _invoke_event_code = 0;
  for (uint8_t i = 0; i < MAX_RESULT_NUM; ++i) {
    if (_result[i]) {
      _result[i]->used = true;
    }
  }
  clear_binary_results();
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s=1,0,1" CMD_SUF, AT_INVOKE);
  write(cmd, strlen(cmd));

#if DFR_HUMANPOSE_SMALL_RAM_PROFILE
  const uint32_t rsp_timeout = 1600;
  const uint32_t evt_timeout = 1600;
#else
  const uint32_t rsp_timeout = 1000;
  const uint32_t evt_timeout = 1000;
#endif

  if (wait(CMD_TYPE_RESPONSE, AT_INVOKE, rsp_timeout) == eOK) {
    if (wait(CMD_TYPE_EVENT, EVENT_INVOKE, evt_timeout) == eOK) {
      return eOK;
    } else {
      LDBG("Wait for EVENT_INVOKE timeout");
    }
  } else {
    LDBG("Wait for AT_INVOKE response timeout");
  }

  return eTimedOut;
}

static bool hp_is_percent_0_100(uint8_t v)
{
  return v <= 100;
}

static bool hp_is_valid_model(DFRobot_HumanPose::eModel_t model)
{
  return (model == DFRobot_HumanPose::eHand || model == DFRobot_HumanPose::ePose || model == DFRobot_HumanPose::eGes);
}

static bool hp_is_valid_baud(DFRobot_HumanPose_UART::eBaudConfig_t baud)
{
  switch (baud) {
    case DFRobot_HumanPose_UART::eBaud_9600:
    case DFRobot_HumanPose_UART::eBaud_14400:
    case DFRobot_HumanPose_UART::eBaud_19200:
    case DFRobot_HumanPose_UART::eBaud_38400:
    case DFRobot_HumanPose_UART::eBaud_57600:
    case DFRobot_HumanPose_UART::eBaud_115200:
    case DFRobot_HumanPose_UART::eBaud_230400:
    case DFRobot_HumanPose_UART::eBaud_460800:
    case DFRobot_HumanPose_UART::eBaud_921600:
      return true;
    default:
      return false;
  }
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::setConfidence(uint8_t confidence)
{
  if (!hp_is_percent_0_100(confidence)) {
    return eINVAL;
  }
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s=%d" CMD_SUF, AT_TSCORE, confidence);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_TSCORE) == eOK) {
    LDBG("SCORE OK");
    return eOK;
  }
  return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::setIOU(uint8_t iou)
{
  if (!hp_is_percent_0_100(iou)) {
    return eINVAL;
  }
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s=%d" CMD_SUF, AT_TIOU, iou);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_TIOU) == eOK) {
    return eOK;
  }
  return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::setModelType(eModel_t model)
{
  if (!hp_is_valid_model(model)) {
    return eINVAL;
  }
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s=%d" CMD_SUF, AT_MODEL, (int)model);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_MODEL) == eOK) {
    _current_model = model;
    if (model == eHand || model == ePose) {
      // Binary stream has id/box only for class label; refresh model list for name mapping.
      (void)getLearnList(model);
    }
    return eOK;
  }
  return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::setLearnSimilarity(uint8_t Similarity)
{
  if (!hp_is_percent_0_100(Similarity)) {
    return eINVAL;
  }
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s=%d" CMD_SUF, AT_TSIMILARITY, (int)Similarity);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_TSIMILARITY) == eOK) {
    return eOK;
  }
  return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::getConfidence(uint8_t *confidence)
{
  if (!confidence) {
    return eINVAL;
  }
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s?" CMD_SUF, AT_TSCORE);
  write(cmd, strlen(cmd));
  *confidence = 0;
  if (wait(CMD_TYPE_RESPONSE, AT_TSCORE) == eOK) {
    *confidence = _ret_data;
    return eOK;
  }
  return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::getIOU(uint8_t *iou)
{
  if (!iou) {
    return eINVAL;
  }
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s?" CMD_SUF, AT_TIOU);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_TIOU) == eOK) {
    *iou = _ret_data;
    return eOK;
  }
  return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::getLearnSimilarity(uint8_t *Similarity)
{
  if (!Similarity) {
    return eINVAL;
  }
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s?" CMD_SUF, AT_TSIMILARITY);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_TSIMILARITY) == eOK) {
    *Similarity = _ret_data;
    return eOK;
  }
  return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::setKeypointOutput(bool enable)
{
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s=%d" CMD_SUF, AT_TKPTS, enable ? 1 : 0);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_TKPTS) == eOK) {
    return eOK;
  }
  return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::getKeypointOutput(uint8_t *enable)
{
  if (!enable) {
    return eINVAL;
  }
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s?" CMD_SUF, AT_TKPTS);
  write(cmd, strlen(cmd));

  if (wait(CMD_TYPE_RESPONSE, AT_TKPTS) == eOK) {
    *enable = _ret_data ? 1 : 0;
    return eOK;
  }
  return eTimedOut;
}

LearnList DFRobot_HumanPose::getLearnList(eModel_t model)
{
  char cmd[64] = { 0 };
  if (!hp_is_valid_model(model)) {
    LearnList empty;
    return empty;
  }
  // GES: fixed class names only (id 0..14); no learn list AT on device.
  if (model == eGes) {
    LearnList empty;
    return empty;
  }
  if (model == ePose) {
    snprintf(cmd, sizeof(cmd), CMD_PRE "%s" CMD_SUF, AT_POSELIST);
    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, AT_POSELIST) == eOK) {
      LDBG("POSE OK");
      return _pose_class_list;
    }
    return _pose_class_list;
  } else {
    snprintf(cmd, sizeof(cmd), CMD_PRE "%s" CMD_SUF, AT_HANDLIST);
    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, AT_HANDLIST) == eOK) {
      LDBG("HAND OK");
      return _hand_class_list;
    }
    return _hand_class_list;
  }
}

bool DFRobot_HumanPose::availableResult()
{
  bool ret = false;
  for (uint8_t i = 0; i < MAX_RESULT_NUM; ++i) {
    if (_result[i] != NULL) {
      if (!_result[i]->used) {
        ret = true;
        break;
      }
    }
  }
  return ret;
}

Result *DFRobot_HumanPose::popResult()
{
  for (uint8_t i = 0; i < MAX_RESULT_NUM; ++i) {
    if (_result[i] != NULL) {
      if (_result[i]->used) {
        continue;
      }

      _result[i]->used = true;
      return _result[i];
    }
  }
  return NULL;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::getName(char *name)
{
  if (!name) {
    return eINVAL;
  }
  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s?" CMD_SUF, AT_NAME);
  write(cmd, strlen(cmd));

  const int ret = wait(CMD_TYPE_RESPONSE, AT_NAME);
#if DFR_HUMANPOSE_DEBUG
  Serial.print(F("[HPDBG] getName wait ret="));
  Serial.println(ret);
#endif
  if (ret == eOK) {
    strcpy(name, _name);
    return eOK;
  }
  return eTimedOut;
}

// ============ Derived Class: DFRobot_HumanPose_I2C ============

/**
 * @fn DFRobot_HumanPose_I2C::DFRobot_HumanPose_I2C
 * @brief Constructor of DFRobot_HumanPose_I2C class
 * @param wire Pointer to TwoWire object (typically &Wire)
 * @param address I2C device address (default is 0x3A)
 */
DFRobot_HumanPose_I2C::DFRobot_HumanPose_I2C(TwoWire *wire, uint8_t address)
{
  _wire     = wire;
  __address = address;
}

/**
 * @fn DFRobot_HumanPose_I2C::~DFRobot_HumanPose_I2C
 * @brief Destructor of DFRobot_HumanPose_I2C class
 */
DFRobot_HumanPose_I2C::~DFRobot_HumanPose_I2C() {}

/**
 * @fn DFRobot_HumanPose_I2C::begin
 * @brief Initialize the I2C communication and sensor
 * @details Initializes the I2C bus, sets the I2C clock speed, and calls the base class begin() method
 * @return True if initialization is successful, otherwise false
 */
bool DFRobot_HumanPose_I2C::begin(void)
{
  _wire->begin();
  _wire->setClock(I2C_CLOCK);
  _wait_delay = 2;

  return DFRobot_HumanPose::begin();
}

int DFRobot_HumanPose_I2C::available()
{
  uint8_t buf[2] = { 0 };
  delay(_wait_delay);
  _wire->beginTransmission(__address);
  _wire->write((uint8_t)FEATURE_TRANSPORT);
  _wire->write((uint8_t)FEATURE_TRANSPORT_CMD_AVAILABLE);
  _wire->write((uint8_t)0);
  _wire->write((uint8_t)0);
  // TODO checksum
  _wire->write((uint8_t)0);
  _wire->write((uint8_t)0);
  const uint8_t tx_ret = _wire->endTransmission();
  if (tx_ret == 0) {
    delay(_wait_delay);
    const int got = _wire->requestFrom(__address, (uint8_t)2);
    if (got >= 2) {
      const int rd = _wire->readBytes(buf, (uint8_t)2);
      if (rd < 2) {
        return 0;
      }
    } else {
      return 0;
    }
  } else {
#if DFR_HUMANPOSE_DEBUG
    Serial.print(F("[HPDBG] i2c available endTx err="));
    Serial.println((int)tx_ret);
#endif
    return 0;
  }

  return (buf[0] << 8) | buf[1];
}

int DFRobot_HumanPose_I2C::read(char *data, int len)
{
  if (!data || len <= 0) {
    return 0;
  }

  int total_read = 0;
  while (total_read < len) {
    const int remain = len - total_read;
    const uint16_t chunk = (uint16_t)((remain < (int)MAX_PL_LEN) ? remain : (int)MAX_PL_LEN);
    delay(_wait_delay);
    _wire->beginTransmission(__address);
    _wire->write((uint8_t)FEATURE_TRANSPORT);
    _wire->write((uint8_t)FEATURE_TRANSPORT_CMD_READ);
    _wire->write((uint8_t)(chunk >> 8));
    _wire->write((uint8_t)(chunk & 0xFF));
    // TODO checksum
    _wire->write((uint8_t)0);
    _wire->write((uint8_t)0);
    const uint8_t tx_ret = _wire->endTransmission();
    if (tx_ret == 0) {
      delay(_wait_delay);
      const int got = _wire->requestFrom(__address, (uint8_t)chunk);
      if (got <= 0) {
        break;
      }
      const int rd = _wire->readBytes(data + total_read, (size_t)got);
      if (rd <= 0) {
        break;
      }
      total_read += rd;
      if (rd < got || got < (int)chunk) {
        break;
      }
    } else {
#if DFR_HUMANPOSE_DEBUG
      Serial.print(F("[HPDBG] i2c read endTx err="));
      Serial.println((int)tx_ret);
#endif
      break;
    }
  }

  return total_read;
}

int DFRobot_HumanPose_I2C::write(const char *data, int len)
{
  if (!data || len <= 0) {
    return 0;
  }

  int total_write = 0;
  while (total_write < len) {
    const int remain = len - total_write;
    const uint16_t chunk = (uint16_t)((remain < (int)MAX_PL_LEN) ? remain : (int)MAX_PL_LEN);
    delay(_wait_delay);
    _wire->beginTransmission(__address);
    _wire->write((uint8_t)FEATURE_TRANSPORT);
    _wire->write((uint8_t)FEATURE_TRANSPORT_CMD_WRITE);
    _wire->write((uint8_t)(chunk >> 8));
    _wire->write((uint8_t)(chunk & 0xFF));
    _wire->write((const uint8_t *)(data + total_write), chunk);
    // TODO checksum
    _wire->write((uint8_t)0);
    _wire->write((uint8_t)0);
    const uint8_t tx_ret = _wire->endTransmission();
    if (tx_ret != 0) {
#if DFR_HUMANPOSE_DEBUG
      Serial.print(F("[HPDBG] i2c write endTx err="));
      Serial.println((int)tx_ret);
#endif
      break;
    }
    total_write += chunk;
  }

  return total_write;
}

// ============ Derived Class: DFRobot_HumanPose_UART ============

#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
/**
 * @fn DFRobot_HumanPose_UART::DFRobot_HumanPose_UART
 * @brief Constructor of DFRobot_HumanPose_UART class (for UNO/ESP8266)
 * @param sSerial Pointer to SoftwareSerial object
 * @param baud Baud rate value (default UART_BAUD, 9600)
 */
DFRobot_HumanPose_UART::DFRobot_HumanPose_UART(SoftwareSerial *sSerial, uint32_t baud)
{
  _serial = sSerial;
  __baud  = baud;
}
#else
/**
 * @fn DFRobot_HumanPose_UART::DFRobot_HumanPose_UART
 * @brief Constructor of DFRobot_HumanPose_UART class
 * @param hSerial Pointer to HardwareSerial object (typically &Serial1)
 * @param baud Baud rate value (default UART_BAUD, 9600)
 * @param rxpin RX pin number (default is 0, required for ESP32)
 * @param txpin TX pin number (default is 0, required for ESP32)
 */
DFRobot_HumanPose_UART::DFRobot_HumanPose_UART(HardwareSerial *hSerial, uint32_t baud, uint8_t rxpin, uint8_t txpin)
{
  _serial = hSerial;
  __baud  = baud;
  __rxpin = rxpin;
  __txpin = txpin;
}
#endif

/**
 * @fn DFRobot_HumanPose_UART::~DFRobot_HumanPose_UART
 * @brief Destructor of DFRobot_HumanPose_UART class
 */
DFRobot_HumanPose_UART::~DFRobot_HumanPose_UART() {}

/**
 * @fn DFRobot_HumanPose_UART::begin
 * @brief Initialize the UART communication and sensor
 * @details Initializes the serial port with the configured baud rate, sets timeout, and calls the base class begin() method
 * @return True if initialization is successful, otherwise false
 */
bool DFRobot_HumanPose_UART::begin(void)
{
  _wait_delay = 2;
#ifdef ESP32
  _serial->begin(__baud, SERIAL_8N1, __rxpin, __txpin);
#elif defined(ARDUINO_AVR_UNO) || defined(ESP8266)
  _serial->begin(__baud);
#else
  _serial->begin(__baud);
#endif
  _serial->setTimeout(1000);
  _serial->flush();
  // flush() only waits outgoing TX; drain stale RX bytes to avoid matching old frames in begin().
  while (_serial->available() > 0) {
    (void)_serial->read();
  }

  return DFRobot_HumanPose::begin();
}

bool DFRobot_HumanPose_UART::setBaud(eBaudConfig_t baud)
{
  if (!hp_is_valid_baud(baud)) {
    return false;
  }
  /* Clear parser state and drain UART so the BAUD reply is not mixed with stale RX / flags. */
  _at_rsp_ready = false;
  _invoke_event_ready = false;
  _at_payload_len = 0;
  rx_end = 0;
  while (_serial->available() > 0) {
    (void)_serial->read();
  }

  char cmd[64] = { 0 };
  snprintf(cmd, sizeof(cmd), CMD_PRE "%s=%lu" CMD_SUF, AT_BAUD, (unsigned long)(uint32_t)baud);
  write(cmd, strlen(cmd));

  /* SoftwareSerial / busy MCUs may need a slightly longer window for the binary ACK frame. */
  if (wait(CMD_TYPE_RESPONSE, AT_BAUD, 2000) == eOK) {
    return true;
  }
  return false;
}

int DFRobot_HumanPose_UART::available()
{
  return _serial->available();
}

int DFRobot_HumanPose_UART::read(char *data, int len)
{
  if (!data || len <= 0) {
    return 0;
  }

  int n = 0;
  while (n < len && _serial->available() > 0) {
    const int c = _serial->read();
    if (c < 0) {
      break;
    }
    data[n++] = (char)c;
  }
  return n;
}

int DFRobot_HumanPose_UART::write(const char *data, int len)
{
  return _serial->write(data, len);
}
