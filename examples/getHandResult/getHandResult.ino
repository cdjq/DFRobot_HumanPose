/*!
 * @file getHandResult.ino
 * @brief Example of getting human hand detection data
 * @details This example demonstrates how to get detection results from the human hand detection sensor and print bounding box and keypoint information
 * @copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @License     The MIT License (MIT)
 * @author [thdyyl](yuanlong.yu@dfrobot.com)
 * @version  V1.0.0
 * @date  2026-04-24
 * @url         https://github.com/DFRobot/DFRobot_HumanPose
 */

#include <DFRobot_HumanPose.h>

/* >> Step 1: Please choose your communication method below */
//  #define HUMANPOSE_COMM_UART  // Use UART communication
#define HUMANPOSE_COMM_I2C    // Use I2C communication

#if defined(HUMANPOSE_COMM_UART)
/* ---------------------------------------------------------------------------------------------------------------------
 * Hardware connection table:
 *    Sensor Pin |        MCU Pin        | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *     VCC       |        3.3V/5V         |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *     GND       |         GND            |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 *     RX        |        MCU TX          |     Serial1 TX1      |     5     |   5/D6  |  26/D3|     X      |  tx1  |
 *     TX        |        MCU RX          |     Serial1 RX1      |     4     |   4/D7  |  25/D2|     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------------------------*/
// Initialize UART communication: Serial1, baud rate 9600, RX pin 25, TX pin 26 (ESP32)
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include <SoftwareSerial.h>
SoftwareSerial         mySerial(4, 5);
DFRobot_HumanPose_UART humanPose(&mySerial, 9600);
#elif defined(ESP32)
DFRobot_HumanPose_UART humanPose(&Serial1, 9600, /*RX pin*/ 25, /*TX pin*/ 26);
#elif defined(ARDUINO_BBC_MICROBIT) && !defined(ARDUINO_BBC_MICROBIT_V2)
#error "BBC micro:bit (nRF51, sandeepmistry/nRF5): Serial1 is not defined. Use I2C in this sketch (#define HUMANPOSE_COMM_I2C) or a board with Serial1."
#else
DFRobot_HumanPose_UART humanPose(&Serial1, 9600);
#endif
#elif defined(HUMANPOSE_COMM_I2C)
/**
  * I2C address configuration
  * Default I2C address is 0x3A
  */
const uint8_t I2C_ADDR = 0x3A;
// Initialize I2C communication: Wire, I2C address 0x3A
DFRobot_HumanPose_I2C humanPose(&Wire, I2C_ADDR);
#else
#error "Please define HUMANPOSE_COMM_UART or HUMANPOSE_COMM_I2C"
#endif

/** Print "[HAND] no target..." at streak 1, then every N consecutive empty frames. */
static const uint16_t NO_TARGET_LOG_INTERVAL = 20;

/** Hand ASCII grid: 1 = points + short legend (less serial noise), 0 = skeleton + long legend. */
#ifndef GET_HAND_ASCII_SIMPLE
#define GET_HAND_ASCII_SIMPLE 1
#endif

#ifndef ASCII_GRID_COLS
#define ASCII_GRID_COLS 40
#endif
static void printSeparator()
{
  Serial.println(F("----------------------------------------------------------------"));
}

static void printScoreAndBoxExplained(const Result *result)
{
  Serial.print(F("    score="));
  Serial.print(result->score);
  Serial.println(F(" (0-100; id==0: det.conf else: learn sim)"));
  Serial.print(F("    box(left, top, width, height)=("));
  Serial.print(result->xLeft);
  Serial.print(F(", "));
  Serial.print(result->yTop);
  Serial.print(F(", "));
  Serial.print(result->width);
  Serial.print(F(", "));
  Serial.print(result->height);
  Serial.println(F(")"));
}

#if !DFR_HUMANPOSE_LOW_MEMORY
static void kpItem(const char *name, const PointU16 &p)
{
  Serial.print(name);
  Serial.print(F("=("));
  Serial.print(p.x);
  Serial.print(F(","));
  Serial.print(p.y);
  Serial.print(F(")"));
}

static void kpSep()
{
  Serial.print(F(" | "));
}

static void printHandKeypointsCompact(const HandResult *h)
{
  Serial.println(F("    keypoints:"));
  Serial.print(F("    "));
  kpItem("wrist", h->wrist);
  kpSep();
  kpItem("thumbCmc", h->thumbCmc);
  kpSep();
  kpItem("thumbMcp", h->thumbMcp);
  kpSep();
  kpItem("thumbIp", h->thumbIp);
  kpSep();
  kpItem("thumbTip", h->thumbTip);
  Serial.println();
  Serial.print(F("    "));
  kpItem("indexFingerMcp", h->indexFingerMcp);
  kpSep();
  kpItem("indexFingerPip", h->indexFingerPip);
  kpSep();
  kpItem("indexFingerDip", h->indexFingerDip);
  kpSep();
  kpItem("indexFingerTip", h->indexFingerTip);
  Serial.println();
  Serial.print(F("    "));
  kpItem("middleFingerMcp", h->middleFingerMcp);
  kpSep();
  kpItem("middleFingerPip", h->middleFingerPip);
  kpSep();
  kpItem("middleFingerDip", h->middleFingerDip);
  kpSep();
  kpItem("middleFingerTip", h->middleFingerTip);
  Serial.println();
  Serial.print(F("    "));
  kpItem("ringFingerMcp", h->ringFingerMcp);
  kpSep();
  kpItem("ringFingerPip", h->ringFingerPip);
  kpSep();
  kpItem("ringFingerDip", h->ringFingerDip);
  kpSep();
  kpItem("ringFingerTip", h->ringFingerTip);
  Serial.println();
  Serial.print(F("    "));
  kpItem("pinkyFingerMcp", h->pinkyFingerMcp);
  kpSep();
  kpItem("pinkyFingerPip", h->pinkyFingerPip);
  kpSep();
  kpItem("pinkyFingerDip", h->pinkyFingerDip);
  Serial.println();
  Serial.print(F("    "));
  kpItem("pinkyFingerTip", h->pinkyFingerTip);
  Serial.println();
}

static int clampi(int v, int lo, int hi)
{
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

static void drawPoint(char grid[][ASCII_GRID_COLS + 1], int w, int h, int x, int y, char ch)
{
  if (x < 0 || x >= w || y < 0 || y >= h) {
    return;
  }
  grid[y][x] = ch;
}

static void drawLine(char grid[][ASCII_GRID_COLS + 1], int w, int h, int x0, int y0, int x1, int y1, char ch)
{
  int dx = abs(x1 - x0);
  int sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0);
  int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  while (true) {
    drawPoint(grid, w, h, x0, y0, ch);
    if (x0 == x1 && y0 == y1) {
      break;
    }
    int e2 = err * 2;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

static void printGrid(char grid[][ASCII_GRID_COLS + 1], int h)
{
  for (int y = 0; y < h; ++y) {
    Serial.println(grid[y]);
  }
}

static void mapToGrid(const Result *r, const PointU16 *src, size_t n, int16_t *gx, int16_t *gy, bool *ok, int w, int h)
{
  const int32_t bx = (int32_t)r->xLeft;
  const int32_t by = (int32_t)r->yTop;
  const int32_t bw = (int32_t)r->width;
  const int32_t bh = (int32_t)r->height;
  const int32_t safeW = (bw <= 1) ? 1 : bw;
  const int32_t safeH = (bh <= 1) ? 1 : bh;

  for (size_t i = 0; i < n; ++i) {
    int32_t lx = (int32_t)src[i].x - bx;
    int32_t ly = (int32_t)src[i].y - by;
    if (lx < 0 || ly < 0 || lx > safeW || ly > safeH) {
      ok[i] = false;
      gx[i] = -1;
      gy[i] = -1;
      continue;
    }
    gx[i] = (int16_t)clampi((int)((lx * (w - 1)) / safeW), 0, w - 1);
    gy[i] = (int16_t)clampi((int)((ly * (h - 1)) / safeH), 0, h - 1);
    ok[i] = true;
  }
}

static void drawBoneIfValid(char grid[][ASCII_GRID_COLS + 1], int w, int h, const int16_t *gx, const int16_t *gy, const bool *ok, uint8_t a, uint8_t b)
{
  if (ok[a] && ok[b]) {
    drawLine(grid, w, h, gx[a], gy[a], gx[b], gy[b], '.');
  }
}

static char pointMark(uint8_t idx)
{
  return (char)('A' + (idx % 26));
}

static void asciiHand(const HandResult *p)
{
  const int W = ASCII_GRID_COLS;
  const int H = 16;
  char grid[H][ASCII_GRID_COLS + 1];
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      grid[y][x] = ' ';
    }
    grid[y][W] = '\0';
  }

  PointU16 pts[21] = { p->wrist,          p->thumbCmc,       p->thumbMcp,       p->thumbIp,      p->thumbTip,       p->indexFingerMcp,
                       p->indexFingerPip, p->indexFingerDip, p->indexFingerTip, p->middleFingerMcp, p->middleFingerPip, p->middleFingerDip,
                       p->middleFingerTip, p->ringFingerMcp, p->ringFingerPip, p->ringFingerDip, p->ringFingerTip, p->pinkyFingerMcp,
                       p->pinkyFingerPip, p->pinkyFingerDip, p->pinkyFingerTip };
  int16_t gx[21];
  int16_t gy[21];
  bool ok[21];
  mapToGrid(p, pts, 21, gx, gy, ok, W, H);

#if GET_HAND_ASCII_SIMPLE
  const bool simple = true;
#else
  const bool simple = false;
#endif
  if (!simple) {
    drawBoneIfValid(grid, W, H, gx, gy, ok, 0, 1);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 1, 2);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 2, 3);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 3, 4);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 0, 5);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 5, 6);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 6, 7);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 7, 8);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 0, 9);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 9, 10);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 10, 11);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 11, 12);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 0, 13);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 13, 14);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 14, 15);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 15, 16);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 0, 17);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 17, 18);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 18, 19);
    drawBoneIfValid(grid, W, H, gx, gy, ok, 19, 20);
  }

  for (uint8_t i = 0; i < 21; ++i) {
    if (ok[i]) {
      drawPoint(grid, W, H, gx[i], gy[i], pointMark(i));
    }
  }
  Serial.println(simple ? F("    [Simple ASCII hand]") : F("    [ASCII hand diagram]"));
  printGrid(grid, H);
  if (simple) {
    Serial.println(F("    legend:"));
    Serial.println(F("      A:wrist B:thumbCmc C:thumbMcp D:thumbIp E:thumbTip F:indexFingerMcp G:indexFingerPip"));
    Serial.println(F("      H:indexFingerDip I:indexFingerTip J:middleFingerMcp K:middleFingerPip L:middleFingerDip M:middleFingerTip N:ringFingerMcp"));
    Serial.println(F("      O:ringFingerPip P:ringFingerDip Q:ringFingerTip R:pinkyFingerMcp S:pinkyFingerPip T:pinkyFingerDip U:pinkyFingerTip"));
  }
}
#endif

/**
  * @brief Initialize function
  * @details Set up serial communication, initialize sensor, configure detection model
  */
void setup()
{
  // Initialize serial port for debug output
  Serial.begin(115200);

  // Initialize sensor, retry if failed
  while (!humanPose.begin()) {
    Serial.println(F("Sensor init fail!"));
    delay(1000);
  }
  Serial.println(F("Sensor init success!"));

  // Set detection model: eHand (hand detection) or ePose (human pose detection)
  humanPose.setModelType(DFRobot_HumanPose::eHand);
#if DFR_HUMANPOSE_LOW_MEMORY
  humanPose.setKeypointOutput(false);
#endif
}

/**
  * @brief Main loop function
  * @details Continuously get detection results from sensor and print bounding box and keypoint information
  */
void loop()
{
  static uint32_t s_frame           = 0;
  static uint32_t no_target_streak = 0;

  ++s_frame;

  if (humanPose.getResult() == DFRobot_HumanPose::eOK) {
    bool     any = false;
    uint16_t idx = 0;
    while (humanPose.availableResult()) {
#if DFR_HUMANPOSE_LOW_MEMORY
      Result *result = humanPose.popResult();
#else
      HandResult *result = static_cast<HandResult *>(humanPose.popResult());
#endif
      if (!result) {
        continue;
      }
      if (!any) {
        no_target_streak = 0;
        printSeparator();
        Serial.print(F("[HAND][frame "));
        Serial.print(s_frame);
        Serial.println(F("] target(s):"));
        any = true;
      }
      ++idx;
      Serial.print(F("  #"));
      Serial.print(idx);
      Serial.print(F(" id="));
      Serial.print(result->id);
      Serial.print(F(" name="));
      Serial.println(result->name);
      printScoreAndBoxExplained(result);
#if !DFR_HUMANPOSE_LOW_MEMORY
      printHandKeypointsCompact(result);
      asciiHand(result);
#endif
    }
    if (!any) {
      ++no_target_streak;
      if (no_target_streak == 1u
          || (no_target_streak % NO_TARGET_LOG_INTERVAL) == 0u) {
        Serial.println(F("[HAND] no target"));
      }
    } else {
      printSeparator();
    }
  } else {
    Serial.println(F("[HAND] get_result timeout"));
  }

  // Delay to avoid output too fast
  delay(50);
}
