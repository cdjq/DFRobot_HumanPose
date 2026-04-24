/*!
 * @file getPoseResult.ino
 * @brief Example of getting human pose detection data
 * @details This example demonstrates how to get detection results from the human pose detection sensor and print bounding box and keypoint information
 * @copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @License     The MIT License (MIT)
 * @author [thdyyl](yuanlong.yu@dfrobot.com)
 * @version  V1.0.0
 * @date  2026-04-24
 * @url         https://github.com/DFRobot/DFRobot_HumanPose
 */

#include <DFRobot_HumanPose.h>

/* >> Step 1: Please choose your communication method below */
// #define HUMANPOSE_COMM_UART  // Use UART communication
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

#ifndef ASCII_GRID_COLS
#define ASCII_GRID_COLS 40
#endif


/** Print "[POSE] no target..." at streak 1, then every N consecutive empty frames (reduces log spam). */
static const uint16_t NO_TARGET_LOG_INTERVAL = 20;

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

static void printPoseKeypointsCompact(const PoseResult *p)
{
  Serial.println(F("    keypoints:"));
  Serial.print(F("    "));
  kpItem("nose", p->nose);
  kpSep();
  kpItem("leye", p->leye);
  kpSep();
  kpItem("reye", p->reye);
  kpSep();
  kpItem("lear", p->lear);
  kpSep();
  kpItem("rear", p->rear);
  Serial.println();
  Serial.print(F("    "));
  kpItem("lshoulder", p->lshoulder);
  kpSep();
  kpItem("rshoulder", p->rshoulder);
  kpSep();
  kpItem("lelbow", p->lelbow);
  kpSep();
  kpItem("relbow", p->relbow);
  Serial.println();
  Serial.print(F("    "));
  kpItem("lwrist", p->lwrist);
  kpSep();
  kpItem("rwrist", p->rwrist);
  kpSep();
  kpItem("lhip", p->lhip);
  kpSep();
  kpItem("rhip", p->rhip);
  Serial.println();
  Serial.print(F("    "));
  kpItem("lknee", p->lknee);
  kpSep();
  kpItem("rknee", p->rknee);
  kpSep();
  kpItem("lankle", p->lankle);
  Serial.println();
  Serial.print(F("    "));
  kpItem("rankle", p->rankle);
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

/* Pose points may extend outside bbox; map by point-cloud range (same idea as functionalTestDemo). */
static void mapPointsAutoRange(const PointU16 *src, size_t n, int16_t *gx, int16_t *gy, bool *ok, int w, int h)
{
  if (!src || n == 0) {
    return;
  }

  uint16_t minX = 0xFFFFu, minY = 0xFFFFu, maxX = 0u, maxY = 0u;
  bool any = false;
  for (size_t i = 0; i < n; ++i) {
    const uint16_t x = src[i].x;
    const uint16_t y = src[i].y;
    if (x == 0u && y == 0u) {
      ok[i] = false;
      gx[i] = -1;
      gy[i] = -1;
      continue;
    }
    ok[i] = true;
    any = true;
    if (x < minX) minX = x;
    if (y < minY) minY = y;
    if (x > maxX) maxX = x;
    if (y > maxY) maxY = y;
  }

  if (!any) {
    for (size_t i = 0; i < n; ++i) {
      ok[i] = false;
      gx[i] = -1;
      gy[i] = -1;
    }
    return;
  }

  uint16_t spanX = (maxX > minX) ? (uint16_t)(maxX - minX) : 1u;
  uint16_t spanY = (maxY > minY) ? (uint16_t)(maxY - minY) : 1u;
  const uint16_t marginX = (uint16_t)(spanX / 10u + 1u);
  const uint16_t marginY = (uint16_t)(spanY / 10u + 1u);
  const int32_t loX = (int32_t)minX - marginX;
  const int32_t loY = (int32_t)minY - marginY;
  const int32_t hiX = (int32_t)maxX + marginX;
  const int32_t hiY = (int32_t)maxY + marginY;
  const int32_t rangeX = (hiX > loX) ? (hiX - loX) : 1;
  const int32_t rangeY = (hiY > loY) ? (hiY - loY) : 1;

  for (size_t i = 0; i < n; ++i) {
    if (!ok[i]) {
      continue;
    }
    const int32_t x = (int32_t)src[i].x;
    const int32_t y = (int32_t)src[i].y;
    gx[i] = (int16_t)clampi((int)(((x - loX) * (w - 1)) / rangeX), 0, w - 1);
    gy[i] = (int16_t)clampi((int)(((y - loY) * (h - 1)) / rangeY), 0, h - 1);
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

static void asciiPose(const PoseResult *p)
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

  PointU16 pts[17] = { p->nose, p->leye, p->reye, p->lear, p->rear, p->lshoulder, p->rshoulder, p->lelbow, p->relbow,
                       p->lwrist, p->rwrist, p->lhip, p->rhip, p->lknee, p->rknee, p->lankle, p->rankle };
  int16_t gx[17];
  int16_t gy[17];
  bool ok[17];
  mapPointsAutoRange(pts, 17, gx, gy, ok, W, H);

  drawBoneIfValid(grid, W, H, gx, gy, ok, 0, 1);
  drawBoneIfValid(grid, W, H, gx, gy, ok, 0, 2);
  drawBoneIfValid(grid, W, H, gx, gy, ok, 1, 3);
  drawBoneIfValid(grid, W, H, gx, gy, ok, 2, 4);
  drawBoneIfValid(grid, W, H, gx, gy, ok, 5, 6);
  drawBoneIfValid(grid, W, H, gx, gy, ok, 5, 7);
  drawBoneIfValid(grid, W, H, gx, gy, ok, 7, 9);
  drawBoneIfValid(grid, W, H, gx, gy, ok, 6, 8);
  drawBoneIfValid(grid, W, H, gx, gy, ok, 8, 10);
  drawBoneIfValid(grid, W, H, gx, gy, ok, 5, 11);
  drawBoneIfValid(grid, W, H, gx, gy, ok, 6, 12);
  drawBoneIfValid(grid, W, H, gx, gy, ok, 11, 12);
  drawBoneIfValid(grid, W, H, gx, gy, ok, 11, 13);
  drawBoneIfValid(grid, W, H, gx, gy, ok, 13, 15);
  drawBoneIfValid(grid, W, H, gx, gy, ok, 12, 14);
  drawBoneIfValid(grid, W, H, gx, gy, ok, 14, 16);

  for (uint8_t i = 0; i < 17; ++i) {
    if (ok[i]) {
      drawPoint(grid, W, H, gx[i], gy[i], pointMark(i));
    }
  }
  Serial.println(F("    [ASCII POSE skeleton]"));
  printGrid(grid, H);
  Serial.println(F("    legend:"));
  Serial.println(F("      A:nose B:leye C:reye D:lear E:rear F:lshoulder G:rshoulder"));
  Serial.println(F("      H:lelbow I:relbow J:lwrist K:rwrist L:lhip M:rhip N:lknee"));
  Serial.println(F("      O:rknee P:lankle Q:rankle"));
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
  humanPose.setModelType(DFRobot_HumanPose::ePose);
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
      PoseResult *result = static_cast<PoseResult *>(humanPose.popResult());
#endif
      if (!result) {
        continue;
      }
      if (!any) {
        no_target_streak = 0;
        printSeparator();
        Serial.print(F("[POSE][frame "));
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
      printPoseKeypointsCompact(result);
      asciiPose(result);
#endif
    }
    if (!any) {
      ++no_target_streak;
      if (no_target_streak == 1u
          || (no_target_streak % NO_TARGET_LOG_INTERVAL) == 0u) {
        Serial.println(F("[POSE] no target"));
      }
    } else {
      printSeparator();
    }
  } else {
    Serial.println(F("[POSE] get_result timeout"));
  }

  // Delay to avoid output too fast
  delay(100);
}
