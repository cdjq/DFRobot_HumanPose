/*!
 * @file functionalTestDemo.ino
 * @brief Functional test demo for DFRobot_HumanPose.
 * @details Interactive serial menu to verify runtime features on real hardware.
 */

#include <DFRobot_HumanPose.h>

/* ======================== Communication Selection ======================== */
#define HUMANPOSE_COMM_UART
//#define HUMANPOSE_COMM_I2C

#if defined(HUMANPOSE_COMM_UART)
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include <SoftwareSerial.h>
SoftwareSerial         mySerial(4, 5);
DFRobot_HumanPose_UART humanPose(&mySerial, 9600);
#elif defined(ESP32)
DFRobot_HumanPose_UART humanPose(&Serial1, 9600, 25, 26);
#elif defined(ARDUINO_BBC_MICROBIT) && !defined(ARDUINO_BBC_MICROBIT_V2)
#error "BBC micro:bit (nRF51, sandeepmistry/nRF5): Serial1 is not defined. Use I2C (#define HUMANPOSE_COMM_I2C) or a board with Serial1."
#else
DFRobot_HumanPose_UART humanPose(&Serial1, 9600);
#endif
#elif defined(HUMANPOSE_COMM_I2C)
const uint8_t I2C_ADDR = 0x3A;
DFRobot_HumanPose_I2C humanPose(&Wire, I2C_ADDR);
#else
#error "Please define HUMANPOSE_COMM_UART or HUMANPOSE_COMM_I2C"
#endif

static DFRobot_HumanPose::eModel_t g_model = DFRobot_HumanPose::eGesture;
static bool g_stream = false;
static uint32_t g_frame = 0;
static bool g_asciiViz = true;
static bool g_asciiSimple = true;
static const int ASCII_GRID_COLS = 40;

static const char *modelName(DFRobot_HumanPose::eModel_t m)
{
  if (m == DFRobot_HumanPose::eHand) {
    return "HAND";
  }
  if (m == DFRobot_HumanPose::ePose) {
    return "POSE";
  }
  return "GESTURE";
}

static void printMenu()
{
  Serial.println();
  Serial.println(F("========== Functional Test Menu =========="));
  Serial.println(F("1: Set model = GESTURE"));
  Serial.println(F("2: Set model = HAND"));
  Serial.println(F("3: Set model = POSE"));
  Serial.println(F("k: Keypoint OFF"));
  Serial.println(F("K: Keypoint ON"));
  Serial.println(F("c: Set confidence = 60"));
  Serial.println(F("i: Set IOU = 45"));
  Serial.println(F("s: Set similarity = 80"));
  Serial.println(F("r: Read current thresholds + keypoint mode"));
  Serial.println(F("l: Read learn list of current model"));
  Serial.println(F("g: Run one getResult()"));
  Serial.println(F("t: Toggle stream getResult()"));
  Serial.println(F("v: Toggle ASCII keypoint visualize"));
  Serial.println(F("m: Toggle ASCII mode (simple/skeleton)"));
  Serial.println(F("h: Show this menu"));
  Serial.println(F("=========================================="));
}

static void printBase(const Result *r)
{
  Serial.print(F("id="));
  Serial.print(r->id);
  Serial.print(F(" name="));
  Serial.print(r->name);
  Serial.print(F(" score="));
  Serial.print(r->score);
  Serial.print(F(" box=("));
  Serial.print(r->xLeft);
  Serial.print(F(","));
  Serial.print(r->yTop);
  Serial.print(F(","));
  Serial.print(r->width);
  Serial.print(F(","));
  Serial.print(r->height);
  Serial.println(F(")"));
}

#if !DFR_HUMANPOSE_LOW_MEMORY
static void printPoseBrief(const PoseResult *p)
{
  Serial.print(F("  nose=("));
  Serial.print(p->nose.x);
  Serial.print(F(","));
  Serial.print(p->nose.y);
  Serial.println(F(")"));
}

static void printHandBrief(const HandResult *h)
{
  Serial.print(F("  wrist=("));
  Serial.print(h->wrist.x);
  Serial.print(F(","));
  Serial.print(h->wrist.y);
  Serial.println(F(")"));
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

/* Pose points are sometimes not tightly bounded by bbox; map by point-cloud range for readability. */
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
    /* Heuristic: treat (0,0) as likely "missing point". */
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
  /* Add 10% margin to reduce clipping on edges. */
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
  // Unified letter-only marker: A, B, C...
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

  // POSE: always draw skeleton lines (more intuitive than points-only).
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
  Serial.println(F("  [ASCII POSE skeleton]"));
  printGrid(grid, H);
  Serial.println(F("  legend: A:nose B:leye C:reye D:lear E:rear F:lsho G:rsho H:lelb I:relb J:lwri K:rwri L:lhip M:rhip N:lknee O:rknee P:lank Q:rank"));
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

  if (!g_asciiSimple) {
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
  Serial.println(g_asciiSimple ? F("  [ASCII HAND simple: point index]") : F("  [ASCII HAND skeleton]"));
  printGrid(grid, H);
  if (g_asciiSimple) {
    Serial.println(F("  legend: A:wrist B:Cmc C:Mcp D:Ip E:Tip F:I-Mcp G:I-Pip H:I-Dip I:I-Tip J:M-Mcp K:M-Pip L:M-Dip M:M-Tip N:R-Mcp O:R-Pip P:R-Dip Q:R-Tip R:P-Mcp S:P-Pip T:P-Dip U:P-Tip"));
  }
}
#endif

static void runOneResult()
{
  const int ret = humanPose.getResult();
  if (ret != DFRobot_HumanPose::eOK) {
    Serial.print(F("[ERR] getResult ret="));
    Serial.println(ret);
    return;
  }

  uint16_t cnt = 0;
  ++g_frame;
  Serial.print(F("[FRAME "));
  Serial.print(g_frame);
  Serial.print(F("]["));
  Serial.print(modelName(g_model));
  Serial.println(F("]"));

  while (humanPose.availableResult()) {
    Result *r = humanPose.popResult();
    if (!r) {
      break;
    }
    ++cnt;
    Serial.print(F(" #"));
    Serial.print(cnt);
    Serial.print(F(" "));
    printBase(r);

#if !DFR_HUMANPOSE_LOW_MEMORY
    if (g_model == DFRobot_HumanPose::ePose) {
      PoseResult *pose = (PoseResult *)r;
      printPoseBrief(pose);
      if (g_asciiViz) {
        asciiPose(pose);
      }
    } else if (g_model == DFRobot_HumanPose::eHand) {
      HandResult *hand = (HandResult *)r;
      printHandBrief(hand);
      if (g_asciiViz) {
        asciiHand(hand);
      }
    }
#endif
  }

  if (cnt == 0) {
    Serial.println(F(" no target"));
  }
}

static void setModelAndInfo(DFRobot_HumanPose::eModel_t model)
{
  const int ret = humanPose.setModelType(model);
  if (ret == DFRobot_HumanPose::eOK) {
    g_model = model;
    Serial.print(F("[OK] model -> "));
    Serial.println(modelName(g_model));
  } else {
    Serial.print(F("[ERR] setModelType ret="));
    Serial.println(ret);
  }
}

static void readConfig()
{
  uint8_t v = 0;
  if (humanPose.getConfidence(&v) == DFRobot_HumanPose::eOK) {
    Serial.print(F("confidence="));
    Serial.println(v);
  }
  if (humanPose.getIOU(&v) == DFRobot_HumanPose::eOK) {
    Serial.print(F("iou="));
    Serial.println(v);
  }
  if (humanPose.getLearnSimilarity(&v) == DFRobot_HumanPose::eOK) {
    Serial.print(F("similarity="));
    Serial.println(v);
  }
  if (humanPose.getKeypointOutput(&v) == DFRobot_HumanPose::eOK) {
    Serial.print(F("keypoint="));
    Serial.println(v ? F("ON") : F("OFF"));
  }
}

static void readLearnList()
{
  LearnList list = humanPose.getLearnList(g_model);
  Serial.print(F("learnList size="));
  Serial.println((int)list.size());
  for (size_t i = 0; i < list.size(); ++i) {
    Serial.print(F(" - "));
    Serial.println(list[i]);
  }
}

static void handleCmd(char c)
{
  switch (c) {
    case '1':
      setModelAndInfo(DFRobot_HumanPose::eGesture);
      break;
    case '2':
      setModelAndInfo(DFRobot_HumanPose::eHand);
      break;
    case '3':
      setModelAndInfo(DFRobot_HumanPose::ePose);
      break;
    case 'k':
      Serial.println(humanPose.setKeypointOutput(false) == DFRobot_HumanPose::eOK ? F("[OK] keypoint OFF") : F("[ERR] keypoint OFF"));
      break;
    case 'K':
      Serial.println(humanPose.setKeypointOutput(true) == DFRobot_HumanPose::eOK ? F("[OK] keypoint ON") : F("[ERR] keypoint ON"));
      break;
    case 'c':
      Serial.println(humanPose.setConfidence(60) == DFRobot_HumanPose::eOK ? F("[OK] confidence=60") : F("[ERR] confidence"));
      break;
    case 'i':
      Serial.println(humanPose.setIOU(45) == DFRobot_HumanPose::eOK ? F("[OK] iou=45") : F("[ERR] iou"));
      break;
    case 's':
      Serial.println(humanPose.setLearnSimilarity(80) == DFRobot_HumanPose::eOK ? F("[OK] similarity=80") : F("[ERR] similarity"));
      break;
    case 'r':
      readConfig();
      break;
    case 'l':
      readLearnList();
      break;
    case 'g':
      runOneResult();
      break;
    case 't':
      g_stream = !g_stream;
      Serial.println(g_stream ? F("[OK] stream ON") : F("[OK] stream OFF"));
      break;
    case 'v':
#if DFR_HUMANPOSE_LOW_MEMORY
      Serial.println(F("[INFO] ASCII keypoint viz unavailable in low-memory profile"));
#else
      g_asciiViz = !g_asciiViz;
      Serial.println(g_asciiViz ? F("[OK] ASCII viz ON") : F("[OK] ASCII viz OFF"));
#endif
      break;
    case 'm':
#if DFR_HUMANPOSE_LOW_MEMORY
      Serial.println(F("[INFO] ASCII mode unavailable in low-memory profile"));
#else
      g_asciiSimple = !g_asciiSimple;
      Serial.println(g_asciiSimple ? F("[OK] ASCII mode -> simple(index)") : F("[OK] ASCII mode -> skeleton"));
#endif
      break;
    case 'h':
      printMenu();
      break;
    default:
      break;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("Booting functional test demo..."));

  while (!humanPose.begin()) {
    Serial.println(F("Sensor init fail, retry in 1s..."));
    delay(1000);
  }
  Serial.println(F("[OK] Sensor init success"));

  setModelAndInfo(DFRobot_HumanPose::eGesture);
  (void)humanPose.setConfidence(60);
  (void)humanPose.setIOU(45);
  (void)humanPose.setLearnSimilarity(80);
  printMenu();
}

void loop()
{
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      continue;
    }
    handleCmd(c);
  }

  if (g_stream) {
    runOneResult();
    delay(120);
  } else {
    delay(20);
  }
}

