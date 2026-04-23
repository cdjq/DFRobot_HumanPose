/*!
 * @file unitTestDemo.ino
 * @brief Integration-style unit test demo for DFRobot_HumanPose.
 * @details Runs a grouped assertion suite on real hardware (sensor required).
 */

#include <DFRobot_HumanPose.h>

/* ======================== Communication Selection ======================== */
// #define HUMANPOSE_COMM_UART
#define HUMANPOSE_COMM_I2C

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

/* ======================== Tiny Test Framework ======================== */
static uint32_t g_total = 0;
static uint32_t g_pass = 0;
static uint32_t g_fail = 0;
static bool     g_allDone = false;

static void testLine(bool cond, const __FlashStringHelper *msg)
{
  ++g_total;
  if (cond) {
    ++g_pass;
    Serial.print(F("[PASS] "));
  } else {
    ++g_fail;
    Serial.print(F("[FAIL] "));
  }
  Serial.println(msg);
}

#define EXPECT_TRUE(cond, msg) testLine((cond), F(msg))
#define EXPECT_EQ_U8(a, b, msg) testLine(((uint8_t)(a) == (uint8_t)(b)), F(msg))
#define EXPECT_EQ_I(a, b, msg) testLine(((int)(a) == (int)(b)), F(msg))

static void printSummary()
{
  Serial.println();
  Serial.println(F("========== TEST SUMMARY =========="));
  Serial.print(F("TOTAL: "));
  Serial.println(g_total);
  Serial.print(F("PASS : "));
  Serial.println(g_pass);
  Serial.print(F("FAIL : "));
  Serial.println(g_fail);
  Serial.println(F("=================================="));
}

/* ======================== Test Cases ======================== */
static void testInvalidArgs()
{
  Serial.println(F("\n[CASE] Invalid argument checks"));
  EXPECT_EQ_I(humanPose.setConfidence(101), DFRobot_HumanPose::eINVAL, "setConfidence(101) should be eINVAL");
  EXPECT_EQ_I(humanPose.setIOU(101), DFRobot_HumanPose::eINVAL, "setIOU(101) should be eINVAL");
  EXPECT_EQ_I(humanPose.setLearnSimilarity(101), DFRobot_HumanPose::eINVAL, "setLearnSimilarity(101) should be eINVAL");
  EXPECT_EQ_I(humanPose.getConfidence(NULL), DFRobot_HumanPose::eINVAL, "getConfidence(NULL) should be eINVAL");
  EXPECT_EQ_I(humanPose.getIOU(NULL), DFRobot_HumanPose::eINVAL, "getIOU(NULL) should be eINVAL");
  EXPECT_EQ_I(humanPose.getLearnSimilarity(NULL), DFRobot_HumanPose::eINVAL, "getLearnSimilarity(NULL) should be eINVAL");
  EXPECT_EQ_I(humanPose.getKeypointOutput(NULL), DFRobot_HumanPose::eINVAL, "getKeypointOutput(NULL) should be eINVAL");
}

static void testThresholdRoundTrip()
{
  Serial.println(F("\n[CASE] Threshold round trip"));
  uint8_t v = 0;

  EXPECT_EQ_I(humanPose.setConfidence(60), DFRobot_HumanPose::eOK, "setConfidence(60)");
  EXPECT_EQ_I(humanPose.getConfidence(&v), DFRobot_HumanPose::eOK, "getConfidence()");
  EXPECT_EQ_U8(v, 60, "confidence should be 60");

  EXPECT_EQ_I(humanPose.setIOU(45), DFRobot_HumanPose::eOK, "setIOU(45)");
  EXPECT_EQ_I(humanPose.getIOU(&v), DFRobot_HumanPose::eOK, "getIOU()");
  EXPECT_EQ_U8(v, 45, "IOU should be 45");

  EXPECT_EQ_I(humanPose.setLearnSimilarity(80), DFRobot_HumanPose::eOK, "setLearnSimilarity(80)");
  EXPECT_EQ_I(humanPose.getLearnSimilarity(&v), DFRobot_HumanPose::eOK, "getLearnSimilarity()");
  EXPECT_EQ_U8(v, 80, "similarity should be 80");
}

static void testKeypointOutput()
{
  Serial.println(F("\n[CASE] Keypoint output"));
  uint8_t mode = 0;

  EXPECT_EQ_I(humanPose.setKeypointOutput(false), DFRobot_HumanPose::eOK, "setKeypointOutput(false)");
  EXPECT_EQ_I(humanPose.getKeypointOutput(&mode), DFRobot_HumanPose::eOK, "getKeypointOutput() after false");
  EXPECT_EQ_U8(mode, 0, "keypoint mode should be 0");

#if DFR_HUMANPOSE_LOW_MEMORY
  Serial.println(F("[INFO] Low-memory profile: keeping keypoint mode at 0."));
#else
  EXPECT_EQ_I(humanPose.setKeypointOutput(true), DFRobot_HumanPose::eOK, "setKeypointOutput(true)");
  EXPECT_EQ_I(humanPose.getKeypointOutput(&mode), DFRobot_HumanPose::eOK, "getKeypointOutput() after true");
  EXPECT_EQ_U8(mode, 1, "keypoint mode should be 1");
#endif
}

static void testModelAndLearnList()
{
  Serial.println(F("\n[CASE] Model switching and learn list"));

  EXPECT_EQ_I(humanPose.setModelType((DFRobot_HumanPose::eModel_t)2), DFRobot_HumanPose::eINVAL, "setModelType(invalid)");

  EXPECT_EQ_I(humanPose.setModelType(DFRobot_HumanPose::eHand), DFRobot_HumanPose::eOK, "setModelType(eHand)");
  {
    LearnList handList = humanPose.getLearnList(DFRobot_HumanPose::eHand);
    EXPECT_TRUE(handList.size() <= LearnList::CAP, "hand learn list size within cap");
  }

  EXPECT_EQ_I(humanPose.setModelType(DFRobot_HumanPose::ePose), DFRobot_HumanPose::eOK, "setModelType(ePose)");
  {
    LearnList poseList = humanPose.getLearnList(DFRobot_HumanPose::ePose);
    EXPECT_TRUE(poseList.size() <= LearnList::CAP, "pose learn list size within cap");
  }

  EXPECT_EQ_I(humanPose.setModelType(DFRobot_HumanPose::eGesture), DFRobot_HumanPose::eOK, "setModelType(eGesture)");
  {
    LearnList gesList = humanPose.getLearnList(DFRobot_HumanPose::eGesture);
    EXPECT_EQ_I((int)gesList.size(), 0, "gesture learn list should be empty");
  }
}

static void testResultLifecycleOnce(DFRobot_HumanPose::eModel_t model, const __FlashStringHelper *tag)
{
  Serial.print(F("\n[CASE] getResult lifecycle "));
  Serial.println(tag);

  EXPECT_EQ_I(humanPose.setModelType(model), DFRobot_HumanPose::eOK, "setModelType before getResult");

  const int ret = humanPose.getResult();
  EXPECT_EQ_I(ret, DFRobot_HumanPose::eOK, "getResult should return eOK");
  if (ret != DFRobot_HumanPose::eOK) {
    return;
  }

  uint16_t cnt = 0;
  while (humanPose.availableResult()) {
    Result *r = humanPose.popResult();
    EXPECT_TRUE(r != NULL, "popResult should return non-null when availableResult is true");
    if (!r) {
      break;
    }
    EXPECT_TRUE(r->used, "returned result should be marked used");
    EXPECT_TRUE(r->score <= 100, "score in [0,100]");
    ++cnt;
  }
  EXPECT_TRUE(cnt <= MAX_RESULT_NUM, "result count should not exceed MAX_RESULT_NUM");
  EXPECT_TRUE(!humanPose.availableResult(), "no unread result after draining");
  EXPECT_TRUE(humanPose.popResult() == NULL, "popResult should return NULL when empty");
}

static void runAllTests()
{
  Serial.println(F("\n========== DFRobot_HumanPose Test Demo =========="));
  Serial.println(F("This is an integration-style unit test on real sensor."));
  Serial.println(F("Expected: Sensor powered, wired correctly, and initialized."));

  testInvalidArgs();
  testThresholdRoundTrip();
  testKeypointOutput();
  testModelAndLearnList();

  testResultLifecycleOnce(DFRobot_HumanPose::eGesture, F("(gesture)"));
  testResultLifecycleOnce(DFRobot_HumanPose::eHand, F("(hand)"));
  testResultLifecycleOnce(DFRobot_HumanPose::ePose, F("(pose)"));

  printSummary();
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("Booting test demo..."));
  while (!humanPose.begin()) {
    Serial.println(F("Sensor init fail, retry in 1s..."));
    delay(1000);
  }
  Serial.println(F("Sensor init success."));

  runAllTests();
  g_allDone = true;
}

void loop()
{
  if (!g_allDone) {
    return;
  }
  delay(1000);
}

