/*!
 * @file poseDetectBlink.ino
 * @brief Pose detect blink example
 * @details This example demonstrates how to detect pose and turn on LED indicator when pose is detected
 * @copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @License     The MIT License (MIT)
 * @author [thdyyl](yuanlong.yu@dfrobot.com)
 * @version  V1.0.0
 * @date  2026-04-13
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

/** Print "[BLINK] no learned target..." at streak 1, then every N consecutive frames without id!=0. */
static const uint16_t NO_LEARNED_LOG_INTERVAL = 20;
/**
 * Blink anti-false-trigger policy (demo side, not API side):
 * - LED turns ON after BLINK_ON_CONFIRM_FRAMES consecutive hit frames
 * - LED turns OFF after BLINK_OFF_CONFIRM_FRAMES consecutive non-hit frames
 *
 * This needs only a few bytes of state and keeps raw per-frame results available to user code.
 */
static const uint8_t BLINK_ON_CONFIRM_FRAMES = 2;
static const uint8_t BLINK_OFF_CONFIRM_FRAMES = 2;

static uint8_t saturatingIncU8(uint8_t v)
{
  return (v < 255u) ? (uint8_t)(v + 1u) : 255u;
}

static void printSeparator()
{
  Serial.println(F("----------------------------------------------------------------"));
}

static void printTargetFields(const Result *result)
{
  Serial.print(F("    score: "));
  Serial.println(result->score);
  Serial.print(F("    xLeft: "));
  Serial.println(result->xLeft);
  Serial.print(F("    yTop: "));
  Serial.println(result->yTop);
  Serial.print(F("    width: "));
  Serial.println(result->width);
  Serial.print(F("    height: "));
  Serial.println(result->height);
}

/**
 * @brief Initialize function
 * @details Set up serial communication, initialize sensor, configure detection parameters, get learn list, initialize LED
 */
void setup()
{
  // Initialize serial port for debug output
  Serial.begin(115200);
  delay(3000);
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

  // Configure detection threshold parameters
  humanPose.setIOU(45);                // Set IOU threshold (0-100), used for non-maximum suppression, default is typically 45
  humanPose.setConfidence(60);         // Set detection confidence threshold (0-100), default is typically 60
  humanPose.setLearnSimilarity(80);    // Set similarity threshold (0-100), used for matching learned targets, default is typically 80

  uint8_t iou=0, score=0, similarity=0;
  humanPose.getIOU(&iou);
  humanPose.getConfidence(&score);
  humanPose.getLearnSimilarity(&similarity);
  Serial.print(F("iou: "));
  Serial.print(iou);
  Serial.print(F(", score: "));
  Serial.print(score);
  Serial.print(F(", similarity: "));
  Serial.println(similarity);

  // Get learn list for specified model
  LearnList learnList = humanPose.getLearnList(DFRobot_HumanPose::eHand);

  // Print all learned target names
  Serial.println(F("Learn list:"));
  for (size_t i = 0; i < learnList.size(); ++i) {
    Serial.print(F("id: "));
    Serial.print(i);
    Serial.print(F(", name: "));
    Serial.println(learnList[i].c_str());
  }

  // Initialize built-in LED pin as output
  pinMode(LED_BUILTIN, OUTPUT);
  // Turn off LED initially
  digitalWrite(LED_BUILTIN, LOW);
}

/**
 * @brief Main loop function
 * @details Continuously detect targets, if learned target is detected (target != 0), turn on LED
 */
void loop()
{
  static uint32_t s_frame                   = 0;
  static uint32_t no_learned_target_streak = 0;
  static bool     led_stable_on            = false;
  static uint8_t  hit_streak               = 0;
  static uint8_t  miss_streak              = 0;

  ++s_frame;

  bool hit_this_frame = false;  // Any learned target found in this frame (id != 0)

  // Get detection results
  if (humanPose.getResult() == DFRobot_HumanPose::eOK) {
    bool     printed_header = false;
    uint16_t idx            = 0;
    // Iterate through all detected targets
    while (humanPose.availableResult()) {
#if DFR_HUMANPOSE_LOW_MEMORY
      Result *result = humanPose.popResult();
#else
      HandResult *result = static_cast<HandResult *>(humanPose.popResult());
#endif
      if (!result) {
        continue;
      }
      if (result->id != 0) {
        hit_this_frame = true;
        if (!printed_header) {
          no_learned_target_streak = 0;
          printSeparator();
          Serial.print(F("[BLINK][frame "));
          Serial.print(s_frame);
          Serial.println(F("] learned target(s):"));
          printed_header = true;
        }
        ++idx;
        Serial.print(F("  #"));
        Serial.print(idx);
        Serial.print(F(" id="));
        Serial.print(result->id);
        Serial.print(F(" name="));
        Serial.println(result->name);
        printTargetFields(result);
      }
    }
    if (!printed_header) {
      ++no_learned_target_streak;
      if (no_learned_target_streak == 1u
          || (no_learned_target_streak % NO_LEARNED_LOG_INTERVAL) == 0u) {
        Serial.println(F("[BLINK] no learned target"));
      }
    } else {
      printSeparator();
    }
  } else {
    Serial.println(F("[BLINK] get_result timeout"));
  }

  // Debounce LED state with tiny RAM footprint (no frame array needed).
  // This handles model jitter/hand shake while keeping raw frame data untouched.
  bool prev_led_stable_on = led_stable_on;
  if (hit_this_frame) {
    hit_streak  = saturatingIncU8(hit_streak);
    miss_streak = 0;
    if (!led_stable_on && hit_streak >= BLINK_ON_CONFIRM_FRAMES) {
      led_stable_on = true;
    }
  } else {
    hit_streak  = 0;
    miss_streak = saturatingIncU8(miss_streak);
    if (led_stable_on && miss_streak >= BLINK_OFF_CONFIRM_FRAMES) {
      led_stable_on = false;
    }
  }
  if (prev_led_stable_on != led_stable_on) {
    Serial.print(F("[BLINK] LED -> "));
    Serial.print(led_stable_on ? F("ON") : F("OFF"));
    Serial.print(F(" (hit_streak="));
    Serial.print(hit_streak);
    Serial.print(F(", miss_streak="));
    Serial.print(miss_streak);
    Serial.println(F(")"));
  }

  // Control LED state based on detection results
  digitalWrite(LED_BUILTIN, led_stable_on ? HIGH : LOW);

  // Delay to avoid detection frequency too high
  delay(50);
}
