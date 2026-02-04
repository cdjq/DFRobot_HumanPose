/**
 * @file poseDetectBlink.ino
 * @brief Pose detect blink example
 * @details This example demonstrates how to detect pose and turn on LED indicator when pose is detected
 * @copyright Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author DFRobot
 * @version V1.0.0
 * @date 2026-01-09
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
    Serial.println("Sensor init fail!");
    delay(1000);
  }
  Serial.println("Sensor init success!");

  // Set detection model: eHand (hand detection) or ePose (human pose detection)
  humanPose.setModelType(DFRobot_HumanPose::eHand);

  // Configure detection threshold parameters
  humanPose.setIOU(45);                // Set IOU threshold (0-100), used for non-maximum suppression, default is typically 45
  humanPose.setConfidence(80);         // Set detection confidence threshold (0-100), default is typically 60
  humanPose.setLearnSimilarity(60);    // Set similarity threshold (0-100), used for matching learned targets, default is typically 60

  uint8_t iou, score, similarity;
  humanPose.getIOU(&iou);
  humanPose.getConfidence(&score);
  humanPose.getLearnSimilarity(&similarity);
  char buf[64];
  sprintf(buf, "iou: %d, score: %d, similarity: %d", iou, score, similarity);
  Serial.println(buf);

  // Get learn list for specified model
  LearnList learnList = humanPose.getLearnList(DFRobot_HumanPose::eHand);

  // Print all learned target names
  Serial.println("Learn list:");
  for (size_t i = 0; i < learnList.size(); ++i) {
    Serial.print("id: ");
    Serial.print(i);
    Serial.print(", name: ");
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
  uint8_t led_val = LOW;    // LED state, default is off

  // Get detection results
  if (humanPose.getResult() == DFRobot_HumanPose::eOK) {
    // Iterate through all detected targets
    while (humanPose.availableResult()) {
      HandResult *result = static_cast<HandResult *>(humanPose.popResult());
      if (result->id != 0) {
        Serial.print("ID: ");
        Serial.println(result->id);
        Serial.print("Name: ");
        Serial.println(result->name);
        led_val = HIGH;
      }
    }
  }

  // Control LED state based on detection results
  // led_val == HIGH: Learned target detected, LED on
  // led_val == LOW: No learned target detected or detection failed, LED off
  digitalWrite(LED_BUILTIN, led_val);

  // Delay to avoid detection frequency too high
  delay(50);
}
