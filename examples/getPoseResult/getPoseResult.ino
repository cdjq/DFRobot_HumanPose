/**
 * @file getPoseResult.ino
 * @brief Example of getting human pose detection data
 * @details This example demonstrates how to get detection results from the human pose detection sensor and print bounding box and keypoint information
 * @copyright Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
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
 * @details Set up serial communication, initialize sensor, configure detection model
 */
void setup()
{
  // Initialize serial port for debug output
  Serial.begin(115200);

  // Initialize sensor, retry if failed
  while (!humanPose.begin()) {
    Serial.println("Sensor init fail!");
    delay(1000);
  }
  Serial.println("Sensor init success!");

  // Set detection model: eHand (hand detection) or ePose (human pose detection)
  humanPose.setModelType(DFRobot_HumanPose::ePose);
}

/**
 * @brief Main loop function
 * @details Continuously get detection results from sensor and print bounding box and keypoint information
 */
void loop()
{
  // Get detection results
  if (humanPose.getResult() == DFRobot_HumanPose::eOK) {
    Serial.println("getPoseResult success");
    while (humanPose.availableResult()) {
      PoseResult *result = static_cast<PoseResult *>(humanPose.popResult());
      Serial.println("id: " + String(result->id));
      Serial.println("name: " + result->name);
      /**
             * @brief Score of the result (0–100).
             *
             * Meaning depends on `id`:
             * - if `id == 0`: `score` is the detection confidence (probability/quality of detection).
             * - if `id != 0`: `score` is the similarity score (match degree to a learned class/gesture/pose).
             */
      Serial.println("score: " + String(result->score));
      Serial.println("xLeft: " + String(result->xLeft));
      Serial.println("yTop: " + String(result->yTop));
      Serial.println("width: " + String(result->width));
      Serial.println("height: " + String(result->height));
      Serial.println("nose: " + String(result->nose.x) + ", " + String(result->nose.y));
      Serial.println("leye: " + String(result->leye.x) + ", " + String(result->leye.y));
      Serial.println("reye: " + String(result->reye.x) + ", " + String(result->reye.y));
      Serial.println("lear: " + String(result->lear.x) + ", " + String(result->lear.y));
      Serial.println("rear: " + String(result->rear.x) + ", " + String(result->rear.y));
      Serial.println("lshoulder: " + String(result->lshoulder.x) + ", " + String(result->lshoulder.y));
      Serial.println("rshoulder: " + String(result->rshoulder.x) + ", " + String(result->rshoulder.y));
      Serial.println("lelbow: " + String(result->lelbow.x) + ", " + String(result->lelbow.y));
      Serial.println("relbow: " + String(result->relbow.x) + ", " + String(result->relbow.y));
      Serial.println("lwrist: " + String(result->lwrist.x) + ", " + String(result->lwrist.y));
      Serial.println("rwrist: " + String(result->rwrist.x) + ", " + String(result->rwrist.y));
      Serial.println("lhip: " + String(result->lhip.x) + ", " + String(result->lhip.y));
      Serial.println("rhip: " + String(result->rhip.x) + ", " + String(result->rhip.y));
      Serial.println("lknee: " + String(result->lknee.x) + ", " + String(result->lknee.y));
      Serial.println("rknee: " + String(result->rknee.x) + ", " + String(result->rknee.y));
      Serial.println("lankle: " + String(result->lankle.x) + ", " + String(result->lankle.y));
      Serial.println("rankle: " + String(result->rankle.x) + ", " + String(result->rankle.y));
      Serial.println("--------------------------------");
    }
  } else {
    Serial.println("getResult fail");
  }

  // Delay to avoid output too fast
  delay(100);
}
