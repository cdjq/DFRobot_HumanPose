/**
 * @file getHandResult.ino
 * @brief Example of getting human hand detection data
 * @details This example demonstrates how to get detection results from the human hand detection sensor and print bounding box and keypoint information
 * @copyright Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author DFRobot
 * @version V1.0.0
 * @date 2026-01-09
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
  humanPose.setModelType(DFRobot_HumanPose::eHand);
}

/**
  * @brief Main loop function
  * @details Continuously get detection results from sensor and print bounding box and keypoint information
  */
void loop()
{
  // Get detection results
  if (humanPose.getResult() == DFRobot_HumanPose::eOK) {
    Serial.println("getHandResult success");
    while (humanPose.availableResult()) {
      HandResult *result = static_cast<HandResult *>(humanPose.popResult());
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
      Serial.println("wrist: " + String(result->wrist.x) + ", " + String(result->wrist.y));
      Serial.println("thumbCmc: " + String(result->thumbCmc.x) + ", " + String(result->thumbCmc.y));
      Serial.println("thumbMcp: " + String(result->thumbMcp.x) + ", " + String(result->thumbMcp.y));
      Serial.println("thumbIp: " + String(result->thumbIp.x) + ", " + String(result->thumbIp.y));
      Serial.println("thumbTip: " + String(result->thumbTip.x) + ", " + String(result->thumbTip.y));
      Serial.println("indexFingerMcp: " + String(result->indexFingerMcp.x) + ", " + String(result->indexFingerMcp.y));
      Serial.println("indexFingerPip: " + String(result->indexFingerPip.x) + ", " + String(result->indexFingerPip.y));
      Serial.println("indexFingerDip: " + String(result->indexFingerDip.x) + ", " + String(result->indexFingerDip.y));
      Serial.println("indexFingerTip: " + String(result->indexFingerTip.x) + ", " + String(result->indexFingerTip.y));
      Serial.println("middleFingerMcp: " + String(result->middleFingerMcp.x) + ", " + String(result->middleFingerMcp.y));
      Serial.println("middleFingerPip: " + String(result->middleFingerPip.x) + ", " + String(result->middleFingerPip.y));
      Serial.println("middleFingerDip: " + String(result->middleFingerDip.x) + ", " + String(result->middleFingerDip.y));
      Serial.println("middleFingerTip: " + String(result->middleFingerTip.x) + ", " + String(result->middleFingerTip.y));
      Serial.println("ringFingerMcp: " + String(result->ringFingerMcp.x) + ", " + String(result->ringFingerMcp.y));
      Serial.println("ringFingerPip: " + String(result->ringFingerPip.x) + ", " + String(result->ringFingerPip.y));
      Serial.println("ringFingerDip: " + String(result->ringFingerDip.x) + ", " + String(result->ringFingerDip.y));
      Serial.println("ringFingerTip: " + String(result->ringFingerTip.x) + ", " + String(result->ringFingerTip.y));
      Serial.println("pinkyFingerMcp: " + String(result->pinkyFingerMcp.x) + ", " + String(result->pinkyFingerMcp.y));
      Serial.println("pinkyFingerPip: " + String(result->pinkyFingerPip.x) + ", " + String(result->pinkyFingerPip.y));
      Serial.println("pinkyFingerDip: " + String(result->pinkyFingerDip.x) + ", " + String(result->pinkyFingerDip.y));
      Serial.println("pinkyFingerTip: " + String(result->pinkyFingerTip.x) + ", " + String(result->pinkyFingerTip.y));
      Serial.println("--------------------------------");
    }
  } else {
    Serial.println("getResult fail");
  }

  // Delay to avoid output too fast
  delay(50);
}
