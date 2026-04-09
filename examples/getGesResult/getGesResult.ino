/*!
 * @file getGesResult.ino
 * @brief Example of getting GES (fixed gesture classification) data
 * @details This example demonstrates how to get detection results from the GES model (MODEL 4): bounding box and
 *          class name (fixed labels, id 0..14). No keypoints and no user learn list on the device.
 * @copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @License     The MIT License (MIT)
 * @author [thdyyl](yuanlong.yu@dfrobot.com)
 * @version  V1.0.0
 * @date  2026-04-08
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

void setup()
{
  Serial.begin(115200);

  while (!humanPose.begin()) {
    Serial.println(F("Sensor init fail!"));
    delay(1000);
  }
  Serial.println(F("Sensor init success!"));

  // Set detection model: eGes (MODEL 4) — fixed gesture classification
  humanPose.setModelType(DFRobot_HumanPose::eGes);
#if DFR_HUMANPOSE_LOW_MEMORY
  humanPose.setKeypointOutput(false);
#endif
}

void loop()
{
  // Get detection results (Result: bbox + score + id/name; no keypoints for GES)
  if (humanPose.getResult() == DFRobot_HumanPose::eOK) {
    Serial.println(F("getGesResult success"));
    while (humanPose.availableResult()) {
      Result *result = humanPose.popResult();
      if (!result) {
        continue;
      }
      Serial.print(F("id: "));
      Serial.println(result->id);
      Serial.print(F("name: "));
      Serial.println(result->name);
      Serial.print(F("score: "));
      Serial.println(result->score);
      Serial.print(F("xLeft: "));
      Serial.println(result->xLeft);
      Serial.print(F("yTop: "));
      Serial.println(result->yTop);
      Serial.print(F("width: "));
      Serial.println(result->width);
      Serial.print(F("height: "));
      Serial.println(result->height);
      Serial.println(F("--------------------------------"));
    }
  } else {
    Serial.println(F("getResult fail"));
  }

  delay(100);
}
