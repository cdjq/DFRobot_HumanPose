/**
 * @file setBaud.ino
 * @brief Set UART baud rate example
 * @details This example demonstrates how to configure and modify the sensor's UART communication baud rate
 * @copyright Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author DFRobot
 * @version V1.0.0
 * @date 2026-01-09
 */

#include <DFRobot_HumanPose.h>



/* ---------------------------------------------------------------------------------------------------------------------
 * Hardware connection table:
 *    Sensor Pin |        MCU Pin        | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *     VCC       |        3.3V/5V         |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *     GND       |         GND            |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 *     RX        |        MCU TX          |     Serial1 TX1      |     5     |   5/D6  |  26/D3|     X      |  tx1  |
 *     TX        |        MCU RX          |     Serial1 RX1      |     4     |   4/D7  |  25/D2|     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------------------------*/
// Initialize UART communication: Serial1, baud rate 9600 (initial baud rate), RX pin 25, TX pin 26 (ESP32)
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include <SoftwareSerial.h>
SoftwareSerial mySerial(4, 5);
DFRobot_HumanPose_UART humanPose(&mySerial, 9600);
#elif defined(ESP32)
DFRobot_HumanPose_UART humanPose(&Serial1, 9600, /*RX pin*/25, /*TX pin*/26);
#else
DFRobot_HumanPose_UART humanPose(&Serial1, 9600);
#endif

/**
 * @brief Initialize function
 * @details Set up serial communication, initialize sensor, demonstrate baud rate configuration
 */
void setup() {
    // Initialize serial port for debug output
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("========== DFRobot HumanPose Baud Rate Setting Example ==========");
    
    // Initialize sensor (using initial baud rate 9600)
    Serial.println("Initializing sensor with initial baud rate 9600...");
    while (!humanPose.begin()) {
        Serial.println("Sensor init fail!");
        delay(1000);
    }
    Serial.println("Sensor init success!");
    
    // Test setting different baud rates
    Serial.println("\nStart testing baud rate setting...");
    
    // Available baud rate options:
    // eBaud_9600, eBaud_14400, eBaud_19200, eBaud_38400, eBaud_57600,
    // eBaud_115200, eBaud_230400, eBaud_460800, eBaud_921600
    
    // Example: Set baud rate to 115200
    Serial.println("\nSetting baud rate to 115200...");
    if (humanPose.setBaud(DFRobot_HumanPose_UART::eBaud_115200)) {
        Serial.println("Baud rate set successfully!");
        Serial.println("Note: If baud rate is modified, you need to re-initialize serial port and sensor object");
        Serial.println("And ensure the MCU's serial port baud rate matches the sensor baud rate");
    } else {
        Serial.println("Baud rate set failed!");
    }
    
    Serial.println("\nExample program completed");
}

/**
 * @brief Main loop function
 * @details Keep program running
 */
void loop() {
    // This example is mainly for demonstrating baud rate setting, main loop can be empty
    // Or you can add other functional code here
    
    delay(1000);
}
