/**
 * @file setBaud.ino
 * @brief Set UART baud rate example
 * @details This example demonstrates how to configure and modify the sensor's UART communication baud rate
 * @copyright Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author DFRobot
 * @version V1.0.0
 * @date 2025-01-01
 */

#include <DFRobot_HumanPose.h>

/* >> Step 1: Please choose your communication method below */
#define HUMANPOSE_COMM_UART  // Use UART communication
// #define HUMANPOSE_COMM_I2C  // Use I2C communication (I2C does not support baud rate setting)

/**
 * I2C address configuration
 * Default I2C address is 0x3A
 */
const uint8_t I2C_ADDR = 0x3A;

#if defined(HUMANPOSE_COMM_UART)
/* ---------------------------------------------------------------------------------------------------------------------
 * Hardware connection table:
 *    Sensor Pin |        MCU Pin        | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *     VCC       |        3.3V/5V         |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *     GND       |         GND            |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 *     RX        |        MCU TX          |     Serial1 TX1      |     5     |   5/D6  |  26/D3|     X      |  tx1  |
 *     TX        |        MCU RX          |     Serial1 RX1      |     4     |   4/D7  |  25/D2|     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------------------------*/
// Initialize UART communication: Serial1, baud rate 921600 (initial baud rate), RX pin 25, TX pin 26 (ESP32)
DFRobot_HumanPose_UART humanPose(&Serial1, 921600, 25, 26);
#elif defined(HUMANPOSE_COMM_I2C)
// Initialize I2C communication: Wire, I2C address 0x3A
DFRobot_HumanPose_I2C humanPose(&Wire, I2C_ADDR);
#else
#error "Please define HUMANPOSE_COMM_UART or HUMANPOSE_COMM_I2C"
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
    
#if defined(HUMANPOSE_COMM_UART)
    // Initialize sensor (using initial baud rate 921600)
    Serial.println("Initializing sensor with initial baud rate 921600...");
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
    
    // Note: After modifying baud rate, you need to re-initialize serial port and sensor object to take effect
    // Example code:
    /*
    Serial1.begin(115200, SERIAL_8N1, 25, 26);
    DFRobot_HumanPose_UART humanPose(&Serial1, 115200, 25, 26);
    if (humanPose.begin()) {
        Serial.println("Re-initialized successfully with new baud rate!");
    }
    */
    
#else
    Serial.println("Note: I2C communication method does not support baud rate setting");
    Serial.println("Please use UART communication method to test baud rate function");
    
    // I2C initialization
    while (!humanPose.begin()) {
        Serial.println("Sensor init fail!");
        delay(1000);
    }
    Serial.println("Sensor init success!");
#endif
    
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
