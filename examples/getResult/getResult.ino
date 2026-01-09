/**
 * @file getResult.ino
 * @brief Example of getting human pose detection data
 * @details This example demonstrates how to get detection results from the human pose detection sensor and print bounding box and keypoint information
 * @copyright Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author DFRobot
 * @version V1.0.0
 * @date 2026-01-09
 */

#include <DFRobot_HumanPose.h>

/* >> Step 1: Please choose your communication method below */
#define HUMANPOSE_COMM_UART  // Use UART communication
// #define HUMANPOSE_COMM_I2C  // Use I2C communication



#if defined(HUMANPOSE_COMM_UART)
/* ---------------------------------------------------------------------------------------------------------------------
 * Hardware connection table:
 *    Sensor Pin |        MCU Pin        | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *     VCC       |        3.3V/5V         |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *     GND       |         GND            |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 *     RX        |        MCU TX          |     Serial1 TX1      |     5     |   5/D6  |  26/D3|     X      |  tx1  |
 *     TX        |        MCU RX          |     Serial1 RX1      |     4     |   4/D7  |  25/D2|     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------------------------*/
// Initialize UART communication: Serial1, baud rate 921600, RX pin 25, TX pin 26 (ESP32)
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include <SoftwareSerial.h>
SoftwareSerial mySerial(4, 5);
DFRobot_HumanPose_UART humanPose(&mySerial, 921600);
#elif defined(ESP32)
DFRobot_HumanPose_UART humanPose(&Serial1, 921600, /*RX pin*/25, /*TX pin*/26);
#else
DFRobot_HumanPose_UART humanPose(&Serial1, 921600);
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
void setup() {
    // Initialize serial port for debug output
    Serial.begin(115200);
    
    // Initialize sensor, retry if failed
    while (!humanPose.begin()) {
        Serial.println("Sensor init fail!");
        delay(1000);
    }
    Serial.println("Sensor init success!");
    
    // Set detection model: eHand (hand detection) or ePose (human pose detection)
    humanPose.setModel(DFRobot_HumanPose::eHand);
}

/**
 * @brief Main loop function
 * @details Continuously get detection results from sensor and print bounding box and keypoint information
 */
void loop() {
    // Get detection results
    if (humanPose.getResult() == DFRobot_HumanPose::eOK) {
        Serial.println("getResult success");
        
        // Iterate through all detected targets (each target contains a bounding box and multiple keypoints)
        for (size_t i = 0; i < humanPose.keypoints().size(); i++) {
            // Print bounding box information
            Serial.print("Box ");
            Serial.print(i);
            Serial.print(" -: x=");
            Serial.print(humanPose.keypoints()[i].box.x);      // Bounding box top-left X coordinate
            Serial.print(", y=");
            Serial.print(humanPose.keypoints()[i].box.y);      // Bounding box top-left Y coordinate
            Serial.print(", w=");
            Serial.print(humanPose.keypoints()[i].box.w);      // Bounding box width
            Serial.print(", h=");
            Serial.print(humanPose.keypoints()[i].box.h);      // Bounding box height
            Serial.print(", score=");
            Serial.print(humanPose.keypoints()[i].box.score);  // Detection confidence (0-100)
            Serial.print(", target=");
            Serial.print(humanPose.keypoints()[i].box.target); // Target identifier (0 means unknown, non-zero means learned target)
            Serial.print(" | Keypoints: ");
            Serial.print(humanPose.keypoints()[i].points.size());
            Serial.println(" points");
            
            // Iterate and print all keypoints for this target
            for (size_t j = 0; j < humanPose.keypoints()[i].points.size(); j++) {
                Serial.print("  Point ");
                Serial.print(j);
                Serial.print(": x=");
                Serial.print(humanPose.keypoints()[i].points[j].x);      // Keypoint X coordinate
                Serial.print(", y=");
                Serial.print(humanPose.keypoints()[i].points[j].y);      // Keypoint Y coordinate
                Serial.print(", score=");
                Serial.print(humanPose.keypoints()[i].points[j].score);  // Keypoint confidence
                Serial.print(", target=");
                Serial.println(humanPose.keypoints()[i].points[j].target); // Keypoint target identifier
            }
        }
    } else {
        Serial.println("getResult fail");
    }
    
    // Delay to avoid output too fast
    delay(100);
}
