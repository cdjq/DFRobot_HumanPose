#include <DFRobot_HumanPose.h>

/* >> 1.Please choose your communication method below: */
// #define HUMANPOSE_COMM_UART
#define HUMANPOSE_COMM_I2C

/**
 * I2C address configuration
 * Default I2C address is 0x62
 */
const uint8_t I2C_ADDR = 0x62;

#if defined(HUMANPOSE_COMM_UART)
/* ---------------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  26/D3|     X      |  tx1  |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  25/D2|     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------------------------*/
DFRobot_HumanPose_UART humanPose(&Serial1);
#elif defined(HUMANPOSE_COMM_I2C)
DFRobot_HumanPose_I2C humanPose(&Wire, I2C_ADDR);
#else
#error "Please define HUMANPOSE_COMM_UART or HUMANPOSE_COMM_I2C"
#endif

void setup() {
    Serial.begin(115200);
    while (!humanPose.begin()) {
        Serial.println("Sensor init fail!");
        delay(1000);
    }
    Serial.println("Sensor init success!");
    humanPose.setModel(DFRobot_HumanPose::ePose);
}

void loop() {
    if (humanPose.invoke() == DFRobot_HumanPose::eOK) {
        Serial.println("Invoke success");
        
        // Print keypoints (kps contains both box and points information)
        for (size_t i = 0; i < humanPose.kps().size(); i++) {
            Serial.print("Person ");
            Serial.print(i);
            Serial.print(" - Box: x=");
            Serial.print(humanPose.kps()[i].box.x);
            Serial.print(", y=");
            Serial.print(humanPose.kps()[i].box.y);
            Serial.print(", w=");
            Serial.print(humanPose.kps()[i].box.w);
            Serial.print(", h=");
            Serial.print(humanPose.kps()[i].box.h);
            Serial.print(", score=");
            Serial.print(humanPose.kps()[i].box.score);
            Serial.print(", target=");
            Serial.print(humanPose.kps()[i].box.target);
            Serial.print(" | Keypoints: ");
            Serial.print(humanPose.kps()[i].points.size());
            Serial.println(" points");
            
            // Print each keypoint
            for (size_t j = 0; j < humanPose.kps()[i].points.size(); j++) {
                Serial.print("  Point ");
                Serial.print(j);
                Serial.print(": x=");
                Serial.print(humanPose.kps()[i].points[j].x);
                Serial.print(", y=");
                Serial.print(humanPose.kps()[i].points[j].y);
                Serial.print(", score=");
                Serial.print(humanPose.kps()[i].points[j].score);
                Serial.print(", target=");
                Serial.println(humanPose.kps()[i].points[j].target);
            }
        }
        
        delay(1000);
    }
}