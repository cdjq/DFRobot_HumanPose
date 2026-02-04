/**
 * @file wakeup.ino
 * @brief Wakeup pin control example
 * @details This example demonstrates how to control the HumanPose sensor wakeup (enable) pin
 *          using a digital GPIO. LOW level = power off, HIGH level = power on.
 *          After powering on the sensor, you can run other examples (getPoseResult / getHandResult)
 *          to get detection results.
 * @copyright Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author DFRobot
 * @version V1.0.0
 * @date 2026-02-04
 */

#include <Arduino.h>

/* ---------------------------------------------------------------------------------------------------------------------
 * Hardware connection table (wakeup/EN pin):
 *    Sensor Pin |       MCU Pin        | Leonardo/Mega2560/M0 |   UNO   | ESP8266 |  ESP32  |
 *     VCC       |       3.3V/5V        |         VCC          |   VCC   |   VCC   |   VCC   |
 *     GND       |        GND           |         GND          |   GND   |   GND   |   GND   |
 *     EN/WAKE   |   WAKEUP_PIN (D7)    |    any digital pin   |    7    |   D6    |  IO26   |
 * ----------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief GPIO pin used to control the sensor wakeup (enable) pin.
 * @note Change this to the digital pin that you actually connected to the sensor wakeup/EN pin.
 */
const int WAKEUP_PIN = 7;

/**
 * @brief Delay time between power off and power on, in milliseconds.
 */
const unsigned long POWER_CYCLE_DELAY_MS = 1000;

// ------------------------ Helper functions ------------------------
/**
 * @brief Power off the HumanPose sensor.
 * @details LOW level on wakeup pin means disconnect / power off / disable the sensor.
 */
void sensorPowerOff()
{
  digitalWrite(WAKEUP_PIN, LOW);
  Serial.println(F("Sensor OFF (wakeup=LOW, sensor disabled)"));
}

/**
 * @brief Power on the HumanPose sensor.
 * @details HIGH level on wakeup pin means connect / power on / enable the sensor.
 */
void sensorPowerOn()
{
  digitalWrite(WAKEUP_PIN, HIGH);
  Serial.println(F("Sensor ON (wakeup=HIGH, sensor enabled)"));
}

// ------------------------ Arduino setup/loop ------------------------
/**
 * @brief Initialize serial port and wakeup pin.
 * @details Configure wakeup pin as output, power off then power on the sensor.
 */
void setup()
{
  // Initialize serial for debug output
  Serial.begin(115200);
  while (!Serial) {
    ;    // wait for serial port to connect. Needed for some boards (e.g. Leonardo)
  }

  Serial.println(F("DFRobot HumanPose wakeup demo"));

  // Configure wakeup pin as output and default to LOW (sensor off)
  pinMode(WAKEUP_PIN, OUTPUT);
  sensorPowerOff();
  delay(POWER_CYCLE_DELAY_MS);

  // Then power on the sensor
  sensorPowerOn();
  Serial.println(F("You can now run pose/hand result examples to read data."));
}

/**
 * @brief Main loop (optional additional power cycling demo).
 * @details By default, it keeps the sensor powered ON. If you want to
 *          periodically power-cycle the sensor, you can uncomment the code below.
 */
void loop()
{
  // Keep sensor ON and do nothing here.
  // If you want to periodically toggle power, uncomment this block:
  /*
  sensorPowerOff();
  delay(POWER_CYCLE_DELAY_MS);
  sensorPowerOn();
  delay(POWER_CYCLE_DELAY_MS);
  */
}
