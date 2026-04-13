/*!
 * @file wakeup.ino
 * @brief Wakeup (EN) pin control — with a concrete usage scenario
 * @details
 *   **User scenario (why this pin exists)**\n
 *   Many installations need to cut sensor power or put the module in standby when nobody is
 *   around (save energy on battery, reduce heat, or meet a “night mode” requirement). The EN /
 *   wakeup pin lets the host MCU **hard‑disable** the sensor without unplugging cables.\n
 *   Typical flow: **LOW** = sensor off / not responding on I2C‑UART; **HIGH** = sensor on, then you
 *   run pose/hand/gesture examples or your own code.\n
 *   This sketch **simulates one “close shop → open shop” cycle**: pull EN low, wait, pull EN high.
 *   After that, keep EN high and switch to `getPoseResult` / `getHandResult` / `getGesResult` on the
 *   same wiring (or another MCU that shares GND and respects EN).\n
 * @copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @License     The MIT License (MIT)
 * @author [thdyyl](yuanlong.yu@dfrobot.com)
 * @version  V1.0.1
 * @date  2026-04-13
 * @url         https://github.com/DFRobot/DFRobot_HumanPose
 */

#include <Arduino.h>

/* ---------------------------------------------------------------------------------------------------------------------
 * Hardware connection table (wakeup/EN pin):
 *    Sensor Pin |       MCU Pin        | Leonardo/Mega2560/M0 |   UNO   | ESP8266 |  ESP32  |
 *     VCC       |       3.3V/5V        |         VCC          |   VCC   |   VCC   |   VCC   |
 *     GND       |        GND           |         GND          |   GND   |   GND   |   GND   |
 *     EN/WAKE   |   WAKEUP_PIN (D7)    |    any digital pin   |    7    |   D6    |  IO26   |
 * ----------------------------------------------------------------------------------------------------------------------*/

/** Set to 1: board LED blinks slowly while EN is held HIGH (visual “program running, sensor enabled”). */
#define WAKEUP_DEMO_LED_HEARTBEAT 1

/**
 * @brief GPIO connected to sensor EN / wakeup (output).
 * @note Must match your actual wiring.
 */
const int WAKEUP_PIN = 7;

/** Pause between OFF and ON (ms); increase if your board needs a longer power-down settle time. */
const unsigned long POWER_CYCLE_DELAY_MS = 1000;

// ------------------------ Helper functions ------------------------
void sensorPowerOff()
{
  digitalWrite(WAKEUP_PIN, LOW);
  Serial.println(F("Sensor OFF (wakeup=LOW, sensor disabled)"));
}

void sensorPowerOn()
{
  digitalWrite(WAKEUP_PIN, HIGH);
  Serial.println(F("Sensor ON (wakeup=HIGH, sensor enabled)"));
}

static void printScenario()
{
  Serial.println();
  Serial.println(F("========== Scenario (when to use EN / wakeup) =========="));
  Serial.println(F("Example: kiosk / classroom / battery project — turn the sensor off when idle,"));
  Serial.println(F("turn it on before business hours or when someone approaches. EN=LOW cuts"));
  Serial.println(F("the sensor side; EN=HIGH lets I2C/UART examples talk to the module."));
  Serial.println(F("=========================================================="));
  Serial.println();
  Serial.println(F("[Step 1] Configure EN as OUTPUT, start LOW (sensor off)."));
  Serial.println(F("[Step 2] Wait so the module fully powers down."));
  Serial.println(F("[Step 3] Drive EN HIGH (sensor on) — ready for detection sketches."));
  Serial.println();
  Serial.println(F("Next: upload & run getPoseResult / getHandResult / getGesResult"));
  Serial.println(F("(same GND; keep EN HIGH while using those examples)."));
  Serial.println();
}

// ------------------------ Arduino setup/loop ------------------------
void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  printScenario();

  pinMode(WAKEUP_PIN, OUTPUT);
#if WAKEUP_DEMO_LED_HEARTBEAT
  pinMode(LED_BUILTIN, OUTPUT);
#endif

  sensorPowerOff();
  delay(POWER_CYCLE_DELAY_MS);

  sensorPowerOn();

  Serial.println(F("--- EN is now HIGH: you can open another example to read pose/hand/GES. ---"));
  Serial.println(F("(This sketch keeps EN high; optional LED heartbeat = program alive.)"));
  Serial.println();
}

void loop()
{
#if WAKEUP_DEMO_LED_HEARTBEAT
  static uint32_t last_ms;
  static bool     led_on;
  uint32_t        now = millis();
  if (now - last_ms >= 1500) {
    last_ms   = now;
    led_on    = !led_on;
    digitalWrite(LED_BUILTIN, led_on ? HIGH : LOW);
  }
#else
  delay(1000);
#endif

  /* Optional: periodic power cycling for stress test — uncomment:
  sensorPowerOff();
  delay(POWER_CYCLE_DELAY_MS);
  sensorPowerOn();
  delay(POWER_CYCLE_DELAY_MS);
  */
}
