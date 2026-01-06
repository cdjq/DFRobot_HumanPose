/**
 * @file  DFRobot_HumanPose.h
 * @brief  Define the infrastructure of DFRobot_HumanPose class
 * @n      This is a human pose detection sensor that can be controlled through I2C/UART ports.
 * @n      EPII_CM55M_APP_S has functions such as human pose detection, hand detection, etc.
 * @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      DFRobot
 * @version     V1.0.0
 * @date        2025-01-01
 * @url         https://github.com/DFRobot/DFRobot_HumanPose
 */
#ifndef DFRobot_HumanPose_H
#define DFRobot_HumanPose_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <vector>
#include <functional>
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>


#ifdef ENABLE_DBG
#define LDBG(...)  {Serial.print("["); Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define LDBG(...)
#endif

class DFRobot_HumanPose {
protected:

#define I2C_ADDRESS (0x3A)

#define HEADER_LEN (uint8_t)4
#define MAX_PL_LEN (uint8_t)250
#define MAX_SPI_PL_LEN (uint16_t)4095
#define CHECKSUM_LEN (uint8_t)2

#define PACKET_SIZE (uint16_t)(HEADER_LEN + MAX_PL_LEN + CHECKSUM_LEN)

#define FEATURE_TRANSPORT 0x10
#define FEATURE_TRANSPORT_CMD_READ 0x01
#define FEATURE_TRANSPORT_CMD_WRITE 0x02
#define FEATURE_TRANSPORT_CMD_AVAILABLE 0x03
#define FEATURE_TRANSPORT_CMD_RESET 0x06

#ifndef RX_MAX_SIZE
#ifdef ARDUINO_ARCH_ESP32
#define RX_MAX_SIZE 32 * 1024
#else
#define RX_MAX_SIZE 4 * 1024
#endif
#endif

#ifndef TX_MAX_SIZE
#define TX_MAX_SIZE 4 * 1024
#endif

#ifndef I2C_CLOCK
#define I2C_CLOCK 400000
#endif

#ifndef UART_BAUD
#define UART_BAUD 921600
#endif

#define RES_PRE "\r{"
#define RES_SUF "}\n"

#define RES_PRE_LEN (sizeof(RES_PRE) - 1)
#define RES_SUF_LEN (sizeof(RES_SUF) - 1)

#define CMD_PRE "AT+"
#define CMD_SUF "\r\n"

#define CMD_TYPE_RESPONSE 0
#define CMD_TYPE_EVENT 1
#define CMD_TYPE_LOG 2

static constexpr const char AT_ID[] = "ID?";
static constexpr const char AT_NAME[] = "NAME?";
static constexpr const char AT_VERSION[] = "VER?";
static constexpr const char AT_INVOKE[] = "INVOKE";
static constexpr const char AT_TSCORE[] = "TSCORE";
static constexpr const char AT_TIOU[] = "TIOU";
static constexpr const char AT_MODELS[] = "MODELS?";
static constexpr const char AT_MODEL[] = "MODEL";
static constexpr const char EVENT_INVOKE[] = "INVOKE";

public:

typedef enum {
    eOK = 0,
    eAgain = 1,
    eLog = 2,
    eTimedOut = 3,
    eIO = 4,
    eINVAL = 5,
    eNOMEM = 6,
    eBUSY = 7,
    eNOTSUP = 8,
    ePERM = 9,
    eUnknown = 10,
} eCmdCode_t;

typedef enum {
    eHand = 1,
    ePose = 3,
} eModel_t;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    uint8_t score;
    uint8_t target;
} sBox_t;

typedef struct
{
    uint8_t target;
    uint8_t score;
} sClass_t;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint8_t score;
    uint8_t target;
} sPoint_t;

typedef struct
{
    sBox_t box;
    std::vector<sPoint_t> points;
} sKps_t;

protected:
    int _wait_delay;
    uint32_t rx_end;
    std::vector<sBox_t> _boxes;
    std::vector<sClass_t> _classes;
    std::vector<sPoint_t> _points;
    std::vector<sKps_t> _kps;
    #if ARDUINOJSON_VERSION_MAJOR == 7
        JsonDocument response; // for json response
    #else
        StaticJsonDocument<2048> response; // for json response
    #endif
    char *tx_buf;
    uint32_t tx_len;
    char *rx_buf;
    uint32_t rx_len;
    char *payload;
    
    // Command processing helper functions
    int wait(int type, const char *cmd, uint32_t timeout = 1000);
    void parser_event();
    
public:
    DFRobot_HumanPose();
    ~DFRobot_HumanPose();
    bool begin();
    eCmdCode_t getResult();
    virtual int available() = 0;
    virtual int read(char *data, int len) = 0;
    virtual int write(const char *data, int len) = 0;

    eCmdCode_t setTScore(uint8_t tscore);
    eCmdCode_t setTIOU(uint8_t tious);
    eCmdCode_t setModel(eModel_t model);

    eCmdCode_t getTScore();
    eCmdCode_t getTIOU();
    eCmdCode_t getModel();

    std::vector<sBox_t> &boxes() { return _boxes; }
    std::vector<sClass_t> &classes() { return _classes; }
    std::vector<sPoint_t> &points() { return _points; }
    std::vector<sKps_t> &kps() { return _kps; }
};

class DFRobot_HumanPose_I2C : public DFRobot_HumanPose {
protected:
    TwoWire *_wire;
    uint8_t __address;
    
public:
    DFRobot_HumanPose_I2C(TwoWire *wire, uint8_t address);
    ~DFRobot_HumanPose_I2C();
    bool begin(void);

protected:  
    int available();
    int read(char *data, int len);
    int write(const char *data, int len);
};

class DFRobot_HumanPose_UART : public DFRobot_HumanPose {
protected:
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
    SoftwareSerial *_serial;
#else
    HardwareSerial *_serial;
    
    uint8_t __rxpin;
    uint8_t __txpin;
#endif
    uint32_t __baud;

public:
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
    DFRobot_HumanPose_UART(SoftwareSerial *sSerial, uint32_t baud);
#else
    DFRobot_HumanPose_UART(HardwareSerial *hSerial, uint32_t baud, uint8_t rxpin = 0, uint8_t txpin = 0);
#endif
    ~DFRobot_HumanPose_UART();
    bool begin(void);

protected:
    int available();
    int read(char *data, int len);
    int write(const char *data, int len);
};

#endif