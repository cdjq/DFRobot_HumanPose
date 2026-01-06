/**
 * @file  DFRobot_HumanPose.cpp
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

#include "DFRobot_HumanPose.h"

#ifdef ARDUINO_ARCH_RENESAS
char *strnstr(const char *haystack, const char *needle, size_t n)
{
    if (!needle || n == 0)
    {
        return NULL;
    }

    size_t needle_len = 0;
    while (needle[needle_len] != '\0')
    {
        needle_len++;
    }

    if (needle_len == 0)
    {
        return (char *)haystack;
    }

    for (size_t i = 0; i < n && haystack[i] != '\0'; i++)
    {
        if (i + needle_len <= n && haystack[i] == needle[0])
        {
            size_t j = 1;
            while (j < needle_len && haystack[i + j] == needle[j])
            {
                j++;
            }
            if (j == needle_len)
            {
                return (char *)&haystack[i];
            }
        }
    }

    return NULL;
}
#endif

// ============ Base Class: DFRobot_HumanPose ============

DFRobot_HumanPose::DFRobot_HumanPose()
{
    _wait_delay = 2;
    rx_end = 0;
    tx_len = 0;
    rx_len = 0;
    tx_buf = NULL;
    rx_buf = NULL;
    payload = NULL;
}

DFRobot_HumanPose::~DFRobot_HumanPose()
{
    if (tx_buf)
    {
        free(tx_buf);
        tx_buf = NULL;
    }
    if (rx_buf)
    {
        free(rx_buf);
        rx_buf = NULL;
    }
}

bool DFRobot_HumanPose::begin()
{
    // Allocate buffers
    rx_buf = (char *)malloc(RX_MAX_SIZE);
    tx_buf = (char *)malloc(TX_MAX_SIZE);
    rx_len = RX_MAX_SIZE;
    tx_len = TX_MAX_SIZE;

    if (!rx_buf || !tx_buf)
    {
        return false;
    }

    rx_end = 0;
    response.clear();

    return true;
}

int DFRobot_HumanPose::wait(int type, const char *cmd, uint32_t timeout)
{
    int ret = eOK;
    unsigned long startTime = millis();

    while (millis() - startTime <= timeout)
    {
        int len = available();
        if (len == 0)
        {
            delay(1);
            continue;
        }

        if (len + rx_end > rx_len)
        {
            len = rx_len - rx_end;
            if (len <= 0)
            {
                rx_end = 0;
                continue;
            }
        }

        rx_end += read(rx_buf + rx_end, len);
        rx_buf[rx_end] = '\0';

        while (char *suffix = strnstr(rx_buf, RES_SUF, rx_end))
        {
            if (char *prefix = strnstr(rx_buf, RES_PRE, suffix - rx_buf))
            {
                // Extract JSON payload
                len = suffix - prefix + RES_SUF_LEN;
                payload = (char *)malloc(len);

                if (!payload)
                {
                    continue;
                }

                memcpy(payload, prefix + 1, len - 1); // remove "\r" and "\n"
                memmove(rx_buf, suffix + RES_SUF_LEN, rx_end - (suffix - rx_buf) - RES_SUF_LEN);
                rx_end -= (suffix - rx_buf) + RES_SUF_LEN;
                payload[len - 1] = '\0';

                response.clear();
                DeserializationError error = deserializeJson(response, payload);
                free(payload);
                payload = NULL;

                if (error)
                {
                    continue;
                }

                if (response["type"] == CMD_TYPE_EVENT)
                {
                    parser_event();
                }

                ret = response["code"];

                // Match command name
                const char *resp_name = response["name"];
                if (resp_name && response["type"] == type)
                {
                    size_t cmd_len = strlen(cmd);
                    size_t resp_len = strlen(resp_name);
                    if ((cmd_len == resp_len && strcmp(resp_name, cmd) == 0) ||
                        (cmd_len > 0 && resp_len > 0 && strncmp(resp_name, cmd, cmd_len) == 0))
                    {
                        return ret;
                    }
                }
            }
            else
            {
                // discard this reply (no prefix found before suffix)
                memmove(rx_buf, suffix + RES_SUF_LEN, rx_end - (suffix - rx_buf) - RES_SUF_LEN);
                rx_end -= (suffix - rx_buf) + RES_SUF_LEN;
                rx_buf[rx_end] = '\0';
            }
        }
    }

    return eTimedOut;
}

void DFRobot_HumanPose::parser_event()
{
    const char *event_name = response["name"];
    if (event_name && strstr(event_name, EVENT_INVOKE))
    {
        JsonObject data = response["data"].as<JsonObject>();

        // ---- pose_keypoints ----
        if (data["pose_keypoints"].is<JsonArray>())
        {
            _kps.clear();
            JsonArray keypoints = data["pose_keypoints"].as<JsonArray>();

            for (size_t i = 0; i < keypoints.size(); i++)
            {
                sKps_t k;
                JsonArray box = keypoints[i][0].as<JsonArray>();
                JsonArray points = keypoints[i][1].as<JsonArray>();

                k.box.x = box[0] | 0;
                k.box.y = box[1] | 0;
                k.box.w = box[2] | 0;
                k.box.h = box[3] | 0;
                k.box.score = box[4] | 0;
                k.box.target = box[5] | 0;

                for (size_t j = 0; j < points.size(); j++)
                {
                    sPoint_t p;
                    p.x = points[j][0] | 0;
                    p.y = points[j][1] | 0;
                    p.z = 0;
                    p.score = points[j][2] | 0;
                    p.target = points[j][3] | 0;
                    k.points.push_back(p);
                }
                _kps.push_back(k);
            }
        }

        // ---- hand_keypoints ----
        else if (data["hand_keypoints"].is<JsonArray>())
        {
            _kps.clear();
            JsonArray keypoints = data["hand_keypoints"].as<JsonArray>();

            for (size_t i = 0; i < keypoints.size(); i++)
            {
                sKps_t k;
                JsonArray box = keypoints[i][0].as<JsonArray>();
                JsonArray points = keypoints[i][1].as<JsonArray>();

                k.box.x = box[0] | 0;
                k.box.y = box[1] | 0;
                k.box.w = box[2] | 0;
                k.box.h = box[3] | 0;
                k.box.score = box[4] | 0;
                k.box.target = box[5] | 0;

                for (size_t j = 0; j < points.size(); j++)
                {
                    sPoint_t p;
                    p.x = points[j][0] | 0;
                    p.y = points[j][1] | 0;
                    p.z = 0;
                    p.score = 0;
                    p.target = 0;
                    k.points.push_back(p);
                }
                _kps.push_back(k);
            }
        }
    }
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::getResult()
{
    char cmd[64] = {0};
    snprintf(cmd, sizeof(cmd), CMD_PRE "%s=1,0,1" CMD_SUF, AT_INVOKE);
    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, AT_INVOKE) == eOK)
    {
        if (wait(CMD_TYPE_EVENT, EVENT_INVOKE) == eOK)
        {
            return eOK;
        }
    }

    return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::setTScore(uint8_t tscore)
{
    char cmd[64] = {0};
    snprintf(cmd, sizeof(cmd), CMD_PRE "%s=%d" CMD_SUF, AT_TSCORE, tscore);
    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, AT_TSCORE) == eOK)
    {
        return eOK;
    }
    return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::setTIOU(uint8_t tious)
{
    char cmd[64] = {0};
    snprintf(cmd, sizeof(cmd), CMD_PRE "%s=%d" CMD_SUF, AT_TIOU, tious);
    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, AT_TIOU) == eOK)
    {
        return eOK;
    }
    return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::setModel(eModel_t model)
{
    char cmd[64] = {0};
    snprintf(cmd, sizeof(cmd), CMD_PRE "%s=%d" CMD_SUF, AT_MODEL, (int)model);
    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, AT_MODEL) == eOK)
    {
        return eOK;
    }
    return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::getTScore()
{
    char cmd[64] = {0};
    snprintf(cmd, sizeof(cmd), CMD_PRE "%s?" CMD_SUF, AT_TSCORE);
    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, AT_TSCORE) == eOK)
    {
        return eOK;
    }
    return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::getTIOU()
{
    char cmd[64] = {0};
    snprintf(cmd, sizeof(cmd), CMD_PRE "%s?" CMD_SUF, AT_TIOU);
    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, AT_TIOU) == eOK)
    {
        return eOK;
    }
    return eTimedOut;
}

DFRobot_HumanPose::eCmdCode_t DFRobot_HumanPose::getModel()
{
    char cmd[64] = {0};
    snprintf(cmd, sizeof(cmd), CMD_PRE "%s?" CMD_SUF, AT_MODEL);
    write(cmd, strlen(cmd));

    if (wait(CMD_TYPE_RESPONSE, AT_MODEL) == eOK)
    {
        return eOK;
    }
    return eTimedOut;
}

// ============ Derived Class: DFRobot_HumanPose_I2C ============

DFRobot_HumanPose_I2C::DFRobot_HumanPose_I2C(TwoWire *wire, uint8_t address)
{
    _wire = wire;
    __address = address;
}

DFRobot_HumanPose_I2C::~DFRobot_HumanPose_I2C()
{
}

bool DFRobot_HumanPose_I2C::begin(void)
{
    _wire->begin();
    _wire->setClock(I2C_CLOCK);
    _wait_delay = 2;

    return DFRobot_HumanPose::begin();
}

int DFRobot_HumanPose_I2C::available()
{
    uint8_t buf[2] = {0};
    delay(_wait_delay);
    _wire->beginTransmission(__address);
    _wire->write(FEATURE_TRANSPORT);
    _wire->write(FEATURE_TRANSPORT_CMD_AVAILABLE);
    _wire->write(0);
    _wire->write(0);
    // TODO checksum
    _wire->write(0);
    _wire->write(0);
    if (_wire->endTransmission() == 0)
    {
        delay(_wait_delay);
        _wire->requestFrom(__address, (uint8_t)2);
        _wire->readBytes(buf, (uint8_t)2);
    }

    return (buf[0] << 8) | buf[1];
}

int DFRobot_HumanPose_I2C::read(char *data, int len)
{
    uint16_t packets = len / MAX_PL_LEN;
    uint8_t remain = len % MAX_PL_LEN;

    for (uint16_t i = 0; i < packets; i++)
    {
        delay(_wait_delay);
        _wire->beginTransmission(__address);
        _wire->write(FEATURE_TRANSPORT);
        _wire->write(FEATURE_TRANSPORT_CMD_READ);
        _wire->write(MAX_PL_LEN >> 8);
        _wire->write(MAX_PL_LEN & 0xFF);
        // TODO checksum
        _wire->write(0);
        _wire->write(0);
        if (_wire->endTransmission() == 0)
        {
            delay(_wait_delay);
            _wire->requestFrom(__address, MAX_PL_LEN);
            _wire->readBytes(data + i * MAX_PL_LEN, MAX_PL_LEN);
        }
    }

    if (remain)
    {
        delay(_wait_delay);
        _wire->beginTransmission(__address);
        _wire->write(FEATURE_TRANSPORT);
        _wire->write(FEATURE_TRANSPORT_CMD_READ);
        _wire->write(remain >> 8);
        _wire->write(remain & 0xFF);
        // TODO checksum
        _wire->write(0);
        _wire->write(0);
        if (_wire->endTransmission() == 0)
        {
            delay(_wait_delay);
            _wire->requestFrom(__address, remain);
            _wire->readBytes(data + packets * MAX_PL_LEN, remain);
        }
    }

    return len;
}

int DFRobot_HumanPose_I2C::write(const char *data, int len)
{
    uint16_t packets = len / MAX_PL_LEN;
    uint16_t remain = len % MAX_PL_LEN;

    for (uint16_t i = 0; i < packets; i++)
    {
        delay(_wait_delay);
        _wire->beginTransmission(__address);
        _wire->write(FEATURE_TRANSPORT);
        _wire->write(FEATURE_TRANSPORT_CMD_WRITE);
        _wire->write(MAX_PL_LEN >> 8);
        _wire->write(MAX_PL_LEN & 0xFF);
        _wire->write((const uint8_t *)(data + i * MAX_PL_LEN), MAX_PL_LEN);
        // TODO checksum
        _wire->write(0);
        _wire->write(0);
        _wire->endTransmission();
    }

    if (remain)
    {
        delay(_wait_delay);
        _wire->beginTransmission(__address);
        _wire->write(FEATURE_TRANSPORT);
        _wire->write(FEATURE_TRANSPORT_CMD_WRITE);
        _wire->write(remain >> 8);
        _wire->write(remain & 0xFF);
        _wire->write((const uint8_t *)(data + packets * MAX_PL_LEN), remain);
        _wire->write(0);
        _wire->write(0);
        _wire->endTransmission();
    }

    return len;
}

// ============ Derived Class: DFRobot_HumanPose_UART ============

#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
DFRobot_HumanPose_UART::DFRobot_HumanPose_UART(SoftwareSerial *sSerial, uint32_t baud)
{
    _serial = sSerial;
    __baud = baud;
}
#else
DFRobot_HumanPose_UART::DFRobot_HumanPose_UART(HardwareSerial *hSerial, uint32_t baud = UART_BAUD, uint8_t rxpin, uint8_t txpin)
{
    _serial = hSerial;
    __baud = baud;
    __rxpin = rxpin;
    __txpin = txpin;
}
#endif

DFRobot_HumanPose_UART::~DFRobot_HumanPose_UART()
{
}

bool DFRobot_HumanPose_UART::begin(void)
{
    _wait_delay = 2;
#ifdef ESP32
    _serial->begin(__baud, SERIAL_8N1, __rxpin, __txpin);
#elif defined(ARDUINO_AVR_UNO) || defined(ESP8266)
    _serial->begin(__baud);
#else
    _serial->begin(__baud);
#endif
    _serial->setTimeout(1000);
    _serial->flush();

    return DFRobot_HumanPose::begin();
}

int DFRobot_HumanPose_UART::available()
{
    return _serial->available();
}

int DFRobot_HumanPose_UART::read(char *data, int len)
{
    return _serial->readBytes(data, len);
}

int DFRobot_HumanPose_UART::write(const char *data, int len)
{
    return _serial->write(data, len);
}
