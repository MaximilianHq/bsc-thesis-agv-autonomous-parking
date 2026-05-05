#include <Arduino.h>
#include <types.h>
#include "dwm.h"
#include "system_actions.h"

DWM::DWM(Stream &str) : _str(str) {}

void DWM::setup(uint16_t id, uint16_t p_stat, uint16_t p_mov)
{
    delay(200);
}

void DWM::_clear_rx()
{
    while (_str.available())
        _str.read();
}

bool DWM::_read_exact(uint8_t *rx, size_t expected_len, const char *cmd_name)
{
    size_t n = 0;
    unsigned long start = millis();

    while (n < expected_len && (millis() - start) < 500)
    {
        if (_str.available())
        {
            rx[n++] = (uint8_t)_str.read();
        }
    }

    if (g_debug.dwm)
    {
        Serial.print("[DWM] ");
        Serial.print(cmd_name);
        Serial.print(" RX ");
        Serial.print(n);
        Serial.print("/");
        Serial.print(expected_len);
        Serial.print(": ");

        for (size_t i = 0; i < n; i++)
        {
            if (rx[i] < 0x10)
                Serial.print("0");
            Serial.print(rx[i], HEX);
            Serial.print(" ");
        }

        Serial.println();
    }

    if (n != expected_len)
    {
        if (g_debug.dwm)
        {
            Serial.print("[DWM] Response (");
            Serial.print(cmd_name);
            Serial.println(") incomplete/corrupt");
        }

        return false;
    }

    return true;
}

bool DWM::dwm_status_get()
{
    _clear_rx();

    uint8_t msg[2];
    msg[0] = 0x32;
    msg[1] = 0x00;

    _str.write(msg, sizeof(msg));
    _str.flush();

    uint8_t rx[6];

    if (!_read_exact(rx, sizeof(rx), "TYPE_CMD_STATUS_GET"))
        return false;

    if (!_tvl_success(rx))
        return false;

    if (rx[3] != 0x5A || rx[4] != 0x01)
        return false;

    return true;
}

bool DWM::dwm_cfg_get(uint16_t &cfg_node)
{
    _clear_rx();

    uint8_t msg[2];
    msg[0] = 0x08; // TYPE_CMD_CFG_GET
    msg[1] = 0x00;

    _str.write(msg, sizeof(msg));
    _str.flush();

    uint8_t rx[7];

    if (!_read_exact(rx, sizeof(rx), "TYPE_CMD_CFG_GET"))
        return false;

    if (!_tvl_success(rx))
        return false;

    if (rx[3] != 0x46 || rx[4] != 0x02)
        return false;

    uint8_t cfg_lo = rx[5];
    uint8_t cfg_hi = rx[6];
    cfg_node = static_cast<uint16_t>(cfg_lo) | (static_cast<uint16_t>(cfg_hi) << 8);

    if (g_debug.dwm)
    {
        bool low_power_en = (cfg_lo & 0x80) != 0;
        bool loc_engine_en = (cfg_lo & 0x40) != 0;
        bool led_en = (cfg_lo & 0x10) != 0;
        bool ble_en = (cfg_lo & 0x08) != 0;
        bool fw_update_en = (cfg_lo & 0x04) != 0;
        uint8_t uwb_mode = cfg_lo & 0x03;

        bool mode_anchor = (cfg_hi & 0x20) != 0;
        bool initiator = (cfg_hi & 0x10) != 0;
        bool bridge = (cfg_hi & 0x08) != 0;
        bool accel_en = (cfg_hi & 0x04) != 0;
        uint8_t meas_mode = cfg_hi & 0x03;

        Serial.print("[DWM] CFG_GET cfg_node=0x");
        if (cfg_node < 0x1000)
            Serial.print("0");
        if (cfg_node < 0x100)
            Serial.print("0");
        if (cfg_node < 0x10)
            Serial.print("0");
        Serial.println(cfg_node, HEX);
        Serial.print("[DWM]   mode=");
        Serial.print(mode_anchor ? "anchor" : "tag");
        Serial.print(" initiator=");
        Serial.print(initiator);
        Serial.print(" bridge=");
        Serial.print(bridge);
        Serial.print(" accel_en=");
        Serial.print(accel_en);
        Serial.print(" meas_mode=");
        Serial.print(meas_mode);
        Serial.println();
        Serial.print("[DWM]   low_power_en=");
        Serial.print(low_power_en);
        Serial.print(" loc_engine_en=");
        Serial.print(loc_engine_en);
        Serial.print(" led_en=");
        Serial.print(led_en);
        Serial.print(" ble_en=");
        Serial.print(ble_en);
        Serial.print(" fw_update_en=");
        Serial.print(fw_update_en);
        Serial.print(" uwb_mode=");
        Serial.println(uwb_mode);
    }

    return true;
}

bool DWM::dwm_get_pos(DwmState &s)
{
    _clear_rx();

    uint8_t cmd[] = {0x0C, 0x00}; // dwm_loc_get
    _str.write(cmd, sizeof(cmd));
    _str.flush();

    uint8_t buffer[256];
    size_t bufferIndex = 0;

    unsigned long start = millis();

    while ((millis() - start) < 500)
    {
        while (_str.available())
        {
            uint8_t b = (uint8_t)_str.read();

            if (bufferIndex >= sizeof(buffer))
            {
                bufferIndex = 0;
            }

            buffer[bufferIndex++] = b;

            if (bufferIndex >= 2)
            {
                uint8_t packet_len = buffer[1];

                if (packet_len > 250)
                {
                    bufferIndex = 0;
                    continue;
                }

                size_t total_len = packet_len + 2;

                if (bufferIndex >= total_len)
                {
                    if (g_debug.dwm)
                    {
                        Serial.print("[DWM] TYPE_CMD_LOC_GET RX ");
                        Serial.print(total_len);
                        Serial.print(": ");

                        for (size_t i = 0; i < total_len; i++)
                        {
                            if (buffer[i] < 0x10)
                                Serial.print("0");
                            Serial.print(buffer[i], HEX);
                            Serial.print(" ");
                        }

                        Serial.println();
                    }

                    uint8_t type = buffer[0];
                    uint8_t length = buffer[1];

                    if ((type == 0x41 || type == 0x40) && length >= 13 && total_len >= 15)
                    {
                        memcpy(&s.pos.x, &buffer[2], 4);
                        memcpy(&s.pos.y, &buffer[6], 4);
                        memcpy(&s.pos.z, &buffer[10], 4);
                        memcpy(&s.q, &buffer[14], 1);

                        float x_cm = s.pos.x / 1000.0f;
                        float y_cm = s.pos.y / 1000.0f;
                        float z_cm = s.pos.z / 1000.0f;

                        if (g_debug.dwm)
                        {
                            Serial.print("[DWM] X: ");
                            Serial.print(x_cm, 3);
                            Serial.print("  Y: ");
                            Serial.print(y_cm, 3);
                            Serial.print("  Z: ");
                            Serial.println(z_cm, 3);
                        }

                        return true;
                    }

                    bufferIndex = 0;
                }
            }
        }
    }

    if (g_debug.dwm)
    {
        Serial.println("[DWM] TYPE_CMD_LOC_GET timeout/no valid packet");
    }

    return false;
}

bool DWM::_tvl_success(const uint8_t *rx)
{
    return rx[0] == 0x40 && rx[1] == 0x01 && rx[2] == 0x00;
}