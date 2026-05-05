#include <Arduino.h>
#include <types.h>
#include "dwm.h"
#include "system_actions.h"

DWM::DWM(Stream &str) : _str(str) {}

void DWM::setup(uint16_t id, uint16_t p_stat, uint16_t p_mov)
{
    bool ok = true;

    ok &= dwm_cfg_tag_set(id);
    ok &= dwm_upd_rate_set(p_stat, p_mov);

    if (!ok)
    {
        if (g_debug.dwm)
            Serial.println("[DWM] Config failed");
    }
    else
    {
        if (g_debug.dwm)
            Serial.println("[DWM] Waiting for module...");

        for (int i = 0; i < 20; i++) // ~2 seconds
        {
            if (dwm_status_get())
            {
                if (g_debug.dwm)
                    Serial.println("[DWM] Module ready");
                return;
            }
            delay(100);
        }

        if (g_debug.dwm)
            Serial.println("[DWM] Module not ready");
    }
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

bool DWM::dwm_cfg_tag_set(uint16_t cfg_tag)
{
    _clear_rx();

    uint8_t msg[4];
    msg[0] = 0x05;
    msg[1] = 0x02;

    memcpy(&msg[2], &cfg_tag, 2);

    _str.write(msg, sizeof(msg));
    _str.flush();

    uint8_t rx[3];

    if (!_read_exact(rx, sizeof(rx), "TYPE_CMD_CFG_TAG_SET"))
        return false;

    return _tvl_success(rx);
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

    uint8_t status_byte = rx[5];

    bool loc_ready = (status_byte & 0x01) != 0;
    bool uwbmac_joined = (status_byte & 0x02) != 0;

    return loc_ready && uwbmac_joined;
}

bool DWM::dwm_upd_rate_set(uint16_t period_stationary_ms, uint16_t period_moving_ms)
{
    _clear_rx();

    uint16_t rate_s = period_stationary_ms / 100;
    uint16_t rate_m = period_moving_ms / 100;

    uint8_t msg[6];
    msg[0] = 0x03;
    msg[1] = 0x04;

    memcpy(&msg[2], &rate_m, 2);
    memcpy(&msg[4], &rate_s, 2);

    _str.write(msg, sizeof(msg));
    _str.flush();

    uint8_t rx[3];

    if (!_read_exact(rx, sizeof(rx), "TYPE_CMD_UR_SET"))
        return false;

    return _tvl_success(rx);
}

bool DWM::dwm_get_pos(DwmState &s)
{
    _clear_rx();

    uint8_t msg[2];
    msg[0] = 0x02;
    msg[1] = 0x00;

    _str.write(msg, sizeof(msg));
    _str.flush();

    uint8_t rx[18];

    if (!_read_exact(rx, sizeof(rx), "TYPE_POS_XYZ"))
        return false;

    if (!_tvl_success(rx))
        return false;

    if (rx[3] != 0x41 || rx[4] != 0x0D)
        return false;

    memcpy(&s.pos.x, &rx[5], 4);
    memcpy(&s.pos.y, &rx[9], 4);
    memcpy(&s.pos.z, &rx[13], 4);
    memcpy(&s.q, &rx[17], 1);

    return true;
}

bool DWM::_tvl_success(const uint8_t *rx)
{
    return rx[0] == 0x40 && rx[1] == 0x01 && rx[2] == 0x00;
}