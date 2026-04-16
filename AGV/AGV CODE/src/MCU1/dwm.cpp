#include <Arduino.h>
#include <types.h>
#include "dwm.h"

bool _tvl_success(const uint8_t *rx)
{
    // Expect: 40 01 err
    if (rx[0] != 0x40 || rx[1] != 0x01)
        return false; // RET_VAL header
    else if (rx[2] != 0x00)
        return false; // err_code must be 0
    else
        return true;
}

bool dwm_ready(Stream &str)
{
    const uint8_t msg[] = {0x32, 0x00}; // TYPE_CMD_STATUS_GET
    str.write(msg, sizeof(msg));

    uint8_t rx[6];
    size_t n = str.readBytes(rx, sizeof(rx));
    if (n != sizeof(rx))
    {
        if (g_debug.dwm)
            Serial.println("[DWM] Response (TYPE_CMD_STATUS_GET) corrupt");
        return false;
    }

    if (!_tvl_success(rx))
        return false;
    if (rx[3] != 0x5A || rx[4] != 0x01)
        return false; // STATUS header

    uint8_t status = rx[5];
    bool loc_ready = (status & 0x01) != 0;
    bool joined = (status & 0x02) != 0;

    return loc_ready && joined; // true only if both are 1
}

bool dwm_upd_rate_set(Stream &str, uint16_t period_stationary_ms, uint16_t period_moving_ms)
{

    uint16_t rate_s = period_stationary_ms / 100; // convert to DWM format
    uint16_t rate_m = period_moving_ms / 100;     // convert to DWM format

    uint8_t msg[6];
    msg[0] = 0x03; // TYPE_CMD_UR_SET
    msg[1] = 0x04; // length = 4 B

    memcpy(&msg[2], &rate_m, 2);
    memcpy(&msg[4], &rate_s, 2);

    str.write(msg, sizeof(msg));

    uint8_t rx[3];
    size_t n = str.readBytes(rx, sizeof(rx));
    if (n != sizeof(rx))
    {
        if (g_debug.dwm)
            Serial.println("[DWM] Response (TYPE_CMD_UR_SET) corrupt");
        return false;
    }

    if (!_tvl_success(rx))
        return false;

    return true;
}

bool dwm_get_pos(Stream &str, DwmState &s)
{
    uint8_t msg[] = {0x02, 0x00}; // TYPE_POS_XYZ
    str.write(msg, sizeof(msg));

    uint8_t rx[18];
    size_t n = str.readBytes(rx, sizeof(rx));
    if (n != sizeof(rx))
    {
        if (g_debug.dwm)
            Serial.println("[DWM] Response (TYPE_POS_XYZ) corrupt");
        return false;
    }

    if (!_tvl_success(rx))
        return false;
    if (rx[3] != 0x41 || rx[4] != 0x0D)
        return false;

    memcpy(&s.pos.x, &rx[5], 4);
    memcpy(&s.pos.y, &rx[9], 4);
    memcpy(&s.pos.z, &rx[13], 4);
    memcpy(&s.q, &rx[18], 1);

    return true;
}