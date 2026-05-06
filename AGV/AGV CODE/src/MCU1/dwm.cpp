#include <Arduino.h>
#include <types.h>
#include "dwm.h"
#include "system_actions.h"

DWM::DWM(Stream &str, unsigned long d_rate) : _str(str), _d_rate(d_rate) {}

void DWM::_request_pos()
{
    uint8_t cmd[] = {0x0C, 0x00}; // dwm_loc_get
    _str.write(cmd, sizeof(cmd));
    _str.flush();

    _last_req = millis();
    _request_start = millis();
    _waiting_for_response = true;

    _reset_parser();
    _reset_response();
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

void DWM::_reset_parser()
{
    _state = WAIT_TYPE;
    _read_index = 0;
    _expected_len = 0;
}

void DWM::_reset_response()
{
    _response_has_status = false;
    _response_has_position = false;
}

bool DWM::read(DwmState &out)
{
    bool packet_delivered = false;

    if (!_waiting_for_response)
    {
        if (_d_rate == 0 || (millis() - _last_req) >= _d_rate)
            _request_pos();
    }

    while (_str.available())
    {
        uint8_t b = static_cast<uint8_t>(_str.read());

        switch (_state)
        {
        case WAIT_TYPE:
            _pkt[0] = b;
            _read_index = 1;
            _state = READ_LEN;
            break;

        case READ_LEN:
            _pkt[1] = b;
            _read_index = 2;
            _expected_len = static_cast<size_t>(b) + 2;

            if (_expected_len > sizeof(_pkt))
            {
                if (g_debug.dwm)
                    Serial.println("[DWM] Packet too large");

                _waiting_for_response = false;
                _reset_parser();
                _reset_response();
                break;
            }

            _state = READ_BODY;
            break;

        case READ_BODY:
            _pkt[_read_index++] = b;

            if (_read_index >= _expected_len)
            {
                if (g_debug.dwm)
                {
                    Serial.print("[DWM] TLV 0x");
                    if (_pkt[0] < 0x10)
                        Serial.print("0");
                    Serial.print(_pkt[0], HEX);
                    Serial.print(" RX ");
                    Serial.print(_expected_len);
                    Serial.print(": ");

                    for (size_t i = 0; i < _expected_len; i++)
                    {
                        if (_pkt[i] < 0x10)
                            Serial.print("0");
                        Serial.print(_pkt[i], HEX);
                        Serial.print(" ");
                    }
                    Serial.println();
                }

                uint8_t type = _pkt[0];
                uint8_t length = _pkt[1];
                _reset_parser();

                if (type == 0x40 && length >= 1)
                {
                    _response_has_status = (_pkt[2] == 0x00);

                    if (g_debug.dwm)
                    {
                        Serial.print("[DWM] Status: ");
                        Serial.println(_pkt[2], HEX);
                    }

                    if (_pkt[2] != 0x00)
                    {
                        _waiting_for_response = false;
                        _reset_response();
                    }
                }
                else if (type == 0x41 && length >= 13)
                {
                    int32_t x = 0;
                    int32_t y = 0;
                    int32_t z = 0;

                    memcpy(&x, &_pkt[2], 4);
                    memcpy(&y, &_pkt[6], 4);
                    memcpy(&z, &_pkt[10], 4);

                    out.pos.x = static_cast<int16_t>(x);
                    out.pos.y = static_cast<int16_t>(y);
                    out.pos.z = static_cast<int16_t>(z);
                    out.q = _pkt[14];

                    _response_has_position = true;
                    packet_delivered = true;
                }
                else if (type == 0x48 && length >= 1)
                {
                    if (g_debug.dwm)
                    {
                        Serial.print("[DWM] Distances count: ");
                        Serial.println(_pkt[2]);
                    }

                    _waiting_for_response = false;
                    _reset_response();
                }
                else if (type == 0x49 && length >= 1)
                {
                    if (g_debug.dwm)
                    {
                        Serial.print("[DWM] Anchors count: ");
                        Serial.println(_pkt[2]);
                    }

                    _waiting_for_response = false;
                    _reset_response();
                }
                else
                {
                    if (g_debug.dwm)
                    {
                        Serial.print("[DWM] Unhandled TLV type 0x");
                        if (type < 0x10)
                            Serial.print("0");
                        Serial.println(type, HEX);
                    }
                }
            }
            break;
        }
    }

    if (_waiting_for_response && (millis() - _request_start) > RESPONSE_TIMEOUT_MS)
    {
        if (g_debug.dwm)
            Serial.println("[DWM] Response timeout");

        _waiting_for_response = false;
        _reset_parser();
        _reset_response();
    }

    return packet_delivered;
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
            rx[n++] = static_cast<uint8_t>(_str.read());
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

bool DWM::_tvl_success(const uint8_t *rx)
{
    return rx[0] == 0x40 && rx[1] == 0x01 && rx[2] == 0x00;
}
