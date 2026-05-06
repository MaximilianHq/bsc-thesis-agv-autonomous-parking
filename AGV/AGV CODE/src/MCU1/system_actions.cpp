#include "system_actions.h"
#include <status_led.h>

SysCtrl::SysCtrl(Comm &comm_bt, Comm &comm_mcu, StatusLED &led_sys, StatusLED &led_cmd)
    : _comm_bt(comm_bt),
      _comm_mcu(comm_mcu),
      _proto_handler_bt(comm_bt, *this),
      _proto_handler_mcu(comm_mcu, *this),
      _led_sys(led_sys),
      _led_cmd(led_cmd) {}

void SysCtrl::on_bt_no_connect()
{
    on_stop();
    _led_sys.set_status(StatusLED::State::STATUS_BLE_SEARCHING);
}

void SysCtrl::on_bt_pkt_recieved(Comm::Packet &pkt)
{
    _proto_handler_bt.handle(pkt);

    if (!pkt.approved)
        return;

    _process_bt_packet(pkt);
    _led_cmd.set_status(StatusLED::State::STATUS_CMD_RECEIVING);
}

void SysCtrl::on_mcu_pkt_recieved(Comm::Packet &pkt)
{
    _proto_handler_mcu.handle(pkt);

    if (!pkt.approved)
        return;

    _process_mcu_packet(pkt);
}

void SysCtrl::on_new_motion(Comm::Packet &pkt) { _motion_buffert.push_back(pkt); }

void SysCtrl::on_stop()
{
    Comm::Packet p = {'X', _proto_handler_mcu.get_sequence(), {}, 0, 0, true};
    p.crc = Comm::csum(p);

    on_stop(p);
}

void SysCtrl::on_stop(Comm::Packet &pkt)
{
    if (!_forward_to_mcu(pkt))
        if (g_debug.IAction)
            Serial.println("[SysCtrl] \033[31mWATNING\033[0m - Failed to send command: 'stop' to [MCU2]");
    _led_cmd.set_status(StatusLED::State::STATUS_CMD_STOPPING);
}

void SysCtrl::on_obstacle_detected(const Position &pos)
{
    Comm::Packet p = {'H', _proto_handler_bt.get_sequence(), {}, 4, 0, true};
    p.data[0] = static_cast<uint8_t>((pos.x >> 8) & 0xFF);
    p.data[1] = static_cast<uint8_t>(pos.x & 0xFF);
    p.data[2] = static_cast<uint8_t>((pos.y >> 8) & 0xFF);
    p.data[3] = static_cast<uint8_t>(pos.y & 0xFF);
    p.crc = Comm::csum(p);

    if (_comm_mcu.write(p))
    {
        _proto_handler_bt.itterate_sequence();
        _proto_handler_bt.add_buffer_sent(p);
    }
    else if (g_debug.IAction)
        Serial.println("[SysCtrl] \033[31mWATNING\033[0m - Failed send obstacle position to [ÖS]");

    _led_cmd.set_status(StatusLED::State::STATUS_OBSTACLE);
}

void SysCtrl::_process_bt_packet(Comm::Packet &pkt)
{
    switch (pkt.type)
    {
    case 'D':
        on_new_motion(pkt);
        break;
    case 'K':
        switch (pkt.data[0])
        {
        case 'N':
            _next_movement(pkt);
            break;
        case 'H':
            _led_cmd.set_status(StatusLED::State::STATUS_RETURNING);
            break;
        default:
            break;
        }
        break;
    case 'X':
        on_stop(pkt);
        break;
    default:
        break;
    }
}

void SysCtrl::_process_mcu_packet(Comm::Packet &pkt)
{
    switch (pkt.type)
    {
    case 'M':
        break;
    case 'H':
        break;
    default:
        break;
    }
}

bool SysCtrl::_forward_to_mcu(Comm::Packet &pkt)
{
    pkt.seq = _proto_handler_mcu.get_sequence();
    pkt.crc = Comm::csum(pkt);

    if (!_comm_mcu.write(pkt))
        return false;

    _proto_handler_mcu.itterate_sequence();
    _proto_handler_mcu.add_buffer_sent(pkt);
    return true;
}

void SysCtrl::_next_movement(Comm::Packet &pkt)
{
    if (_motion_buffert.size())
        if (_forward_to_mcu(_motion_buffert[0]))
            _motion_buffert.pop(0);
        else
        {
            on_stop();
            if (g_debug.IAction)
                Serial.println("[SYSCTRL] \033[31mWARNING\033[0m - Failed to send command: 'next movement' to [MCU2]");
        }
    else
    {
        Comm::Packet p = {'E', _proto_handler_bt.get_sequence(), {'N', pkt.seq}, 1, 0, true};
        p.crc = Comm::csum(p);

        if (_comm_mcu.write(p))
        {
            _proto_handler_bt.itterate_sequence();
            _proto_handler_bt.add_buffer_sent(p);
        }
        else if (g_debug.IAction)
            Serial.println("[SysCtrl] \033[31mWATNING\033[0m - Failed send no movement error to [ÖS]");
    }
}

void SysCtrl::on_new_position_data(const DwmState &dwm, const ImuState &imu)
{
    if (true)
    {
        Serial.println("[SYSCTRL] Recieved values");
        Serial.print("[DWM] X: ");
        Serial.print(dwm.pos.x / 1000.0f, 3);
        Serial.print("  Y: ");
        Serial.print(dwm.pos.y / 1000.0f, 3);
        Serial.print("  Z: ");
        Serial.println(dwm.pos.z / 1000.0f, 3);

        Serial.print("[IMU] AX: ");
        Serial.print(imu.ax);
        Serial.print("  AY: ");
        Serial.print(imu.ay);
        Serial.print("  DT: ");
        Serial.print(imu.dt);
        Serial.print("  WZ: ");
        Serial.println(imu.wz);
    }

    if (_state.size() == 0)
    {
        AgvState init_state;
        init_state.pos = dwm.pos;
        init_state.theta = 0.0f;
        init_state.t_ms = millis();
        _state.push_front(init_state);
        return;
    }

    const AgvState prev_state = _state[0];
    AgvState pred_state = prev_state;

    // predicted state
    // x = \cos θ, y = \sin θ
    // x_{agv} = \cos θ * v_x + \cos(θ-90\deg) * v_y = \cos θ * v_x - \sin θ * v_y
    // y_{agv} = \sin θ * v_y + \sin(θ-90\deg) * v_y = \sin θ * v_x + \cos θ * v_y

    pred_state.pos.x = prev_state.pos.x + (prev_state.vx * cosf(prev_state.theta) - prev_state.vy * sinf(prev_state.theta)) * imu.dt;
    pred_state.pos.y = prev_state.pos.y + (prev_state.vx * sinf(prev_state.theta) + prev_state.vy * cosf(prev_state.theta)) * imu.dt;
    pred_state.theta = _norm_ang(prev_state.theta + imu.wz * imu.dt);

    pred_state.vx += imu.ax * imu.dt;
    pred_state.vy += imu.ay * imu.dt;

    // position error
    float err_x = dwm.pos.x - pred_state.pos.x;
    float err_y = dwm.pos.y - pred_state.pos.y;

    // position correction
    AgvState upd = pred_state;
    upd.pos.x += _err_co_dwm * err_x;
    upd.pos.y += _err_co_dwm * err_y;
    upd.t_ms = millis();

    float dx = static_cast<float>(upd.pos.x - prev_state.pos.x);
    float dy = static_cast<float>(upd.pos.y - prev_state.pos.y);

    float dist = sqrtf(dx * dx + dy * dy);

    if (dist > 10.0f)
    {
        float theta_meas = atan2f(dy, dx);
        float theta_err = _norm_ang(theta_meas - upd.theta);
        upd.theta = _norm_ang(upd.theta + _err_co_imu * theta_err);
    }

    if (true)
    {
        Serial.println("[SYSCTRL] Calculated values values");
        Serial.print("[DWM] X: ");
        Serial.print(upd.pos.x / 1000.0f, 3);
        Serial.print("  Y: ");
        Serial.print(upd.pos.y / 1000.0f, 3);
        Serial.print("  Z: ");
        Serial.print(upd.pos.z / 1000.0f, 3);
        Serial.print("  ANG: ");
        Serial.print(upd.theta);
        Serial.print("  TIME_NOW: ");
        Serial.println(upd.t_ms);
    }

    _state.push_front(upd);

    auto x = _state[0].pos.x;
    auto y = _state[0].pos.y;

    Comm::Packet p = {'P', _proto_handler_bt.get_sequence(), {static_cast<uint8_t>((x >> 8) & 0xFF), static_cast<uint8_t>(x & 0xFF), static_cast<uint8_t>((y >> 8) & 0xFF), static_cast<uint8_t>(y & 0xFF)}, 4, 0, true};
    p.crc = Comm::csum(p);

    if (_comm_mcu.write(p))
    {
        _proto_handler_bt.itterate_sequence();
        _proto_handler_bt.add_buffer_sent(p);
    }
    else if (g_debug.IAction)
        Serial.println("[SysCtrl] \033[31mWATNING\033[0m - Failed send position update to [ÖS]");
};
