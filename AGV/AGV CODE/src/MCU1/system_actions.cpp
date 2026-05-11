#include "system_actions.h"
#include <status_led.h>
#include <coords.h>
#include <dwm.h>

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
        if (g_debug.sysctrl)
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
        _proto_handler_bt.iterate_sequence();
        _proto_handler_bt.add_buffer_sent(p);
    }
    else if (g_debug.sysctrl)
        Serial.println("[SysCtrl] \033[31mWARNING\033[0m - Failed send obstacle position to [ÖS]");

    _led_cmd.set_status(StatusLED::State::STATUS_OBSTACLE);
}

void SysCtrl::on_new_position_data(DwmState &dwm, const ImuState &imu, float dwm_offset)
{
    if (true)
    {
        Serial.println("[SYSCTRL] Received values");
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
        Serial.print(imu.dt * 1000.0f);
        Serial.print("  WZ: ");
        Serial.println(imu.wz);
    }

    // handle initial state
    if (_state.size() == 0)
    {
        AgvState init_state;
        init_state.pos = dwm.pos;
        init_state.theta = 0;
        init_state.t_ms = millis();
        _state.push_front(init_state);
        return;
    }

    const AgvState prev_state = _state[0];
    AgvState pred_state = prev_state;

    // predict new position
    pred_state.pos.x = prev_state.pos.x + (prev_state.vx * cosf(prev_state.theta) - prev_state.vy * sinf(prev_state.theta)) * imu.dt;
    pred_state.pos.y = prev_state.pos.y + (prev_state.vx * sinf(prev_state.theta) + prev_state.vy * cosf(prev_state.theta)) * imu.dt;
    pred_state.theta = _norm_ang(prev_state.theta + imu.wz * imu.dt);

    // add dwm offset to agv center
    Position dwm_pos_agv = dwm.pos;
    abs2agv(dwm_pos_agv, pred_state.theta, dwm_offset);

    // update body velocities from IMU
    pred_state.vx += imu.ax * imu.dt;
    pred_state.vy += imu.ay * imu.dt;

    // compute prediction error against DWM
    float err_x = dwm_pos_agv.x - pred_state.pos.x;
    float err_y = dwm_pos_agv.y - pred_state.pos.y;

    // pull predicted pose toward DWM using a gain
    AgvState upd = pred_state;
    upd.pos.x += _err_co_dwm * err_x;
    upd.pos.y += _err_co_dwm * err_y;
    upd.t_ms = millis();

    // compare against last state for a longer step
    const AgvState &baseline_state = _state[_state.size() - 1];
    float dx = static_cast<float>(upd.pos.x - baseline_state.pos.x);
    float dy = static_cast<float>(upd.pos.y - baseline_state.pos.y);
    float dist = sqrtf(dx * dx + dy * dy);
    float body_speed = sqrtf(pred_state.vx * pred_state.vx + pred_state.vy * pred_state.vy);

    // compute movement in AGV body frame
    float dx_body = dx * cosf(baseline_state.theta) + dy * sinf(baseline_state.theta);
    float dy_body = -dx * sinf(baseline_state.theta) + dy * cosf(baseline_state.theta);
    const float theta_body = atan2f(dy_body, dx_body);
    const bool aligned_body = _heading_adjustment_allowed(theta_body);

    // adjust heading only when motion is straight enough over a long enough step
    if (dist > _heading_dist_threshold_mm && body_speed > _heading_speed_threshold_mm_s && aligned_body)
    {
        float theta_meas = atan2f(dy, dx);
        float theta_err = _norm_ang(theta_meas - upd.theta);
        upd.theta = _norm_ang(upd.theta + _err_co_imu * theta_err);
    }

    if (true)
    {
        const float theta_rep103_deg = upd.theta * 180.0f / PI;
        const float theta_y_forward_deg = _norm_ang((PI / 2.0f) - upd.theta) * 180.0f / PI;

        Serial.println("[SYSCTRL] Calculated values values");
        Serial.print("[DWM] X: ");
        Serial.print(upd.pos.x / 1000.0f, 3);
        Serial.print("  Y: ");
        Serial.print(upd.pos.y / 1000.0f, 3);
        Serial.print("  Z: ");
        Serial.print(upd.pos.z / 1000.0f, 3);
        Serial.print("  ANG_REP103: ");
        Serial.print(theta_rep103_deg);
        Serial.print("  ANG_REL: ");
        Serial.print(theta_y_forward_deg);
        Serial.print("  BODY_MOVE_ANG: ");
        Serial.print(theta_body * 180.0f / PI);
        Serial.print("  ALIGN_OK: ");
        Serial.println(aligned_body ? "YES" : "NO");
        Serial.print("  TIME_NOW: ");
        Serial.println(upd.t_ms);
    }

    _state.push_front(upd);

    // send position update
    Comm::Packet p = {'P', _proto_handler_bt.get_sequence(), {}, 7, 0, true};

    const int16_t x = static_cast<int16_t>(upd.pos.x);
    const int16_t y = static_cast<int16_t>(upd.pos.y);
    const int16_t theta = static_cast<int16_t>(upd.theta * 100); // 2 decimaler

    p.data[0] = static_cast<uint8_t>((x >> 8) & 0xFF);
    p.data[1] = static_cast<uint8_t>(x & 0xFF);

    p.data[2] = static_cast<uint8_t>((y >> 8) & 0xFF);
    p.data[3] = static_cast<uint8_t>(y & 0xFF);

    p.data[4] = static_cast<uint8_t>((theta >> 8) & 0xFF);
    p.data[5] = static_cast<uint8_t>(theta & 0xFF);

    p.data[6] = 0x00;

    p.crc = Comm::csum(p);

    if (_comm_bt.write(p))
    {
        _proto_handler_bt.iterate_sequence();
        _proto_handler_bt.add_buffer_sent(p);
    }
    else if (g_debug.sysctrl)
        Serial.println("[SysCtrl] \033[31mWATNING\033[0m - Failed to send AGV position to [ÖS]");
};

bool SysCtrl::on_startup(DWM &dwm, float dwm_offset)
{
    if (g_debug.sysctrl)
        Serial.print("[SYSCTRL] Calibrating Position...");

    DwmState d1, d2;

    // Ta första mätvärdet
    for (int i = 0; i < 10; i++)
    {
        if (dwm.read(d1))
            break;
        else if (i == 9)
            return false;
        delay(50);
    }

    // Kör fram på MCU2
    Comm::Packet p = {'D', _proto_handler_mcu.get_sequence(), {0x01, 0x32}, 2, 0, true};
    p.crc = Comm::csum(p);
    if (!_forward_to_mcu(p))
    {
        if (g_debug.sysctrl)
            Serial.println("[SysCtrl] \033[31mWATNING\033[0m - Failed to send command: Drive to [MCU2]");
        return false;
    }
    delay(2000);
    on_stop();

    for (int i = 0; i < 10; i++)
    {
        if (dwm.read(d2))
            break;
        else if (i == 9)
            return false;
        delay(50);
    }

    AgvState s;
    s.pos = d2.pos;

    const float dx = static_cast<float>(d2.pos.x - d1.pos.x);
    const float dy = static_cast<float>(d2.pos.y - d1.pos.y);
    s.theta = atan2f(dy, dx);
    abs2agv(s, dwm_offset);
    _state.push_back(s);

    if (g_debug.sysctrl)
        Serial.print("[SYSCTRL] Calibration Complete");

    return true;
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

    _proto_handler_mcu.iterate_sequence();
    _proto_handler_mcu.add_buffer_sent(pkt);
    return true;
}

void SysCtrl::_next_movement(Comm::Packet &pkt)
{
    if (_motion_buffert.size())
        if (_forward_to_mcu(_motion_buffert[0]))
        {
            _motion_buffert.pop(0);

            // clear state buffert except for last (collected) state
            for (int i = 0; i < _state.size() - 1; i++)
                _state.pop_back();
        }
        else
        {
            on_stop();
            if (g_debug.sysctrl)
                Serial.println("[SYSCTRL] \033[31mWARNING\033[0m - Failed to send command: 'next movement' to [MCU2]");
        }
    else
    {
        Comm::Packet p = {'E', _proto_handler_bt.get_sequence(), {'N', pkt.seq}, 1, 0, true};
        p.crc = Comm::csum(p);

        if (_comm_mcu.write(p))
        {
            _proto_handler_bt.iterate_sequence();
            _proto_handler_bt.add_buffer_sent(p);
        }
        else if (g_debug.sysctrl)
            Serial.println("[SysCtrl] \033[31mWATNING\033[0m - Failed send no movement error to [ÖS]");
    }
}
