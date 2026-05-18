#include "system_actions.h"
#include <types.h>
#include <status_led.h>
#include <dwm.h>
#include "lift.h"

SysCtrl::SysCtrl(Comm &comm_bt, Comm &comm_mcu, StatusLED &led_sys, StatusLED &led_cmd, DWM &dwm, Lift &crane)
    : _comm_bt(comm_bt),
      _comm_mcu(comm_mcu),
      _proto_handler_bt(comm_bt, *this),
      _proto_handler_mcu(comm_mcu, *this),
      _led_sys(led_sys),
      _led_cmd(led_cmd),
      _dwm(dwm),
      _crane(crane) {}

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

void SysCtrl::on_new_motion(Comm::Packet &pkt)
{
    Comm::Packet clean_pkt = {'D', 0, {}, 2, 0, true};
    clean_pkt.data[0] = pkt.data[0];
    clean_pkt.data[1] = pkt.data[1];
    _motion_buffert.push_back(clean_pkt);
}

void SysCtrl::on_stop()
{
    Comm::Packet p = {'X', 0, {}, 0, 0, true};

    on_stop(p);
}

void SysCtrl::on_stop(Comm::Packet &pkt)
{
    (void)pkt;

    _motion_buffert.clear();
    _proto_handler_mcu.clear_send_queue();

    Comm::Packet stop_mcu = {'X', 0, {}, 0, 0, true};
    Comm::Packet stop_bt = {'X', 0, {}, 0, 0, true};

    if (!_proto_handler_mcu.send_pkt(stop_mcu) && g_debug.sysctrl)
        Serial.println("[SysCtrl] WATNING - Failed to send command: 'stop' to [MCU2]");
    if (!_proto_handler_bt.send_pkt(stop_bt) && g_debug.sysctrl)
        Serial.println("[SysCtrl] WATNING - Failed to send status: 'stop' to [ÖS]");
    _led_cmd.set_status(StatusLED::State::STATUS_CMD_STOPPING);
}

void SysCtrl::on_obstacle_detected(const Position &pos)
{
    Comm::Packet p = {'H', 0, {}, 4, 0, true, false};
    p.data[0] = static_cast<uint8_t>((pos.x >> 8) & 0xFF);
    p.data[1] = static_cast<uint8_t>(pos.x & 0xFF);
    p.data[2] = static_cast<uint8_t>((pos.y >> 8) & 0xFF);
    p.data[3] = static_cast<uint8_t>(pos.y & 0xFF);

    _proto_handler_bt.send_pkt(p, false);

    _led_cmd.set_status(StatusLED::State::STATUS_OBSTACLE);
}

void SysCtrl::on_new_position_data(DwmState &dwm, const ImuState &imu)
{
    if (g_debug.positioning)
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
    _to_agv_center(dwm_pos_agv, pred_state.theta);

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
    const bool meaningful_motion = dist > _heading_dist_threshold_mm && body_speed > _heading_speed_threshold_mm_s;

    // compute movement in AGV body frame only when the step is large enough
    float theta_body = _last_body_move_ang;
    bool aligned_body = false;
    if (meaningful_motion)
    {
        float dx_body = dx * cosf(baseline_state.theta) + dy * sinf(baseline_state.theta);
        float dy_body = -dx * sinf(baseline_state.theta) + dy * cosf(baseline_state.theta);
        theta_body = atan2f(dy_body, dx_body);
        _last_body_move_ang = theta_body;
        aligned_body = _heading_adjustment_allowed(theta_body);
    }

    // adjust heading only when motion is straight enough over a long enough step
    if (meaningful_motion && aligned_body)
    {
        float theta_meas = atan2f(dy, dx);
        float theta_body_axis = _snap_body_motion_axis(theta_body);
        float theta_from_motion = _norm_ang(theta_meas - theta_body_axis);
        float theta_err = _norm_ang(theta_from_motion - upd.theta);
        upd.theta = _norm_ang(upd.theta + _err_co_imu * theta_err);
    }

    if (g_debug.positioning)
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
    Comm::Packet p = {'P', 0, {}, 7, 0, true, false};

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

    !_proto_handler_bt.send_pkt(p, false);
};

bool SysCtrl::on_startup()
{
    if (g_debug.sysctrl)
        Serial.println("[SYSCTRL] Calibrating AGV Angle...");

    _led_cmd.set_status(StatusLED::State::STATUS_BOOT);

    DwmState d1, d2;

    // Ta första mätvärdet
    for (int i = 0; i < 10; i++)
    {
        if (_dwm.read(d1))
            break;
        else if (i == 9)
            return false;
        delay(50);
    }

    // Kör fram på MCU2
    Comm::Packet p_drive = {'D', 0, {0xD0, 0x32}, 2, 0, true};

    if (!_proto_handler_mcu.send_pkt(p_drive))
    {
        if (g_debug.sysctrl)
            Serial.println("[SYSCTRL] WARNING - Failed to send command: Drive to [MCU2]");
        return false;
    }

    if (g_debug.sysctrl)
        Serial.println("[SYSCTRL] WARNING - Entering endless loop");
    do
    {
        Comm::Packet mcu_pkt;
        if (_comm_mcu.read(mcu_pkt))
            on_mcu_pkt_recieved(mcu_pkt);

        _proto_handler_mcu.update();
        delay(20);
    } while (!_proto_handler_mcu.get_sequence().avalible);

    delay(1000);
    on_stop();

    if (g_debug.sysctrl)
        Serial.println("[SYSCTRL] WARNING - Entering endless loop");
    do
    {
        Comm::Packet mcu_pkt;
        if (_comm_mcu.read(mcu_pkt))
            on_mcu_pkt_recieved(mcu_pkt);

        _proto_handler_mcu.update();
        delay(20);
    } while (!_proto_handler_mcu.get_sequence().avalible);

    for (int i = 0; i < 10; i++)
    {
        if (_dwm.read(d2))
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
    _to_agv_center(s.pos, s.theta);
    _state.push_back(s);

    Comm::Packet p_comp = {'K', 0, {'C'}, 0, 0, true};

    if (!_proto_handler_bt.send_pkt(p_comp) && g_debug.sysctrl)
        Serial.println("[SysCtrl] WARNING - Failed to send status: 'Calibration Complete' to [ÖS]");

    if (g_debug.sysctrl)
        Serial.println("[SYSCTRL] WARNING - Entering endless loop");
    do
    {
        Comm::Packet bt_pkt;
        if (_comm_bt.read(bt_pkt))
            on_bt_pkt_recieved(bt_pkt);

        _proto_handler_bt.update();
        delay(20);
    } while (!_proto_handler_bt.get_sequence().avalible);

    if (g_debug.sysctrl)
        Serial.println("[SYSCTRL] Calibration Complete");

    return true;
}

void SysCtrl::on_lift(bool dir)
{
    _crane_done = false;
    if (dir)
    {
        Serial.println("[SYSCTRL] Lifting up");
        _crane.lift(100, _crane_done);
    }
    else
    {
        Serial.println("[SYSCTRL] Lowering down");
        _crane.lower(100, _crane_done);
    }
}

void SysCtrl::on_lift_done()
{
    Comm::Packet p = {'L', 0, {}, 0, 0, true};
    Serial.println("[SYSCTRL] Lift done");
}

void SysCtrl::_process_bt_packet(Comm::Packet &pkt)
{
    switch (pkt.type)
    {
    case 'D':
        if (g_debug.sysctrl)
        {
            Serial.print("[SYSCTRL] New movement: ");
            Serial.println(pkt.data[0]);
        }
        on_new_motion(pkt);
        _led_sys.set_status(StatusLED::State::STATUS_MOVING);
        break;
    case 'K':
        switch (pkt.data[0])
        {
        case 'N':
            if (g_debug.sysctrl)
                Serial.println("[SYSCTRL] Next movement");
            _next_movement(pkt);
            break;
        case 'C':
            on_startup();
            break;
        case 'L': // lift command
            on_lift(pkt.data[0] != 0);
            _crane_lifting = true;
            _led_sys.set_status(StatusLED::State::STATUS_LIFTING);
            break;
        default:
            break;
        }
        break;
    case 'X': // critical stop
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

void SysCtrl::_next_movement(Comm::Packet &pkt)
{
    if (_motion_buffert.size())
        if (_proto_handler_mcu.send_pkt(_motion_buffert[0]))
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
                Serial.println("[SYSCTRL] WARNING - Failed to send command: 'next movement' to [MCU2]");
        }
    else
    {
        Comm::Packet p = {'E', 0, {'N', pkt.seq}, 1, 0, true};

        if (!_proto_handler_bt.send_pkt(p) && g_debug.sysctrl)
            Serial.println("[SYSCTRL] WARNING - Failed send no movement error to [ÖS]");
    }
}

void SysCtrl::_to_agv_center(Position &p, float ang) const
{
    p.x = p.x - _dwm_offset * cosf(ang);
    p.y = p.y - _dwm_offset * sinf(ang);
}
