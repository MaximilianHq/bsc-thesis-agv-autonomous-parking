#include "system_actions.h"
#include <types.h>
#include <comm.h>
#include <status_led.h>
#include <dwm.h>
#include "lift.h"

AGVCtrl::AGVCtrl(Comm &comm_mcu, StatusLED &led_sys, StatusLED &led_cmd, DWM &dwm, Lift &crane)
    : _comm_mcu(comm_mcu),
      _proto_handler_mcu(comm_mcu, *this),
      _led_sys(led_sys),
      _led_cmd(led_cmd),
      _dwm(dwm),
      _crane(crane)
{
}

void AGVCtrl::update()
{
    _proto_handler_mcu.update();

    if (_crane_lifting && _crane_done)
    {
        _crane_lifting = false;
        on_lift_done();
    }
}

const AgvState *AGVCtrl::latest_state() const
{
    if (_state.size() == 0)
        return nullptr;

    return &_state[0];
}

void AGVCtrl::on_mcu_pkt_recieved(Comm::Packet &pkt)
{
    _proto_handler_mcu.handle(pkt);

    if (!pkt.approved)
        return;

    _process_mcu_packet(pkt);
}

void AGVCtrl::on_stop()
{
    Comm::Packet p = {'X', 0, {}, 0, 0, true};

    on_stop(p);
}

void AGVCtrl::on_stop(Comm::Packet &pkt)
{
    (void)pkt;

    _proto_handler_mcu.clear_send_queue();

    Comm::Packet stop_mcu = {'X', 0, {}, 0, 0, true};

    if (!_proto_handler_mcu.send_pkt(stop_mcu) && g_debug.sysctrl)
        Serial.println("[SYSCTRL] WATNING - Failed to send command: 'stop' to [MCU2]");
    _led_cmd.set_status(StatusLED::State::STATUS_CMD_STOPPING);
}

void AGVCtrl::on_obstacle_detected(const Position &pos) { _led_cmd.set_status(StatusLED::State::STATUS_OBSTACLE); }

void AGVCtrl::on_new_position_data(DwmState &dwm, const ImuState &imu)
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
        const int theta_0_360_deg = static_cast<int>(_wrap_ang_0_2pi(upd.theta) * 180.0f / PI + 0.5f);

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
        Serial.print("  ANG_360: ");
        Serial.print(theta_0_360_deg);
        Serial.print("  BODY_MOVE_ANG: ");
        Serial.print(theta_body * 180.0f / PI);
        Serial.print("  ALIGN_OK: ");
        Serial.println(aligned_body ? "YES" : "NO");
        Serial.print("  TIME_NOW: ");
        Serial.println(upd.t_ms);
    }

    _state.push_front(upd);
};

bool AGVCtrl::on_startup()
{
    if (g_debug.sysctrl)
        Serial.println("[SYSCTRL] Calibrating AGV Angle...");

    _led_cmd.set_status(StatusLED::State::STATUS_BOOT);

    DwmState d1, d2;

    if (!_read_dwm_average(d1))
        return false;

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

    delay(3000);
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

    if (!_read_dwm_average(d2))
        return false;

    AgvState s;
    s.pos = d2.pos;

    const float dx = static_cast<float>(d2.pos.x - d1.pos.x);
    const float dy = static_cast<float>(d2.pos.y - d1.pos.y);
    s.theta = atan2f(dy, dx);

    if (g_debug.positioning)
    {
        Serial.println("[SYSCTRL] Startup angle calibration");
        Serial.print("[DWM] P1 X: ");
        Serial.print(d1.pos.x / 1000.0f, 3);
        Serial.print("  Y: ");
        Serial.print(d1.pos.y / 1000.0f, 3);
        Serial.print("  Z: ");
        Serial.println(d1.pos.z / 1000.0f, 3);

        Serial.print("[DWM] P2 X: ");
        Serial.print(d2.pos.x / 1000.0f, 3);
        Serial.print("  Y: ");
        Serial.print(d2.pos.y / 1000.0f, 3);
        Serial.print("  Z: ");
        Serial.println(d2.pos.z / 1000.0f, 3);

        Serial.print("[DWM] DX: ");
        Serial.print(dx / 1000.0f, 3);
        Serial.print("  DY: ");
        Serial.println(dy / 1000.0f, 3);

        Serial.print("[SYSCTRL] Startup ANG_REP103: ");
        Serial.print(s.theta * 180.0f / PI);
        Serial.print("  ANG_REL: ");
        Serial.println(_norm_ang((PI / 2.0f) - s.theta) * 180.0f / PI);
    }

    _to_agv_center(s.pos, s.theta);
    _state.clear();
    _state.push_front(s);

    return true;
}

void AGVCtrl::on_lift(bool dir)
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

void AGVCtrl::on_lift_done()
{
    Comm::Packet p = {'L', 0, {}, 0, 0, true};
    Serial.println("[SYSCTRL] Lift done");
}

float AGVCtrl::_norm_ang(float ang)
{
    while (ang > PI)
        ang -= 2.0f * PI;
    while (ang < -PI)
        ang += 2.0f * PI;
    return ang;
};

float AGVCtrl::_wrap_ang_0_2pi(float ang)
{
    while (ang >= 2.0f * PI)
        ang -= 2.0f * PI;
    while (ang < 0.0f)
        ang += 2.0f * PI;
    return ang;
};

bool AGVCtrl::_heading_adjustment_allowed(float ang)
{
    ang = _norm_ang(ang);
    const float nearest = roundf(ang / (PI / 2.0f)) * (PI / 2.0f);
    const float err = fabs(_norm_ang(ang - nearest));
    return err <= _heading_alignment_tolerance_rad;
};

float AGVCtrl::_snap_body_motion_axis(float ang)
{
    ang = _norm_ang(ang);
    return _norm_ang(roundf(ang / (PI / 2.0f)) * (PI / 2.0f));
};

bool AGVCtrl::_read_dwm_average(DwmState &avg, uint8_t samples)
{
    if (samples == 0)
        return false;

    long sum_x = 0;
    long sum_y = 0;
    long sum_z = 0;
    uint8_t last_q = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        DwmState dwm;
        for (int attempts = 0; attempts < 10; attempts++)
        {
            if (_dwm.read(dwm))
                break;
            else if (attempts == 9)
                return false;
            delay(50);
        }

        sum_x += dwm.pos.x;
        sum_y += dwm.pos.y;
        sum_z += dwm.pos.z;
        last_q = dwm.q;
        delay(50);
    }

    avg.pos.x = sum_x / samples;
    avg.pos.y = sum_y / samples;
    avg.pos.z = sum_z / samples;
    avg.q = last_q;
    return true;
}

void AGVCtrl::_process_mcu_packet(Comm::Packet &pkt)
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

void AGVCtrl::_to_agv_center(Position &p, float ang) const
{
    p.x = p.x - _dwm_offset * cosf(ang);
    p.y = p.y - _dwm_offset * sinf(ang);
}

RemoteCtrl::RemoteCtrl(Comm &comm_bt, Comm &comm_mcu, StatusLED &led_sys, StatusLED &led_cmd, DWM &dwm, Lift &crane)
    : AGVCtrl(comm_mcu, led_sys, led_cmd, dwm, crane),
      _comm_bt(comm_bt),
      _proto_handler_bt(comm_bt, *this)
{
}

void RemoteCtrl::update()
{
    _proto_handler_bt.update();
    AGVCtrl::update();
}

void RemoteCtrl::on_bt_no_connect()
{
    on_stop();
    _led_sys.set_status(StatusLED::State::STATUS_BLE_SEARCHING);
}

void RemoteCtrl::on_bt_pkt_recieved(Comm::Packet &pkt)
{
    _proto_handler_bt.handle(pkt);

    if (!pkt.approved)
        return;

    _process_bt_packet(pkt);
    _led_cmd.set_status(StatusLED::State::STATUS_CMD_RECEIVING);
}

void RemoteCtrl::on_stop(Comm::Packet &pkt)
{
    _motion_buffert.clear();
    AGVCtrl::on_stop(pkt);

    if (!_proto_handler_bt.send_pkt(pkt) && g_debug.sysctrl)
        Serial.println("[SYSCTRL] WATNING - Failed to send status: 'stop' to [ÖS]");
}

void RemoteCtrl::on_obstacle_detected(const Position &pos)
{
    Comm::Packet p = {'H', 0, {}, 4, 0, true, false};
    p.data[0] = static_cast<uint8_t>((pos.x >> 8) & 0xFF);
    p.data[1] = static_cast<uint8_t>(pos.x & 0xFF);
    p.data[2] = static_cast<uint8_t>((pos.y >> 8) & 0xFF);
    p.data[3] = static_cast<uint8_t>(pos.y & 0xFF);

    _proto_handler_bt.send_pkt(p, false);
}

void RemoteCtrl::on_new_position_data(DwmState &dwm, const ImuState &imu)
{
    AGVCtrl::on_new_position_data(dwm, imu);
    if (_state.size() == 0)
        return;

    const AgvState &upd = _state[0];

    // send position update
    Comm::Packet p = {'P', 0, {}, 7, 0, true, false};

    const int16_t x = static_cast<int16_t>(upd.pos.x);
    const int16_t y = static_cast<int16_t>(upd.pos.y);
    const float theta_wrapped = _wrap_ang_0_2pi(upd.theta);
    const int16_t theta = static_cast<int16_t>(theta_wrapped * 100); // 2 decimaler, 0..2pi

    p.data[0] = static_cast<uint8_t>((x >> 8) & 0xFF);
    p.data[1] = static_cast<uint8_t>(x & 0xFF);

    p.data[2] = static_cast<uint8_t>((y >> 8) & 0xFF);
    p.data[3] = static_cast<uint8_t>(y & 0xFF);

    p.data[4] = static_cast<uint8_t>((theta >> 8) & 0xFF);
    p.data[5] = static_cast<uint8_t>(theta & 0xFF);

    p.data[6] = 0x00;

    _proto_handler_bt.send_pkt(p, false);
};

bool RemoteCtrl::on_startup()
{
    bool calibrate = AGVCtrl::on_startup();

    Comm::Packet p_comp = {'K', 0, {'C'}, 0, 0, true};

    if (!_proto_handler_bt.send_pkt(p_comp) && g_debug.sysctrl)
        Serial.println("[SYSCTRL] WARNING - Failed to send status: 'Calibration Complete' to [ÖS]");

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

    return calibrate;
}

void RemoteCtrl::on_new_motion(Comm::Packet &pkt)
{
    Comm::Packet clean_pkt = {'D', 0, {}, 2, 0, true};
    clean_pkt.data[0] = pkt.data[0];
    clean_pkt.data[1] = pkt.data[1];
    _motion_buffert.push_back(clean_pkt);
}

void RemoteCtrl::_process_bt_packet(Comm::Packet &pkt)
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
        _led_cmd.set_status(StatusLED::State::STATUS_MOVING);
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

bool RemoteCtrl::_next_movement(Comm::Packet &pkt)
{
    if (_motion_buffert.size())
    {
        if (_proto_handler_mcu.send_pkt(_motion_buffert[0]))
        {
            _motion_buffert.pop(0);

            // clear state buffert except for last (collected) state
            for (int i = 0; i < _state.size() - 1; i++)
                _state.pop_back();
            return true;
        }
        else
        {
            on_stop();
            if (g_debug.sysctrl)
                Serial.println("[SYSCTRL] WARNING - Failed to send command: 'next movement' to [MCU2]");
            return false;
        }
    }

    Comm::Packet p = {'E', 0, {'N', pkt.seq}, 1, 0, true};

    if (!_proto_handler_bt.send_pkt(p) && g_debug.sysctrl)
        Serial.println("[SYSCTRL] WARNING - Failed send no movement error to [ÖS]");

    return false;
}

LocalCtrl::LocalCtrl(Comm &comm_mcu, StatusLED &led_sys, StatusLED &led_cmd, DWM &dwm, Lift &crane)
    : AGVCtrl(comm_mcu, led_sys, led_cmd, dwm, crane)
{
}

bool LocalCtrl::on_startup()
{
    if (g_debug.sysctrl)
        Serial.println("[SYSCTRL] Calibrating local start position...");

    _led_cmd.set_status(StatusLED::State::STATUS_BOOT);

    DwmState dwm_avg;
    if (!_read_dwm_average(dwm_avg))
        return false;

    AgvState s;
    s.pos = dwm_avg.pos;
    s.theta = PI;
    s.t_ms = millis();

    if (g_debug.positioning)
    {
        Serial.println("[SYSCTRL] Local startup position average");
        Serial.print("[DWM] AVG X: ");
        Serial.print(s.pos.x / 1000.0f, 3);
        Serial.print("  Y: ");
        Serial.print(s.pos.y / 1000.0f, 3);
        Serial.print("  Z: ");
        Serial.println(s.pos.z / 1000.0f, 3);
        Serial.print("[SYSCTRL] Startup ANG_REP103: ");
        Serial.print(s.theta * 180.0f / PI);
        Serial.print("  ANG_REL: ");
        Serial.println(_norm_ang((PI / 2.0f) - s.theta) * 180.0f / PI);
    }

    _to_agv_center(s.pos, s.theta);
    _state.clear();
    _state.push_front(s);

    return true;
}

void LocalCtrl::on_new_move(uint8_t move, uint8_t spd)
{
    Comm::Packet p = {'D', 0, {move, spd}, 2, 0, true};
    if (!_proto_handler_mcu.send_pkt(p) && g_debug.sysctrl)
        Serial.println("[SYSCTRL] WATNING - Failed to send movement to [MCU2]");
}

DemoCtrl::DemoCtrl(Comm &comm_mcu, StatusLED &led_sys, StatusLED &led_cmd, DWM &dwm, Lift &crane)
    : LocalCtrl(comm_mcu, led_sys, led_cmd, dwm, crane)
{
}

void DemoCtrl::begin_demo(const DemoPoint &point)
{
    if (!_home_pos_valid)
    {
        const AgvState *state = latest_state();
        if (state != nullptr)
        {
            _home_pos = state->pos;
            _home_pos_valid = true;
        }
    }

    _active_demo_point = point;
    _active_step = 0;
    _step_started = false;
    _demo_is_active = true;
    _demo_is_done = false;
    _theta_err_valid = false;
    _theta_err_has_improved = false;
    _last_theta_abs_err = 0.0f;
    _build_demo_program(point);
}

bool DemoCtrl::demo_active() const { return _demo_is_active; }

bool DemoCtrl::demo_done() const { return _demo_is_done; }

bool DemoCtrl::on_startup()
{
    _program.clear();
    _active_step = 0;
    _step_started = false;
    _demo_is_active = false;
    _demo_is_done = false;
    _theta_err_valid = false;
    _theta_err_has_improved = false;
    _last_theta_abs_err = 0.0f;

    const bool ok = LocalCtrl::on_startup();
    const AgvState *state = latest_state();
    if (ok && state != nullptr)
    {
        _home_pos = state->pos;
        _home_pos_valid = true;
    }

    return ok;
}

bool DemoCtrl::_theta_within_margin(float theta, float target_theta, float margin_deg)
{
    const float margin_rad = margin_deg * PI / 180.0f;
    return _theta_abs_err(theta, target_theta) <= margin_rad;
}

float DemoCtrl::_theta_abs_err(float theta, float target_theta)
{
    return fabs(_norm_ang(theta - target_theta));
}

uint8_t DemoCtrl::_align_cmd_for_target(float theta, float target_theta)
{
    const float err = _norm_ang(target_theta - theta);
    return (err >= 0.0f) ? 0xD9 : 0xD8;
}

bool DemoCtrl::_handle_align_to_theta(DemoStep &step, const char *label)
{
    const AgvState *state = latest_state();
    if (state == nullptr)
        return false;

    if (_theta_within_margin(state->theta, step.target_theta))
    {
        if (g_debug.sysctrl)
        {
            Serial.print("[DEMO] Step ");
            Serial.print(_active_step);
            Serial.print(": ");
            Serial.print(label);
            Serial.print(" satisfied at theta_deg=");
            Serial.println(state->theta * 180.0f / PI);
        }
        on_stop();
        _align_cmd = 0x00;
        _advance_step();
        return true;
    }

    const uint8_t desired_cmd = _align_cmd_for_target(state->theta, step.target_theta);
    if (desired_cmd != _align_cmd)
    {
        _align_cmd = desired_cmd;
        if (g_debug.sysctrl)
        {
            Serial.print("[DEMO] Step ");
            Serial.print(_active_step);
            Serial.print(": ");
            Serial.print(label);
            Serial.print(" correcting cmd=0x");
            Serial.println(_align_cmd, HEX);
        }
        on_new_move(_align_cmd, step.speed == 0x00 ? 0x20 : step.speed);
        return false;
    }

    return false;
}

bool DemoCtrl::_y_within_margin(int16_t y, int16_t target_y, int16_t margin_mm)
{
    return abs(static_cast<int>(y) - static_cast<int>(target_y)) <= margin_mm;
}

uint8_t DemoCtrl::_center_cmd_for_target(int16_t y, int16_t target_y)
{
    return (y < target_y) ? 0xD2 : 0xD3;
}

void DemoCtrl::update()
{
    LocalCtrl::update();

    if (!_demo_is_active)
        return;

    bool advanced = false;
    bool command_issued = false;
    do
    {
        advanced = false;

        if (_active_step >= _program.size())
        {
            if (g_debug.sysctrl)
                Serial.println("[DEMO] Program complete");
            _demo_is_active = false;
            _demo_is_done = true;
            return;
        }

        DemoStep &step = _program[_active_step];

        if (!_step_started)
        {
            _step_started = true;

            switch (step.type)
            {
            case StepType::Move:
                if (g_debug.sysctrl)
                {
                    Serial.print("[DEMO] Step ");
                    Serial.print(_active_step);
                    Serial.print(": Move cmd=0x");
                    Serial.print(step.move, HEX);
                    Serial.print(" speed=");
                    Serial.println(step.speed);
                }
                on_new_move(step.move, step.speed);
                _advance_step();
                advanced = true;
                command_issued = true;
                break;

            case StepType::MoveUntilTheta:
            {
                const AgvState *state = latest_state();
                if (g_debug.sysctrl)
                {
                    Serial.print("[DEMO] Step ");
                    Serial.print(_active_step);
                    Serial.print(": MoveUntilTheta cmd=0x");
                    Serial.print(step.move, HEX);
                    Serial.print(" target_deg=");
                    Serial.println(step.target_theta * 180.0f / PI);
                }
                _align_cmd = 0x00;
                _theta_err_valid = state != nullptr;
                _theta_err_has_improved = false;
                if (_theta_err_valid)
                    _last_theta_abs_err = _theta_abs_err(state->theta, step.target_theta);
                on_new_move(step.move, step.speed);
                command_issued = true;
                break;
            }

            case StepType::LiftUp:
                if (g_debug.sysctrl)
                {
                    Serial.print("[DEMO] Step ");
                    Serial.print(_active_step);
                    Serial.println(": LiftUp");
                }
                on_lift(true);
                _crane_lifting = true;
                _led_sys.set_status(StatusLED::State::STATUS_LIFTING);
                command_issued = true;
                break;

            case StepType::LiftDown:
                if (g_debug.sysctrl)
                {
                    Serial.print("[DEMO] Step ");
                    Serial.print(_active_step);
                    Serial.println(": LiftDown");
                }
                on_lift(false);
                _crane_lifting = true;
                _led_sys.set_status(StatusLED::State::STATUS_LIFTING);
                command_issued = true;
                break;

            case StepType::WaitUntil:
                if (g_debug.sysctrl)
                {
                    Serial.print("[DEMO] Step ");
                    Serial.print(_active_step);
                    Serial.println(": WaitUntil");
                }
                break;

            case StepType::RecalibrateAngle:
                if (g_debug.sysctrl)
                {
                    Serial.print("[DEMO] Step ");
                    Serial.print(_active_step);
                    Serial.println(": RecalibrateAngle");
                }
                if (!AGVCtrl::on_startup())
                {
                    if (g_debug.sysctrl)
                        Serial.println("[DEMO] Recalibration failed");
                    _demo_is_active = false;
                    _demo_is_done = true;
                    on_stop();
                    return;
                }
                _advance_step();
                advanced = true;
                break;

            case StepType::AlignToTheta:
                if (g_debug.sysctrl)
                {
                    Serial.print("[DEMO] Step ");
                    Serial.print(_active_step);
                    Serial.print(": AlignToTheta target_deg=");
                    Serial.println(step.target_theta * 180.0f / PI);
                }
                advanced = _handle_align_to_theta(step, "AlignToTheta");
                command_issued = !advanced;
                break;

            case StepType::AlignToPi:
            {
                if (g_debug.sysctrl)
                {
                    Serial.print("[DEMO] Step ");
                    Serial.print(_active_step);
                    Serial.println(": AlignToPi");
                }
                step.target_theta = PI;
                advanced = _handle_align_to_theta(step, "AlignToPi");
                command_issued = !advanced;
                break;
            }

            case StepType::AlignToCenter:
            {
                const AgvState *state = latest_state();
                if (state == nullptr)
                    break;

                if (_y_within_margin(state->pos.y, step.target_y))
                {
                    if (g_debug.sysctrl)
                    {
                        Serial.print("[DEMO] Step ");
                        Serial.print(_active_step);
                        Serial.println(": AlignToCenter already satisfied");
                    }
                    on_stop();
                    _center_cmd = 0x00;
                    _advance_step();
                    advanced = true;
                    break;
                }

                _center_cmd = _center_cmd_for_target(state->pos.y, step.target_y);
                if (g_debug.sysctrl)
                {
                    Serial.print("[DEMO] Step ");
                    Serial.print(_active_step);
                    Serial.print(": AlignToCenter cmd=0x");
                    Serial.print(_center_cmd, HEX);
                    Serial.print(" target_y=");
                    Serial.println(step.target_y);
                }
                on_new_move(_center_cmd, step.speed == 0x00 ? 0x20 : step.speed);
                command_issued = true;
                break;
            }

            case StepType::Delay:
                if (g_debug.sysctrl)
                {
                    Serial.print("[DEMO] Step ");
                    Serial.print(_active_step);
                    Serial.print(": Delay ms=");
                    Serial.println(step.delay_ms);
                }
                _delay_start_ms = millis();
                command_issued = true;
                break;

            case StepType::Stop:
                if (g_debug.sysctrl)
                {
                    Serial.print("[DEMO] Step ");
                    Serial.print(_active_step);
                    Serial.println(": Stop");
                }
                on_stop();
                _advance_step();
                advanced = true;
                command_issued = true;
                break;
            }
        }
        else if ((step.type == StepType::LiftUp || step.type == StepType::LiftDown) && !_crane_lifting)
        {
            _advance_step();
            advanced = true;
        }
        else if (step.type == StepType::MoveUntilTheta)
        {
            const AgvState *state = latest_state();
            if (state != nullptr && _theta_within_margin(state->theta, step.target_theta))
            {
                if (g_debug.sysctrl)
                {
                    Serial.print("[DEMO] Step ");
                    Serial.print(_active_step);
                    Serial.print(": MoveUntilTheta satisfied at theta_deg=");
                    Serial.println(state->theta * 180.0f / PI);
                }
                on_stop();
                _align_cmd = 0x00;
                _theta_err_valid = false;
                _theta_err_has_improved = false;
                _advance_step();
                advanced = true;
            }
            else if (state != nullptr)
            {
                const float theta_abs_err = _theta_abs_err(state->theta, step.target_theta);
                const float err_hysteresis = 0.5f * PI / 180.0f;
                const bool theta_error_improved = _theta_err_valid && theta_abs_err < _last_theta_abs_err - err_hysteresis;
                const bool theta_error_growing = _theta_err_has_improved && theta_abs_err > _last_theta_abs_err + err_hysteresis;

                if (!_theta_err_valid)
                    _last_theta_abs_err = theta_abs_err;
                else if (theta_error_improved)
                {
                    _last_theta_abs_err = theta_abs_err;
                    _theta_err_has_improved = true;
                }
                _theta_err_valid = true;

                if (theta_error_growing)
                {
                    const uint8_t desired_cmd = _align_cmd_for_target(state->theta, step.target_theta);
                    _last_theta_abs_err = theta_abs_err;
                    if (desired_cmd != _align_cmd)
                    {
                        _align_cmd = desired_cmd;
                        if (g_debug.sysctrl)
                        {
                            Serial.print("[DEMO] Step ");
                            Serial.print(_active_step);
                            Serial.print(": MoveUntilTheta correcting cmd=0x");
                            Serial.print(_align_cmd, HEX);
                            Serial.print(" err_deg=");
                            Serial.println(theta_abs_err * 180.0f / PI);
                        }
                        on_new_move(_align_cmd, step.speed == 0x00 ? 0x20 : step.speed);
                        command_issued = true;
                    }
                }
            }
        }
        else if (step.type == StepType::AlignToTheta)
        {
            advanced = _handle_align_to_theta(step, "AlignToTheta");
            command_issued = !advanced;
        }
        else if (step.type == StepType::AlignToPi)
        {
            step.target_theta = PI;
            advanced = _handle_align_to_theta(step, "AlignToPi");
            command_issued = !advanced;
        }
        else if (step.type == StepType::AlignToCenter)
        {
            const AgvState *state = latest_state();
            if (state != nullptr && _y_within_margin(state->pos.y, step.target_y))
            {
                if (g_debug.sysctrl)
                {
                    Serial.print("[DEMO] Step ");
                    Serial.print(_active_step);
                    Serial.print(": AlignToCenter satisfied at y=");
                    Serial.println(state->pos.y);
                }
                on_stop();
                _center_cmd = 0x00;
                _advance_step();
                advanced = true;
            }
            else if (state != nullptr)
            {
                const uint8_t desired_cmd = _center_cmd_for_target(state->pos.y, step.target_y);
                if (desired_cmd != _center_cmd)
                {
                    _center_cmd = desired_cmd;
                    if (g_debug.sysctrl)
                    {
                        Serial.print("[DEMO] Step ");
                        Serial.print(_active_step);
                        Serial.print(": AlignToCenter correcting cmd=0x");
                        Serial.println(_center_cmd, HEX);
                    }
                    on_new_move(_center_cmd, step.speed == 0x00 ? 0x20 : step.speed);
                    command_issued = true;
                }
            }
        }
        else if (step.type == StepType::Delay)
        {
            if (millis() - _delay_start_ms >= step.delay_ms)
            {
                _advance_step();
                advanced = true;
            }
        }
        else if (step.type == StepType::WaitUntil &&
                 step.wait_condition != nullptr)
        {
            const AgvState *state = latest_state();
            if (state != nullptr && step.wait_condition(*state, _active_demo_point))
            {
                if (g_debug.sysctrl)
                {
                    Serial.print("[DEMO] Step ");
                    Serial.print(_active_step);
                    Serial.print(": WaitUntil satisfied at x=");
                    Serial.print(state->pos.x);
                    Serial.print(" y=");
                    Serial.print(state->pos.y);
                    Serial.print(" theta_deg=");
                    Serial.println(state->theta * 180.0f / PI);
                }
                _advance_step();
                advanced = true;
            }
        }

        if (command_issued)
            break;
    } while (advanced);
}

void DemoCtrl::_build_demo_program(const DemoPoint &point)
{
    _program.clear();
    const uint8_t spd = 50;
    const int16_t car_len = 730;       // TODO: set actual car length in mm
    const int16_t garage_mid_y = 1500; // TODO: set actual garage midpoint on x
    const Position home = _home_pos_valid ? _home_pos : latest_state()->pos;

    switch (point.approach_type)
    {
    case 'V':
        // _program.push_back({StepType::LiftUp, 0x00, 0x00, nullptr});

        // Forward
        _program.push_back({StepType::Move, 0xD0, spd, nullptr});
        _program.push_back({StepType::WaitUntil, 0x00, 0x00,
                            [](const AgvState &state, const DemoPoint &point) -> bool
                            {
                                (void)point;
                                return state.pos.x <= point.p.x - car_len * 3 / 4;
                            }});

        // Lateral arc left
        _program.push_back({StepType::MoveUntilTheta, 0xDD, spd, nullptr, 0, 3.0f * PI / 2.0f});

        // Backward
        _program.push_back({StepType::Move, 0xD1, spd, nullptr});
        _program.push_back({StepType::WaitUntil, 0x00, 0x00,
                            [](const AgvState &state, const DemoPoint &point) -> bool
                            {
                                (void)point;
                                return state.pos.y >= point.p.y - car_len;
                            }});

        _program.push_back({StepType::Stop, 0xD0, 0x00, nullptr});
        _program.push_back({StepType::Delay, 0x00, 0x00, nullptr, 0, 0.0f, 1000});
        // _program.push_back({StepType::LiftDown, 0x00, 0x00, nullptr});

        // Forward
        _program.push_back({StepType::Move, 0xD0, spd, nullptr});
        _program.push_back({StepType::WaitUntil, 0x00, 0x00,
                            [garage_mid_y](const AgvState &state, const DemoPoint &point) -> bool
                            {
                                (void)point;
                                return state.pos.y <= garage_mid_y;
                            }});

        // Rotate right
        _program.push_back({StepType::MoveUntilTheta, 0xD8, spd / 2, nullptr, 0, PI});

        // Backward
        _program.push_back({StepType::Move, 0xD1, spd, nullptr});
        _program.push_back({StepType::WaitUntil, 0x00, 0x00,
                            [home](const AgvState &state, const DemoPoint &point) -> bool
                            {
                                (void)point;
                                return state.pos.x >= home.x - car_len;
                            }});

        _program.push_back({StepType::Stop, 0x00, 0x00, nullptr});
        break;
    case 'H':
        // _program.push_back({StepType::LiftUp, 0x00, 0x00, nullptr});

        // Forward
        _program.push_back({StepType::Move, 0xD0, spd, nullptr});
        _program.push_back({StepType::WaitUntil, 0x00, 0x00,
                            [](const AgvState &state, const DemoPoint &point) -> bool
                            {
                                (void)point;
                                return state.pos.x <= point.p.x - car_len / 2.5;
                            }});

        // Lateral arc right
        _program.push_back({StepType::MoveUntilTheta, 0xDC, spd, nullptr, 0, PI / 2.0f});

        // Backward
        _program.push_back({StepType::Move, 0xD1, spd, nullptr});
        _program.push_back({StepType::WaitUntil, 0x00, 0x00,
                            [](const AgvState &state, const DemoPoint &point) -> bool
                            {
                                (void)point;
                                return state.pos.y <= point.p.y + car_len;
                            }});

        // Lateral arc left
        _program.push_back({StepType::MoveUntilTheta, 0xDD, spd, nullptr, 0, PI});

        _program.push_back({StepType::Delay, 0x00, 0x00, nullptr, 0, 0.0f, 1000});
        // _program.push_back({StepType::LiftDown, 0x00, 0x00, nullptr});

        // Forward
        _program.push_back({StepType::Move, 0xD0, spd, nullptr});
        _program.push_back({StepType::WaitUntil, 0x00, 0x00,
                            [garage_mid_y](const AgvState &state, const DemoPoint &point) -> bool
                            {
                                (void)point;
                                return state.pos.x <= point.p.x - car_len * 2 / 3;
                            }});

        // Align to garage center on y-axis
        _program.push_back({StepType::AlignToCenter, 0x00, spd, nullptr, garage_mid_y});

        // Backward
        _program.push_back({StepType::Move, 0xD1, spd, nullptr});
        _program.push_back({StepType::WaitUntil, 0x00, 0x00,
                            [home](const AgvState &state, const DemoPoint &point) -> bool
                            {
                                (void)point;
                                return state.pos.x >= home.x - car_len;
                            }});

        _program.push_back({StepType::Stop, 0x00, 0x00, nullptr});
        break;
    default:
        break;
    }

    _program.push_back({StepType::RecalibrateAngle, 0x00, 0x20, nullptr});
    // Adjust angle
    _program.push_back({StepType::AlignToPi, 0x00, 0x20, nullptr});
    // Align to garage center on y-axis
    _program.push_back({StepType::AlignToCenter, 0x00, 0x20, nullptr, garage_mid_y});

    // Backward
    _program.push_back({StepType::Move, 0xD1, spd, nullptr});
    _program.push_back({StepType::WaitUntil, 0x00, 0x00,
                        [home](const AgvState &state, const DemoPoint &point) -> bool
                        {
                            (void)point;
                            return state.pos.x >= home.x;
                        }});

    _program.push_back({StepType::Stop, 0xD0, 0x00, nullptr});
    // _program.push_back({StepType::LiftUp, 0x00, 0x00, nullptr});
    _program.push_back({StepType::Delay, 0x00, 0x00, nullptr, 0, 0.0f, 2000});
}

void DemoCtrl::_advance_step()
{
    if (_active_step < _program.size())
        _active_step++;

    _step_started = false;
}
