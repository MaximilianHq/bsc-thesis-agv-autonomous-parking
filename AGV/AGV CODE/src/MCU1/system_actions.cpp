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
};

bool AGVCtrl::on_startup()
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

    delay(2000);
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
    _active_demo_point = point;
    _active_step = 0;
    _step_started = false;
    _demo_is_active = true;
    _demo_is_done = false;
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

    return AGVCtrl::on_startup();
}

void DemoCtrl::update()
{
    LocalCtrl::update();

    if (!_demo_is_active)
        return;

    bool advanced = false;
    do
    {
        advanced = false;

        if (_active_step >= _program.size())
        {
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
                on_new_move(step.move, step.speed);
                _advance_step();
                advanced = true;
                break;

            case StepType::LiftUp:
                on_lift(true);
                _crane_lifting = true;
                _led_sys.set_status(StatusLED::State::STATUS_LIFTING);
                break;

            case StepType::LiftDown:
                on_lift(false);
                _crane_lifting = true;
                _led_sys.set_status(StatusLED::State::STATUS_LIFTING);
                break;

            case StepType::WaitUntil:
                break;

            case StepType::Stop:
                on_stop();
                _advance_step();
                advanced = true;
                break;
            }
        }
        else if ((step.type == StepType::LiftUp || step.type == StepType::LiftDown) && !_crane_lifting)
        {
            _advance_step();
            advanced = true;
        }
        else if (step.type == StepType::WaitUntil &&
                 step.wait_condition != nullptr)
        {
            const AgvState *state = latest_state();
            if (state != nullptr && step.wait_condition(*state, _active_demo_point))
            {
                _advance_step();
                advanced = true;
            }
        }
    } while (advanced);
}

void DemoCtrl::_build_demo_program(const DemoPoint &point)
{
    _program.clear();
    const uint8_t spd = 50;
    const int16_t car_len = 0;      // TODO: set actual car length in mm
    const int16_t garage_mid_x = 0; // TODO: set actual garage midpoint on x
    const Position home = latest_state() ? latest_state()->pos : Position{};
    const float start_theta = latest_state() ? latest_state()->theta : 0.0f;
    const float left_turn_target = _wrap_ang_0_2pi(start_theta + (PI / 2.0f));
    const float right_turn_target = _wrap_ang_0_2pi(start_theta);
    const int16_t target_x_with_car = point.p.x + car_len;
    const int16_t target_y_with_half_car = point.p.y + (car_len / 2);

    delay(5000);

    _program.push_back({StepType::LiftUp, 0x00, 0x00, nullptr});

    // Forward
    _program.push_back({StepType::Move, 0x00, spd, nullptr});
    _program.push_back({StepType::WaitUntil, 0x00, 0x00,
                        [target_x_with_car](const AgvState &state, const DemoPoint &point) -> bool
                        {
                            (void)point;
                            return state.pos.x > target_x_with_car;
                        }});

    // Lateral arc left
    _program.push_back({StepType::Move, 0xDD, spd, nullptr});
    _program.push_back({StepType::WaitUntil, 0x00, 0x00,
                        [left_turn_target](const AgvState &state, const DemoPoint &point) -> bool
                        {
                            (void)point;
                            return DemoCtrl::_wrap_ang_0_2pi(state.theta) >= left_turn_target;
                        }});

    // Backward
    _program.push_back({StepType::Move, 0x01, spd, nullptr});
    _program.push_back({StepType::WaitUntil, 0x00, 0x00,
                        [target_y_with_half_car](const AgvState &state, const DemoPoint &point) -> bool
                        {
                            (void)point;
                            return state.pos.y > target_y_with_half_car;
                        }});

    _program.push_back({StepType::LiftDown, 0x00, 0x00, nullptr});

    // Forward
    _program.push_back({StepType::Move, 0xD0, spd, nullptr});
    _program.push_back({StepType::WaitUntil, 0x00, 0x00,
                        [garage_mid_x](const AgvState &state, const DemoPoint &point) -> bool
                        {
                            (void)point;
                            return state.pos.x >= garage_mid_x;
                        }});

    // Rotate right
    _program.push_back({StepType::Move, 0xD8, spd, nullptr});
    _program.push_back({StepType::WaitUntil, 0x00, 0x00,
                        [right_turn_target](const AgvState &state, const DemoPoint &point) -> bool
                        {
                            (void)point;
                            return DemoCtrl::_wrap_ang_0_2pi(state.theta) <= right_turn_target;
                        }});

    // Backward
    _program.push_back({StepType::Move, 0x01, spd, nullptr});
    _program.push_back({StepType::WaitUntil, 0x00, 0x00,
                        [home](const AgvState &state, const DemoPoint &point) -> bool
                        {
                            (void)point;
                            return state.pos.x > home.x;
                        }});

    _program.push_back({StepType::Stop, 0x00, 0x00, nullptr});
}

void DemoCtrl::_advance_step()
{
    if (_active_step < _program.size())
        _active_step++;

    _step_started = false;
}
