#pragma once
#include <Arduino.h>
#include <types.h>
#include <comm.h>
#include <static_vector.h>

class StatusLED;
class DWM;
class Lift;

class SysCtrl
{
public:
    SysCtrl(Comm &comm_bt, Comm &comm_mcu, StatusLED &led_sys, StatusLED &led_cmd, DWM &dwm, Lift &crane);

    void update()
    {
        _proto_handler_bt.update();
        _proto_handler_mcu.update();
    };

    // ========== IActions ==========
    void on_bt_no_connect();
    void on_bt_pkt_recieved(Comm::Packet &pkt);
    void on_mcu_pkt_recieved(Comm::Packet &pkt);
    void on_new_motion(Comm::Packet &pkt);
    void on_stop();
    void on_stop(Comm::Packet &pkt);
    void on_obstacle_detected(const Position &pos);
    void on_new_position_data(DwmState &dwm, const ImuState &imu);
    bool on_startup();
    void on_lift(bool dir);
    void on_lift_done();

private:
    Comm &_comm_bt;
    Comm &_comm_mcu;
    StaticVector<AgvState, 10> _state;
    StaticVector<Comm::Packet, 20> _motion_buffert;
    ProtocolHandler _proto_handler_bt, _proto_handler_mcu;

    StatusLED &_led_sys;
    StatusLED &_led_cmd;

    DWM &_dwm;
    const float _dwm_offset = 75.0f;
    const float _err_co_dwm = 0.25f;
    const float _err_co_imu = 0.05f;
    float _last_body_move_ang = 0.0f;
    static constexpr float _heading_dist_threshold_mm = 50.0f;
    static constexpr float _heading_speed_threshold_mm_s = 100.0f;
    static constexpr float _heading_alignment_tolerance_rad = PI / 18.0f; // 10 degrees

    Lift &_crane;
    bool _crane_done = false;

    static float _norm_ang(float ang)
    {
        while (ang > PI)
            ang -= 2.0f * PI;
        while (ang < -PI)
            ang += 2.0f * PI;
        return ang;
    };

    static bool _heading_adjustment_allowed(float ang)
    {
        ang = _norm_ang(ang);
        const float nearest = roundf(ang / (PI / 2.0f)) * (PI / 2.0f);
        const float err = fabs(_norm_ang(ang - nearest));
        return err <= _heading_alignment_tolerance_rad;
    };

    static float _snap_body_motion_axis(float ang)
    {
        ang = _norm_ang(ang);
        return _norm_ang(roundf(ang / (PI / 2.0f)) * (PI / 2.0f));
    };

    void _process_bt_packet(Comm::Packet &pkt);
    void _process_mcu_packet(Comm::Packet &pkt);
    void _next_movement(Comm::Packet &pkt);
    void _to_agv_center(Position &p, float ang) const;
};
