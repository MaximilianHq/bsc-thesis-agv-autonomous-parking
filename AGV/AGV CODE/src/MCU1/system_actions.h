#pragma once
#include <Arduino.h>
#include <types.h>
#include <comm.h>
#include <static_vector.h>

class StatusLED;
class DWM;
class IMU;

class SysCtrl
{
public:
    SysCtrl(Comm &comm_bt, Comm &comm_mcu, StatusLED &led_sys, StatusLED &led_cmd);

    // ========== IActions ==========
    void on_bt_no_connect();
    void on_bt_pkt_recieved(Comm::Packet &pkt);
    void on_mcu_pkt_recieved(Comm::Packet &pkt);
    void on_new_motion(Comm::Packet &pkt);
    void on_stop();
    void on_stop(Comm::Packet &pkt);
    void on_obstacle_detected(const Position &pos);
    void on_new_position_data(DwmState &dwm, const ImuState &imu, float dwm_offset);
    bool on_startup(DWM &dwm, float dwm_offset);

    void test_move()
    {
        Comm::Packet p = {'D', _proto_handler_bt.get_sequence(), {0x01, 0x32}, 2, 0, true};
        p.crc = Comm::csum(p);

        if (!_forward_to_mcu(p))
            if (g_debug.sysctrl)
                Serial.println("[SysCtrl] \033[31mWATNING\033[0m - Failed send test move command to [ÖS]");
    };

private:
    Comm &_comm_bt;
    Comm &_comm_mcu;
    StatusLED &_led_sys;
    StatusLED &_led_cmd;

    StaticVector<AgvState, 10> _state;
    float _err_co_dwm = 0.25f;
    float _err_co_imu = 0.05f;
    static constexpr float _heading_dist_threshold_mm = 50.0f;
    static constexpr float _heading_speed_threshold_mm_s = 100.0f;
    static constexpr float _heading_alignment_tolerance_rad = PI / 18.0f; // 10 degrees
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
        const float forward_err = fabs(ang);
        const float backward_err = fabs(_norm_ang(ang - PI));
        return forward_err <= _heading_alignment_tolerance_rad ||
               backward_err <= _heading_alignment_tolerance_rad;
    };
    StaticVector<Comm::Packet, 20> _motion_buffert;

    ProtocolHandler _proto_handler_bt, _proto_handler_mcu;

    void _process_bt_packet(Comm::Packet &pkt);
    void _process_mcu_packet(Comm::Packet &pkt);
    bool _forward_to_mcu(Comm::Packet &pkt);
    void _next_movement(Comm::Packet &pkt);
};
