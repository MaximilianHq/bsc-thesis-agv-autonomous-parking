#pragma once
#include <Arduino.h>
#include <types.h>
#include <comm.h>
#include <static_vector.h>
#include <functional>

class StatusLED;
class Comm;
class DWM;
class Lift;

class AGVCtrl
{
public:
    AGVCtrl(Comm &comm_mcu, StatusLED &led_sys,
            StatusLED &led_cmd, DWM &dwm, Lift &crane);

    virtual void update();
    const AgvState *latest_state() const;

    void on_mcu_pkt_recieved(Comm::Packet &pkt);
    void on_stop();
    virtual void on_stop(Comm::Packet &pkt);
    virtual void on_obstacle_detected(const Position &pos);
    virtual void on_new_position_data(DwmState &dwm, const ImuState &imu);
    virtual bool on_startup();
    void on_lift(bool dir);
    void on_lift_done();

protected:
    Comm &_comm_mcu;
    StaticVector<AgvState, 10> _state;
    ProtocolHandler _proto_handler_mcu;

    StatusLED &_led_sys;
    StatusLED &_led_cmd;

    DWM &_dwm;
    const float _dwm_offset = 5.0f;
    const float _err_co_dwm = 0.75f;
    const float _err_co_imu = 0.00f;
    float _last_body_move_ang = 0.0f;
    static constexpr float _heading_dist_threshold_mm = 50.0f;
    static constexpr float _heading_speed_threshold_mm_s = 100.0f;
    static constexpr float _heading_alignment_tolerance_rad = PI / 18.0f; // 10 degrees

    Lift &_crane;
    bool _crane_lifting = false;
    bool _crane_done = false;

    static float _norm_ang(float ang);
    static float _wrap_ang_0_2pi(float ang);
    static bool _heading_adjustment_allowed(float ang);
    static float _snap_body_motion_axis(float ang);

    bool _read_dwm_average(DwmState &avg, uint8_t samples = 10);
    void _process_mcu_packet(Comm::Packet &pkt);
    void _to_agv_center(Position &p, float ang) const;
};

class RemoteCtrl : public AGVCtrl
{
public:
    using AGVCtrl::on_stop;

    RemoteCtrl(Comm &comm_bt, Comm &comm_mcu, StatusLED &led_sys,
               StatusLED &led_cmd, DWM &dwm, Lift &crane);

    void update() override;

    void on_bt_no_connect();
    void on_bt_pkt_recieved(Comm::Packet &pkt);
    void on_stop(Comm::Packet &pkt) override;
    void on_obstacle_detected(const Position &pos) override;
    void on_new_position_data(DwmState &dwm, const ImuState &imu) override;
    bool on_startup() override;

private:
    void on_new_motion(Comm::Packet &pkt);
    void _process_bt_packet(Comm::Packet &pkt);
    bool _next_movement(Comm::Packet &pkt);

    Comm &_comm_bt;
    StaticVector<Comm::Packet, 20> _motion_buffert;
    ProtocolHandler _proto_handler_bt;
};

class LocalCtrl : public AGVCtrl
{
public:
    LocalCtrl(Comm &comm_mcu, StatusLED &led_sys,
              StatusLED &led_cmd, DWM &dwm, Lift &crane);

    bool on_startup() override;
    void on_new_move(uint8_t move, uint8_t spd = 0x32);
};

class DemoCtrl : public LocalCtrl
{
public:
    enum class StepType
    {
        Move,
        LiftUp,
        LiftDown,
        WaitUntil,
        RecalibrateAngle,
        AlignToPi,
        AlignToCenter,
        Stop,
    };

    struct DemoPoint
    {
        Position p;
        char approach_type = 0x00;
    };

    using WaitCondition = std::function<bool(const AgvState &state, const DemoPoint &point)>;

    struct DemoStep
    {
        StepType type = StepType::Stop;
        uint8_t move = 0x00;
        uint8_t speed = 0x00;
        WaitCondition wait_condition = nullptr;
        int16_t target_y = 0;
    };

    DemoCtrl(Comm &comm_mcu, StatusLED &led_sys,
             StatusLED &led_cmd, DWM &dwm, Lift &crane);

    void begin_demo(const DemoPoint &point);
    bool demo_active() const;
    bool demo_done() const;
    bool on_startup() override;
    void update() override;

private:
    static bool _theta_within_margin(float theta, float target_theta, float margin_deg = 1.5f);
    static uint8_t _align_cmd_for_target(float theta, float target_theta);
    static bool _y_within_margin(int16_t y, int16_t target_y, int16_t margin_mm = 20);
    static uint8_t _center_cmd_for_target(int16_t y, int16_t target_y);
    void _build_demo_program(const DemoPoint &point);
    void _advance_step();

    StaticVector<DemoStep, 24> _program;
    DemoPoint _active_demo_point;
    uint8_t _active_step = 0;
    bool _step_started = false;
    bool _demo_is_active = false;
    bool _demo_is_done = false;
    uint8_t _align_cmd = 0x00;
    uint8_t _center_cmd = 0x00;
};
