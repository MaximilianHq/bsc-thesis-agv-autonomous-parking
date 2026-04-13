#pragma once
#include <Arduino.h>
#include <types.h>
#include <comm.h>
#include <static_vector.h>

class StatusLED;

class IActions
{
public:
    virtual ~IActions() = default;

    virtual void on_bt_no_connect() = 0;
    virtual void on_bt_pkt_recieved(Comm::Packet &pkt) = 0;
    virtual void on_mcu_pkt_recieved(Comm::Packet &pkt) = 0;
    virtual void on_new_motion(Comm::Packet &pkt) = 0;
    virtual void on_stop() = 0;
    virtual void on_obstacle_detected(const Position &pos) = 0;
    virtual void on_new_position_data(const DwmState &dwm, const ImuState &imu) = 0;
};

class SysCtrl : public IActions
{
public:
    SysCtrl(Comm &comm_bt, Comm &comm_mcu, StatusLED &led_sys, StatusLED &led_cmd);

    // ========== IActions ==========
    void on_bt_no_connect() override;
    void on_bt_pkt_recieved(Comm::Packet &pkt) override;
    void on_mcu_pkt_recieved(Comm::Packet &pkt) override;
    void on_new_motion(Comm::Packet &pkt) override;
    void on_stop() override;
    void on_stop(Comm::Packet &pkt);
    void on_obstacle_detected(const Position &pos) override;
    void on_new_position_data(const DwmState &dwm, const ImuState &imu) override { return; };

private:
    Comm &_comm_bt;
    Comm &_comm_mcu;
    StatusLED &_led_sys;
    StatusLED &_led_cmd;

    StaticVector<AgvState, 10> _state;
    StaticVector<Comm::Packet, 20> _motion_buffert;

    ProtocolHandler _proto_handler_bt, _proto_handler_mcu;

    void _process_bt_packet(Comm::Packet &pkt);
    void _process_mcu_packet(Comm::Packet &pkt);
    bool _forward_to_mcu(Comm::Packet &pkt);
    void _next_movement(Comm::Packet &pkt);
};
