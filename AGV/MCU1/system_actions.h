#pragma once
#include "types.h"
#include "agv_state.h"
#include "static_vector.h"
#include "comm.h"
#include <Arduino.h>

class IActions
{
public:
    virtual ~IActions() = default;

    // CONNECTION
    virtual void on_bt_no_connect() = 0;
    // BT
    virtual void on_bt_pkt_recieved(Comm::Packet &pkt) = 0;
    virtual void on_new_motion(uint8_t motion, uint8_t speed) = 0;
    virtual void on_stop() = 0;
    // MCU

    // SONAR
    virtual void on_obstacle_detected(Position &pos) = 0;
};

class SysCtrl : public IActions
{
public:
    SysCtrl(Comm &comm_bt, Comm &comm_mcu);

    void on_bt_pkt_recieved(Comm::Packet &pkt) override;
    void on_new_motion(uint8_t motion, uint8_t speed) override;
    void on_obstacle_detected(Position &pos) override;

private:
    Comm &_comm_bt;
    Comm &_comm_mcu;

    StaticVector<AgvState, 10> _state;
    StaticVector<AgvMotion, 20> _motion;

    BTPacketHandler _bt_pkt_handler;
    MCUPacketHandler _mcu_pkt_handler;
};
