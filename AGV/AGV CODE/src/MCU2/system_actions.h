#pragma once
#include <Arduino.h>
#include <types.h>
#include <comm.h>

class MotorDriver;

class AGVCtrl;

class AGVCtrl
{
public:
    AGVCtrl(Comm &comm_mcu, MotorDriver &mdriver);

    void on_mcu_pkt_recieved(Comm::Packet &pkt);
    void on_new_motion(const Comm::Packet &pkt);
    void on_stop();

private:
    Comm &_comm_mcu;
    MotorDriver &_mdriver;

    ProtocolHandler _proto_handler_mcu;

    void _process_mcu_packet(Comm::Packet &pkt);
};
