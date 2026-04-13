#pragma once
#include <Arduino.h>
#include <types.h>
#include <comm.h>

class MotorDriver;

class IActions
{
public:
    virtual ~IActions() = default;

    virtual void on_mcu_pkt_recieved(Comm::Packet &pkt) = 0;
    virtual void on_new_motion(const Comm::Packet &pkt) = 0;
    virtual void on_stop() = 0;
};

class SysCtrl : public IActions
{
public:
    SysCtrl(Comm &comm_mcu, MotorDriver &mdriver);

    void on_mcu_pkt_recieved(Comm::Packet &pkt) override;
    void on_new_motion(const Comm::Packet &pkt) override;
    void on_stop() override;

private:
    Comm &_comm_mcu;
    MotorDriver &_mdriver;

    ProtocolHandler _proto_handler_mcu;

    void _process_mcu_packet(Comm::Packet &pkt);
};
