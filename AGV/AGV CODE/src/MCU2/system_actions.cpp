#include "system_actions.h"
#include "motor_driver.h"

AGVCtrl::AGVCtrl(Comm &comm_mcu, MotorDriver &mdriver)
    : _comm_mcu(comm_mcu),
      _proto_handler_mcu(comm_mcu, *this),
      _mdriver(mdriver) {}

void AGVCtrl::on_mcu_pkt_recieved(Comm::Packet &pkt)
{
    _proto_handler_mcu.handle(pkt);

    if (!pkt.approved)
        return;

    _process_mcu_packet(pkt);
}

void AGVCtrl::on_new_motion(const Comm::Packet &pkt) { _mdriver.move(pkt.data[0], pkt.data[1]); }

void AGVCtrl::on_stop() { _mdriver.channels_stop_all(); }

void AGVCtrl::_process_mcu_packet(Comm::Packet &pkt)
{
    switch (pkt.type)
    {
    case 'D':
        on_new_motion(pkt);
        break;
    case 'X':
        on_stop();
        break;
    case 'K':
        switch (pkt.data[0])
        {
        default:
            break;
        }
    default:
        break;
    }
}
