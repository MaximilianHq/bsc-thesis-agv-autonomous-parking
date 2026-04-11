#include "system_actions.h"

SysCtrl::SysCtrl(Comm &comm_bt, Comm &comm_mcu)
    : _comm_bt(comm_bt), _comm_mcu(comm_mcu),
      _bt_pkt_handler(comm_bt, *this), _mcu_pkt_handler(comm_mcu, *this) {}

void SysCtrl::on_bt_pkt_recieved(const Comm::Packet &pkt) { _bt_pkt_handler.handle(pkt); }

void SysCtrl::on_new_motion(uint8_t motion, uint8_t speed)
{
    AgvMotion new_motion = {motion, speed}; // motion, speed
    _motion.push_back(new_motion);
}

void SysCtrl::on_obstacle_detected(const Position &pos)
{
    // TODO write to ös
}