#include "system_actions.h"

SysCtrl::SysCtrl(Comm &comm_bt, Comm &comm_mcu, StatusLED &led_sys, StatusLED &led_cmd)
    : _comm_bt(comm_bt),
      _comm_mcu(comm_mcu),
      _pkt_handler_bt(comm_bt, *this),
      _pkt_handler_mcu(comm_mcu, *this),
      _led_sys(led_sys),
      _led_cmd(led_cmd) {}

void SysCtrl::on_bt_no_connect()
{
    on_stop();
    _led_sys.set_status(StatusLED::State::STATUS_BLE_SEARCHING);
}

void SysCtrl::on_bt_pkt_recieved(const Comm::Packet &pkt)
{
    _pkt_handler_bt.handle(pkt);
    _led_cmd.set_status(StatusLED::State::STATUS_CMD_RECEIVING);
}
void SysCtrl::on_mcu_pkt_recieved(const Comm::Packet &pkt) { _pkt_handler_mcu.handle(pkt); }

void SysCtrl::on_new_motion(uint8_t motion, uint8_t speed)
{
    AgvMotion new_motion = {motion, speed};
    _motion.push_back(new_motion);
}

void SysCtrl::on_stop()
{
    Comm::Packet p = {'X', _pkt_handler_mcu.get_sequence(), {}, 0, 0, true};
    p.crc = Comm::csum(p);
    _comm_mcu.write(p);

    _led_cmd.set_status(StatusLED::State::STATUS_CMD_STOPPING);
}

void SysCtrl::on_obstacle_detected(const Position &pos)
{
    Comm::Packet p = {'H', _pkt_handler_bt.get_sequence(), {(pos.x >> 8) & 0xFF, pos.x & 0xFF, (pos.y >> 8) & 0xFF, pos.y & 0xFF}, 4, 0, true};
    p.crc = Comm::csum(p);
    _comm_bt.write(p);

    _led_cmd.set_status(StatusLED::State::STATUS_OBSTACLE);
}