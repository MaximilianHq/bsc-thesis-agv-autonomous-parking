#include "system_actions.h"
#include "status_led.h"

SysCtrl::SysCtrl(Comm &comm_bt, Comm &comm_mcu, StatusLED &led_sys, StatusLED &led_cmd)
    : _comm_bt(comm_bt),
      _comm_mcu(comm_mcu),
      _proto_handler_bt(comm_bt, *this),
      _proto_handler_mcu(comm_mcu, *this),
      _led_sys(led_sys),
      _led_cmd(led_cmd) {}

void SysCtrl::on_bt_no_connect()
{
    on_stop();
    _led_sys.set_status(StatusLED::State::STATUS_BLE_SEARCHING);
}

void SysCtrl::on_bt_pkt_recieved(Comm::Packet &pkt)
{
    _proto_handler_bt.handle(pkt);

    if (!pkt.approved)
        return;

    _process_bt_packet(pkt);
    _led_cmd.set_status(StatusLED::State::STATUS_CMD_RECEIVING);
}

void SysCtrl::on_mcu_pkt_recieved(Comm::Packet &pkt)
{
    _proto_handler_bt.handle(pkt);

    if (!pkt.approved)
        return;

    _process_mcu_packet(pkt);
}

void SysCtrl::on_new_motion(Comm::Packet &pkt)
{
    if (!_forward_to_mcu(pkt))
        if (g_debug.IAction)
            Serial.println("[SYSCTRL] - Failed to send movement command to [MCU2]");
}

void SysCtrl::on_stop()
{
    Comm::Packet p = {'X', _proto_handler_mcu.get_sequence(), {}, 0, 0, true};
    p.crc = Comm::csum(p);

    on_stop(p);
}

void SysCtrl::on_stop(Comm::Packet &pkt)
{
    if (!_forward_to_mcu(pkt))
        if (g_debug.IAction)
            Serial.println("[SYSCTRL] \033[31mWATNING\033[0m - Failed to send stop command to [MCU2]");

    _led_cmd.set_status(StatusLED::State::STATUS_CMD_STOPPING);
}

void SysCtrl::on_obstacle_detected(const Position &pos)
{
    Comm::Packet p = {'H', _proto_handler_bt.get_sequence(), {}, 4, 0, true};
    p.data[0] = static_cast<uint8_t>((pos.x >> 8) & 0xFF);
    p.data[1] = static_cast<uint8_t>(pos.x & 0xFF);
    p.data[2] = static_cast<uint8_t>((pos.y >> 8) & 0xFF);
    p.data[3] = static_cast<uint8_t>(pos.y & 0xFF);
    p.crc = Comm::csum(p);

    if (_comm_mcu.write(p))
    {
        _proto_handler_bt.itterate_sequence();
        _proto_handler_bt.add_buffer_sent(p);
    }
    else if (g_debug.IAction)
        Serial.println("[SYSCTRL] \033[31mWATNING\033[0m - Failed send obstacle position to [ÖS]");

    _led_cmd.set_status(StatusLED::State::STATUS_OBSTACLE);
}

void SysCtrl::_process_bt_packet(Comm::Packet &pkt)
{
    switch (pkt.type)
    {
    case 'D':
        on_new_motion(pkt);
        break;
    case 'K':
        switch (pkt.data[0])
        {
        default:
            break;
        }
        break;
    case 'X':
        on_stop(pkt);
        break;
    case 'R':
        break;
    default:
        break;
    }
}

void SysCtrl::_process_mcu_packet(Comm::Packet &pkt)
{
    switch (pkt.type)
    {
    case 'M':
        break;
    case 'H':
        break;
    default:
        break;
    }
}

bool SysCtrl::_forward_to_mcu(Comm::Packet &pkt)
{
    pkt.seq = _proto_handler_mcu.get_sequence();
    pkt.crc = Comm::csum(pkt);

    if (!_comm_mcu.write(pkt))
        return false;

    _proto_handler_mcu.itterate_sequence();
    _proto_handler_mcu.add_buffer_sent(pkt);
    return true;
}