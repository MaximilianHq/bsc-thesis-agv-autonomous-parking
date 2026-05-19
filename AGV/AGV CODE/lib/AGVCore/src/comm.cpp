#include <Arduino.h>
#include <types.h>
#include "comm.h"

uint8_t Comm::csum(const Packet &pkt)
{
    uint8_t crc = 0;
    crc ^= static_cast<uint8_t>(pkt.type);
    for (size_t i = 0; i < DATA_BYTES; i++)
        crc ^= pkt.data[i];
    crc ^= pkt.seq;
    return crc;
}

Comm::Comm(Stream &s, const String &debug_id) : _str(s), _dbg_name(debug_id) {}

bool Comm::read(Packet &out)
{
    bool started_packet = false;

    while (_str.available())
    {
        uint8_t b = static_cast<uint8_t>(_str.read());

        switch (_state)
        {
        case WAIT_START:
            if (b == '$')
            {
                started_packet = true;
                _pkt = Packet{};
                _read_index = 0;
                _state = READ_TYPE;
                if (g_debug.comm)
                    Serial.println("[COMM " + _dbg_name + "] Packet recieving...");
            }
            break;

        case READ_TYPE:
            if (b == '$')
            {
                _pkt = Packet{};
                _read_index = 0;
                if (g_debug.comm)
                    Serial.println("[COMM " + _dbg_name + "] Restarting packet read");
                break;
            }

            _pkt.type = static_cast<char>(b);
            _pkt.data_len = DATA_BYTES;
            _state = READ_BODY;
            break;

        case READ_BODY:
            if (_read_index < DATA_BYTES)
                _pkt.data[_read_index++] = b;
            else if (_read_index == DATA_BYTES)
            {
                _pkt.seq = b;
                _read_index++;
            }
            else if (_read_index == DATA_BYTES + 1)
            {
                _pkt.crc = b;
                _pkt.approved = (_pkt.crc == csum(_pkt));

                if (g_debug.comm)
                {
                    Serial.println("[COMM " + _dbg_name + "] Packet recieved!");
                    const uint8_t calc_crc = csum(_pkt);
                    Serial.print("[COMM " + _dbg_name + "] start: ");
                    if (static_cast<uint8_t>('$') < 0x10)
                        Serial.print("0");
                    Serial.println(static_cast<uint8_t>('$'), HEX);
                    Serial.print("[COMM " + _dbg_name + "] type: ");
                    if (static_cast<uint8_t>(_pkt.type) < 0x10)
                        Serial.print("0");
                    Serial.println(static_cast<uint8_t>(_pkt.type), HEX);
                    pds(_pkt.data, DATA_BYTES, "[COMM " + _dbg_name + "] data");
                    Serial.print("[COMM " + _dbg_name + "] seq: ");
                    if (_pkt.seq < 0x10)
                        Serial.print("0");
                    Serial.println(_pkt.seq, HEX);
                    Serial.print("[COMM " + _dbg_name + "] crc: ");
                    if (_pkt.crc < 0x10)
                        Serial.print("0");
                    Serial.println(_pkt.crc, HEX);
                    Serial.print("[COMM " + _dbg_name + "] calc: ");
                    if (calc_crc < 0x10)
                        Serial.print("0");
                    Serial.println(calc_crc, HEX);
                    Serial.print("[COMM " + _dbg_name + "] approved=");
                    Serial.println(_pkt.approved ? "yes" : "no");
                }

                out = _pkt;
                _state = WAIT_START;
                _read_index = 0;
                return true;
            }
            else
            {
                if (g_debug.comm)
                    Serial.println("[COMM " + _dbg_name + "] Packet corrupt");
                _state = WAIT_START;
                _read_index = 0;
            }
            break;
        }
    }

    if (g_debug.comm && started_packet)
        Serial.println("[COMM " + _dbg_name + "] Read paused");

    return false;
}

bool Comm::write(const Packet &pkt)
{
    if (!pkt.approved)
        return false;

    const uint8_t crc_to_send = (pkt.crc != 0) ? pkt.crc : csum(pkt);

    if (g_debug.comm)
    {
        const uint8_t calc_crc = csum(pkt);
        Serial.print("[COMM " + _dbg_name + "] start: ");
        if (static_cast<uint8_t>('$') < 0x10)
            Serial.print("0");
        Serial.println(static_cast<uint8_t>('$'), HEX);
        Serial.print("[COMM " + _dbg_name + "] type: ");
        if (static_cast<uint8_t>(pkt.type) < 0x10)
            Serial.print("0");
        Serial.println(static_cast<uint8_t>(pkt.type), HEX);
        pds(pkt.data, DATA_BYTES, "[COMM " + _dbg_name + "] data");
        Serial.print("[COMM " + _dbg_name + "] seq: ");
        if (pkt.seq < 0x10)
            Serial.print("0");
        Serial.println(pkt.seq, HEX);
        Serial.print("[COMM " + _dbg_name + "] crc: ");
        if (crc_to_send < 0x10)
            Serial.print("0");
        Serial.println(crc_to_send, HEX);
        Serial.print("[COMM " + _dbg_name + "] calc: ");
        if (calc_crc < 0x10)
            Serial.print("0");
        Serial.println(calc_crc, HEX);
        Serial.print("[COMM " + _dbg_name + "] approved=");
        Serial.println(pkt.approved ? "yes" : "no");
    }

    _str.write(static_cast<uint8_t>('$'));
    _str.write(static_cast<uint8_t>(pkt.type));
    _str.write(pkt.data, DATA_BYTES);
    _str.write(pkt.seq);
    _str.write(crc_to_send);

    return true;
}

ProtocolHandler::ProtocolHandler(Comm &comm, AGVCtrl &actions)
    : _comm(comm), _actions(actions) {}

void ProtocolHandler::update()
{
    unsigned long now = millis();
    if (_avalible || (now - _last_message_sent < _force_update_interval))
        return;

    if (_pkt_last_sent.approved && _pkt_last_sent.is_critical)
    {
        _comm.write(_pkt_last_sent);
        _last_message_sent = now;
        return;
    }

    _avalible = true;
    _pkt_last_sent = Comm::Packet{0};

    if (_pkt_send_que.size() > 0)
    {
        auto pkt = _pkt_send_que.pop_front();
        send_pkt(pkt);
    }
}

ProtocolHandler::Sequence ProtocolHandler::get_sequence() const { return {_seq, _avalible}; }

void ProtocolHandler::handle(const Comm::Packet &pkt)
{
    // Handle asnwers
    if (pkt.type == 'A')
    {
        if (pkt.approved)
        {
            if (!_avalible)
                _iterate_sequence();

            _pkt_last_sent = Comm::Packet{0};
            if (_pkt_send_que.size() > 0)
            {
                auto pkt = _pkt_send_que.pop_front();
                send_pkt(pkt);
            }
            else
                _avalible = true;
        }
        else
        {
            _comm.write(_pkt_last_sent);
            _last_message_sent = millis();
        }
        return;
    }

    // Handle commands
    auto ans = _make_ack(pkt);
    if (_comm.write(ans))
    {
        if (g_debug.comm)
            Serial.println("[PROTOHANDLER] Sent response succesfully");
    }
    else if (g_debug.comm)
        Serial.println("[PROTOHANDLER] Failed to respond");
}

bool ProtocolHandler::send_pkt(Comm::Packet &pkt, bool que)
{
    Sequence seq = get_sequence();
    if (!seq.avalible && que)
    {
        _pkt_send_que.push_back(pkt);
        return true;
    }
    else if (!seq.avalible)
        return false;

    pkt.seq = seq.seq;
    pkt.crc = Comm::csum(pkt);

    if (!_comm.write(pkt))
        return false;

    _pkt_last_sent = pkt;
    _avalible = false;
    _last_message_sent = millis();

    return true;
}

void ProtocolHandler::clear_send_queue() { _pkt_send_que.clear(); }

Comm::Packet ProtocolHandler::_make_ack(const Comm::Packet &pkt)
{
    Comm::Packet p = {'A', pkt.seq, {((pkt.approved) ? static_cast<uint8_t>(0x01) : static_cast<uint8_t>(0x00))}, 1, 0, true};
    return p;
}

void ProtocolHandler::_iterate_sequence()
{
    uint8_t prev = _seq;
    _seq++;
    if (prev > _seq && g_debug.comm)
        Serial.println("[PROTOHANDLER] WARNING - Sequence overflow");
}
