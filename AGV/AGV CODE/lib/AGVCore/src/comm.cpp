#include <Arduino.h>
#include <types.h>
#include "comm.h"

uint8_t Comm::csum(const Packet &pkt)
{
    uint8_t crc = 0;
    crc ^= static_cast<uint8_t>(pkt.type);
    for (size_t i = 0; i < pkt.data_len; i++)
        crc ^= pkt.data[i];
    crc ^= pkt.seq;
    return crc;
}

Comm::Comm(Stream &s, const String &debug_id) : _str(s), _dbg_name(debug_id) {}

bool Comm::read(Packet &out)
{
    bool packet_delivered = false;
    bool started_packet = false;

    while (_str.available())
    {
        uint8_t b = static_cast<uint8_t>(_str.read());

        switch (_state)
        {
        case WAIT_START:
            if (g_debug.comm)
                Serial.println("[COMM " + _dbg_name + "] Packet recieving...");
            if (b == '$')
            {
                started_packet = true;
                _pkt = Packet{}; // nollställ
                _read_index = 0;
                _state = READ_TYPE;
            }
            break;

        case READ_TYPE:
            _pkt.type = static_cast<char>(b);
            _state = READ_BODY;
            break;

        case READ_BODY:
            if (b == '\n')
            {
                if (g_debug.comm)
                    Serial.println("[COMM " + _dbg_name + "] Packet recieved!");
                if (_read_index >= 2)
                {
                    _pkt.data_len = _read_index - 2; // points to seq
                    _pkt.seq = _pkt.data[_pkt.data_len];
                    _pkt.crc = _pkt.data[_pkt.data_len + 1];
                    _pkt.data[_pkt.data_len] = 0;
                    _pkt.data[_pkt.data_len + 1] = 0;
                    _pkt.approved = (_pkt.crc == csum(_pkt));

                    out = _pkt; // leverera färdigt paket
                    packet_delivered = true;

                    // === NY DEBUG: skriv hela paketet ===
                    if (g_debug.comm)
                    {
                        Serial.print("[COMM " + _dbg_name + "] Recieving message: $ ");
                        Serial.write(_pkt.type);

                        for (size_t i = 0; i < _pkt.data_len; i++)
                        {
                            Serial.print(_pkt.data[i], HEX);
                            Serial.print(" ");
                        }

                        Serial.println();

                        Serial.println();
                        Serial.print("Received crc: ");
                        Serial.print("CHAR = ");
                        Serial.print(static_cast<char>(_pkt.crc));
                        Serial.print(", HEX = ");
                        Serial.println(_pkt.crc);

                        Serial.print("Calculated crc: ");
                        Serial.print("CHAR = ");
                        Serial.print(static_cast<char>(csum(_pkt)));
                        Serial.print(", HEX = ");
                        Serial.println(csum(_pkt));
                    }
                }
                _state = WAIT_START;
            }
            else
            {
                if (_read_index < sizeof(_pkt.data))
                {
                    _pkt.data[_read_index++] = b;
                }
                else
                {
                    // overflow: kasta paket och vänta på ny start
                    if (g_debug.comm)
                        Serial.println("[COMM " + _dbg_name + "] Packet corrupt");
                    _state = WAIT_START;
                }
            }
            break;
        }
    }

    if (g_debug.comm && started_packet && !packet_delivered)
        Serial.println("[COMM " + _dbg_name + "] Read paused");

    return packet_delivered;
}

bool Comm::write(const Packet &pkt)
{
    if (!pkt.approved)
        return false;

    if (g_debug.comm)
    {
        Serial.println("[COMM " + _dbg_name + "] Sending message: $ ");
        Serial.write(pkt.type);

        for (size_t i = 0; i < pkt.data_len; i++)
        {
            Serial.print(pkt.data[i], HEX);
            Serial.print(" ");
        }

        Serial.println();

        Serial.print("Calculated crc: ");
        Serial.print("CHAR = ");
        Serial.print(static_cast<char>(csum(pkt)));
        Serial.print(", HEX = ");
        Serial.println(csum(pkt));
    }

    _str.write(static_cast<uint8_t>('$'));
    _str.write(static_cast<uint8_t>(pkt.type));

    _str.write(pkt.data, pkt.data_len);
    _str.write(pkt.seq);
    _str.write((pkt.crc != 0) ? pkt.crc : csum(pkt));

    _str.write(static_cast<uint8_t>('\n'));

    return true;
}

ProtocolHandler::ProtocolHandler(Comm &comm, SysCtrl &actions) : _comm(comm), _actions(actions) {}

uint8_t ProtocolHandler::get_sequence() const { return _seq; }
void ProtocolHandler::iterate_sequence()
{
    uint8_t prev = _seq;
    _seq++;
    if (prev > _seq && g_debug.comm)
        Serial.println("[PROTOHANDLER] \033[33mWARNING\033[0m - Sequence overflow");
}

void ProtocolHandler::add_buffer_sent(const Comm::Packet &pkt) { _pkt_buffer_sent.push_back(pkt); }
void ProtocolHandler::add_buffer_rcvd(const Comm::Packet &pkt) { _pkt_buffer_rcvd.push_back(pkt); }

void ProtocolHandler::handle(const Comm::Packet &pkt)
{
    // Handle asnwers
    if (pkt.type == 'A')
    {
        if (pkt.approved)
            _pkt_buffer_sent.pop_if([&pkt](const Comm::Packet &p)
                                    { return p.seq == pkt.seq; });
        else
            _comm.write(*_pkt_buffer_sent.find_if([&pkt](const Comm::Packet &p)
                                                  { return p.seq == pkt.seq; }));

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

    if (pkt.approved)
        _pkt_buffer_rcvd.push_back(pkt);
}

Comm::Packet ProtocolHandler::_make_ack(const Comm::Packet &pkt)
{
    Comm::Packet p = {'A', pkt.seq, {((pkt.approved) ? static_cast<uint8_t>(0x01) : static_cast<uint8_t>(0x00))}, 1, 0, true};
    p.crc = Comm::csum(p);
    return p;
}
