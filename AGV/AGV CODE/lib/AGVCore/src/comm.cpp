#include <Arduino.h>
#include <types.h>
#include <system_actions.h>
#include "comm.h"

uint8_t Comm::csum(const Packet &pkt)
{
    uint8_t crc = 0;
    crc ^= (uint8_t)pkt.type;
    for (size_t i = 0; i < pkt.data_len; i++)
        crc ^= pkt.data[i];
    return crc;
}

Comm::Comm(Stream &s, const String &debug_id) : _str(s), _dbg_name(debug_id) {}

bool Comm::read(Packet &out)
{
    bool packet_delivered = false;
    bool started_packet = false;

    while (_str.available())
    {
        uint8_t b = (uint8_t)_str.read();

        switch (_state)
        {
        case WAIT_START:
            if (g_debug.comm)
                Serial.println("[" + _dbg_name + "] Packet recieving...");
            if (b == '$')
            {
                started_packet = true;
                _pkt = Packet{}; // nollställ
                _read_index = 0;
                _state = READ_TYPE;
            }
            break;

        case READ_TYPE:
            _pkt.type = (char)b;
            _state = READ_BODY;
            break;

        case READ_BODY:
            if (b == '\n')
            {
                if (g_debug.comm)
                    Serial.println("[" + _dbg_name + "] Packet recieved!");
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
                        Serial.print("[" + _dbg_name + "] Recieving message: $");
                        Serial.write(_pkt.type);
                        for (size_t j = 0; j < _pkt.data_len; j++)
                            Serial.print((uint8_t)_pkt.data[j]);

                        Serial.println();
                        Serial.print("Received crc: ");
                        Serial.print("CHAR = ");
                        Serial.print((char)_pkt.crc);
                        Serial.print(", HEX = ");
                        Serial.println(_pkt.crc);

                        Serial.print("Calculated crc: ");
                        Serial.print("CHAR = ");
                        Serial.print((char)csum(_pkt));
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
                        Serial.println("[" + _dbg_name + "] Packet corrupt");
                    _state = WAIT_START;
                }
            }
            break;
        }
    }

    if (g_debug.comm && started_packet && !packet_delivered)
        Serial.println("[" + _dbg_name + "] Read paused");

    return packet_delivered;
}

bool Comm::write(const Packet &pkt)
{
    if (!pkt.approved)
        return false;

    if (g_debug.comm)
    {
        Serial.print("[" + _dbg_name + "] Sending message: $");
        Serial.write(pkt.type);
        for (size_t j = 0; j < pkt.data_len; j++)
            Serial.print((uint8_t)pkt.data[j]);

        Serial.print("Calculated crc: ");
        Serial.print("CHAR = ");
        Serial.print((char)csum(pkt));
        Serial.print(", HEX = ");
        Serial.println(csum(pkt));
    }

    _str.write((uint8_t)'$');
    _str.write((uint8_t)pkt.type);

    _str.write(pkt.data, pkt.data_len);
    _str.write(pkt.seq);
    _str.write(pkt.crc);

    _str.write((uint8_t)'\n');

    return true;
}

PacketHandler::PacketHandler(Comm &comm, IActions &actions) : _comm(comm), _actions(actions) {}

uint16_t PacketHandler::get_sequence() const { return _seq; }

void PacketHandler::handle(const Comm::Packet &pkt)
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
    respond(pkt);

    if (!pkt.approved)
    {
        if (g_debug.comm)
            Serial.println("[COMM::BTPKTHANDLER handle()] Packet not approved");
        return;
    }

    handle_approved(pkt); // Anropar barn-klass
}

Comm::Packet PacketHandler::make_ack(const Comm::Packet &pkt)
{
    Comm::Packet p = {'A', pkt.seq, {(pkt.approved ? 0x01 : 0x00)}, 1, 0, true};
    p.crc = Comm::csum(p);
    return p;
}

void PacketHandler::respond(const Comm::Packet &pkt)
{
    auto ans = make_ack(pkt);
    if (_comm.write(ans))
    {
        if (g_debug.comm)
            Serial.println("[COMM::PKTHANDLER respond()] Sent answer succesfully");
    }
    else if (g_debug.comm)
        Serial.println("[COMM::PKTHANDLER respond()] Failed to answer");
}

// BTPacketHandler::BTPacketHandler(Comm &comm) : PacketHandler(comm) {}
// MCUPacketHandler::MCUPacketHandler(Comm &comm) : PacketHandler(comm) {}

void BTPacketHandler::handle_approved(const Comm::Packet &pkt)
{
    switch (pkt.type)
    {
    case 'D':
        _actions.on_new_motion(pkt.data[0], pkt.data[1]);
        break;
    case 'K':
        switch (pkt.data[0])
        {
        default:
            break;
        }
        break;
    case 'X':
        _actions.on_stop();
        break;
    case 'R':
        break;
    default:
        break;
    }
}

void MCUPacketHandler::handle_approved(const Comm::Packet &pkt)
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
