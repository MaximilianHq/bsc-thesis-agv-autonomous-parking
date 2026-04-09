#include "comm.h"
#include "types.h"
#include <Arduino.h>

uint8_t csum(const Packet &pkt)
{
    uint8_t crc = 0;
    crc ^= (uint8_t)pkt.type;
    for (size_t i = 0; i < pkt.data_len; i++)
        crc ^= pkt.data[i];
    return crc;
}

void PacketHandler::respond(const Comm::Packet &pkt)
{
    Comm::Packet ans;
    ans.type = 'A';
    ans.seq = pkt.seq + 1;
    ans.data[0] = 'O';
    ans.data[1] = 'K';
    ans.data_len = 2;
    ans.crc = csum(ans);
    ans.approved = true;

    if (_comm.write(ans))
    {
        if (g_debug.comm)
            Serial.println("[COMM::PKTHANDLER respond()] Sent answer succesfully");
    }
    else if (g_debug.comm)
        Serial.println("[COMM::PKTHANDLER respond()] Failed to answer");
}

BTPacketHandler::BTPacketHandler(Comm &comm) : PacketHandler(comm) {}
MCUPacketHandler::MCUPacketHandler(Comm &comm) : PacketHandler(comm) {}

void BTPackethandler::handle(const Comm::Packet &pkt)
{
    if (!pkt.approved)
    {
        if (g_debug.comm)
            Serial.println("[COMM::BTPKTHANDLER handle()] Packet not approved")
    }

    respond(pkt);

    switch (pkt.type)
    {
    case 'T': // test

        break;
    default:
        break;
    }
}

void MCUPackethandler::handle(const Comm::Packet &pkt)
{
    if (!pkt.approved)
    {
        if (g_debug.comm)
            Serial.println("[COMM::MCUPKTHANDLER handle()] Packet not approved")
    }

    switch (pkt.type)
    {
    case 'T': // test

        break;
    default:
        break;
    }
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
                i = 0;
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
                if (i >= 2)
                {
                    _pkt.data_len = i - 1; // points to crc
                    _pkt.crc = _pkt.data[_pkt.data_len];
                    _pkt.data[_pkt.data_len] = 0;
                    if (g_debug.comm)
                    {
                    }
                    _pkt.approved = (_pkt.crc == PacketHandler::csum(_pkt));

                    out = _pkt; // leverera färdigt paket
                    packet_delivered = true;

                    // === NY DEBUG: skriv hela paketet ===
                    if (g_debug.comm)
                    {
                        Serial.print("[" + _dbg_name + "] Full message: $" + _pkt.type);
                        for (size_t j = 0; j < _pkt.data_len; j++)
                            Serial.write(_pkt.data[j]);

                        Serial.println();
                        Serial.print("Received crc: ");
                        Serial.println(_pkt.crc, HEX);

                        Serial.print("Calculated crc: ");
                        Serial.println(PacketHandler::csum(_pkt), HEX);
                    }
                }
                _state = WAIT_START;
            }
            else
            {
                if (i < sizeof(_pkt.data))
                {
                    _pkt.data[i++] = b;
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
    {
        Serial.println("[" + _dbg_name + "] Read paused");
    }

    return packet_delivered;
}

bool Comm::write(const Packet &pkt)
{
    if (!pkt.approved)
    {
        if (g_debug.comm)
            Serial.println("[" + _dbg_name + "] Failed to send packet");
        return false;
    }

    _str.write((uint8_t)'$');
    _str.write((uint8_t)pkt.type);

    _str.write(pkt.data, pkt.data_len);
    _str.write(pkt.seq);
    _str.write(pkt.crc);

    _str.write((uint8_t)'\n');

    if (g_debug.comm)
        Serial.println("[" + _dbg_name + "] Sent packet sucessfully");

    return true;
}
