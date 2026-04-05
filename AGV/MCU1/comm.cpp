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

Comm::Comm(Stream &s, const String &debug_id) : _str(s), _dbg_name(debug_id) {}

void Comm::read(Packet &out)
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
                    _pkt.approved = (_pkt.crc == csum(_pkt));

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
                        Serial.println(csum(_pkt), HEX);
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
}

bool Comm::write(const Packet &pkt)
{
    _str.write((uint8_t)'$');
    _str.write((uint8_t)pkt.type);

    _str.write(pkt.data, pkt.data_len);

    _str.write((pkt.crc != 0) ? pkt.crc : csum(pkt));

    _str.write((uint8_t)'\n');
    if (g_debug.comm)
        Serial.println("[" + _dbg_name + "] Sent packet sucessfully!");
    return true;
}
