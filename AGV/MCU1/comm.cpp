#include "comm.h"
#include "types.h"
#include <Arduino.h>
#include <BluetoothSerial.h>

uint8_t csum(const Packet &pkt)
{
    uint8_t crc = 0;
    crc ^= (uint8_t)pkt.type;
    for (size_t i = 0; i < pkt.data_len; i++)
        crc ^= pkt.data[i];
    return crc;
}

Comm::Comm(Stream &s, const String &dbgn) : str(s), debug_name(dbgn) {}

void Comm::read(Packet &out)
{
    while (str.available())
    {
        uint8_t b = (uint8_t)str.read();

        switch (state)
        {
        case WAIT_START:
            if (g_debug.comm)
                Serial.println("[" + debug_name + "]" + " Packet recieving...");
            if (b == '$')
            {
                pkt = Packet{}; // nollställ
                i = 0;
                state = READ_TYPE;
            }
            break;

        case READ_TYPE:
            pkt.type = (char)b;
            state = READ_BODY;
            break;

        case READ_BODY:
            if (b == '\n')
            {
                if (g_debug.comm)
                    Serial.println("[" + debug_name + "]" + " Packet recieved!");
                if (i >= 2)
                {
                    pkt.data_len = i - 1; // points to crc
                    pkt.crc = pkt.data[pkt.data_len];
                    pkt.data[pkt.data_len] = 0;
                    if (g_debug.comm)
                    {
                        
                    }
                    pkt.approved = (pkt.crc == csum(pkt));

                    out = pkt; // leverera färdigt paket

                    // === NY DEBUG: skriv hela paketet ===
                    if (g_debug.comm)
                    {
                        Serial.print("[" + debug_name + "]" + " Full message: $" + pkt.type);
                        for (size_t j = 0; j < pkt.data_len; j++)
                            Serial.write(pkt.data[j]);

                        Serial.println();
                        Serial.print("Received crc: ");
                        Serial.println(pkt.crc, HEX);

                        Serial.print("Calculated crc: ");
                        Serial.println(csum(pkt), HEX);
                    }
                }
                state = WAIT_START;
            }
            else
            {
                if (i < sizeof(pkt.data))
                {
                    pkt.data[i++] = b;
                }
                else
                {
                    // overflow: kasta paket och vänta på ny start
                    if (g_debug.comm)
                        Serial.println("[" + debug_name + "]" + " Packet corrupt");
                    state = WAIT_START;
                }
            }
            break;
        }
    }
}

bool Comm::write(const Packet &pkt)
{
    str.write((uint8_t)'$');
    str.write((uint8_t)pkt.type);

    str.write(pkt.data, pkt.data_len);

    str.write((pkt.crc != 0) ? pkt.crc : csum(pkt));

    str.write((uint8_t)'\n');
    if (g_debug.comm)
        Serial.println("[" + debug_name + "]" + " Sent packet sucessfully!");
    return true;
}
