#include <comm.h>
#include <Arduino.h>

uint8_t csum(const Packet &pkt)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < pkt.data_len; i++)
        crc ^= pkt.data[i];
    return crc;
}

void read_bt(Packet &out)
{
    static enum { WAIT_START,
                  READ_TYPE,
                  READ_BODY } state = WAIT_START;
    static Packet pkt;
    static size_t i = 0;

    while (SerialBT.available())
    {
        uint8_t b = (uint8_t)SerialBT.read();

        switch (state)
        {
        case WAIT_START:
            Serial.println("BT-Packet recieving...");
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
                Serial.println("BT-packet recieved!");
                if (i >= 1)
                {
                    pkt.crc = pkt.data[i - 1];
                    pkt.data_len = i - 1;
                    pkt.approved = (pkt.crc == csum(pkt));

                    out = pkt; // leverera färdigt paket
                }
                state = WAIT_START; // redo för nästa
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
                    Serial.println("BT-Packet corrupt");
                    state = WAIT_START;
                }
            }
            break;
        }
        if (out != pkt)
            Serial.println("BT-Read paused");
    }
}

bool write_bt(const Packet &pkt)
{
    if (!SerialBT.hasClient())
        return false;

    SerialBT.write((uint8_t)'$');
    SerialBT.write((uint8_t)pkt.type);

    SerialBT.write(pkt.data, pkt.data_len);

    SerialBT.write((pkt.crc != 0) ? pkt.crc : csum(pkt));

    SerialBT.write((uint8_t)'\n');
    return true;
}