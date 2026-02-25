#include "agv_state.h"
#include "comm_os.h"
#include "imu.h"
#include "uwb.h"

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

const int UART_BAUD = 115200;
const int BT_DELAY = 20000; // ms

struct Packet
{
    char type;             // type
    size_t data_len = 0;   // data_length
    uint8_t data[9] = {0}; // data
    uint8_t crc = 0;       // checksum
    bool approved = false; // data integrity
};

uint8_t csum(const Packet &bt_pkt)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < bt_pkt.data_len; i++)
        crc ^= bt_pkt.data[i];
    return crc;
}

void read_bt(Packet &bt_pkt)
{
    char start = 0;

    // hitta '$'
    while (SerialBT.available())
    {
        start = (char)SerialBT.read();
        if (start == '$')
            break;
    }
    if (start != '$')
        return;

    // läs type
    if (!SerialBT.available())
        return;
    bt_pkt.type = (char)SerialBT.read();

    // läs data + crc tills '\n'
    size_t i = 0;
    while (SerialBT.available() && i < sizeof(bt_pkt.data))
    {
        uint8_t b = (uint8_t)SerialBT.read();

        if (b == '\n')
        {
            if (i < 1)
                return;                      // måste minst ha CRC
            bt_pkt.crc = bt_pkt.data[i - 1]; // sista byte före \n = crc
            bt_pkt.data_len = i - 1;         // resten = payload
            bt_pkt.approved = (bt_pkt.crc == csum(bt_pkt));
            break;
        }

        bt_pkt.data[i++] = b;
    }
}

void setup()
{
    Serial.begin(UART_BAUD);     // PC
    Serial1.begin(UART_BAUD);    // MCU2
    Serial2.begin(UART_BAUD);    // DWM
    SerialBT.begin("AGV_BT_G2"); // ÖS
    Serial.println("Bluethooth started");
}

void loop()
{
    // read BT. packet: ös movement, ös command
    // $TXY*CC\n or $TC*CC\n

    Packet bt_pkt; // last byte defines data data_length
    read_bt(bt_pkt);
}