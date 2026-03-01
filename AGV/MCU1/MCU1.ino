#include "agv_state.h"
#include "comm.h"
#include "imu.h"
#include "uwb.h"

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

const int UART_BAUD = 115200;
const int BT_DELAY = 20000; // ms

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
    // $TCXXYYTTC\n or $TCCC\n

    Packet bt_pkt;
    read_bt(bt_pkt);
    Packet answer;
    answer.type = 'A';
    answer.data[0] = 'O';
    answer.data[1] = 'K';
    answer.data_len = 2;
    answer.crc = csum(answer);
    if (bt_pkt.approved)
    {
        if (write_bt(answer))
            Serial.println("Sent answer succesfully!");
        else
            Serial.println("Failed to answer :/");
    }
    else
        Serial.println("Recieved packet not approved :/");
}