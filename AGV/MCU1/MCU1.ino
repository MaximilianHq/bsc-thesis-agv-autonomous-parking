#include "agv_state.h"
#include "comm.h"
#include "imu.h"
#include "uwb.h"
#include "types.h"
#include "ota.h"

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

BluetoothSerial SerialBT;

const int UART_BAUD = 115200;

Comm comm_mcu(Serial1, "MCU");
Comm comm_bt(SerialBT, "BT");

void setup()
{
    Serial.begin(UART_BAUD);  // PC
    Serial1.begin(UART_BAUD); // MCU2
    Serial2.begin(UART_BAUD); // DWM
    Serial.setTimeout(30);    // PC
    Serial1.setTimeout(30);   // MCU2
    Serial2.setTimeout(30);   // DWM

    SerialBT.begin("AGV_BT_G2"); // ÖS
    Serial.println("Bluethooth started");

    ota_begin("QBit", "internet");
}

void loop()
{
    ota_handle();
    // read BT. packet: ös movement, ös command
    // $TCXXYYTTC\n or $TCCC\n

    Packet bt_pkt;
    comm_bt.read(bt_pkt);
    Packet ans;
    ans.type = 'A';
    ans.data[0] = 'O';
    ans.data[1] = 'K';
    ans.data_len = 2;
    ans.crc = csum(ans);
    if (bt_pkt.approved)
    {
        if (comm_bt.write(ans))
            Serial.println("Sent answer succesfully!");
        else
            Serial.println("Failed to answer :/");
    }
    else
        Serial.println("Recieved packet not approved :/");
}