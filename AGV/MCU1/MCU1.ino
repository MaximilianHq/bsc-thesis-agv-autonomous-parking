#include "agv_state.h"
#include "comm.h"
#include "imu.h"
#include "dwm.h"
#include "types.h"
#include "ota.h"
#include "sonar.h"

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ServoEasing.hpp> // ☠️ .hpp only in .ino file ! ☠️
#include <WiFi.h>
// #include <ArduinoOTA.h>

#define PIN_SONAR_SERVO 0
#define PIN_SONAR_TRIG 1
#define PIN_SONAR_ECHO 2
#define UART_BAUD 115200

constexpr int32_t SONAR_RANGE = 200; // mm

BluetoothSerial SerialBT;

Comm comm_mcu(Serial1, "MCU");
Comm comm_bt(SerialBT, "BT");
Sonar sonar(PIN_SONAR_SERVO, PIN_SONAR_TRIG, PIN_SONAR_ECHO, SONAR_RANGE, 0, -90, 90, false);

Debug g_debug;
AgvState g_state;
AgvState g_state_prev;
AgvMotion g_motion;

void setup()
{
    // UART
    Serial.begin(UART_BAUD);  // PC
    Serial1.begin(UART_BAUD); // MCU2
    Serial2.begin(UART_BAUD); // DWM
    Serial.setTimeout(30);    // PC
    Serial1.setTimeout(30);   // MCU2
    Serial2.setTimeout(30);   // DWM

    // Network
    SerialBT.begin("AGV_BT_G2"); // ÖS
    Serial.println("Bluethooth started");
    // ota_begin("QBit", "internet");

    // Sonar
    sonar.sonar_init();
}

void loop()
{
    // ota_handle();
    //  read BT. packet: ös movement, ös command
    //  $TCXXYYTTC\n or $TCCC\n

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
    delay(200);
}