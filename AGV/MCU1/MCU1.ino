#include "agv_state.h"
#include "comm.h"
#include "imu.h"
#include "dwm.h"
#include "types.h"
#include "ota.h"
#include "sonar.h"
#include "sreg_handler.h"
#include "status_led.h"

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ServoEasing.hpp> // ☠️ .hpp only in .ino file ! ☠️
#include <WiFi.h>
// #include <ArduinoOTA.h>

// ========== PINS ==========
// Sonar
#define PIN_SONAR_SERVO 18
#define PIN_SONAR_TRIG 19
#define PIN_SONAR_ECHO 35
// UART
#define PIN_MTM_TXD 17
#define PIN_MTM_RXD 16
#define PIN_DWM_TXD 32
#define PIN_DWM_RXD 33
// Shift-Register
#define PIN_SHREG_DATA 27
#define PIN_SHREG_LATCH 26
#define PIN_SHREG_CLK 25
// Car crane
#define PIN_CRANE_SERVO 23
// IMU
#define PIN_SCL 22
#define PIN_SDA 21

// ========== DEFINITIONS ==========
#define UART_BAUD 115200
constexpr int32_t SONAR_RANGE = 200; // mm

BluetoothSerial SerialBT;

Comm comm_mcu(Serial1, "MCU");
Comm comm_bt(SerialBT, "BT");
Sonar sonar(PIN_SONAR_SERVO, PIN_SONAR_TRIG, PIN_SONAR_ECHO, SONAR_RANGE, 0, -90, 90, false);

// ========== GLOBALS ==========
Debug g_debug;
AgvState g_state;
AgvState g_state_prev;
AgvMotion g_motion;
AgvStatus g_led_status = {
    StatusLED::State::STATUS_BOOT,
    StatusLED::State::STATUS_BOOT,
};

// ========== LED STATUS ==========
SRegHandler sreg(PIN_SHREG_DATA, PIN_SHREG_CLK, PIN_SHREG_LATCH);
StatusLED led_sys(sreg, 0, 1, 2, true);
StatusLED led_cmd(sreg, 3, 4, 5, true);

// ========== PROTOTYPES ==========
void blt_status_routine();
void bt_test();

void setup()
{
    // ========== State ==========
    // Updated by led_x.update, do not set manually!
    sreg.setup();
    led_sys.setup();
    led_cmd.setup();

    // ========== UART ==========
    Serial.begin(UART_BAUD);  // PC
    Serial1.begin(UART_BAUD); // MCU2
    Serial2.begin(UART_BAUD); // DWM
    Serial.setTimeout(30);    // PC
    Serial1.setTimeout(30);   // MCU2
    Serial2.setTimeout(30);   // DWM

    Serial.println("[MAIN] Running setup...");

    // ========== Network ==========
    SerialBT.begin("AGV_BT_G2"); // ÖS
    Serial.println("Bluethooth started");
    // ota_begin("QBit", "internet");

    // ========== Sonar ==========
    sonar.setup();

    // ========== END ==========
    Serial.println("[MAIN] Setup finished");
    g_led_status.sys = StatusLED::State::STATUS_READY;
}

void loop()
{
    // ========== UPDATES ==========
    sonar.update();
    led_sys.update(g_led_status.sys);
    led_cmd.update(g_led_status.cmd);

    // ========== ROUTINES ==========

    // ota_handle();
    //  read BT. packet: ös movement, ös command
    //  $TCXXYYTTC\n or $TCCC\n

    blt_status_routine();

    // ========== CODE ==========
}

void blt_status_routine()
{
    if (SerialBT.hasClient())
        g_led_status.sys = StatusLED::State::STATUS_BLE_CONNECTED;
    else
        g_led_status.sys = StatusLED::State::STATUS_BLE_SEARCHING;
}

void bt_test()
{
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