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

constexpr int SONAR_RANGE = 200; // mm
constexpr int SONAR_SPEED = 100; // mm
constexpr int SONAR_ANGLE = 70;  // ∓ deg

BluetoothSerial SerialBT;

Comm comm_bt(SerialBT, "BT");
Comm comm_mcu(Serial1, "MCU");
BTPacketHandler bt_pkt_handler(comm_bt);
MCUPacketHandler mcu_pkt_handler(comm_mcu);

Sonar sonar(PIN_SONAR_SERVO, PIN_SONAR_TRIG, PIN_SONAR_ECHO,
            SONAR_RANGE, SONAR_SPEED, SONAR_ANGLE, 90, 0, 180, false);

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
StatusLED led_sys(sreg, SRegHandler::pin_sreg::QA,
                  SRegHandler::pin_sreg::QB,
                  SRegHandler::pin_sreg::QC,
                  true);
StatusLED led_cmd(sreg, SRegHandler::pin_sreg::QD,
                  SRegHandler::pin_sreg::QE,
                  SRegHandler::pin_sreg::QF,
                  true);

// ========== PROTOTYPES ==========
void blt_status_routine();
void bt_test2();

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
    Serial.println("[BT] Bluethooth active");
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
    Position pos;
    if (sonar.update())
    {
        g_led_status.cmd = StatusLED::State::STATUS_OBSTACLE;
        pos = sonar.get_obstacle_position();
        sonar.reset();
    }

    led_sys.update(g_led_status.sys);
    led_cmd.update(g_led_status.cmd);

    // ========== ROUTINES ==========

    // ota_handle();
    //  read BT. packet: ös movement, ös command
    //  $TCXXYYTTC\n or $TCCC\n

    blt_status_routine();

    // ========== CODE ==========

    bt_test2();
}

void blt_status_routine()
{
    if (SerialBT.hasClient())
        g_led_status.sys = StatusLED::State::STATUS_BLE_CONNECTED;
    else
        g_led_status.sys = StatusLED::State::STATUS_BLE_SEARCHING;
}

void bt_test2()
{
    Comm::Packet bt_pkt;
    if (!comm_bt.read(bt_pkt))
        return;

    bt_pkt_handler.handle(bt_pkt);

    delay(200);
}