#include "types.h"
#include "system_actions.h"
#include "comm.h"
#include "imu.h"
#include "dwm.h"
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

SRegHandler sreg(PIN_SHREG_DATA, PIN_SHREG_CLK, PIN_SHREG_LATCH);
StatusLED led_sys(sreg, SRegHandler::pin_sreg::QA,
                  SRegHandler::pin_sreg::QB,
                  SRegHandler::pin_sreg::QC,
                  true);
StatusLED led_cmd(sreg, SRegHandler::pin_sreg::QD,
                  SRegHandler::pin_sreg::QE,
                  SRegHandler::pin_sreg::QF,
                  true);

SysCtrl sysctrl(comm_bt, comm_mcu, led_sys, led_cmd);

Sonar::SonarConfig sonar_cfg{
    PIN_SONAR_SERVO,
    PIN_SONAR_TRIG,
    PIN_SONAR_ECHO,
    SONAR_RANGE,
    SONAR_SPEED,
    SONAR_ANGLE,
    90,
    0,
    180,
    false};
Sonar sonar(sonar_cfg, sysctrl);

// ========== GLOBALS ==========
Debug g_debug;

// TODO MESSAGE SENDING BUFFER
// TODO CHECK INCOMMING AFFIRM
// TODO MAP WHOLE COMMUNICATION PROTOCOL
// TODO MOVEMENT SEQUENCE BUFFER

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
    led_sys.set_status(StatusLED::State::STATUS_READY);
}

void loop()
{
    // ========== UPDATES ==========
    sonar.update();
    led_sys.update();
    led_cmd.update();

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
        led_sys.set_status(StatusLED::State::STATUS_BLE_CONNECTED);
    else
        sysctrl.on_bt_no_connect();
}

void bt_test2()
{
    Comm::Packet bt_pkt;
    if (!comm_bt.read(bt_pkt))
        return;

    sysctrl.on_bt_pkt_recieved(bt_pkt);

    delay(200);
}