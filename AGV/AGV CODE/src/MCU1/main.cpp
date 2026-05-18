#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ServoEasing.hpp>
#include <servo_continious.h>
#include <WiFi.h>
#include <types.h>
#include <ota.h>
#include <comm.h>
#include <sreg_handler.h>
#include <status_led.h>
#include <imu.h>
#include <dwm.h>
#include "lift.h"
#include "system_actions.h"
#include "sonar.h"

// ========== PINS ==========
// UART
#define PIN_MTM_TXD 16
#define PIN_MTM_RXD 17
#define PIN_DWM_TXD 32
#define PIN_DWM_RXD 33
// Sonar
#define PIN_SONAR_SERVO 18
#define PIN_SONAR_TRIG 19
#define PIN_SONAR_ECHO 35
// SHIFT REGISTER
#define PIN_SHREG_DATA 27
#define PIN_SHREG_LATCH 26
#define PIN_SHREG_CLK 25
// CRANE
#define PIN_CRANE_SERVO 23
#define PIN_CRANE_ENDSTOP 36
#define PIN_CRANE_BEGINSTOP 39
// IMU
#define PIN_SCL 22
#define PIN_SDA 21

// ========== DEFINITIONS ==========
#define UART_BAUD 115200
#define WATCHDOG 500 // ms

constexpr int SONAR_RANGE = 150; // mm
constexpr int SONAR_SPEED = 150; // mm
constexpr int SONAR_ANGLE = 75;  // ∓ deg

// ---------- COMM ----------
BluetoothSerial SerialBT;

Comm comm_bt(SerialBT, "BT");
Comm comm_mcu(Serial1, "MCU");

// ---------- LEDS ----------
SRegHandler sreg(PIN_SHREG_DATA, PIN_SHREG_CLK, PIN_SHREG_LATCH);
StatusLED led_sys(sreg, SRegHandler::pin_sreg::QA,
                  SRegHandler::pin_sreg::QB,
                  SRegHandler::pin_sreg::QC,
                  true);
StatusLED led_cmd(sreg, SRegHandler::pin_sreg::QD,
                  SRegHandler::pin_sreg::QE,
                  SRegHandler::pin_sreg::QF,
                  true);

Lift crane(PIN_CRANE_SERVO, PIN_CRANE_BEGINSTOP, PIN_CRANE_ENDSTOP, 1, false);

// ---------- POS ----------
DWM dwm(Serial2);
IMU imu(PIN_SDA, PIN_SCL);

SysCtrl sysctrl(comm_bt, comm_mcu, led_sys, led_cmd, dwm, crane);

// ---------- SONAR ----------
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
unsigned long last_packet_time = 0;

// ========== PROTOTYPES ==========
void blt_status_routine();
void watchdog_routine();

void setup()
{
    // ========== STATE ==========
    sreg.setup();
    led_sys.setup();
    led_cmd.setup();
    led_cmd.set_status(StatusLED::State::STATUS_BOOT);

    // ========== UART ==========
    Serial.begin(UART_BAUD, SERIAL_8N1); // PC
    Serial1.begin(UART_BAUD, SERIAL_8N1,
                  PIN_MTM_TXD, PIN_MTM_RXD); // MCU2
    Serial2.begin(UART_BAUD, SERIAL_8N1,
                  PIN_DWM_TXD, PIN_DWM_RXD); // DWM
    Serial.setTimeout(30);                   // PC
    Serial1.setTimeout(30);                  // MCU2
    Serial2.setTimeout(30);                  // DWM

    Serial.println("[MAIN] Running setup...");

    // ========== NETWORK ==========
    SerialBT.begin("AGV_BT_G2"); // ÖS
    Serial.println("[BT] Bluethooth active");
    // ota_begin("QBit", "internet");

    // ========== SONAR ==========
    sonar.setup();

    // ========== CRANE ==========
    // crane.setup();

    // ========== POS ==========
    uint16_t cfg;
    if (dwm.dwm_cfg_get(cfg))
        Serial.println("[DWM] cfg_get ok");
    imu.setup();

    // ========== END ==========
    Serial.println("[MAIN] Setup finished");

    // ========== CALIBRATION ==========
    // if (!sysctrl.on_startup())
    //     Serial.println("[SYSCTRL] WARNING - Calibration Failed");

    // watchdog start
    last_packet_time = millis();
}

void loop()
{
    // ========== ROUTINES ==========
    // ota_handle();
    //  read BT. packet: ös movement, ös command
    //  $TCXXYYTTC\n or $TCCC\n

    blt_status_routine();
    // watchdog_routine();

    // ========== UPDATES ==========
    sonar.update();
    // crane.update();
    led_sys.update();
    led_cmd.update();
    sysctrl.update();

    // ========== CODE ==========

    // ---------- COMM ----------
    // Read from Bluetooth and process packet
    Comm::Packet bt_pkt;
    if (comm_bt.read(bt_pkt))
    {
        sysctrl.on_bt_pkt_recieved(bt_pkt);
        last_packet_time = millis();
    }

    // Read from MCU2 and process packet
    Comm::Packet mcu_pkt;
    if (comm_mcu.read(mcu_pkt))
        sysctrl.on_mcu_pkt_recieved(mcu_pkt);

    // ---------- DWM ----------
    DwmState d;
    ImuState i;
    if (dwm.read(d))
    {
        if (imu.read(i))
            sysctrl.on_new_position_data(d, i);
        else if (g_debug.imu)
            Serial.println("[IMU] no read");
    }
    else if (g_debug.dwm)
        Serial.println("[DWM] no read");
}

void blt_status_routine()
{
    if (SerialBT.hasClient())
        led_sys.set_status(StatusLED::State::STATUS_BLE_CONNECTED);
    else
        sysctrl.on_bt_no_connect();
}

void watchdog_routine()
{
    if (millis() - last_packet_time > WATCHDOG)
    {
        sysctrl.on_stop();
        if (g_debug.mcu1)
            Serial.println("[WATCHDOG] WARNING - Stopping AGV");
        last_packet_time = millis();
    }
}
