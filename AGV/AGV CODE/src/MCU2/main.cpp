#include <Arduino.h>
#include <types.h>
#include <comm.h>
#include <ota.h>
#include "motor_driver.h"
#include "system_actions.h"

// UART
#define PIN_MTM_TXD 23
#define PIN_MTM_RXD 22
// DRIVER
#define PIN_ERR 34
#define PIN_EN 21
#define PIN_DRV 32
// DIR [ 25, 26, 27, 33 ]
#define PIN_DIR1 25 // real 3
#define PIN_DIR2 33 // real 4
#define PIN_DIR3 26 // real 2
#define PIN_DIR4 27 // real 1
// PWM [ 16, 17, 18, 19 ]
#define PIN_PWM1 18 // real 3
#define PIN_PWM2 19 // real 4
#define PIN_PWM3 17 // real 2
#define PIN_PWM4 16 // real 1

// ========== DEFINITIONS ==========
#define UART_BAUD 115200

MotorDriver::MotorDriverConfig cfg = {
    {PIN_DIR1, PIN_PWM1, 0, false},
    {PIN_DIR2, PIN_PWM2, 1, true},
    {PIN_DIR3, PIN_PWM3, 2, false},
    {PIN_DIR4, PIN_PWM4, 3, true},
    PIN_DRV,
    PIN_EN,
    PIN_ERR};

MotorDriver md(cfg);

Comm comm_mcu(Serial1, "MCU");
SysCtrl sysctrl(comm_mcu, md);

// ========== GLOBALS ==========
Debug g_debug;

// ========== PROTOTYPES ==========

void setup()
{
    // ========== UART ==========
    Serial.begin(UART_BAUD, SERIAL_8N1); // PC
    Serial1.begin(UART_BAUD, SERIAL_8N1,
                  PIN_MTM_RXD, PIN_MTM_TXD); // MCU1
    Serial.setTimeout(30);                   // PC
    Serial1.setTimeout(30);                  // MCU1

    Serial.println("[MAIN] Running setup...");

    // ========== NETWORK ==========
    // ota_begin("QBit", "internet");

    // ========== MOTOR DRIVER ==========
    md.setup();
    // md.temperature_error_reset_all();
    md.outputs_enable();
    md.drivers_enable();

    md.move(0x00, 60, 0);
    delay(2000);
    md.move(0x01, 60, 0);
    delay(2000);

    md.move(0x00, 80, 0);
    delay(2000);
    md.move(0x01, 80, 0);
    delay(2000);

    md.move(0x00, 100, 0);
    delay(2000);
    md.move(0x01, 100, 0);
    delay(2000);

    md.channels_stop_all();
    // ========== END ==========
    Serial.println("[MAIN] Setup finished");
}

void loop()
{
    // ========== UPDATES ==========
    md.update();

    // ========== ROUTINES ==========

    // ========== CODE ==========

    // Read from MCU1 and process packet
    Comm::Packet mcu_pkt;
    if (comm_mcu.read(mcu_pkt))
    {
        Serial.println("HELP");
        sysctrl.on_mcu_pkt_recieved(mcu_pkt);
    }
    delay(200);
}
