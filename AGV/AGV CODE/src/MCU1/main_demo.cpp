#include <Arduino.h>
#include <ServoEasing.hpp>
#include <servo_continious.h>
#include <types.h>
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

constexpr int SONAR_RANGE = 150; // mm
constexpr int SONAR_SPEED = 150; // mm
constexpr int SONAR_ANGLE = 75;  // ∓ deg

// ---------- COMM ----------
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

DemoCtrl sysctrl(comm_mcu, led_sys, led_cmd, dwm, crane);

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
StaticVector<DemoCtrl::DemoPoint, 1> demo_points;
bool demo_started = false;

// ========== PROTOTYPES ==========
void setup_demo_points();
void demo_routine();

void setup()
{
    sreg.setup();
    led_sys.setup();
    led_cmd.setup();
    led_cmd.set_status(StatusLED::State::STATUS_BOOT);

    Serial.begin(UART_BAUD, SERIAL_8N1); // PC
    Serial1.begin(UART_BAUD, SERIAL_8N1,
                  PIN_MTM_TXD, PIN_MTM_RXD); // MCU2
    Serial2.begin(UART_BAUD, SERIAL_8N1,
                  PIN_DWM_TXD, PIN_DWM_RXD); // DWM
    Serial.setTimeout(30);
    Serial1.setTimeout(30);
    Serial2.setTimeout(30);

    Serial.println("[MAIN DEMO] Running setup...");

    sonar.setup();
    crane.setup();

    uint16_t cfg;
    if (dwm.dwm_cfg_get(cfg))
        Serial.println("[DWM] cfg_get ok");

    imu.setup();
    setup_demo_points();

    Serial.println("[MAIN DEMO] Setup finished");

    if (!sysctrl.on_startup())
        Serial.println("[SYSCTRL] WARNING - Calibration Failed");
}

void loop()
{
    sonar.update();
    crane.update();
    led_sys.update();
    led_cmd.update();
    sysctrl.update();

    demo_routine();

    Comm::Packet mcu_pkt;
    if (comm_mcu.read(mcu_pkt))
        sysctrl.on_mcu_pkt_recieved(mcu_pkt);

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

void setup_demo_points()
{
    demo_points.clear();

    // DETTA ÄR POSITIONEN MITTEN AV RUTAN
    demo_points.push_back({{4000, 0, 0}, 0x01});
}

void demo_routine()
{
    if (demo_started || demo_points.size() == 0)
        return;

    sysctrl.begin_demo(demo_points[0]);
    demo_started = true;
}
