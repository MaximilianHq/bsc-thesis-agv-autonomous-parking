#include <Arduino.h>
#include <types.h>
#include "motor_driver.h"
#include "system_actions.h"

#define PIN_ERR 34
#define PIN_EN 21
#define PIN_DRV 32
#define PIN_DIR1 33
#define PIN_DIR2 25
#define PIN_DIR3 26
#define PIN_DIR4 27
#define PIN_PWM1 19
#define PIN_PWM2 18
#define PIN_PWM3 17
#define PIN_PWM4 16

#define UART_BAUD 115200

MotorDriver::MotorDriverConfig cfg = {
    {PIN_DIR1, PIN_PWM1},
    {PIN_DIR2, PIN_PWM2},
    {PIN_DIR3, PIN_PWM3},
    {PIN_DIR4, PIN_PWM4},
    PIN_DRV,
    PIN_EN,
    PIN_ERR};

MotorDriver md(cfg);

Debug g_debug;

void setup()
{
    Serial.begin(UART_BAUD); // PC
    md.setup();
    // md.temperature_error_reset_all();
    md.outputs_enable();
    md.drivers_enable();
    md.move(0x00, 20, 5000000);
}

void loop()
{
    md.update();
}