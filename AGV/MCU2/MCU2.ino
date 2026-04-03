#include "types.h"
#include "motor_driver.h"
#include <Arduino.h>

#define PIN_ERR 34
#define PIN_EN 18
#define PIN_DRV 19
#define PIN_DIR1 21
#define PIN_DIR2 22
#define PIN_DIR3 23
#define PIN_DIR4 25
#define PIN_PWM1 26
#define PIN_PWM2 27
#define PIN_PWM3 32
#define PIN_PWM4 33

MotorDriverConfig cfg = {
    {PIN_DIR1, PIN_PWM1},
    {PIN_DIR2, PIN_PWM2},
    {PIN_DIR3, PIN_PWM3},
    {PIN_DIR4, PIN_PWM4},
    PIN_DRV,
    PIN_EN,
    PIN_ERR};

MotorDriver md(cfg);

Debug g_debug;
AgvState g_state;
AgvState g_state_prev;
AgvMotion g_motion;

void setup()
{
    md.setup();
    md.move(0x00, 50, 5000);
}

void loop()
{
    md.update();
}