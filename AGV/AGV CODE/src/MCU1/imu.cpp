#include "imu.h"
#include <MPU6050.h>
#include <Wire.h>
#include <math.h>

static MPU6050 mpu;
static unsigned long last_time = 0;

// Skalningskonstanter
static const float ACC_SCALE = 16384.0f; // LSB/g (±2g default)
static const float G_TO_MS2 = 9.81f;

static const float GYRO_SCALE = 131.0f;  // LSB/(deg/s)

void imu_init() {
    Wire.begin(8, 9);

    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1) delay(10);
    }

    // Valfri config (default är oftast ok)
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

    last_time = millis();
}

ImuState imu_read() {
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0f;
    last_time = now;

    ImuState imu = {};

    // === KONVERTERA TILL RIKTIGA ENHETER ===
    float ax = (ax_raw / ACC_SCALE) * G_TO_MS2;
    float ay = (ay_raw / ACC_SCALE) * G_TO_MS2;
    float wz = (gz_raw / GYRO_SCALE) * DEG_TO_RAD;

    // === DEADZONE ===
    if (fabs(ax) < 0.05f) ax = 0.0f;
    if (fabs(ay) < 0.05f) ay = 0.0f;
    if (fabs(wz) < 0.01f) wz = 0.0f;

    // === KOORDINATSYSTEM (justera vid behov) ===
    imu.ax = ax;
    imu.ay = ay;
    imu.wz = wz;
    imu.dt = dt;

    return imu;
}