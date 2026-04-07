#include "imu.h"
#include "types.h"
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

imuData::imuData() {}

bool imuData::begin() {
    Serial.println("Sensor test!");

    if (!imu.begin()) {
        if(g_debug.imu)
            Serial.println("Failed to find MPU6050 chip");
        return false;
    }

    Serial.println("MPU6050 found!");

    imu.setAccelerometerRange(MPU6050_RANGE_8_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    printSettings();

    return true;
}

void imuData::read_print() {
    sensors_event_t a;
    sensors_event_t g;
    sensors_event_t temp;

    imu.getEvent(&a, &g, &temp);

    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");

    Serial.println("");
}




