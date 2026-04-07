#include "imu.h"
#include "types.h"
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

imuData::imuData() {}

bool imuData::begin() {

    if (!imu.begin()) {
        if(g_debug.imu)
            Serial.println("Failed to find MPU6050 chip");
        return false;
    }

    Serial.println("MPU6050 found!");

    imu.setAccelerometerRange(MPU6050_RANGE_8_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    return true;
}


void imuData::update() {
    sensors_event_t a;
    sensors_event_t g;
    sensors_event_t temp;

    imu.getEvent(&a, &g, &temp);

    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;

    gx = g.gyro.x;
    gy = g.gyro.y;
    gz = g.gyro.z;

    temperature = temp.temperature;

}




