#pragma once
#include "types.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#ifndef MPU6050
#define MPU6050

class imuData {
    public:
        imuData();

        bool begin();
        void update();

        float ax;
        float ay;
        float az;

        float gx;
        float gy;
        float gz;

        float temp;

    private:
    Adafruit_MPU6050 imu;
};


