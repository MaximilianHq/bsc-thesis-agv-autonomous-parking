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
        void read_print();

    private:
    Adafruit_MPU6050 mpu;
    void printSettings();
};


