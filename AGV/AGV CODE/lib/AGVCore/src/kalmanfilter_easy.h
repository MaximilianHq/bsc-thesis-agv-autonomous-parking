#pragma once
#include <types.h>
#include <kalmanfilter.h>

class KalmanFEasy : public KalmanFilter
{
public:
    KalmanFEasy(float dt) : KalmanFilter(dt) {}

    // Converts IMU and DWM state measurements to format Kalman filter can process
    // Returns estimated position as AgvState
    AgvState update_position(const ImuState &imu, const DwmState &dwm)
    {
        // Prepare measurement vector: [dwm.x, dwm.y, imu_derived_vx, imu_derived_vy]
        float z[meas_dim] = {
            (float)dwm.pos.x,
            (float)dwm.pos.y,
            0.0f, // TODO: derive from IMU if available
            0.0f  // TODO: derive from IMU if available
        };

        // Predict step
        _predict();

        // Update with measurements
        update(z);

        // Extract and return estimated state
        AgvState result;
        result.pos.x = (int16_t)x[0];
        result.pos.y = (int16_t)x[1];
        result.vx = x[2];
        result.vy = x[3];
        result.theta = 0.0f; // TODO: calculate from imu.wz if needed

        return result;
    }
};
