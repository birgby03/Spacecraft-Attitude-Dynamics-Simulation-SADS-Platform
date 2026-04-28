#include "shared_state.h"
#include "timing.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <algorithm>

void estimator_task(SharedState& shared, std::atomic<bool>& stop_flag)
{
    using namespace std::chrono;
    const auto period = 10ms; // 100 Hz (once per measurement)

    auto next = steady_clock::now();
    while (!stop_flag.load())
    {
        next += period;

        {
            // In a real implementation, this is where the sensor fusion / estimation algorithm would go.
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.est_imu = shared.meas_imu; // identity estimator


            // Convert quaternion to Euler angles (roll, pitch, yaw)
            const auto& q = shared.est_imu.q; // assuming scalar-first

            float q0 = q[0];
            float q1 = q[1];
            float q2 = q[2];
            float q3 = q[3];

            // Roll
            float sinr_cosp = 2.0f * (q0*q1 + q2*q3);
            float cosr_cosp = 1.0f - 2.0f * (q1*q1 + q2*q2);
            shared.est_euler.roll_rad = std::atan2(sinr_cosp, cosr_cosp);

            // Pitch
            float sinp = 2.0f * (q0*q2 - q3*q1);
            sinp = std::clamp(sinp, -1.0f, 1.0f);
            shared.est_euler.pitch_rad = std::asin(sinp);

            // Yaw
            float siny_cosp = 2.0f * (q0*q3 + q1*q2);
            float cosy_cosp = 1.0f - 2.0f * (q2*q2 + q3*q3);
            shared.est_euler.yaw_rad = std::atan2(siny_cosp, cosy_cosp);

            shared.last_est_us = now_us();
        }

        sleep_until(next);
    }
}
