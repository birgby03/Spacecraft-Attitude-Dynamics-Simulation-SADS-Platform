#include "shared_state.h"
#include "timing.h"

#include <atomic>
#include <chrono>

void imu_task(SharedState& shared, std::atomic<bool>& stop_flag, MTi03Driver& imu)
{
    using namespace std::chrono;
    const auto period = 10ms; // 100 Hz

    auto next = steady_clock::now();
    while (!stop_flag.load())
    {
        next += period;

        ImuState s = imu.getState();

        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.meas_imu = s;
            shared.last_meas_us = now_us();
        }

        sleep_until(next);
    }
}
