#include "shared_state.h"
#include "timing.h"
#include "rw_bus.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cmath>

// Must match ESP32 register map
static constexpr uint8_t REG_TORQUE_CMD = 0x00;

void control_task(SharedState& shared,
                  std::atomic<bool>& stop_flag,
                  std::array<RwBus, NUM_WHEELS>& rws)
{
    using namespace std::chrono;
    const auto period = 200ms; // 5 Hz
    const float period_sec = 0.200; // [seconds]
    const float torque_command = 0.01f; // [Nm]
    const float rpm_range = 500; // [rpm]

    std::array<float, NUM_WHEELS> torque_cmd{};
    torque_cmd.fill(0.0f);
    int counter = 0;
    int bounds = (1/period_sec)/torque_command*rpm_range/2500; // 0.05 torque w/ 500rpms --> 20 bounds,  0.001 torque w/ 50 rpms --> 100 bounds

    auto next = steady_clock::now();
    while (!stop_flag.load())
    {
        next += period;

        // Calculate Torque Command
        if (counter < bounds/2)
        {
            torque_cmd.fill(torque_command);   // 0–9 → 10 commands
        }
        else if(counter >= bounds/2 && counter < bounds)
        {
            torque_cmd.fill(-torque_command);  // 10–19 → 10 commands
        }
        else
        {
            counter = 0;
        }

        // Publish commanded torque for telemetry (even if wheel absent)
        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.wheel_torque_cmd_nm = torque_cmd;
            shared.last_ctrl_us = now_us();
        }

        // Command and readback
        for (int i = 0; i < NUM_WHEELS; ++i)
        {
            if (!rws[i].ok) continue;

            (void)rws[i].bus.setSlave(rws[i].addr);

            if (std::fabs(torque_cmd[i]) > 1e-5f)
                if (!rws[i].bus.writeFloat(REG_TORQUE_CMD, torque_cmd[i]))
                {
                    std::fprintf(stderr,
                        "[control] RW%d write torque failed (addr 0x%02X) — disabling\n",
                        i, rws[i].addr);
                    rws[i].ok = false;

                    std::lock_guard<std::mutex> lock(shared.mtx);
                    shared.wheels[i].present = false;
                    continue;
                }

            float rpm = 0.0f; uint32_t faults = 0;
            if (rws[i].bus.readRPMandFaults(rpm, faults))
            {
                std::lock_guard<std::mutex> lock(shared.mtx);
                shared.wheels[i].present = true;
                shared.wheels[i].rpm = rpm;
                shared.wheels[i].faults = faults;
                shared.last_rw_us = now_us();
            }
        }

        counter += 1;
        sleep_until(next);
    }
}
