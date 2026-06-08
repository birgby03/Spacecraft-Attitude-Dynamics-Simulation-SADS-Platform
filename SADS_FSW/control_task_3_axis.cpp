#include "shared_state.h"
#include "timing.h"
#include "rw_bus.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cmath>

// Must match ESP32 register map
static constexpr uint8_t REG_TORQUE_CMD = 0x00;

// Pyramid wheel-axis geometry.
// Body-frame convention used here:
//   +X points toward the top of the platform diagram
//   +Y points toward the left of the platform diagram
//   +Z points out of the platform diagram
// Wheel numbering is RW1 at top-left, then clockwise:
//   RW1: +X, +Y quadrant
//   RW2: +X, -Y quadrant
//   RW3: -X, -Y quadrant
//   RW4: -X, +Y quadrant
//
// A maps wheel-axis torque components into body-frame torque direction.
// The later leading minus sign is intentional because commanded wheel torque
// produces an equal-and-opposite reaction torque on the platform.
float a = 57.5f * M_PI / 180.0f; // 57.5 degrees in radians

float A[3][4] = {
    {  sin(a)/sqrt(2),  sin(a)/sqrt(2), -sin(a)/sqrt(2), -sin(a)/sqrt(2) },
    {  sin(a)/sqrt(2), -sin(a)/sqrt(2), -sin(a)/sqrt(2),  sin(a)/sqrt(2) },
    {  cos(a),          cos(a),          cos(a),          cos(a)           }
};

static constexpr float pinv_scale = 0.75f;

static float clampf(float x, float lo, float hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

void control_task(SharedState& shared,
                  std::atomic<bool>& stop_flag,
                  std::array<RwBus, NUM_WHEELS>& rws)
{
    // Timing Parameters
    using namespace std::chrono;
    const auto period = 50ms;       // 20 Hz

    // Control Variables
    std::array<float, 3> eul{};
    std::array<float, 3> omega{};
    std::array<float, 3> body_torque_cmd{};
    std::array<float, NUM_WHEELS> torque_cmd{};
    torque_cmd.fill(0.0f);

    // Inertia estimates from Zorio thesis
    const float Ixx = 1.298f;
    const float Iyy = 1.025f;
    const float Izz = 1.243f;

    // Roll/pitch leveling controller.
    // These command restoring torques toward roll = 0 and pitch = 0.
    const float Wn_xy = 0.3f;
    const float zeta_xy = 1.0f;

    const float Kp_x = Ixx * Wn_xy * Wn_xy;
    const float Kp_y = Iyy * Wn_xy * Wn_xy;
    const float Kp_z = Izz * Wn_xy * Wn_xy;

    const float Kd_x = 2.0f * zeta_xy * Wn_xy * Ixx;
    const float Kd_y = 2.0f * zeta_xy * Wn_xy * Iyy;
    const float Kd_z = 2.0f * zeta_xy * Wn_xy * Izz;

    // Per-wheel safety limit.
    const float T_MAX_WHEEL = 0.05f;     // [N*m]

    auto next = steady_clock::now();
    while (!stop_flag.load())
    {
        next += period;

        {
            std::lock_guard<std::mutex> lock(shared.mtx);

            eul = { shared.est_euler.roll_rad,
                    shared.est_euler.pitch_rad,
                    shared.est_euler.yaw_rad };

            omega = { shared.est_imu.omega[0],
                      shared.est_imu.omega[1],
                      shared.est_imu.omega[2] };
        }

        
        // Restorative roll/pitch leveling control.
        // Positive roll/pitch angle or rate commands negative body torque.
        body_torque_cmd[0] = -(Kp_x * eul[0] + Kd_x * omega[0]); // roll / body X
        body_torque_cmd[1] = -(Kp_y * eul[1] + Kd_y * omega[1]); // pitch / body Y
        body_torque_cmd[2] = -(Kd_z * omega[2]); // yaw / body Z

        // Allocate body torque to four pyramid wheels.
        for (int i = 0; i < NUM_WHEELS; ++i)
        {
            torque_cmd[i] = -pinv_scale *
                (A[0][i] * body_torque_cmd[0] +
                 A[1][i] * body_torque_cmd[1] +
                 A[2][i] * body_torque_cmd[2]);

            // Clamp after allocation so the actual wheel command is limited.
            torque_cmd[i] = clampf(torque_cmd[i], -T_MAX_WHEEL, T_MAX_WHEEL);
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

            // Always write the torque command, including zero or near-zero commands.
            // This prevents an old nonzero torque from persisting on the wheel controller.
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

            float rpm = 0.0f;
            uint32_t faults = 0;
            if (rws[i].bus.readRPMandFaults(rpm, faults))
            {
                std::lock_guard<std::mutex> lock(shared.mtx);
                shared.wheels[i].present = true;
                shared.wheels[i].rpm = rpm;
                shared.wheels[i].faults = faults;
                shared.last_rw_us = now_us();
            }
        }

        sleep_until(next);
    }
}
