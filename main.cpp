#include "shared_state.h"
#include "timing.h"
#include "rw_bus.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

// Task entry points
void imu_task(SharedState& shared, std::atomic<bool>& stop_flag, MTi03Driver& imu);
void estimator_task(SharedState& shared, std::atomic<bool>& stop_flag);
void control_task(SharedState& shared, std::atomic<bool>& stop_flag, std::array<RwBus, NUM_WHEELS>& rws);
void comm_task(SharedState& shared, std::atomic<bool>& stop_flag);
void server_task(SharedState& shared, std::atomic<bool>& stop_flag, int port, const std::string& web_root);

static std::atomic<bool> g_stop{false};

static void on_sigint(int)
{
    g_stop.store(true);
}

struct Args
{
    std::string imu_dev = "/dev/ttyUSB0";
    std::string i2c_dev = "/dev/i2c-1";
    std::array<uint8_t, NUM_WHEELS> rw_addrs{ {0x20, 0x21, 0x22, 0x23} };
    int http_port = 8080;
    std::string web_root = "../web";
};

static Args parse_args(int argc, char** argv)
{
    Args a;
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--imu") == 0 && i + 1 < argc) a.imu_dev = argv[++i];
        else if (std::strcmp(argv[i], "--i2c") == 0 && i + 1 < argc) a.i2c_dev = argv[++i];
        else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) a.http_port = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--web") == 0 && i + 1 < argc) a.web_root = argv[++i];
    }
    return a;
}

int main(int argc, char** argv)
{
    auto args = parse_args(argc, argv);
    std::signal(SIGINT, on_sigint);

    SharedState shared;

    // IMU driver
    MTi03Driver imu(args.imu_dev.c_str());
    if (!imu.start())
    {
        std::perror("[main] Failed to start IMU driver");
        return 1;
    }
    std::printf("[main] IMU started on %s\n", args.imu_dev.c_str());

    // RW buses
    std::array<RwBus, NUM_WHEELS> rws;
    for (int i = 0; i < NUM_WHEELS; ++i)
    {
        rws[i] = RwBus(args.i2c_dev);
        rws[i].addr = args.rw_addrs[i];

        bool ok = rws[i].bus.open() && rws[i].bus.setSlave(rws[i].addr);
        rws[i].ok = ok;

        std::lock_guard<std::mutex> lock(shared.mtx);
        shared.wheels[i].present = ok; // NOTE: may still be absent physically; will be disabled on first write failure
        if (ok)
            std::printf("[main] RW%d ok on %s addr 0x%02X\n", i, args.i2c_dev.c_str(), rws[i].addr);
        else
            std::printf("[main] RW%d failed on %s addr 0x%02X\n", i, args.i2c_dev.c_str(), rws[i].addr);
    }

    std::atomic<bool> stop_flag{false};

    std::thread t_imu(imu_task, std::ref(shared), std::ref(stop_flag), std::ref(imu));
    std::thread t_est(estimator_task, std::ref(shared), std::ref(stop_flag));
    std::thread t_ctrl(control_task, std::ref(shared), std::ref(stop_flag), std::ref(rws));
    std::thread t_comm(comm_task, std::ref(shared), std::ref(stop_flag));
    std::thread t_srv(server_task, std::ref(shared), std::ref(stop_flag), args.http_port, args.web_root);

    std::puts("[main] Running. Press Ctrl+C to quit.");
    while (!g_stop.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::puts("\n[main] Shutting down...");
    stop_flag.store(true);

    t_srv.join();
    t_comm.join();
    t_ctrl.join();
    t_est.join();
    t_imu.join();

    imu.stop();
    std::puts("[main] Done.");
    return 0;
}
