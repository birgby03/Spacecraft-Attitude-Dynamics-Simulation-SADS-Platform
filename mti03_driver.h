#pragma once

#include <cstdint>
#include <atomic>
#include <mutex>
#include <thread>

struct ImuState {
    float q[4];        // quaternion
    float omega[3];    // angular rate (rad/s)
    float mag[3];      // magnetic field (sensor units)
    uint64_t timestamp; // device timestamp (ticks or usec, config-dependent)
    bool valid;
};

class MTi03Driver {
public:
    explicit MTi03Driver(const char* device);
    ~MTi03Driver();

    bool start();
    void stop();

    ImuState getState();

private:
    void run();
    void parsePacket(const uint8_t* payload, size_t len);

    int fd;
    std::atomic<bool> running;
    std::thread rx_thread;

    std::mutex state_mutex;
    ImuState state;
};