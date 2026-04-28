#pragma once
#include <chrono>
#include <cstdint>
#include <thread>

inline uint64_t now_us()
{
    using namespace std::chrono;
    return (uint64_t)duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

template <class TP>
inline void sleep_until(const TP& tp)
{
    std::this_thread::sleep_until(tp);
}
