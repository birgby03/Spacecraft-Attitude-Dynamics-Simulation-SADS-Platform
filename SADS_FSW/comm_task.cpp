#include "shared_state.h"
#include "timing.h"

#include <atomic>
#include <chrono>

// Comm task placeholder.
// In the future this is where you would stream telemetry to InfluxDB / UDP / etc.
// For the first draft, HTTP/AJAX is handled by server_task.cpp
void comm_task(SharedState& shared, std::atomic<bool>& stop_flag)
{
    using namespace std::chrono;
    const auto period = 200ms; // 5 Hz

    auto next = steady_clock::now();
    while (!stop_flag.load())
    {
        next += period;

        // Touch shared state so the thread isn't "dead code".
        (void)shared.snapshot();

        sleep_until(next);
    }
}
