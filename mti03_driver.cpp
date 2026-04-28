#include "mti03_driver.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>

// ---------------- Xbus constants ----------------
constexpr uint8_t XBUS_PREAMBLE = 0xFA;
constexpr uint8_t XBUS_BID      = 0xFF;
constexpr uint8_t MID_MTDATA2   = 0x36;

// ---------------- Endian helpers ----------------
static inline uint16_t read_u16_be(const uint8_t* p) {
    return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}

static inline uint32_t read_u32_be(const uint8_t* p) {
    return (uint32_t(p[0]) << 24) |
           (uint32_t(p[1]) << 16) |
           (uint32_t(p[2]) <<  8) |
           (uint32_t(p[3]));
}

static inline float read_f32_be(const uint8_t* p) {
    uint32_t u = read_u32_be(p);
    float f;
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

// ---------------- Serial open ----------------
static int openSerial(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    termios tty{};
    tcgetattr(fd, &tty);

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = 0;

    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

// ---------------- Constructor / Destructor ----------------
MTi03Driver::MTi03Driver(const char* device)
    : fd(-1), running(false)
{
    std::memset(&state, 0, sizeof(state));
    state.valid = false;
    fd = openSerial(device);
}

MTi03Driver::~MTi03Driver() {
    stop();
    if (fd >= 0)
        close(fd);
}

// ---------------- Public API ----------------
bool MTi03Driver::start() {
    if (fd < 0) return false;
    running = true;
    rx_thread = std::thread(&MTi03Driver::run, this);
    return true;
}

void MTi03Driver::stop() {
    running = false;
    if (rx_thread.joinable())
        rx_thread.join();
}

ImuState MTi03Driver::getState() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return state;
}

// ---------------- RX thread ----------------
void MTi03Driver::run() {
    uint8_t rxbuf[2048];
    size_t rxlen = 0;

    while (running) {
        int n = read(fd, rxbuf + rxlen, sizeof(rxbuf) - rxlen);
        if (n <= 0) continue;
        rxlen += n;

        size_t idx = 0;

        while (rxlen - idx >= 6) {
            if (rxbuf[idx] != XBUS_PREAMBLE ||
                rxbuf[idx + 1] != XBUS_BID) {
                idx++;
                continue;
            }

            uint8_t mid = rxbuf[idx + 2];
            uint8_t len = rxbuf[idx + 3];
            size_t packet_len = 4 + len + 1;

            if (rxlen - idx < packet_len)
                break;

            uint8_t cs = 0;
            for (size_t i = idx + 1; i < idx + 4 + len; i++)
                cs += rxbuf[i];
            cs = 0x100 - cs;

            if (cs != rxbuf[idx + 4 + len]) {
                idx++;
                continue;
            }

            if (mid == MID_MTDATA2)
                parsePacket(&rxbuf[idx + 4], len);

            idx += packet_len;
        }

        if (idx > 0) {
            memmove(rxbuf, rxbuf + idx, rxlen - idx);
            rxlen -= idx;
        }
    }
}

// ---------------- MTData2 parser ----------------
void MTi03Driver::parsePacket(const uint8_t* payload, size_t len) {
    size_t k = 0;

    float q[4] = {0};
    float w[3] = {0};
    float m[3] = {0};
    uint64_t ts = 0;

    bool haveQuat = false;
    bool haveRate = false;
    bool haveMag  = false;
    bool haveTime = false;

    while (k + 3 <= len) {
        uint16_t dataId = read_u16_be(&payload[k]);
        uint8_t size = payload[k + 2];
        k += 3;

        if (k + size > len) break;

        switch (dataId) {

        case 0x2010: // Quaternion
            if (size == 16) {
                q[0] = read_f32_be(&payload[k + 0]);
                q[1] = read_f32_be(&payload[k + 4]);
                q[2] = read_f32_be(&payload[k + 8]);
                q[3] = read_f32_be(&payload[k + 12]);
                haveQuat = true;
            }
            break;

        case 0x8020: // Angular rate (rad/s)
            if (size == 12) {
                w[0] = read_f32_be(&payload[k + 0]);
                w[1] = read_f32_be(&payload[k + 4]);
                w[2] = read_f32_be(&payload[k + 8]);
                haveRate = true;
            }
            break;

        case 0xC020: // Magnetic field
            if (size == 12) {
                m[0] = read_f32_be(&payload[k + 0]);
                m[1] = read_f32_be(&payload[k + 4]);
                m[2] = read_f32_be(&payload[k + 8]);
                haveMag = true;
            }
            break;

        case 0x1060: // Timestamp (device)
            if (size == 4) {
                ts = read_u32_be(&payload[k]);
                haveTime = true;
            }
            break;

        default:
            break;
        }

        k += size;
    }

    if (haveQuat && haveRate) {std::memcpy(state.q, q, sizeof(q));}

    if (haveRate) {std::memcpy(state.omega, w, sizeof(w));}

    if (haveMag) {std::memcpy(state.mag, m, sizeof(m));}

    if (haveTime) {state.timestamp = ts;}

    state.valid = true;
}