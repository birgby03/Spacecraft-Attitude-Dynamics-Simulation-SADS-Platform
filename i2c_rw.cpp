#include "i2c_rw.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

I2CRW::I2CRW(const std::string& dev)
    : dev_(dev), fd_(-1), addr_(0)
{
}

I2CRW::~I2CRW()
{
    close();
}

bool I2CRW::open()
{
    if (fd_ >= 0) return true;
    fd_ = ::open(dev_.c_str(), O_RDWR);
    return fd_ >= 0;
}

void I2CRW::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

bool I2CRW::setSlave(uint8_t addr)
{
    addr_ = addr;
    // Some operations (like simple write/read) require this, but we use I2C_RDWR below.
    // Still set it so basic tools (i2cdetect) and fallback calls work.
    return (fd_ >= 0) && (ioctl(fd_, I2C_SLAVE, addr) >= 0);
}

bool I2CRW::writeReg(uint8_t reg, const uint8_t* data, uint16_t len)
{
    if (fd_ < 0) return false;

    // Buffer: [reg][payload...]
    uint8_t buf[1 + 64];
    if (len > 64) return false;
    buf[0] = reg;
    if (len > 0 && data) {
        std::memcpy(&buf[1], data, len);
    }

    struct i2c_msg msg {
        .addr  = addr_,
        .flags = 0,
        .len   = static_cast<__u16>(1 + len),
        .buf   = buf
    };

    struct i2c_rdwr_ioctl_data xfer {
        .msgs  = &msg,
        .nmsgs = 1
    };

    return ioctl(fd_, I2C_RDWR, &xfer) >= 0;
}

bool I2CRW::readReg(uint8_t reg, uint8_t* data, uint16_t len)
{
    if (fd_ < 0) return false;
    if (!data || len == 0) return false;

    // Two-message transaction:
    //  1) write register pointer
    //  2) read len bytes
    uint8_t regbuf[1] = { reg };

    struct i2c_msg msgs[2];
    msgs[0].addr  = addr_;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = regbuf;

    msgs[1].addr  = addr_;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = len;
    msgs[1].buf   = data;

    struct i2c_rdwr_ioctl_data xfer {
        .msgs  = msgs,
        .nmsgs = 2
    };

    return ioctl(fd_, I2C_RDWR, &xfer) >= 0;
}

bool I2CRW::writeFloat(uint8_t reg, float value)
{
    static_assert(sizeof(float) == 4, "float must be 4 bytes");
    uint8_t buf[4];
    std::memcpy(buf, &value, 4);
    return writeReg(reg, buf, 4);
}

bool I2CRW::readFloat(uint8_t reg, float& value)
{
    static_assert(sizeof(float) == 4, "float must be 4 bytes");
    uint8_t buf[4];
    if (!readReg(reg, buf, 4)) return false;
    std::memcpy(&value, buf, 4);
    return true;
}

bool I2CRW::readRPMandFaults(float& rpm, uint32_t& faults)
{
    uint8_t buf[8];
    if (!readReg(0x04, buf, 8)) return false; // REG_RPM_AND_FAULTS

    std::memcpy(&rpm,    &buf[0], 4);
    std::memcpy(&faults, &buf[4], 4);
    return true;
}

bool I2CRW::readFaults(uint32_t& faults)
{
    uint8_t buf[4];
    if (!readReg(0x08, buf, 4)) return false; // REG_FAULTS
    std::memcpy(&faults, buf, 4);
    return true;
}

bool I2CRW::readByte(uint8_t reg, uint8_t& value)
{
    return readReg(reg, &value, 1);
}

bool I2CRW::isReady()
{
    uint8_t ready = 0;
    if (!readByte(0x01, ready))   // REG_READY = 0x01
        return false;

    return (ready == 1);
}

bool I2CRW::writeFloatIfReady(uint8_t reg, float value)
{
    if (!isReady())
        return false;

    return writeFloat(reg, value);
}
