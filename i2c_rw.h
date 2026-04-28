#ifndef I2C_RW_H
#define I2C_RW_H

#include <cstdint>
#include <string>

// Minimal Linux I2C helper for talking to the ESP32 RW node.
// Uses /dev/i2c-* and the I2C_RDWR ioctl.

class I2CRW
{
public:
    explicit I2CRW(const std::string& dev = "/dev/i2c-1");
    ~I2CRW();

    bool open();
    void close();

    // Set current slave address
    bool setSlave(uint8_t addr);

    // Register write/read helpers
    bool writeReg(uint8_t reg, const uint8_t* data, uint16_t len);
    bool readReg(uint8_t reg, uint8_t* data, uint16_t len);

    // Convenience types for this project
    bool writeFloat(uint8_t reg, float value);
    bool readFloat(uint8_t reg, float& value);

    bool readByte(uint8_t reg, uint8_t& value);
    bool isReady();
    bool writeFloatIfReady(uint8_t reg, float value);

    bool readRPMandFaults(float& rpm, uint32_t& faults);
    bool readFaults(uint32_t& faults);

    int fd() const { return fd_; }

private:
    std::string dev_;
    int fd_;
    uint8_t addr_;
};

#endif // I2C_RW_H
