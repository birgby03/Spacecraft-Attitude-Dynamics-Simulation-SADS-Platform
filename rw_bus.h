#pragma once
#include <cstdint>
#include <string>
#include "i2c_rw.h"

struct RwBus
{
    I2CRW bus;
    uint8_t addr = 0;
    bool ok = false;

    RwBus() = default;
    explicit RwBus(const std::string& dev): bus(dev) {}

    bool init(const std::string& dev, uint8_t address)
    {
        bus = I2CRW(dev);
        addr = address;
        ok = bus.open() && bus.setSlave(addr);
        return ok;
    }
};
