#pragma once

#include <string>
#include "imu/imu.hpp"

class ISM330DHCXDriver : public ImuDriver
{
public:
    ISM330DHCXDriver(const std::string &i2c_bus, uint8_t address);
    ~ISM330DHCXDriver();

    bool initialize() override;
    bool read(ImuSample &imu) override;

private:
    bool open_i2c();
    bool write_register(uint8_t reg, uint8_t value);
    bool read_register(uint8_t reg, uint8_t &value);
    bool read_bytes(uint8_t reg, uint8_t *buffer, size_t length);

    std::string i2c_bus_;
    uint8_t address_;
    int i2c_fd_{-1};
};
