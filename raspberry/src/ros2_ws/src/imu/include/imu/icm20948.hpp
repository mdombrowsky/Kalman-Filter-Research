#pragma once

#include <string>
#include "imu/imu.hpp"

class ICM20948Driver : public ImuDriver
{
public:
    ICM20948Driver(const std::string &i2c_bus, uint8_t address);
    ~ICM20948Driver();

    bool initialize() override;
    bool read(ImuSample &imu) override;

    bool has_magnetometer() const override { return true; }
    bool read_magnetometer(MagSample &mag) override;

private:
    bool open_i2c();
    bool write_register(uint8_t reg, uint8_t value);
    bool read_register(uint8_t reg, uint8_t &value);
    bool read_bytes(uint8_t reg, uint8_t *buffer, size_t length);

    bool select_bank(uint8_t bank);
    bool init_magnetometer();

    std::string i2c_bus_;
    uint8_t address_;
    int i2c_fd_{-1};
};
