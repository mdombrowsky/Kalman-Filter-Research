#include "imu/ism330dhcx.hpp"

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cmath>

// Registers
#define WHO_AM_I_REG  0x0F
#define WHO_AM_I_VAL  0x6B

#define CTRL1_XL      0x10
#define CTRL2_G       0x11

#define OUTX_L_G      0x22
#define OUTX_L_XL     0x28

ISM330DHCXDriver::ISM330DHCXDriver(const std::string &i2c_bus, uint8_t address)
: i2c_bus_(i2c_bus), address_(address)
{
}

ISM330DHCXDriver::~ISM330DHCXDriver()
{
    if (i2c_fd_ >= 0)
	{
        close(i2c_fd_);
    }
}

bool ISM330DHCXDriver::open_i2c()
{
    i2c_fd_ = open(i2c_bus_.c_str(), O_RDWR);
    if (i2c_fd_ < 0) return false;

    if (ioctl(i2c_fd_, I2C_SLAVE, address_) < 0) return false;

    return true;
}

bool ISM330DHCXDriver::write_register(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    return ::write(i2c_fd_, buffer, 2) == 2;
}

bool ISM330DHCXDriver::read_register(uint8_t reg, uint8_t &value)
{
    if (::write(i2c_fd_, &reg, 1) != 1) return false;
    if (::read(i2c_fd_, &value, 1) != 1) return false;
    return true;
}

bool ISM330DHCXDriver::read_bytes(uint8_t reg, uint8_t *buffer, size_t length)
{
    if (::write(i2c_fd_, &reg, 1) != 1) return false;
    if (::read(i2c_fd_, buffer, length) != static_cast<int>(length)) return false;
    return true;
}

bool ISM330DHCXDriver::initialize()
{
    if (!open_i2c()) return false;

    uint8_t whoami = 0;
    if (!read_register(WHO_AM_I_REG, whoami) || whoami != WHO_AM_I_VAL)
	{
        return false;
    }

    // Accel: 104 Hz, ±2g
    if (!write_register(CTRL1_XL, 0x40)) return false;

    // Gyro: 104 Hz, ±250 dps
    if (!write_register(CTRL2_G, 0x40)) return false;

    return true;
}

bool ISM330DHCXDriver::read(ImuSample &imu)
{
    uint8_t buffer[12];

    if (!read_bytes(OUTX_L_G, buffer, 6)) return false;
    if (!read_bytes(OUTX_L_XL, buffer + 6, 6)) return false;

    int16_t gx = (int16_t)(buffer[1] << 8 | buffer[0]);
    int16_t gy = (int16_t)(buffer[3] << 8 | buffer[2]);
    int16_t gz = (int16_t)(buffer[5] << 8 | buffer[4]);

    int16_t ax = (int16_t)(buffer[7] << 8 | buffer[6]);
    int16_t ay = (int16_t)(buffer[9] << 8 | buffer[8]);
    int16_t az = (int16_t)(buffer[11] << 8 | buffer[10]);

    constexpr double accel_sensitivity = 0.061 * 9.80665 / 1000.0;
    constexpr double gyro_sensitivity  = 8.75 / 1000.0 * (M_PI / 180.0);

    imu.ax = ax * accel_sensitivity;
    imu.ay = ay * accel_sensitivity;
    imu.az = az * accel_sensitivity;

    imu.gx = gx * gyro_sensitivity;
    imu.gy = gy * gyro_sensitivity;
    imu.gz = gz * gyro_sensitivity;

    return true;
}
