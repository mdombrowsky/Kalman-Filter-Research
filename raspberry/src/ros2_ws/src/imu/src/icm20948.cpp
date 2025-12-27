#include "imu/icm20948.hpp"

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cmath>

/* ================= Registers ================= */

// Bank select
#define REG_BANK_SEL     0x7F
#define BANK_0           0x00
#define BANK_2           0x20

// WHO_AM_I
#define WHO_AM_I_REG     0x00
#define WHO_AM_I_VAL     0xEA

// Accel / Gyro config (Bank 2)
#define ACCEL_CONFIG     0x14
#define GYRO_CONFIG_1    0x01

// Data registers (Bank 0)
#define ACCEL_XOUT_H     0x2D
#define GYRO_XOUT_H      0x33

// Bank 0 Registers
#define REG_PWR_MGMT_1    0x06
#define REG_INT_PIN_CFG   0x0F  // for magnetometer bypass

// Magnetometer (AK09916)
#define AK09916_I2C_ADDR 0x0C
#define AK09916_CNTL2   0x31
#define AK09916_ST1     0x10
#define AK09916_HXL     0x11

/* ================= Constructor ================= */

ICM20948Driver::ICM20948Driver(const std::string &i2c_bus, uint8_t address)
	: i2c_bus_(i2c_bus), address_(address)
{
}

ICM20948Driver::~ICM20948Driver()
{
	if (i2c_fd_ >= 0)
	{
		close(i2c_fd_);
	}
}

/* ================= I2C ================= */

bool ICM20948Driver::open_i2c()
{
	i2c_fd_ = open(i2c_bus_.c_str(), O_RDWR);
	if (i2c_fd_ < 0) return false;

	if (ioctl(i2c_fd_, I2C_SLAVE, address_) < 0) return false;
	return true;
}

bool ICM20948Driver::write_register(uint8_t reg, uint8_t value)
{
	uint8_t buffer[2] = { reg, value };
	return ::write(i2c_fd_, buffer, 2) == 2;
}

bool ICM20948Driver::read_register(uint8_t reg, uint8_t &value)
{
	if (::write(i2c_fd_, &reg, 1) != 1) return false;
	if (::read(i2c_fd_, &value, 1) != 1) return false;
	return true;
}

bool ICM20948Driver::read_bytes(uint8_t reg, uint8_t *buffer, size_t length)
{
	if (::write(i2c_fd_, &reg, 1) != 1) return false;
	if (::read(i2c_fd_, buffer, length) != static_cast<int>(length)) return false;
	return true;
}

bool ICM20948Driver::select_bank(uint8_t bank)
{
	return write_register(REG_BANK_SEL, bank);
}

/* ================= Initialization ================= */

bool ICM20948Driver::initialize()
{
	if (!open_i2c()) return false;

	select_bank(BANK_0);

	uint8_t whoami = 0;
	if (!read_register(WHO_AM_I_REG, whoami) || whoami != WHO_AM_I_VAL)
	{
		return false;
	}

	if (!write_register(REG_PWR_MGMT_1, 0x01)) return false;

	if (!write_register(REG_INT_PIN_CFG, 0x02)) return false;

	select_bank(BANK_2);
	write_register(ACCEL_CONFIG, 0x01); // Enable Accel + Filter
	write_register(GYRO_CONFIG_1, 0x01); // Enable Gyro + Filter

	select_bank(BANK_0);

	return init_magnetometer();
}

/* ================= Read Accel + Gyro ================= */

bool ICM20948Driver::read(ImuSample &imu)
{
	uint8_t buffer[12];

	if (!read_bytes(ACCEL_XOUT_H, buffer, 12)) return false;

	int16_t ax = (buffer[0] << 8) | buffer[1];
	int16_t ay = (buffer[2] << 8) | buffer[3];
	int16_t az = (buffer[4] << 8) | buffer[5];

	int16_t gx = (buffer[6] << 8) | buffer[7];
	int16_t gy = (buffer[8] << 8) | buffer[9];
	int16_t gz = (buffer[10] << 8) | buffer[11];

	constexpr double accel_scale = 2.0 * 9.80665 / 32768.0;
	constexpr double gyro_scale = 250.0 * M_PI / (180.0 * 32768.0);

	imu.ax = ax * accel_scale;
	imu.ay = ay * accel_scale;
	imu.az = az * accel_scale;

	imu.gx = gx * gyro_scale;
	imu.gy = gy * gyro_scale;
	imu.gz = gz * gyro_scale;

	return true;
}

/* ================= Magnetometer ================= */

bool ICM20948Driver::init_magnetometer()
{
	// switch to mag I2C address
	if (ioctl(i2c_fd_, I2C_SLAVE, AK09916_I2C_ADDR) < 0) return false;

	uint8_t mag_id = 0;
	read_register(0x01, mag_id);
	if (mag_id != 0x09)
	{
		// If this fails, the Bypass mode is not active
		ioctl(i2c_fd_, I2C_SLAVE, address_);
		return false;
	}

	// continuous measurement mode 100 Hz
	write_register(AK09916_CNTL2, 0x08);

	// switch back to main IMU
	ioctl(i2c_fd_, I2C_SLAVE, address_);
	return true;
}

bool ICM20948Driver::read_magnetometer(MagSample &mag)
{
	// switch to mag
	if (ioctl(i2c_fd_, I2C_SLAVE, AK09916_I2C_ADDR) < 0) return false;

	uint8_t st1;
	if (!read_register(AK09916_ST1, st1))
	{
		ioctl(i2c_fd_, I2C_SLAVE, address_);
		return false;
	}

	if (!(st1 & 0x01))
	{
		ioctl(i2c_fd_, I2C_SLAVE, address_);
		return false;
	}

	uint8_t buffer[8]; // read 6 data bytes + ST2
	if (!read_bytes(AK09916_HXL, buffer, 8)) {
		ioctl(i2c_fd_, I2C_SLAVE, address_);
		return false;
	}

	int16_t mx = (buffer[1] << 8) | buffer[0];
	int16_t my = (buffer[3] << 8) | buffer[2];
	int16_t mz = (buffer[5] << 8) | buffer[4];

	constexpr double mag_scale = 0.15; // micro-Tesla per LSB

	mag.mx = mx * mag_scale;
	mag.my = my * mag_scale;
	mag.mz = mz * mag_scale;

	// switch back
	ioctl(i2c_fd_, I2C_SLAVE, address_);
	return true;
}