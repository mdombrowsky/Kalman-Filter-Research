#pragma once

// =====================
// Core IMU data types
// =====================

// Minimum common IMU data (6-DoF)
struct ImuSample
{
    double ax, ay, az;  // linear acceleration [m/s^2]
    double gx, gy, gz;  // angular velocity [rad/s]
    //double temperature; // 
};

// Optional magnetometer data (9-DoF IMUs)
struct MagSample
{
    double mx, my, mz;  // magnetic field [T]
};

// =====================
// IMU driver interface
// =====================

class ImuDriver
{
public:
    virtual ~ImuDriver() = default;
    virtual bool initialize() = 0;
    virtual bool read(ImuSample &imu) = 0;

    // ---- optional ----

    // Override if IMU has a magnetometer
    virtual bool has_magnetometer() const
    {
        return false;
    }

    // Only if has_magnetometer() == true
    virtual bool read_magnetometer(MagSample &mag)
    {
        (void)mag;
        return false;
    }
};
