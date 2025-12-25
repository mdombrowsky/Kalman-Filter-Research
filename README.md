# Kalman Filter Research



## DSP - Filter and Sensor Fusion Research


### Project Overview
This research investigates the efficacy of various Kalman filter architectures. Specifically the Basic Kalman Filter (KF), Extended Kalman Filter (EKF), and Unscented Kalman Filter (UKF). For the sensor fusion of low-cost Inertial Measurement Unit (IMU) and Global Positioning System (GPS) data.

The primary challenge is mitigating IMU drift during GPS signal loss (dead reckoning) and filtering GPS multipath noise during periods of coverage.

### Hardware Architecture
The system utilizes a Raspberry Pi 4B as the primary compute node, interfacing with high-precision SparkFun breakouts.

Component	Purpose	Interface
Raspberry Pi 4B	Central Processor (ROS2 Humble / Docker)	N/A
u-blox ZED-F9P	High-precision RTK GPS	UART/USB
ISM330DHCX	Industrial-grade 6DoF IMU (Motion Master)	I2C (0x6B)
ICM-20948	9DoF IMU (Orientation Reference/Magnetometer)	I2C (0x69)


### Sensor Fusion Strategy
ISM330DHCX: Selected as the primary source for linear acceleration and angular velocity due to its superior bias stability.

ICM-20948: Utilized primarily for its magnetometer to provide a stable absolute heading (Yaw), preventing the gyroscope integration from drifting over time.

Cross-Checking: The 6DoF data from the ISM330 is compared against the ICM-20948 in real-time to detect and reject sensor-specific vibration noise or anomalies.


### Software Stack & Methodology
The project is containerized using Docker to ensure environment parity across the ROS2 Humble ecosystem.

Pipeline
Data Acquisition: ROS2 nodes interface with hardware via I2C/UART, publishing raw /imu and /gps topics.

Preprocessing

Filtering: The core estimation engine implements:

EKF: Linearization via Jacobians for the non-linear process model.

UKF: Deterministic sampling (Sigma Points) for higher-order accuracy in non-linear propagation.

Data Logging:

### Key Results (Preliminary)

### Installation & Usage
 see subcategories for details.