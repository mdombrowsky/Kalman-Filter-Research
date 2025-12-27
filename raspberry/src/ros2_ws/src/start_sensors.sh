#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch gps gps.launch.xml &
GPS_PID=$!

ros2 launch imu imu.launch.xml &
IMU_PID=$!

trap "kill $GPS_PID $IMU_PID; wait" SIGINT SIGTERM

wait
