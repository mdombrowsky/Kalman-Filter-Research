cmake_minimum_required(VERSION 3.8)
project(imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#ament_package()

include_directories(include)

add_library(imu_drivers
  src/ism330dhcx.cpp
  src/icm20948.cpp
)

ament_target_dependencies(imu_drivers
  rclcpp
)

target_include_directories(imu_drivers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#add_executable(imu_node src/imu.cpp)
#add_executable(imu_publisher_node src/imu_publisher.cpp)
add_executable(imu_node
  src/imu_publisher.cpp
)

#ament_target_dependencies(imu_node rclcpp sensor_msgs)
#ament_target_dependencies(imu_publisher_node rclcpp sensor_msgs)

ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
)

target_link_libraries(imu_node
  imu_drivers
)

install(TARGETS
  imu_drivers
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(imu_drivers)

ament_package()
