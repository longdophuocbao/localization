cmake_minimum_required(VERSION 3.8)
project(localization_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_localization REQUIRED)

# Include directories
include_directories(src)

# Add IMU driver executable
add_executable(imu_driver src/imu_driver.cpp)
target_link_libraries(imu_driver PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

# Add GPS driver executable
add_executable(gps_driver src/gps_driver.cpp)
target_link_libraries(gps_driver PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

# Install executables
install(TARGETS
  imu_driver
  gps_driver
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
