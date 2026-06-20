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
find_package(rosidl_default_generators REQUIRED)

# Generate custom service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CalibrateImu.srv"
)

# Include directories
include_directories(src)

# Add IMU driver executable
add_executable(imu_driver src/imu_driver.cpp)

# Get typesupport target for local service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(imu_driver PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${cpp_typesupport_target}
)

# Ensure executable compiles after interface generation
add_dependencies(imu_driver ${PROJECT_NAME}__rosidl_typesupport_cpp)

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
