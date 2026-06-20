cmake_minimum_required(VERSION 3.20)
project(path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Fields2Cover REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robot_localization REQUIRED)
find_package(geographic_msgs REQUIRED)

# Generate custom service definitions
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RecordVertex.srv"
  "srv/GeneratePath.srv"
  "srv/SavePath.srv"
  "srv/LoadPath.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

# Add Fields2Cover path planner executable
add_executable(path_planner_node src/path_planner_node.cpp)

# Get typesupport target for local service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(path_planner_node PUBLIC
  Fields2Cover::Fields2Cover
  ${cpp_typesupport_target}
  ${geographic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${robot_localization_TARGETS}
  geographic_msgs::validation
  rclcpp::rclcpp
  robot_localization::rl_lib
  tf2_ros::tf2_ros
)

# Ensure executable compiles after message/service interfaces are generated
add_dependencies(path_planner_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Install executables
install(TARGETS
  path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
