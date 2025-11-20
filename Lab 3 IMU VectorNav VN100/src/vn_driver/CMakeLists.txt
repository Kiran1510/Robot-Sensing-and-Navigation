cmake_minimum_required(VERSION 3.8)
project(vn_driver)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Vectornav.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# Install the Python driver node
install(
  PROGRAMS
    scripts/vn_driver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vn_driver
)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

# Install launch files so ros2 launch can find
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Finalize package
ament_package()
