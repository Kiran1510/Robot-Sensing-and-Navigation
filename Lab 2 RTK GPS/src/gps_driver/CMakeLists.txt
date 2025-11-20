cmake_minimum_required(VERSION 3.8)
project(gps_driver)

#required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#generate RTK message only
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Customrtk.msg"
  DEPENDENCIES std_msgs
)

#ixport message runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

# Install RTK Python executable only
install(PROGRAMS
  python/rtk_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

#install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
