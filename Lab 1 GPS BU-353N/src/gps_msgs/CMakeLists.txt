cmake_minimum_required(VERSION 3.5)
project(gps_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate both messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GPSmsg.msg"
  "msg/Customgps.msg"
  DEPENDENCIES std_msgs
)

# Make the runtime deps visible to downstream packages & tools
ament_export_dependencies(rosidl_default_runtime)

ament_package()

