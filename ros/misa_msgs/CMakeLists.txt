cmake_minimum_required(VERSION 3.8)
project(misa_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMode.srv"
  "srv/SetGait.srv"
  "srv/PlayPose.srv"
  "srv/SetBodyAttitude.srv"
  "srv/SetHeight.srv"
)

ament_package()
