cmake_minimum_required(VERSION 3.8)
project(gps_nav_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OpticalFlow.msg" "msg/LocalizationStatus.msg" "msg/NavigationStatus.msg"
  "msg/RobotCommand.msg" "msg/RobotAck.msg" "msg/RobotLinkStatus.msg"
  "msg/RobotControlStatus.msg"
  "srv/SetSession.srv" "srv/SetDestination.srv"
  DEPENDENCIES std_msgs)
ament_package()
