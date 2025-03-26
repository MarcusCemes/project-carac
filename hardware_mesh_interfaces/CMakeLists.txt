cmake_minimum_required(VERSION 3.8)
project(hardware_mesh_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotArmPosition.msg"
  "srv/RobotArmConfig.srv"
  "srv/RobotArmTool.srv"
  "srv/WindShape.srv"
)

ament_package()
