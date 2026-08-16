cmake_minimum_required(VERSION 3.16)
project(sbgc_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoardInfo.msg"
  "msg/GimbalStatus.msg"
  "srv/GetBoardInfo.srv"
  "srv/SetControlMode.srv"
  "action/CalibrateGyro.action"
  DEPENDENCIES action_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
