cmake_minimum_required(VERSION 3.16)
project(sbgc_driver CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sbgc_interfaces REQUIRED)
find_package(sbgc_protocol REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ---------------------------------------------------------------------------
# The decision layer. No ROS dependency on purpose: the interesting behaviour
# here is a set of safety rules, and rules that need a node, a serial port and
# an executor to exercise do not get exercised.
# ---------------------------------------------------------------------------
add_library(gimbal_core src/gimbal_core.cpp)
target_include_directories(gimbal_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(gimbal_core PUBLIC sbgc_protocol::sbgc_protocol)

# ---------------------------------------------------------------------------
# Parameters, declared once in YAML and generated into a validated struct.
# ---------------------------------------------------------------------------
generate_parameter_library(sbgc_driver_parameters
  src/sbgc_driver_parameters.yaml)

# ---------------------------------------------------------------------------
# The node, built as a component with a thin executable alongside it.
# ---------------------------------------------------------------------------
add_library(sbgc_driver_component SHARED
  src/driver_node.cpp
  src/sbgc_link.cpp)
target_include_directories(sbgc_driver_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(sbgc_driver_component PUBLIC
  gimbal_core
  sbgc_driver_parameters)
ament_target_dependencies(sbgc_driver_component PUBLIC
  control_msgs
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  lifecycle_msgs
  rclcpp_lifecycle
  sbgc_interfaces
  sensor_msgs
  std_srvs)
rclcpp_components_register_node(sbgc_driver_component
  PLUGIN "sbgc_driver::SbgcDriverNode"
  EXECUTABLE sbgc_driver_component_node)

add_executable(sbgc_driver_node src/main.cpp)
target_link_libraries(sbgc_driver_node sbgc_driver_component)

# Only the pieces another package can actually consume are exported.
#
# gimbal_core and the link wrapper are ordinary libraries and are exported.
# The component is not: it is loaded by name through pluginlib, nothing links
# it, and exporting it would drag in the generated parameter target, which is
# an implementation detail rather than an interface. driver_node.hpp is
# likewise not installed, because it includes the generated parameter header
# and would not compile outside this build.
install(FILES
  include/sbgc_driver/gimbal_core.hpp
  include/sbgc_driver/sbgc_link.hpp
  DESTINATION include/sbgc_driver)

install(TARGETS gimbal_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)

install(TARGETS sbgc_driver_component LIBRARY DESTINATION lib)
install(TARGETS sbgc_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(sbgc_protocol)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gimbal_core test/test_gimbal_core.cpp)
  target_link_libraries(test_gimbal_core gimbal_core)

  # The integration suite drives the real node against the upstream project's
  # own board simulator on a pty: real framing, real lifecycle transitions, no
  # mock that agrees with whatever the code happens to do. Isolated so it does
  # not see, or get seen by, anything else running concurrently.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_driver_integration.py TIMEOUT 180)
endif()

ament_package()
