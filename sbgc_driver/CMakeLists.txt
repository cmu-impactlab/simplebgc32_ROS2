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
find_package(sbgc_protocol REQUIRED)

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

install(DIRECTORY include/ DESTINATION include)
install(TARGETS gimbal_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(sbgc_protocol)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gimbal_core test/test_gimbal_core.cpp)
  target_link_libraries(test_gimbal_core gimbal_core)
endif()

ament_package()
