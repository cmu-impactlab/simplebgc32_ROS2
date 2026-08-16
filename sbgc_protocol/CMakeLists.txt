cmake_minimum_required(VERSION 3.16)
project(sbgc_protocol C)

find_package(ament_cmake REQUIRED)

# --------------------------------------------------------------------------
# The vendored upstream sources.
#
# Carried in-tree rather than referenced as a submodule, so that a plain clone
# of this repository builds. See vendor/README.md for the upstream commit these
# correspond to and vendor/sync.sh for taking a newer one.
# --------------------------------------------------------------------------
set(VENDOR "${CMAKE_CURRENT_SOURCE_DIR}/vendor")

# --------------------------------------------------------------------------
# Only the three translation units that decode or transmit the protocol are
# built. httpd.c, gamepad.c and the two standalone applications belong to the
# upstream tools and have no place in a ROS driver.
# --------------------------------------------------------------------------
add_library(sbgc_protocol STATIC
  "${VENDOR}/src/sbgc_api.c"
  "${VENDOR}/src/sbgc_params.c"
  "${VENDOR}/src/sbgc_gui_config.c")

set_target_properties(sbgc_protocol PROPERTIES
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  POSITION_INDEPENDENT_CODE ON)

# Upstream's own warning set, kept verbatim. -Wconversion in particular is what
# keeps the fixed-point unit conversions honest; dropping it here would silently
# weaken code that is currently built clean under it.
target_compile_options(sbgc_protocol PRIVATE
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion)

target_include_directories(sbgc_protocol PUBLIC
  "$<BUILD_INTERFACE:${VENDOR}/include>"
  "$<INSTALL_INTERFACE:include>")

# sbgc_api.c calls floor(), so the math library has to reach consumers of the
# static archive too — hence PUBLIC rather than PRIVATE.
#
# Left unguarded on purpose. A NOT-WIN32 guard would suggest this package can
# build somewhere without a separate libm, and it cannot: the vendored
# transport is POSIX termios and poll. -lm is exactly as portable as the rest
# of the library, and a guard would only let a doomed build fail later and
# less clearly.
target_link_libraries(sbgc_protocol PUBLIC m)

# --------------------------------------------------------------------------
# Install
# --------------------------------------------------------------------------
install(FILES
  "${VENDOR}/include/sbgc_api.h"
  "${VENDOR}/include/sbgc_params.h"
  "${VENDOR}/include/sbgc_gui_config.h"
  DESTINATION include)

install(TARGETS sbgc_protocol
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# --------------------------------------------------------------------------
# Tests
#
# Upstream's protocol suite asserts the encoder byte-for-byte against BaseCam's
# published "SBGC32 API cmd examples". Running it here means a submodule bump
# that changes the wire format fails this build — which is the entire reason
# for vendoring rather than reimplementing.
# --------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Every source file in this package is a verbatim copy of upstream's. Style
  # linting them would report on code this project does not own and must not
  # reformat: an edit here forks the wire format silently, which is exactly the
  # failure the copy-with-a-recorded-commit arrangement exists to prevent.
  #
  # The linters that cover files we DO own -- lint_cmake for this file, xmllint
  # for package.xml -- are deliberately left enabled.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cppcheck
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_pep257
    ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  add_executable(test_sbgc_api_vendor "${VENDOR}/test/test_sbgc_api.c")
  target_link_libraries(test_sbgc_api_vendor sbgc_protocol)
  add_test(NAME sbgc_api_vendor_bytes COMMAND test_sbgc_api_vendor)
endif()

ament_package()
