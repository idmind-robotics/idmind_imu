cmake_minimum_required(VERSION 3.14)  # FetchContent_MakeAvailable
project(idmind_imu)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------------------------
# TinkerForge C bindings, fetched from the official release.
#
# TinkerForge deliberately ships no precompiled library and no distro package: their C/C++
# bindings guide says "the recommended way of using the bindings is to include their source
# code directly into your C/C++ project". So there is nothing to find_package() or link
# against - we pull the official release and compile the two files this device needs.
#
# Pinned to an exact version with a checksum: their "_latest" URL would make every build
# non-reproducible and silently change the driver under us.
#
# NOTE: this makes the build require network access on a clean build tree. Set
# TINKERFORGE_C_BINDINGS_DIR to a pre-downloaded `source/` directory to build offline.
# ---------------------------------------------------------------------------------------------
set(TINKERFORGE_C_BINDINGS_VERSION "2.1.34")
set(TINKERFORGE_C_BINDINGS_SHA256
  "1ac937fd2a4442e1fafe811c8774c1c42cda01792ec2914e4b22c10c8a9b6b4a")
set(TINKERFORGE_C_BINDINGS_DIR "" CACHE PATH
  "Path to an already-extracted tinkerforge C bindings 'source' directory. \
When empty, the pinned release is downloaded at configure time.")

if(TINKERFORGE_C_BINDINGS_DIR)
  set(TINKERFORGE_SOURCE_DIR "${TINKERFORGE_C_BINDINGS_DIR}")
  message(STATUS "Using local TinkerForge bindings: ${TINKERFORGE_SOURCE_DIR}")
else()
  string(REPLACE "." "_" _tf_version_underscored "${TINKERFORGE_C_BINDINGS_VERSION}")
  # CMake >= 3.24 warns unless told how to stamp extracted files. The DOWNLOAD_EXTRACT_TIMESTAMP
  # option does not exist on 3.22 (Humble) and would be silently swallowed into URL_HASH there,
  # so set the policy instead - it is a no-op on older CMake.
  if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
  endif()
  include(FetchContent)
  fetchcontent_declare(
    tinkerforge_c_bindings
    URL "https://download.tinkerforge.com/bindings/c/tinkerforge_c_bindings_${_tf_version_underscored}.zip"
    URL_HASH "SHA256=${TINKERFORGE_C_BINDINGS_SHA256}"
  )
  # The archive carries no CMakeLists.txt, so this only downloads and extracts.
  fetchcontent_makeavailable(tinkerforge_c_bindings)
  set(TINKERFORGE_SOURCE_DIR "${tinkerforge_c_bindings_SOURCE_DIR}/source")
endif()

foreach(_tf_file ip_connection.c ip_connection.h brick_imu_v2.c brick_imu_v2.h)
  if(NOT EXISTS "${TINKERFORGE_SOURCE_DIR}/${_tf_file}")
    message(FATAL_ERROR
      "TinkerForge bindings incomplete: ${TINKERFORGE_SOURCE_DIR}/${_tf_file} not found")
  endif()
endforeach()

# Stage the headers under a `tinkerforge/` prefix so includes read
# "tinkerforge/ip_connection.h" - cpplint requires the directory component, and the upstream
# archive has them flat in `source/`.
set(TINKERFORGE_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/tinkerforge_include")
configure_file(
  "${TINKERFORGE_SOURCE_DIR}/ip_connection.h"
  "${TINKERFORGE_INCLUDE_DIR}/tinkerforge/ip_connection.h" COPYONLY)
configure_file(
  "${TINKERFORGE_SOURCE_DIR}/brick_imu_v2.h"
  "${TINKERFORGE_INCLUDE_DIR}/tinkerforge/brick_imu_v2.h" COPYONLY)

add_library(tinkerforge_bindings STATIC
  "${TINKERFORGE_SOURCE_DIR}/ip_connection.c"
  "${TINKERFORGE_SOURCE_DIR}/brick_imu_v2.c"
)
target_include_directories(tinkerforge_bindings PUBLIC
  $<BUILD_INTERFACE:${TINKERFORGE_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
# Third-party C: built with upstream's own expectations, not this project's warning flags.
target_compile_options(tinkerforge_bindings PRIVATE -w)
target_link_libraries(tinkerforge_bindings PUBLIC Threads::Threads)
set_property(TARGET tinkerforge_bindings PROPERTY POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------------------------
# The package library: conversions, driver layer, and the hardware-agnostic node.
# ---------------------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/conversions.cpp
  src/imu_node.cpp
  src/drivers/registry.cpp
  src/drivers/brick_v2.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC tinkerforge_bindings)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  rcl_interfaces
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

# ---------------------------------------------------------------------------------------------
# Executables. `imu_node` is the real name; the other two are the pre-refactor console-script
# names, kept so already-deployed launch files keep working.
# ---------------------------------------------------------------------------------------------
foreach(executable_name imu_node imu_brick_node_v2 imu_brick_node)
  add_executable(${executable_name} src/main.cpp)
  target_link_libraries(${executable_name} PRIVATE ${PROJECT_NAME})
  install(TARGETS ${executable_name} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(
  TARGETS ${PROJECT_NAME} tinkerforge_bindings
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
# brick_v2.hpp is installed and includes "tinkerforge/...", so consumers need these too.
install(DIRECTORY ${TINKERFORGE_INCLUDE_DIR}/ DESTINATION include)
install(DIRECTORY launch config models DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} tinkerforge_bindings)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  diagnostic_msgs diagnostic_updater geometry_msgs rcl_interfaces
  rclcpp sensor_msgs std_msgs std_srvs
)

# ---------------------------------------------------------------------------------------------
# Tests. Every one of them runs with no hardware and no brickd: `test/fake_brickd.cpp` speaks
# the real BrickDaemon wire protocol on an ephemeral port.
# ---------------------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # cppcheck chokes on the fetched third-party C; everything else runs over our sources only,
  # since the bindings now live in the build tree rather than the repo.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Shared by every hardware-facing test.
  add_library(fake_brickd STATIC test/fake_brickd.cpp)
  target_include_directories(fake_brickd PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
  )
  target_link_libraries(fake_brickd PUBLIC Threads::Threads)

  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions ${PROJECT_NAME})

  ament_add_gtest(test_fake_brickd test/test_fake_brickd.cpp)
  target_link_libraries(test_fake_brickd fake_brickd tinkerforge_bindings)

  ament_add_gtest(test_brick_v2_driver test/test_brick_v2_driver.cpp)
  target_link_libraries(test_brick_v2_driver fake_brickd ${PROJECT_NAME})

  # This one owns rclcpp::init/shutdown itself, so it supplies its own main().
  ament_add_gtest(test_imu_node test/test_imu_node.cpp)
  target_link_libraries(test_imu_node fake_brickd ${PROJECT_NAME})
endif()

ament_package()
