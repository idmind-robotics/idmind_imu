cmake_minimum_required(VERSION 3.8)
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
# Vendored TinkerForge C bindings.
#
# TinkerForge ships these only as a source download (CC0 1.0 licensed), with no apt package and
# therefore no rosdep rule. Vendoring them keeps `rosdep install` working and the build
# reproducible. They are third-party C and are deliberately compiled without this project's
# warning flags and excluded from linting.
# ---------------------------------------------------------------------------------------------
add_library(tinkerforge_bindings STATIC
  vendor/tinkerforge/ip_connection.c
  vendor/tinkerforge/brick_imu_v2.c
)
target_include_directories(tinkerforge_bindings PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor>
)
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
  # The vendored bindings are third-party CC0 code: upstream style, upstream copyright header.
  # Linting them would mean either 3000 failures or reformatting code we want to keep
  # byte-identical to the upstream release so it can be re-downloaded and diffed.
  set(ament_cmake_copyright_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck)
  file(GLOB_RECURSE VENDORED_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/vendor/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/vendor/*.h"
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${VENDORED_SOURCES})
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
