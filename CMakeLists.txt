# Portions of this file are derived from ros2_control_cmake (Apache 2.0 License)
# Copyright 2025 AIT - Austrian Institute of Technology GmbH
# See the LICENSE file for details.

cmake_minimum_required(VERSION 3.16)
project(dynamixel_hardware_interface)

# ==== Begin inlined content from ros2_control_cmake/cmake/ros2_control.cmake ====
# Macro to extract GCC_MAJOR_VERSION and GCC_MINOR_VERSION
macro(extract_gcc_version)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    string(REPLACE "." ";" VERSION_LIST ${CMAKE_CXX_COMPILER_VERSION})
    list(GET VERSION_LIST 0 GCC_MAJOR_VERSION)
    list(GET VERSION_LIST 1 GCC_MINOR_VERSION)

    message(STATUS "Detected GCC Version: ${CMAKE_CXX_COMPILER_VERSION} (Major: ${GCC_MAJOR_VERSION}, Minor: ${GCC_MINOR_VERSION})")

    # Convert to a number to avoid string comparison issues
    if(GCC_MAJOR_VERSION MATCHES "^[0-9]+$")
      math(EXPR GCC_MAJOR_VERSION "${GCC_MAJOR_VERSION}")
    endif()
    if(GCC_MINOR_VERSION MATCHES "^[0-9]+$")
      math(EXPR GCC_MINOR_VERSION "${GCC_MINOR_VERSION}")
    endif()
  endif()
endmacro()

# set compiler options depending on detected compiler
macro(set_compiler_options)
  if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                        -Werror=return-type -Werror=shadow -Werror=format
                        -Werror=missing-braces)
    message(STATUS "Compiler warnings enabled for ${CMAKE_CXX_COMPILER_ID}")

    # https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Documentation.html#compiler-and-linker-options
    if(NOT CMAKE_C_STANDARD)
      set(CMAKE_C_STANDARD 99)
    endif()
    if(NOT CMAKE_CXX_STANDARD)
      set(CMAKE_CXX_STANDARD 17)
    endif()

    # Extract major version if g++ is used
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      extract_gcc_version()
      if(DEFINED GCC_MAJOR_VERSION AND GCC_MAJOR_VERSION GREATER 10)
        # GCC 11 introduced -Werror=range-loop-construct
        add_compile_options(-Werror=range-loop-construct)
      endif()
    endif()
  endif()
endmacro()

# using this instead of visibility macros
# S1 from https://github.com/ros-controls/ros2_controllers/issues/1053
macro(export_windows_symbols)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endmacro()
# ==== End inlined content from ros2_control_cmake/cmake/ros2_control.cmake ====

set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  realtime_tools
  dynamixel_sdk
  std_srvs
  dynamixel_interfaces
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

################################################################################
# Build
################################################################################
add_library(
  ${PROJECT_NAME}
  SHARED
  src/dynamixel_hardware_interface.cpp
  src/dynamixel/dynamixel_info.cpp
  src/dynamixel/dynamixel.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/dynamixel_hardware_interface>
  ${dynamixel_sdk_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${dynamixel_sdk_LIBRARIES}
  ${std_srvs_TARGETS}
  ${dynamixel_interfaces_TARGETS}
)

pluginlib_export_plugin_description_file(hardware_interface dynamixel_hardware_interface_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/dynamixel_hardware_interface
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(PROGRAMS
  scripts/create_udev_rules
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
