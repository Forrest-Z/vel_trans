cmake_minimum_required(VERSION 3.5)
project(can_bus)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Boost REQUIRED thread)
find_package(tf2 REQUIRED)
find_package(diagnostic_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
ament_auto_find_build_dependencies()

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanBusMessage.msg"
  "msg/Cmd.msg"
  "srv/CanBusService.srv"
  "srv/ControlService.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
  Boost)


# Generate exe file
ament_auto_add_library(can_bus_service_node SHARED
  src/can_bus_service_node.cpp
  include/can_bus/can_bus_service_node.hpp
)

rosidl_target_interfaces(can_bus_service_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(can_bus_service_node
  ${PROJECT_SOURCE_DIR}/lib/libECanVci.so
  ${PROJECT_SOURCE_DIR}/lib/libusb-1.0.so
  ${PROJECT_SOURCE_DIR}/lib/libusb.so
  ${Boost_LIBRARIES}
)

rclcpp_components_register_node(can_bus_service_node
  PLUGIN "CanBusServiceNode"
  EXECUTABLE can_bus_service
)

ament_auto_add_library(can_bus_pmdriver_node SHARED
  src/can_bus_pmdriver_node.cpp
  include/can_bus/can_bus_pmdriver_node.hpp
)

rclcpp_components_register_node(can_bus_pmdriver_node
  PLUGIN "CanBusPmdriver"
  EXECUTABLE can_bus_pmdriver
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  lib
)
