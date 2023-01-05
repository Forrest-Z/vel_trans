cmake_minimum_required(VERSION 3.5)
project(path_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

ament_auto_find_build_dependencies()

## Generate services in the 'srv' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPathName.srv"
  "srv/GlobalPath.srv"
  DEPENDENCIES
  nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Generate exe file
ament_auto_add_library(record_path_node SHARED
  src/record_path_node.cpp
)
# For using message definitions from the same package
rosidl_target_interfaces(record_path_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rclcpp_components_register_node(record_path_node
  PLUGIN "RecordPathNode"
  EXECUTABLE record_path
)

# Generate exe file
ament_auto_add_library(load_path_node SHARED
  src/load_path_node.cpp
)

# For using message definitions from the same package
rosidl_target_interfaces(load_path_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rclcpp_components_register_node(load_path_node
  PLUGIN "LoadPathNode"
  EXECUTABLE load_path
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE)
