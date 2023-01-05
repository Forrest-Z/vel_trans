cmake_minimum_required(VERSION 3.5)
project(topic_tools)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(topic_tools_node SHARED src/relay.cpp)

rclcpp_components_register_node(topic_tools_node
  PLUGIN "topic_tools::RelayNode"
  EXECUTABLE relay
)

ament_auto_add_library(hz_checker SHARED src/hz_checker.cpp)

rclcpp_components_register_node(hz_checker
  PLUGIN "topic_tools::HzCheckerNode"
  EXECUTABLE hz_checker_node
)

ament_auto_add_library(node_list_component SHARED src/node_list.cpp)

rclcpp_components_register_node(node_list_component
  PLUGIN "topic_tools::NodeList"
  EXECUTABLE node_list
)

ament_auto_add_library(topic_list_component SHARED src/topic_list.cpp)

rclcpp_components_register_node(topic_list_component
  PLUGIN "topic_tools::TopicList"
  EXECUTABLE topic_list
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
