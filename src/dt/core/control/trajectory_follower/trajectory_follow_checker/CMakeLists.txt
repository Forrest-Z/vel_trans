cmake_minimum_required(VERSION 3.5)
project(trajectory_follow_checker)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()


include_directories(
  include
)

# Target
## trajectory_follow_checker_node
ament_auto_add_library(trajectory_follow_checker SHARED
  src/trajectory_follow_checker_node.cpp
)

rclcpp_components_register_node(trajectory_follow_checker
  PLUGIN "TrajectoryFollowCheckerNode"
  EXECUTABLE trajectory_follow_checker_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
