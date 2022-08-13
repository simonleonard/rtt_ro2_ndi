cmake_minimum_required(VERSION 3.5)
project(rtt_ros2_ndi)

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

# find dependencies
find_package(orocos-rtt REQUIRED HINTS /usr/local/orocos_toolchain/lib/cmake/orocos-rtt)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rtt_ros2 REQUIRED)
find_package(rtt_ros2_node REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

include_directories(
  include 
  )

orocos_component(rtt_ros2_ndi src/rtt_ros2_ndi.cpp)
target_link_libraries(rtt_ros2_ndi
  rtt_ros2::rtt_ros2
  rtt_ros2_node::rtt_ros2_node
)
orocos_generate_package()
