cmake_minimum_required(VERSION 3.15)
project(av_dm_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy)
find_package(example_interfaces)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  nodes/speed_control.py
  nodes/dummy_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()