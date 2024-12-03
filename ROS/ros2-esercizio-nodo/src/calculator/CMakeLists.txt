cmake_minimum_required(VERSION 3.8)
project(calculator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(node src/node.cpp)
ament_target_dependencies(node rclcpp std_msgs)

install(TARGETS
  node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
