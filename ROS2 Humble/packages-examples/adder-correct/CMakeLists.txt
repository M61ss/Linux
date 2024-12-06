cmake_minimum_required(VERSION 3.8)
project(adder-correct)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs message_filters)

add_executable(${PROJECT_NAME}_node src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs message_filters)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}/launch
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
