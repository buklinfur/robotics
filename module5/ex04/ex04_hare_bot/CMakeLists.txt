cmake_minimum_required(VERSION 3.8)
project(ex04_hare_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(circle_movement src/circle_movement.cpp)
ament_target_dependencies(circle_movement rclcpp geometry_msgs)

install(TARGETS circle_movement
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config urdf worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
