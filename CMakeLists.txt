cmake_minimum_required(VERSION 3.8)
project(tangent_bug_pmr)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(tangent_bug src/tangent_bug.cpp)

ament_target_dependencies(tangent_bug
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS tangent_bug
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
