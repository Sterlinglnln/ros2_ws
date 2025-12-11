cmake_minimum_required(VERSION 3.8)
project(realsense_head_reconstruction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters io kdtree surface registration)

add_executable(head_reconstruction_node src/head_reconstruction_node.cpp)
ament_target_dependencies(head_reconstruction_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_msgs
  std_srvs)

target_include_directories(head_reconstruction_node PUBLIC
  ${PCL_INCLUDE_DIRS})

target_link_libraries(head_reconstruction_node
  ${PCL_LIBRARIES})

target_compile_definitions(head_reconstruction_node PUBLIC ${PCL_DEFINITIONS})

install(TARGETS head_reconstruction_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
