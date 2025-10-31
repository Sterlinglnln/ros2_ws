cmake_minimum_required(VERSION 3.8)
project(mg6010_can_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/mg6010_hardware.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface pluginlib rclcpp rclcpp_lifecycle
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "MG6010_CAN_CONTROL_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME})

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
