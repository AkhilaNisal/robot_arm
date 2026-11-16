cmake_minimum_required(VERSION 3.8)
project(robot_arm_rviz_panel)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(${PROJECT_NAME} SHARED
  include/robot_arm_rviz_panel/planning_progress_panel.hpp
  src/planning_progress_panel.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib rclcpp rviz_common std_msgs
)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_package()
