cmake_minimum_required(VERSION 3.22)
project(qi_robot_pinocchio_ik)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/pinocchio_ik_plugin.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs moveit_core moveit_kinematics moveit_msgs pluginlib rclcpp)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen pinocchio::pinocchio)

pluginlib_export_plugin_description_file(moveit_core pinocchio_ik_plugin_description.xml)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES pinocchio_ik_plugin_description.xml DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3 geometry_msgs moveit_core moveit_kinematics moveit_msgs pinocchio pluginlib rclcpp)
ament_package()
