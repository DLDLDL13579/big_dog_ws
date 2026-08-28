cmake_minimum_required(VERSION 3.8)
project(dog_brain)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# RefreshPlanningPose BT 节点插件
add_library(refresh_planning_pose_bt_node SHARED
  src/refresh_planning_pose.cpp
)
target_include_directories(refresh_planning_pose_bt_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# 必须定义 BT_PLUGIN_EXPORT，否则 BTCPP_EXPORT 被定义为 static，符号不导出
target_compile_definitions(refresh_planning_pose_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(refresh_planning_pose_bt_node
  rclcpp
  geometry_msgs
  behaviortree_cpp_v3
)

install(TARGETS refresh_planning_pose_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
