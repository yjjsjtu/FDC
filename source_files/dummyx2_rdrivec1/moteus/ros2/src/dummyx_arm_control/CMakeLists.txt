cmake_minimum_required(VERSION 3.8)
project(dummyx_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(joint_controller src/joint_controller.cpp)
ament_target_dependencies(
  joint_controller
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(pose_controller src/pose_controller.cpp)
ament_target_dependencies(
  pose_controller
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(circle_controller src/circle_controller.cpp)
ament_target_dependencies(
  circle_controller
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_geometry_msgs
)

# TCP工具中心点演示节点：TCP保持位置固定，J1/J2周期摆动
add_executable(tool_center_point_controller src/tool_center_point_controller.cpp)
ament_target_dependencies(
  tool_center_point_controller
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_geometry_msgs
)

# Xbox手柄控制节点：通过Joy话题实时控制末端位姿
add_executable(joy_stick_controller src/joy_stick_controller.cpp)
ament_target_dependencies(
  joy_stick_controller
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  sensor_msgs
  trajectory_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  joint_controller
  pose_controller
  circle_controller
  tool_center_point_controller
  joy_stick_controller
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Python 绘图脚本到 share，使 ros2 pkg prefix 路径可找到
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  FILES_MATCHING PATTERN "*.py"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
              WORLD_EXECUTE WORLD_READ
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
