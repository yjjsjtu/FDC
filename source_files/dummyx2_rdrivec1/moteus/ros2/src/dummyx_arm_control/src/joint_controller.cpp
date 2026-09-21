#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("dummyx_arm_joint_control_node");

    // 设置日志级别（可选）
    rclcpp::Logger logger = node->get_logger();
    RCLCPP_INFO(logger, "DummyX Arm 关节控制节点已启动");
    // 动作组设置
    moveit::planning_interface::MoveGroupInterface move_group(node, "dummyx_arm");
    // 运动规划与执行
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // 参数配置
    move_group.setPlanningTime(10.0);          // 规划超时时间（秒）
    move_group.setNumPlanningAttempts(10);     // 规划尝试次数
    move_group.setGoalJointTolerance(0.02);    // 关节目标公差（弧度）
    move_group.setMaxVelocityScalingFactor(0.2); // 速度缩放（0-1，新手建议设小）
    move_group.setMaxAccelerationScalingFactor(0.1); // 加速度缩放
    // 各个关节名称设置
    std::vector<std::string> joint_names = {
        "Joint1", 
        "Joint2", 
        "Joint3",
        "Joint4", 
        "Joint5", 
        "Joint6"
    };    

    //设置目标关节角度（弧度值，示例值，可根据需求修改）
    std::vector<double> target_joint_values = {
        0.0,    // Joint1
        -0.785, // Joint2 (≈-45°)
        0.0,    // Joint3
        0.785,  // Joint4 (≈45°)
        0.0,    // Joint5
        0.0     // Joint6
    };

    // 校验关节数量是否匹配
    if (target_joint_values.size() != joint_names.size()) {
        RCLCPP_ERROR(logger, "目标关节值数量与关节名称数量不匹配！");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::sleep_for(std::chrono::seconds(5));
    // 设置目标关节位置
    move_group.setJointValueTarget(joint_names, target_joint_values);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // 规划成功，执行
    if (success) {
        RCLCPP_INFO(logger, "运动规划成功，开始执行轨迹");
        move_group.execute(my_plan); // 执行规划好的轨迹
    } else {
        RCLCPP_ERROR(logger, "运动规划失败，请检查关节限制/碰撞/目标值！");
    }
    
    // 回到初始位姿（示例）
    RCLCPP_INFO(logger, "5秒后返回初始位姿...");
    rclcpp::sleep_for(std::chrono::seconds(5));
    move_group.setNamedTarget("home"); // 需确保MoveIt配置中定义了"home"位姿
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
        move_group.execute(my_plan);
        RCLCPP_INFO(logger, "已返回初始位姿");
    }

    // 关闭节点
    RCLCPP_INFO(logger, "关节控制节点执行完成");
    rclcpp::shutdown();
    return 0;
}