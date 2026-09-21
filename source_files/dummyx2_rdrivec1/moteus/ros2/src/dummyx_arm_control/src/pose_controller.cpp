#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


// 封装单次运动序列（下探→后探→右探→左探）的函数
bool execute_movement_sequence(
    moveit::planning_interface::MoveGroupInterface& move_group,
    rclcpp::Logger logger,
    moveit::planning_interface::MoveGroupInterface::Plan& my_plan,
    geometry_msgs::msg::Pose& base_pose  // 基础位姿（保留orientation，仅修改position）
) {
    // 定义四个目标位姿（基于基础位姿修改position）
    // 1. 末端下探
    geometry_msgs::msg::Pose pose_down = base_pose;
    pose_down.position.z = 0.06;
    // 2. 末端后探
    geometry_msgs::msg::Pose pose_back = pose_down;
    pose_back.position.y = 0.187;
    // 3. 末端右探
    geometry_msgs::msg::Pose pose_right = pose_back;
    pose_right.position.x = 0.187;
    // 4. 末端左探
    geometry_msgs::msg::Pose pose_left = pose_back;
    pose_left.position.x = -0.187;

    // 定义运动动作的数组，方便遍历执行
    std::vector<std::pair<std::string, geometry_msgs::msg::Pose>> movement_steps = {
        {"下探", pose_down},
        {"后探", pose_back},
        {"右探", pose_right},
        {"左探", pose_left}
    };

    // 遍历执行每个动作
    for (auto& step : movement_steps) {
        std::string step_name = step.first;
        geometry_msgs::msg::Pose target_pose = step.second;

        rclcpp::sleep_for(std::chrono::seconds(3));  // 动作间等待3秒
        move_group.setPoseTarget(target_pose);

        // 规划轨迹
        bool plan_success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (plan_success) {
            RCLCPP_INFO(logger, "🔹 【%s】轨迹规划成功，开始执行", step_name.c_str());
            // 延长执行超时时间（15秒），避免控制器超时
            rclcpp::Duration exec_timeout = rclcpp::Duration::from_seconds(15.0);
            move_group.execute(my_plan);
        } else {
            RCLCPP_ERROR(logger, "❌ 【%s】轨迹规划失败！检查位姿/工作空间/碰撞", step_name.c_str());
            return false;  // 某一步失败，终止本次序列
        }
    }
    return true;  // 所有步骤执行成功
}


int main(int argc, char *argv[])
{
    // 初始化ROS2节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("dummyx_arm_pose_control_node");
    auto logger = node->get_logger();
    RCLCPP_INFO(logger, "DummyX Arm 末端位姿控制节点已启动");

    // 创建MoveGroupInterface实例，指定动作组
    moveit::planning_interface::MoveGroupInterface move_group(node, "dummyx2_arm");

    // 配置规划参数
    move_group.setPlanningTime(15.0);                // 规划超时时间（逆解可能需要更长时间）
    move_group.setNumPlanningAttempts(15);           // 逆解尝试次数
    move_group.setGoalPositionTolerance(0.005);      // 位置公差（米）
    move_group.setGoalOrientationTolerance(0.01);    // 姿态公差（弧度）
    move_group.setMaxVelocityScalingFactor(0.3);     // 速度缩放（新手建议0.1）
    move_group.setMaxAccelerationScalingFactor(0.3); // 加速度缩放

    // 设置末端执行器链接名称
    std::string ee_link_name = "link6_1";
    move_group.setEndEffectorLink(ee_link_name);

    // 定义目标位姿（位置+姿态）
    geometry_msgs::msg::Pose target_pose;

    // 设置目标位置（x/y/z，单位：米，示例值，需根据你的机械臂修改）
    target_pose.position.x = 0.0;  // 末端在x轴的位置
    target_pose.position.y = 0.287;  // 末端在y轴的位置
    target_pose.position.z = 0.14;  // 末端在z轴的位置

    // 设置目标姿态（欧拉角转四元数，更直观）
    // 欧拉角：roll(绕x轴)、pitch(绕y轴)、yaw(绕z轴)，单位：弧度
    double roll = -1.5707;    // 绕x轴旋转0°
    double pitch = 0.0;   // 绕y轴旋转0°
    double yaw = 0.0;  // 绕z轴旋转90°

    // 将欧拉角转换为四元数（MoveIt2要求姿态用四元数表示）
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw); // 欧拉角转四元数
    q.normalize(); // 归一化，确保是单位四元数
    target_pose.orientation = tf2::toMsg(q); // 转换为ROS2的Quaternion消息

    // 设置目标位姿并规划
    rclcpp::sleep_for(std::chrono::seconds(3));
    move_group.setPoseTarget(target_pose); // 核心：设置末端目标位姿
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
        RCLCPP_INFO(logger, "末端位姿规划成功，开始执行轨迹");
        move_group.execute(my_plan); // 执行规划的轨迹
    } else {
        RCLCPP_ERROR(logger, "末端位姿规划失败！请检查：");
        RCLCPP_ERROR(logger, "1. 末端链接名称是否正确");
        RCLCPP_ERROR(logger, "2. 目标位姿是否在机械臂工作空间内");
        RCLCPP_ERROR(logger, "3. 目标位姿是否存在碰撞/逆解无解");
    }

    geometry_msgs::msg::Pose base_pose = target_pose;
    // 核心：循环执行10次运动序列
    int total_cycles = 10;
    for (int cycle = 0; cycle < total_cycles; cycle++) {
        RCLCPP_INFO(logger, "\n=====================================");
        RCLCPP_INFO(logger, "🚀 开始执行第 %d/%d 轮运动序列", cycle + 1, total_cycles);
        RCLCPP_INFO(logger, "=====================================");

        // 执行单次运动序列（下探→后探→右探→左探）
        bool cycle_success = execute_movement_sequence(move_group, logger, my_plan, base_pose);
        
        if (cycle_success) {
            RCLCPP_INFO(logger, "✅ 第 %d 轮运动序列执行完成", cycle + 1);
        } else {
            RCLCPP_ERROR(logger, "❌ 第 %d 轮运动序列执行失败，终止循环", cycle + 1);
            break;  // 某一轮失败，可选择终止循环（也可注释掉继续执行下一轮）
        }
    }

    RCLCPP_INFO(logger, "\n🎉 所有 %d 轮运动序列执行完毕", total_cycles);

    // 回到home位姿
    RCLCPP_INFO(logger, "5秒后返回初始位姿...");
    rclcpp::sleep_for(std::chrono::seconds(5));
    move_group.setNamedTarget("home"); // 需确保MoveIt配置中有"home"位姿
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
        move_group.execute(my_plan);
        RCLCPP_INFO(logger, "已返回初始位姿");
    }

    // 关闭节点
    RCLCPP_INFO(logger, "末端位姿控制节点执行完成");
    rclcpp::shutdown();
    return 0;
}