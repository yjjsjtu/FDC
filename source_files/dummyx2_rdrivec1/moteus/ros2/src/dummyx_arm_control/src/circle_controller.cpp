#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>    // 用于sin/cos计算圆的点
#include <fstream>  // CSV文件写入
#include <iomanip>  // std::setprecision
#include <cstdlib>  // system()

int main(int argc, char *argv[])
{
    // 初始化ROS2节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("dummyx_arm_cartesian_circle_node");
    auto logger = node->get_logger();
    RCLCPP_INFO(logger, "DummyX Arm 笛卡尔画圆控制节点已启动");

    // 创建MoveGroupInterface实例
    moveit::planning_interface::MoveGroupInterface move_group(node, "dummyx2_arm");

    // 配置规划参数（支持 ros2 run 时通过 --ros-args -p 动态传入）
    // 用法示例：ros2 run dummyx_arm_control circle_controller --ros-args -p vel_scaling:=0.3 -p acc_scaling:=0.2
    move_group.setPlanningTime(30.0);
    node->declare_parameter("vel_scaling", 0.2);
    node->declare_parameter("acc_scaling", 0.2);
    node->declare_parameter("y_scale", 1.0);    // Y轴半径缩放比（>1=补偿Y方向跟踪不足）
    node->declare_parameter("z_tilt", 0.0);     // Z轴倾斜补偿（补偿底座/桌面倾斜导致的左高右低）
    // z_tilt 公式: z_cmd = center_z + z_tilt * x
    // 左侧高(x<0)右侧低(x>0) → z_tilt 为正值（约 0.025 对应 3mm/60mm半径）
    // 调参: 从 z_tilt:=0.025 开始，若过补偿则减小，若仍有偏差则增大
    const double vel_scaling = node->get_parameter("vel_scaling").as_double();
    const double acc_scaling = node->get_parameter("acc_scaling").as_double();
    // TOTG 内部当 acceleration_bounded_=false 时默认用 1 rad/s² 作为上限
    // （urdfdom 不解析 URDF <limit acceleration=...>，且 joint_limits.yaml 仅加载到 move_group 节点）
    // 补偿：将 acc_scaling 乘以 ROBOT_MAX_ACCEL/TOTG_DEFAULT，使 TOTG 实际按真实抠限运行
    // acc_scaling_totg 被局限于 1.0，给予 acc_scaling<=0.398 时完全准确，更高时取限至最大加速度
    constexpr double ROBOT_MAX_ACCEL = 2.513;   // rad/s² from joint_limits.yaml
    constexpr double TOTG_DEFAULT_ACCEL = 1.0;  // rad/s² TOTG 默认上限
    const double acc_scaling_totg = std::min(1.0, acc_scaling * (ROBOT_MAX_ACCEL / TOTG_DEFAULT_ACCEL));
    RCLCPP_INFO(logger, "规划参数: vel_scaling=%.2f, acc_scaling=%.2f (TOTG 补偿系数=%.3f, 实际加速度上限=%.3f rad/s²)",
                vel_scaling, acc_scaling, acc_scaling_totg, acc_scaling_totg * TOTG_DEFAULT_ACCEL);
    move_group.setMaxVelocityScalingFactor(vel_scaling);
    move_group.setMaxAccelerationScalingFactor(acc_scaling);

    // 设置末端执行器链接名称
    std::string ee_link_name = "link6_1";
    move_group.setEndEffectorLink(ee_link_name);

    // 定义目标位姿（位置+姿态）
    geometry_msgs::msg::Pose target_pose;

    // 设置目标位置（x/y/z，单位：米，示例值，需根据你的机械臂修改）
    target_pose.position.x = 0.0;  // 末端在x轴的位置
    target_pose.position.y = 0.22;  // 末端在y轴的位置
    target_pose.position.z = 0.15;  // 0.15m: J4/J5处于中位角，左右重力矩对称，Z方向最稳定

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
    move_group.setPoseTarget(target_pose); // 核心：设置末端目标位姿

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(logger, "末端位姿规划成功，开始执行轨迹");
        move_group.execute(my_plan); // 执行规划的轨迹
        // ✅ 修改1：移除 sleep_for(1s)
        // 原因：PD保持期间（KP=900）会产生低频振荡，应立即衔接下一段轨迹
    } else {
        RCLCPP_ERROR(logger, "末端位姿规划失败！请检查：");
        RCLCPP_ERROR(logger, "1. 末端链接名称是否正确");
        RCLCPP_ERROR(logger, "2. 目标位姿是否在机械臂工作空间内");
        RCLCPP_ERROR(logger, "3. 目标位姿是否存在碰撞/逆解无解");
    }

    // 获取当前末端位姿（作为画圆的基准位姿）
    geometry_msgs::msg::Pose start_pose = target_pose;
    RCLCPP_INFO(logger, "当前末端初始位姿（圆心）：x=%.3f, y=%.3f, z=%.3f",
                start_pose.position.x, start_pose.position.y, start_pose.position.z);

    // 定义圆的参数（可根据需求修改）
    double circle_radius = 0.06;   // 圆半径 0.06m（直径12cm）
    int num_waypoints = 36;        // 圆的控制路点数（36=每10度一个控制点）
                                   // eef_step=1mm 负责每段弧长的密集插值（~10个IK点/段）
    // 几何预补偿：若实际画出的是椭圆（Y轴压缩），可通过 y_scale > 1.0 放大Y轴半径
    // 使机械臂在Y方向走更远，让跟踪误差补偿后恰好形成正圆
    // 例如：y_scale=1.4 → Y轴指令半径=0.084m，若实际跟踪比X轴少30%，结果接近正圆
    // 通过 ROS param 动态调整（无需重新编译）：--ros-args -p y_scale:=1.3
    double y_scale = node->get_parameter("y_scale").as_double();
    double z_tilt  = node->get_parameter("z_tilt").as_double();   // 左高右低补偿
    double circle_radius_x = circle_radius;
    double circle_radius_y = circle_radius * y_scale;
    double center_x = start_pose.position.x;
    double center_y = start_pose.position.y;
    double center_z = start_pose.position.z;

    // ✅ 关键修复：消除圆终点的 90° 方向突变
    // 旧方案：圆心→[圆弧360°]→圆弧顶点→圆心
    //   问题：圆弧顶点处切线方向为+X，而返回圆心方向为-Y，形成90°折角
    //         TOTG在此处强制减速到v≈0，然后重新加速，物理上表现为跳变/顿挫
    // 新方案：圆心→(独立规划)→圆弧起始点→[闭合圆弧360°]→圆弧起始点
    //   优点：闭合圆弧起点=终点，速度约束为v=0（自然减速），无方向突变，无跳变

    // 圆弧起始点（angle=0：x=center_x, y=center_y+circle_radius_y）
    geometry_msgs::msg::Pose circle_start_pose = start_pose;
    circle_start_pose.position.x = center_x + circle_radius_x * sin(0.0);  // = center_x
    circle_start_pose.position.y = center_y + circle_radius_y * cos(0.0);  // = center_y + R_y
    // 起始点 x=center_x → z_tilt 修正量为 0（sin(0)=0）
    circle_start_pose.position.z = center_z + z_tilt * (circle_start_pose.position.x - center_x);

    // 步骤A：从圆心移动到圆弧起始点（独立规划，以v=0结束，无角点影响后续圆弧）
    RCLCPP_INFO(logger, "移动到圆弧起始点：(%.3f, %.3f, %.3f)",
                circle_start_pose.position.x, circle_start_pose.position.y, circle_start_pose.position.z);
    move_group.setPoseTarget(circle_start_pose);
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    bool approach_ok = (move_group.plan(approach_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (approach_ok) {
        move_group.execute(approach_plan);
        RCLCPP_INFO(logger, "已到达圆弧起始点，准备执行闭合圆弧");
    } else {
        RCLCPP_ERROR(logger, "移动到圆弧起始点失败！请检查目标位姿是否在工作空间内。");
        rclcpp::shutdown();
        return 1;
    }

    // 步骤B：生成闭合圆弧路点（从 circle_start_pose 出发，绕一圈回到 circle_start_pose）
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(circle_start_pose);  // 起点：圆上（angle=0）

    for (int i = 1; i <= num_waypoints; ++i)
    {
        double angle = 2 * M_PI * i / num_waypoints;
        double wx = center_x + circle_radius_x * sin(angle);
        double wy = center_y + circle_radius_y * cos(angle);
        // Z轴倒斜补偿：z_cmd = center_z + z_tilt * (x - center_x)
        // 左侧(x<0)高时用 z_tilt>0 让指令z更低，实际z抵消计差后恢复平坦
        double wz = center_z + z_tilt * (wx - center_x);

        geometry_msgs::msg::Pose waypoint_pose = start_pose;
        waypoint_pose.position.x = wx;
        waypoint_pose.position.y = wy;
        waypoint_pose.position.z = wz;
        waypoint_pose.orientation = start_pose.orientation;

        waypoints.push_back(waypoint_pose);
    }
    // 最后一个生成路点 (i=36, angle=2π) ≈ circle_start_pose → 轨迹自然闭合，无方向突变
    RCLCPP_INFO(logger, "圆弧参数: R_x=%.3fm, R_y=%.3fm (y_scale=%.2f), z_tilt=%.4f (Z±%.2fmm)",
                circle_radius_x, circle_radius_y, y_scale,
                z_tilt, fabs(z_tilt * circle_radius_x) * 1000.0);

    // 笛卡尔路径规划（核心函数）
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;    // 跳跃阈值（0=不允许关节跳跃）
    const double eef_step = 0.001;       // 末端执行器步长（米，1mm，更密集的插值点，减少加速度尖刺）
    
    // 计算笛卡尔路径，返回路径分数（0-1，1=所有路点都规划成功）
    double fraction = move_group.computeCartesianPath(
        waypoints,   // 路点列表
        eef_step,    // 末端步长
        jump_threshold, // 跳跃阈值
        trajectory   // 输出轨迹
    );

    // 检查规划结果并执行
    if (fraction > 0.9) // 路径分数>0.9视为规划成功（允许少量路点失败）
    {
        RCLCPP_INFO(logger, "笛卡尔画圆路径规划成功，路径分数=%.2f", fraction);

        // ---------------------------------------------------------------
        // 关键修复：computeCartesianPath 不自动应用 setMaxVelocityScalingFactor
        // 和 setMaxAccelerationScalingFactor，必须手动做时间参数化！
        // 用轨迹起始点构建参考状态，完全不依赖 getCurrentState()（避免 joint_states
        // 时间戳过期导致的 nullptr 崩溃）
        // ---------------------------------------------------------------
        {
            // 构建参考 RobotState：从 URDF 模型获取关节限制信息
            auto ref_state = std::make_shared<moveit::core::RobotState>(move_group.getRobotModel());
            ref_state->setToDefaultValues();
            // 用轨迹第一个路点设置关节位置（更精确）
            if (!trajectory.joint_trajectory.points.empty()) {
                const auto& pt    = trajectory.joint_trajectory.points.front();
                const auto& names = trajectory.joint_trajectory.joint_names;
                for (size_t j = 0; j < names.size(); ++j) {
                    ref_state->setJointPositions(names[j], &pt.positions[j]);
                }
            }
            ref_state->update();

            robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), "dummyx2_arm");
            rt.setRobotTrajectoryMsg(*ref_state, trajectory);

            // ✅ 修改3：TOTG resampled at 40 Hz (resample_dt=0.025s)
            // TOTG默认resample_dt=0.1s（10Hz），对13s轨迹只有~133个输出点
            // 降为0.025s后输出~530个点，加速度曲线分辨率提高4倍，尖刺更清晰可见
            // （真实执行时点越多越平滑，usb2can_node以更高频率执行插值）
            trajectory_processing::TimeOptimalTrajectoryGeneration totg(
                0.025,   // resample_dt: 40 Hz output (default=0.1s=10Hz)
                0.001    // min_angle_change: keep small for smooth arcs (default=0.001)
            );
            bool time_ok = totg.computeTimeStamps(rt, vel_scaling, acc_scaling_totg);

            if (time_ok) {
                rt.getRobotTrajectoryMsg(trajectory);
                // 打印轨迹总时长，验证 TOTG 是否真正缩放了速度
                const auto& pts = trajectory.joint_trajectory.points;
                const auto& joint_names = trajectory.joint_trajectory.joint_names;
                double traj_duration = 0.0;
                if (!pts.empty()) {
                    const auto& last = pts.back();
                    traj_duration = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9;
                }
                RCLCPP_INFO(logger, "时间参数化(TOTG)成功：vel=%.2f，acc_scale=%.3f，实际加速度=%.3f rad/s²，轨迹总时长=%.2f秒，路点数=%zu",
                            vel_scaling, acc_scaling_totg, acc_scaling_totg * TOTG_DEFAULT_ACCEL, traj_duration, pts.size());

                // ---------------------------------------------------------------
                // 导出轨迹数据到 CSV，用于后续绘图分析
                // 格式：time, j1_pos, j1_vel, j1_acc, j2_pos, j2_vel, j2_acc, ...
                // ---------------------------------------------------------------
                const std::string csv_path = "/home/liyq/moteus/tmp/circle_traj_data.csv";
                std::ofstream csv(csv_path);
                if (csv.is_open()) {
                    // 写表头
                    csv << "time";
                    for (const auto& jname : joint_names) {
                        csv << "," << jname << "_pos"
                            << "," << jname << "_vel"
                            << "," << jname << "_acc";
                    }
                    csv << "\n";
                    csv << std::fixed << std::setprecision(6);

                    for (const auto& pt : pts) {
                        double t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
                        csv << t;
                        for (size_t j = 0; j < joint_names.size(); ++j) {
                            double pos = (j < pt.positions.size())     ? pt.positions[j]     : 0.0;
                            double vel = (j < pt.velocities.size())    ? pt.velocities[j]    : 0.0;
                            double acc = (j < pt.accelerations.size()) ? pt.accelerations[j] : 0.0;
                            csv << "," << pos << "," << vel << "," << acc;
                        }
                        csv << "\n";
                    }
                    csv.close();
                    RCLCPP_INFO(logger, "轨迹数据已导出到：%s（%zu 行）", csv_path.c_str(), pts.size());

                    // 自动调用 Python 绘图脚本生成图表
                    // 优先使用 src 路径（开发环境，python 环境完整）
                    const std::string plot_out = "/home/liyq/moteus/tmp/circle_traj_plot.png";
                    const std::string src_script =
                        "/home/liyq/moteus/ros2/src/dummyx_arm_control/scripts/plot_circle_traj.py";
                    const std::string plot_cmd =
                        "python3 " + src_script + " " + csv_path + " " + plot_out + " &";
                    int ret = system(plot_cmd.c_str());
                    if (ret == 0) {
                        RCLCPP_INFO(logger, "绘图脚本已启动，图表将保存至：%s", plot_out.c_str());
                    } else {
                        // 回退：installed 路径
                        const std::string fallback =
                            "python3 $(ros2 pkg prefix dummyx_arm_control)/share/dummyx_arm_control/scripts/plot_circle_traj.py "
                            + csv_path + " " + plot_out + " &";
                        system(fallback.c_str());
                        RCLCPP_WARN(logger, "绘图脚本路径回退，请确认脚本已安装。图表：/home/liyq/moteus/tmp/circle_traj_plot.png");
                    }
                } else {
                    RCLCPP_WARN(logger, "无法写入CSV文件：%s", csv_path.c_str());
                }
            } else {
                RCLCPP_WARN(logger, "时间参数化失败，将以全速执行");
            }
        }
        // ---------------------------------------------------------------

        // 将规划好的轨迹封装为MoveIt Plan
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory_ = trajectory;
        // ✅ 修改4：简化执行逻辑，「返回圆心」已包含在轨迹末尾，无需单独执行
        // 轨迹路径：圆心 → 圆弧起点 → 完整圆弧 → 圆心（一条连续轨迹，无中间停顿）
        RCLCPP_INFO(logger, "执行闭合圆弧（起点=终点，无方向折角）...");
        move_group.execute(my_plan);
        RCLCPP_INFO(logger, "闭合圆弧执行完成，机械臂已平滑减速至圆弧起始点（v=0，无跳变）");
    }
    else
    {
        RCLCPP_ERROR(logger, "笛卡尔画圆路径规划失败，路径分数=%.2f！", fraction);
        RCLCPP_ERROR(logger, "可能原因：1. 圆超出工作空间 2. 步长/半径过大 3. 逆解无解");
    }

    // 可选：回到初始位姿
    RCLCPP_INFO(logger, "5秒后返回初始位姿...");
    rclcpp::sleep_for(std::chrono::seconds(5));
    move_group.setNamedTarget("home");
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) move_group.execute(my_plan);

    // 10. 关闭节点
    RCLCPP_INFO(logger, "笛卡尔画圆控制节点执行完成");
    rclcpp::shutdown();
    return 0;
}