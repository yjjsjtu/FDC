/**
 * chicken_head_controller.cpp - TCP 效果演示
 *
 * TCP (link6_1前方1cm) 保持位置不变，J2上下摆动（大臂），
 * J3/J4/J5通过3D数值IK补偿TCP位置。
 *
 * ★ 关键:
 *   1. 所有 update() 必须使用 update(true) 强制FK刷新
 *   2. 仅约束位置(3D), 不约束姿态 → 3关节恰好可解
 *   3. 使用MoveIt enforcePositionBounds 防止超出关节限位
 */
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>

static constexpr double ROBOT_MAX_ACCEL = 2.513;
static constexpr double TOTG_DEFAULT_ACCEL = 0.25;
static constexpr double TCP_OFFSET = 0.009;  // link6_1前方1cm

// ─── TCP位置 ──────────────────────────────────────────────────────
static Eigen::Vector3d getTcpPos(moveit::core::RobotState& st)
{
    st.update(true);
    const auto& T = st.getGlobalLinkTransform("link6_1");
    // TCP = link6_1 + 1cm沿local Y轴 (Joint6轴=Y)
    return T.translation() + T.rotation() * Eigen::Vector3d(0, TCP_OFFSET, 0);
}

// ─── 3D位置IK求解器 (DLS) ──────────────────────────────────────────
// 补偿关节: J3, J4, J5  →  3 DOF 对 3D 位置约束 (恰好可解)
static bool solvePositionIK(
    moveit::core::RobotState& st,
    const Eigen::Vector3d& target_pos,
    const std::vector<std::string>& comp_joints,
    const moveit::core::JointModelGroup* jmg,
    double tol = 0.002,       // 2mm
    int max_iter = 300,
    rclcpp::Logger* dbg = nullptr,
    int dbg_id = -1)          // waypoint ID for debug
{
    const int N = comp_joints.size();
    const double delta = 0.0005;
    bool do_log = (dbg != nullptr && dbg_id >= 0);

    for (int iter = 0; iter < max_iter; ++iter) {
        Eigen::Vector3d cur = getTcpPos(st);
        Eigen::Vector3d err = target_pos - cur;
        double e = err.norm();

        if (do_log && (iter == 0 || iter == 5 || iter == 50 || iter == max_iter-1)) {
            RCLCPP_WARN(*dbg,
                "  IK[wp%d] iter=%d err=%.2fmm (%.4f,%.4f,%.4f)",
                dbg_id, iter, e*1000, err.x(), err.y(), err.z());
        }

        if (e < tol) return true;

        // 构建 3×N 位置Jacobian
        Eigen::MatrixXd J(3, N);
        for (int j = 0; j < N; ++j) {
            double q0 = st.getVariablePosition(comp_joints[j]);
            st.setVariablePosition(comp_joints[j], q0 + delta);
            Eigen::Vector3d pert = getTcpPos(st);
            J.col(j) = (pert - cur) / delta;
            st.setVariablePosition(comp_joints[j], q0);
        }

        // DLS: dq = J^T (J J^T + λ²I)^{-1} err
        double lambda = 0.005 + 0.02 * e;
        Eigen::Matrix3d JJt = J * J.transpose()
                            + lambda * lambda * Eigen::Matrix3d::Identity();
        Eigen::VectorXd dq = J.transpose() * JJt.ldlt().solve(err);

        // 限制步长
        if (dq.norm() > 0.15)
            dq *= 0.15 / dq.norm();

        for (int j = 0; j < N; ++j)
            st.setVariablePosition(comp_joints[j],
                st.getVariablePosition(comp_joints[j]) + dq(j));

        // 关节限位裁剪
        st.enforceBounds(jmg);
    }

    // 最终检查
    double final_e = (target_pos - getTcpPos(st)).norm();
    if (do_log) {
        RCLCPP_ERROR(*dbg, "  IK[wp%d] 最终失败: err=%.2fmm", dbg_id, final_e*1000);
    }
    return final_e < tol * 2.0;
}

// ─── 正弦摆动序列 ───────────────────────────────────────────────
// 生成连续多周期正弦序列, 含端点自然闭合
static std::vector<double> makeSinSwing(
    double center, double amp, int pts_per_cycle, int cycles = 1, double phase = 0.0)
{
    const int total = pts_per_cycle * cycles;
    std::vector<double> s;
    s.reserve(total + 1);
    for (int i = 0; i <= total; ++i) {
        double t = 2.0 * M_PI * i / pts_per_cycle;
        s.push_back(center + amp * std::sin(t + phase));
    }
    return s;
}

// ══════════════════════════════════════════════════════════════════
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("dummyx_chicken_head_node");
    auto logger = node->get_logger();
    RCLCPP_INFO(logger, "====== TCP 效果控制节点已启动 ======");

    node->declare_parameter("swing_amplitude", 0.4);     // J2: ≈23°
    node->declare_parameter("j1_amplitude", 0.3);         // J1: ≈17°
    node->declare_parameter("num_waypoints", 80);
    node->declare_parameter("vel_scaling", 0.3);
    node->declare_parameter("acc_scaling", 0.2);
    node->declare_parameter("num_cycles", 3);

    const double swing_amp = node->get_parameter("swing_amplitude").as_double();
    const double j1_amp = node->get_parameter("j1_amplitude").as_double();
    const int num_wp = node->get_parameter("num_waypoints").as_int();
    const double vel_sc = node->get_parameter("vel_scaling").as_double();
    const double acc_sc = node->get_parameter("acc_scaling").as_double();
    const int num_cycles = node->get_parameter("num_cycles").as_int();
    const double acc_totg = std::min(1.0, acc_sc * (ROBOT_MAX_ACCEL / TOTG_DEFAULT_ACCEL));

    RCLCPP_INFO(logger, "参数: J2_amp=%.2f(%.1f°), J1_amp=%.2f(%.1f°), wp=%d, vel=%.2f, acc=%.2f, cycles=%d",
        swing_amp, swing_amp*180/M_PI, j1_amp, j1_amp*180/M_PI,
        num_wp, vel_sc, acc_sc, num_cycles);

    // kinematics params
    node->declare_parameter("robot_description_kinematics.dummyx2_arm.kinematics_solver",
        std::string("kdl_kinematics_plugin/KDLKinematicsPlugin"));
    node->declare_parameter("robot_description_kinematics.dummyx2_arm.kinematics_solver_search_resolution", 0.5);
    node->declare_parameter("robot_description_kinematics.dummyx2_arm.kinematics_solver_timeout", 0.05);

    moveit::planning_interface::MoveGroupInterface mg(node, "dummyx2_arm");
    mg.setPlanningTime(30.0);
    mg.setMaxVelocityScalingFactor(vel_sc);
    mg.setMaxAccelerationScalingFactor(acc_sc);
    mg.setEndEffectorLink("link6_1");

    const auto model = mg.getRobotModel();
    const auto jmg = model->getJointModelGroup("dummyx2_arm");

    // ─── 移动到前伸下垂位姿 ──────────────────────────────────────
    // TCP位置稍高、稍近 → 关节更居中，补偿余量更大
    RCLCPP_INFO(logger, "移动到前伸下垂位姿...");
    geometry_msgs::msg::Pose init_pose;
    init_pose.position.x = 0.0;
    init_pose.position.y = 0.18;   // 比0.22近，肘部更弯
    init_pose.position.z = 0.18;   // 比0.15高，减少J5极限
    tf2::Quaternion q;
    q.setRPY(-1.5707, 0.0, 0.0);
    q.normalize();
    init_pose.orientation = tf2::toMsg(q);

    mg.setPoseTarget(init_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "无法规划到初始位姿！");
        rclcpp::shutdown(); return 1;
    }
    mg.execute(plan);
    RCLCPP_INFO(logger, "已到达初始位姿");

    // ─── 从轨迹末端路点提取初始关节角度 ──────────────────────────
    auto init_state = std::make_shared<moveit::core::RobotState>(model);
    init_state->setToDefaultValues();
    {
        const auto& tj = plan.trajectory_.joint_trajectory;
        const auto& lp = tj.points.back();
        for (size_t i = 0; i < tj.joint_names.size(); ++i)
            init_state->setVariablePosition(tj.joint_names[i], lp.positions[i]);
    }
    init_state->update(true);

    // 记录初始关节角度
    std::vector<double> init_jvals;
    init_state->copyJointGroupPositions(jmg, init_jvals);
    RCLCPP_INFO(logger, "初始关节角: J1=%.1f° J2=%.1f° J3=%.1f° J4=%.1f° J5=%.1f° J6=%.1f°",
        init_jvals[0]*180/M_PI, init_jvals[1]*180/M_PI, init_jvals[2]*180/M_PI,
        init_jvals[3]*180/M_PI, init_jvals[4]*180/M_PI, init_jvals[5]*180/M_PI);

    // 打印关节限位
    for (int ji = 0; ji < 6; ++ji) {
        std::string jn = "Joint" + std::to_string(ji+1);
        const auto* jm = model->getJointModel(jn);
        auto bounds = jm->getVariableBounds()[0];
        RCLCPP_INFO(logger, "  %s 限位: [%.1f°, %.1f°] 当前=%.1f°",
            jn.c_str(), bounds.min_position_*180/M_PI, bounds.max_position_*180/M_PI,
            init_jvals[ji]*180/M_PI);
    }

    // ─── 记录TCP固定目标位置 ─────────────────────────────────────
    Eigen::Vector3d tcp_target = getTcpPos(*init_state);
    RCLCPP_INFO(logger, "TCP固定位置: (%.4f, %.4f, %.4f)",
        tcp_target.x(), tcp_target.y(), tcp_target.z());

    // ─── FK健全性测试 ─────────────────────────────────────────────
    {
        auto test = std::make_shared<moveit::core::RobotState>(*init_state);
        Eigen::Vector3d p0 = getTcpPos(*test);

        for (int ji = 2; ji <= 4; ++ji) {
            std::string jn = "Joint" + std::to_string(ji+1);
            double q_orig = test->getVariablePosition(jn);
            test->setVariablePosition(jn, q_orig + 0.1);
            Eigen::Vector3d dp = getTcpPos(*test) - p0;
            test->setVariablePosition(jn, q_orig);
            RCLCPP_INFO(logger, "FK测试: %s+0.1rad → TCP移动(%.4f,%.4f,%.4f) %.1fmm",
                jn.c_str(), dp.x(), dp.y(), dp.z(), dp.norm()*1000);
        }

        // 测试J2对TCP的影响
        double j2_orig = test->getVariablePosition("Joint2");
        test->setVariablePosition("Joint2", j2_orig + 0.1);
        Eigen::Vector3d dp2 = getTcpPos(*test) - p0;
        test->setVariablePosition("Joint2", j2_orig);
        RCLCPP_INFO(logger, "FK测试: J2+0.1rad → TCP移动(%.4f,%.4f,%.4f) %.1fmm",
            dp2.x(), dp2.y(), dp2.z(), dp2.norm()*1000);
    }

    // ─── 设置补偿参数 ─────────────────────────────────────────────
    const double j1_center = init_jvals[0];
    const double j2_center = init_jvals[1];
    const double j6_fix = init_jvals[5];
    // 补偿关节: J3/J4/J5/J6 (4 DOF对3D位置, DLS取最小范数解)
    const std::vector<std::string> comp_joints = {"Joint3", "Joint4", "Joint5", "Joint6"};

    RCLCPP_INFO(logger, "开始TCP 效果: J1摆动%.1f°±%.1f°, J2摆动%.1f°±%.1f°, J3-J6补偿",
        j1_center*180/M_PI, j1_amp*180/M_PI,
        j2_center*180/M_PI, swing_amp*180/M_PI);

    // 调试标志: 只对前3个失败打印详细日志
    int dbg_fail_cnt = 0;

    // ═════════════════════════════════════════════════════════════════
    // 构建一条连续轨迹: 阶段1(J1+J2) + 阶段2(J2-only), 一次执行
    // ═════════════════════════════════════════════════════════════════
    robot_trajectory::RobotTrajectory rt(model, "dummyx2_arm");
    auto last_good = std::make_shared<moveit::core::RobotState>(*init_state);
    int total_ok = 0, total_fail = 0;

    // ─── 阶段1: J1+J2 画圆(偏置) ──────────────────────────────
    {
        int circle_cycles = 5;
        RCLCPP_INFO(logger, "═══ 阶段1: J1+J2 画圆运动 ×%d次 ═══", circle_cycles);
        int ok_cnt = 0, fail_cnt = 0;
        const int total_pts = num_wp * circle_cycles;

        for (int i = 0; i <= total_pts; ++i) {
            double t = 2.0 * M_PI * i / num_wp;
            // 正常的圆是 sin(t) 和 cos(t)。
            // 为了让它无缝地从上一阶段的中心点(j1_center, j2_center)平滑起步，绝对不能有跳变。
            // 我们使用 1-cos(t)，这样当 t=0 和 t=2PI 时，偏移量恰好为 0。
            double j1_t = j1_center + j1_amp * std::sin(t);
            // 将J2振幅适当缩小，避免单侧偏置后超过安全范围
            double j2_t = j2_center + (swing_amp * 0.75) * (1.0 - std::cos(t));

            auto state = std::make_shared<moveit::core::RobotState>(*last_good);
            state->setVariablePosition("Joint1", j1_t);
            state->setVariablePosition("Joint2", j2_t);
            state->update(true);

            bool solved = solvePositionIK(
                *state, tcp_target, comp_joints, jmg,
                0.002, 300,
                (dbg_fail_cnt < 3) ? &logger : nullptr,
                (int)i);

            if (solved) {
                state->setVariablePosition("Joint1", j1_t);
                state->setVariablePosition("Joint2", j2_t);
                state->enforceBounds(jmg);
                state->update(true);
                rt.addSuffixWayPoint(*state, 0.0);
                *last_good = *state;
                ++ok_cnt;

                size_t sample_interval = total_pts / (5 * circle_cycles) + 1;
                if (i % sample_interval == 0) {
                    std::vector<double> jv;
                    state->copyJointGroupPositions(jmg, jv);
                    double pe = (tcp_target - getTcpPos(*state)).norm();
                    RCLCPP_INFO(logger,
                        "  [%d] J1=%.1f° J2=%.1f° J3=%.1f° J4=%.1f° J5=%.1f° | err=%.2fmm",
                        i, jv[0]*180/M_PI, jv[1]*180/M_PI, jv[2]*180/M_PI,
                        jv[3]*180/M_PI, jv[4]*180/M_PI, pe*1000);
                }
            } else {
                dbg_fail_cnt++;
                RCLCPP_WARN(logger, "圆轨迹点%d IK失败", i);
                ++fail_cnt;
            }
        }
        RCLCPP_INFO(logger, "阶段1(画圆) IK完成: 成功=%d", ok_cnt);
        total_ok += ok_cnt;
        total_fail += fail_cnt;
    }

    // ─── 阶段2: J1+J2 反相运动 ──────────────────────────────────
    {
        RCLCPP_INFO(logger, "═══ 阶段2: J1+J2 反相运动 ×%d次 ═══", 1);
        auto j1_seq = makeSinSwing(j1_center, j1_amp, num_wp, 1, 0.0);
        auto j2_seq = makeSinSwing(j2_center, swing_amp, num_wp, 1, M_PI);
        int ok_cnt = 0, fail_cnt = 0;

        for (size_t i = 0; i < j1_seq.size(); ++i) {
            auto state = std::make_shared<moveit::core::RobotState>(*last_good);
            double t = 2.0 * M_PI * i / num_wp;
            // 注入零边界正交偏移(椭圆呼吸), 保证初末物理状态严格为0不跳变，同时峰值速度不为零从而消灭点头！
            double j2_val = j2_seq[i] + (swing_amp * 0.05) * (1.0 - std::cos(t));
            
            state->setVariablePosition("Joint1", j1_seq[i]);
            state->setVariablePosition("Joint2", j2_val);
            state->update(true);

            bool solved = solvePositionIK(
                *state, tcp_target, comp_joints, jmg,
                0.002, 300,
                (dbg_fail_cnt < 3) ? &logger : nullptr,
                (int)i);

            if (solved) {
                state->setVariablePosition("Joint1", j1_seq[i]);
                state->setVariablePosition("Joint2", j2_val);
                state->enforceBounds(jmg);
                state->update(true);
                rt.addSuffixWayPoint(*state, 0.0);
                *last_good = *state;
                ++ok_cnt;

                size_t sample_interval = j2_seq.size() / 5 + 1;
                if (i % sample_interval == 0) {
                    std::vector<double> jv;
                    state->copyJointGroupPositions(jmg, jv);
                    double pe = (tcp_target - getTcpPos(*state)).norm();
                    RCLCPP_INFO(logger,
                        "  [%zu] J1=%.1f° J2=%.1f° J3=%.1f° J4=%.1f° J5=%.1f° | err=%.2fmm",
                        i, jv[0]*180/M_PI, jv[1]*180/M_PI, jv[2]*180/M_PI,
                        jv[3]*180/M_PI, jv[4]*180/M_PI, pe*1000);
                }
            } else {
                dbg_fail_cnt++;
                RCLCPP_WARN(logger, "路点%zu IK失败", i);
                ++fail_cnt;
            }
        }
        RCLCPP_INFO(logger, "阶段2(反相) IK完成: 成功=%d", ok_cnt);
        total_ok += ok_cnt;
        total_fail += fail_cnt;
    }

    // ─── 阶段2.2: J1+J2 同相运动 ────────────────────────────────
    {
        RCLCPP_INFO(logger, "═══ 阶段2.2: J1+J2 同相运动 ×%d次 ═══", 1);
        auto j1_seq = makeSinSwing(j1_center, j1_amp, num_wp, 1, 0.0);
        auto j2_seq = makeSinSwing(j2_center, swing_amp, num_wp, 1, 0.0);
        int ok_cnt = 0, fail_cnt = 0;

        for (size_t i = 0; i < j1_seq.size(); ++i) {
            auto state = std::make_shared<moveit::core::RobotState>(*last_good);
            double t = 2.0 * M_PI * i / num_wp;
            double j2_val = j2_seq[i] + (swing_amp * 0.05) * (1.0 - std::cos(t));
            
            state->setVariablePosition("Joint1", j1_seq[i]);
            state->setVariablePosition("Joint2", j2_val);
            state->update(true);

            bool solved = solvePositionIK(
                *state, tcp_target, comp_joints, jmg,
                0.002, 300,
                (dbg_fail_cnt < 3) ? &logger : nullptr,
                (int)i);

            if (solved) {
                state->setVariablePosition("Joint1", j1_seq[i]);
                state->setVariablePosition("Joint2", j2_val);
                state->enforceBounds(jmg);
                state->update(true);
                rt.addSuffixWayPoint(*state, 0.0);
                *last_good = *state;
                ++ok_cnt;

                size_t sample_interval = j2_seq.size() / 5 + 1;
                if (i % sample_interval == 0) {
                    std::vector<double> jv;
                    state->copyJointGroupPositions(jmg, jv);
                    double pe = (tcp_target - getTcpPos(*state)).norm();
                    RCLCPP_INFO(logger,
                        "  [%zu] J1=%.1f° J2=%.1f° J3=%.1f° J4=%.1f° J5=%.1f° | err=%.2fmm",
                        i, jv[0]*180/M_PI, jv[1]*180/M_PI, jv[2]*180/M_PI,
                        jv[3]*180/M_PI, jv[4]*180/M_PI, pe*1000);
                }
            } else {
                dbg_fail_cnt++;
                RCLCPP_WARN(logger, "路点%zu IK失败", i);
                ++fail_cnt;
            }
        }
        RCLCPP_INFO(logger, "阶段2.2(同相) IK完成: 成功=%d", ok_cnt);
        total_ok += ok_cnt;
        total_fail += fail_cnt;
    }

    // ─── 阶段3: 仅J2摆动, J1固定 ───────────────────────────────
    {
        RCLCPP_INFO(logger, "═══ 阶段3: 仅J2摆动, J1固定 ×%d次 ═══", 1);
        const std::vector<std::string> comp_joints_j2only = {"Joint3", "Joint4", "Joint5"};
        auto j2_seq = makeSinSwing(j2_center, swing_amp, num_wp, 1, 0.0);
        int ok_cnt = 0, fail_cnt = 0;

        for (size_t i = 0; i < j2_seq.size(); ++i) {
            auto state = std::make_shared<moveit::core::RobotState>(*last_good);
            double t = 2.0 * M_PI * i / num_wp;
            // 引入对偶微弱正交呼吸运动，并保持端点绝对归零
            double j1_val = j1_center + (j1_amp * 0.05) * (1.0 - std::cos(t));
            
            state->setVariablePosition("Joint1", j1_val);
            state->setVariablePosition("Joint2", j2_seq[i]);
            state->update(true);

            bool solved = solvePositionIK(
                *state, tcp_target, comp_joints_j2only, jmg,
                0.002, 300, nullptr, -1);

            if (solved) {
                state->setVariablePosition("Joint1", j1_val);
                state->enforceBounds(jmg);
                state->update(true);
                rt.addSuffixWayPoint(*state, 0.0);
                *last_good = *state;
                ++ok_cnt;

                size_t sample_interval = j2_seq.size() / 5 + 1;
                if (i % sample_interval == 0) {
                    std::vector<double> jv;
                    state->copyJointGroupPositions(jmg, jv);
                    double pe = (tcp_target - getTcpPos(*state)).norm();
                    RCLCPP_INFO(logger,
                        "  [%zu] J2=%.1f° J3=%.1f° J4=%.1f° J5=%.1f° | err=%.2fmm",
                        i, jv[1]*180/M_PI, jv[2]*180/M_PI,
                        jv[3]*180/M_PI, jv[4]*180/M_PI, pe*1000);
                }
            } else {
                RCLCPP_WARN(logger, "路点%zu IK失败", i);
                ++fail_cnt;
            }
        }
        RCLCPP_INFO(logger, "阶段3(J2摆动) IK完成: 成功=%d", ok_cnt);
        total_ok += ok_cnt;
        total_fail += fail_cnt;
    }

    // ─── 阶段4: 仅J1摆动, J2固定 ───────────────────────────────
    {
        RCLCPP_INFO(logger, "═══ 阶段4: 仅J1摆动, J2固定 ×%d次 ═══", 1);
        auto j1_seq = makeSinSwing(j1_center, j1_amp, num_wp, 1, 0.0);
        int ok_cnt = 0, fail_cnt = 0;

        for (size_t i = 0; i < j1_seq.size(); ++i) {
            auto state = std::make_shared<moveit::core::RobotState>(*last_good);
            double t = 2.0 * M_PI * i / num_wp;
            // 为固定的J2注入边界对齐的法向呼吸运动
            double j2_val = j2_center + (swing_amp * 0.05) * (1.0 - std::cos(t));
            
            state->setVariablePosition("Joint1", j1_seq[i]);
            state->setVariablePosition("Joint2", j2_val);
            state->update(true);

            // 让 J3, J4, J5 进行补偿
            bool solved = solvePositionIK(
                *state, tcp_target, comp_joints, jmg,
                0.002, 300, nullptr, -1);

            if (solved) {
                state->setVariablePosition("Joint1", j1_seq[i]);
                state->setVariablePosition("Joint2", j2_val);
                state->enforceBounds(jmg);
                state->update(true);
                rt.addSuffixWayPoint(*state, 0.0);
                *last_good = *state;
                ++ok_cnt;

                size_t sample_interval = j1_seq.size() / 5 + 1;
                if (i % sample_interval == 0) {
                    std::vector<double> jv;
                    state->copyJointGroupPositions(jmg, jv);
                    double pe = (tcp_target - getTcpPos(*state)).norm();
                    RCLCPP_INFO(logger,
                        "  [%zu] J1=%.1f° J3=%.1f° J4=%.1f° J5=%.1f° | err=%.2fmm",
                        i, jv[0]*180/M_PI, jv[2]*180/M_PI,
                        jv[3]*180/M_PI, jv[4]*180/M_PI, pe*1000);
                }
            } else {
                RCLCPP_WARN(logger, "路点%zu IK失败", i);
                ++fail_cnt;
            }
        }
        RCLCPP_INFO(logger, "阶段4(J1摆动) IK完成: 成功=%d", ok_cnt);
        total_ok += ok_cnt;
        total_fail += fail_cnt;
    }


    RCLCPP_INFO(logger, "总IK: 成功=%d 失败=%d (%.0f%%)",
        total_ok, total_fail, 100.0*total_ok/(total_ok+total_fail+1e-9));

    // ─── 一次性执行整条轨迹 ─────────────────────────────────────
    if (rt.getWayPointCount() >= 3) {
        trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.025, 0.001);
        totg.computeTimeStamps(rt, vel_sc, acc_totg);
        double dur = rt.getWayPointDurationFromStart(rt.getWayPointCount()-1);
        RCLCPP_INFO(logger, "TOTG成功: %zu点, %.2f秒", rt.getWayPointCount(), dur);

        moveit_msgs::msg::RobotTrajectory tmsg;
        rt.getRobotTrajectoryMsg(tmsg);
        
        moveit::planning_interface::MoveGroupInterface::Plan ep;
        ep.trajectory_ = tmsg;

        RCLCPP_INFO(logger, "执行完整演示...");
        auto res = mg.execute(ep);
        RCLCPP_INFO(logger, "演示 %s",
            res == moveit::core::MoveItErrorCode::SUCCESS ? "完成✓" : "异常");
    }

    // 返回home
    RCLCPP_INFO(logger, "3秒后返回home...");
    rclcpp::sleep_for(std::chrono::seconds(3));
    mg.setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan rp;
    if (mg.plan(rp) == moveit::core::MoveItErrorCode::SUCCESS)
        mg.execute(rp);

    RCLCPP_INFO(logger, "====== TCP 效果控制节点执行完毕 ======");
    rclcpp::shutdown();
    return 0;
}
