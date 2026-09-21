/**
 * joy_stick_controller.cpp - Xbox手柄丝滑控制机械臂末端(link6_1)
 *
 * ★ 核心: 完全绕过MoveIt规划, Jacobian IK (~0.1ms) + 直接发布轨迹
 *
 * 架构:
 *   [joy_node] → /joy → [本节点] → /dummyx2_arm_controller/joint_trajectory
 *                         ↑ /joint_states (反馈)
 *
 * Xbox映射 (末端笛卡尔空间控制):
 *   左摇杆 X/Y → 左右/前后平移    右摇杆 Y → 上下平移
 *   右摇杆 X → Yaw旋转            LT/RT → Roll旋转
 *   LB/RB → Pitch旋转             十字键 → 微调
 *   A=切换模式  B=急停回Home  X=回初始位姿  Start=退出
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

// ═══════════════════════ 常量 ═══════════════════════════════════════
static constexpr double DEADZONE         = 0.10;
static constexpr double DEF_LIN_STEP     = 0.001;  // m/步 (1mm)
static constexpr double DEF_ANG_STEP     = 0.016;  // rad/步 (~0.9°)
static constexpr double DEF_SMOOTH_ALPHA = 0.35;
static constexpr double DEF_CTRL_RATE    = 50.0;   // Hz
static constexpr double DLS_LAMBDA       = 0.01;   // DLS阻尼因子
static constexpr double DEF_MAX_JNT_VEL  = 0.4;    // rad/s 关节最大速度

// 工作空间限制 (笛卡尔, 米)
static constexpr double WS_X_MIN = -0.25, WS_X_MAX = 0.25;
static constexpr double WS_Y_MIN =  0.05, WS_Y_MAX = 0.35;
static constexpr double WS_Z_MIN =  0.01, WS_Z_MAX = 0.40;
static constexpr double WS_R_MIN =  0.08, WS_R_MAX = 0.35;

static const std::vector<std::string> JOINT_NAMES =
    {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};

// ═══════════════════════ 工具函数 ═══════════════════════════════════
static double applyDeadzone(double v, double dz) {
    if (std::fabs(v) < dz) return 0.0;
    double s = (v > 0) ? 1.0 : -1.0;
    return s * (std::fabs(v) - dz) / (1.0 - dz);
}

static bool clampWorkspace(Eigen::Vector3d& p) {
    bool c = false;
    auto cl = [&](double& v, double lo, double hi) {
        if (v < lo) { v = lo; c = true; } if (v > hi) { v = hi; c = true; }
    };
    cl(p.x(), WS_X_MIN, WS_X_MAX);
    cl(p.y(), WS_Y_MIN, WS_Y_MAX);
    cl(p.z(), WS_Z_MIN, WS_Z_MAX);
    double r = p.norm();
    if (r > 1e-6 && r < WS_R_MIN) { p *= WS_R_MIN / r; c = true; }
    if (r > WS_R_MAX) { p *= WS_R_MAX / r; c = true; }
    return c;
}

// ═══════════════════════ 控制器 ═══════════════════════════════════
class JoyStickController : public rclcpp::Node {
public:
    JoyStickController() : Node("dummyx_joy_stick_controller"),
        mode_trans_(true), has_joy_(false), has_joints_(false),
        jog_active_(true), was_moving_(false), speed_idx_(2)
    {
        declare_parameter("linear_step",   DEF_LIN_STEP);
        declare_parameter("angular_step",  DEF_ANG_STEP);
        declare_parameter("max_joint_vel", DEF_MAX_JNT_VEL);
        declare_parameter("smooth_alpha",  DEF_SMOOTH_ALPHA);
        declare_parameter("control_rate",  DEF_CTRL_RATE);
        declare_parameter("vel_scaling",   0.5);
        declare_parameter("acc_scaling",   0.5);
        lin_step_   = get_parameter("linear_step").as_double();
        ang_step_   = get_parameter("angular_step").as_double();
        max_jnt_vel_= get_parameter("max_joint_vel").as_double();
        alpha_      = get_parameter("smooth_alpha").as_double();
        ctrl_rate_  = get_parameter("control_rate").as_double();
        vel_sc_     = get_parameter("vel_scaling").as_double();
        acc_sc_     = get_parameter("acc_scaling").as_double();
        lin_speed_  = lin_step_ * ctrl_rate_;
        ang_speed_  = ang_step_ * ctrl_rate_;
        max_jnt_step_ = max_jnt_vel_ / ctrl_rate_;

        declare_parameter("robot_description_kinematics.dummyx2_arm.kinematics_solver",
            std::string("kdl_kinematics_plugin/KDLKinematicsPlugin"));
        declare_parameter("robot_description_kinematics.dummyx2_arm.kinematics_solver_search_resolution", 0.5);
        declare_parameter("robot_description_kinematics.dummyx2_arm.kinematics_solver_timeout", 0.05);

        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Joy::SharedPtr m) {
                std::lock_guard<std::mutex> lk(joy_mu_);
                joy_ = *m; has_joy_ = true;
            });
        js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::JointState::SharedPtr m) {
                std::lock_guard<std::mutex> lk(js_mu_);
                for (size_t i = 0; i < m->name.size(); ++i)
                    for (size_t j = 0; j < 6; ++j)
                        if (m->name[i] == JOINT_NAMES[j] && i < m->position.size())
                            cur_q_[j] = m->position[i];
                has_joints_ = true;
            });

        traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/dummyx2_arm_controller/joint_trajectory", 1);

        sv_ = Eigen::VectorXd::Zero(6);
        std::fill(cur_q_.begin(), cur_q_.end(), 0.0);

        RCLCPP_INFO(get_logger(),
            "参数: lin_step=%.1fmm ang_step=%.1f° rate=%.0fHz max_jnt=%.2frad/s",
            lin_step_*1000, ang_step_*180/M_PI, ctrl_rate_, max_jnt_vel_);
        RCLCPP_INFO(get_logger(),
            "  → speed=%.3fm/s ang=%.2frad/s alpha=%.2f",
            lin_speed_, ang_speed_, alpha_);
    }

    void run()
    {
        auto log = get_logger();

        // ─── MoveIt初始化 + 移动到初始位姿 ─────────────────────────
        RCLCPP_INFO(log, "初始化MoveIt...");
        auto mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "dummyx2_arm");
        mg->setPlanningTime(5.0);
        mg->setMaxVelocityScalingFactor(vel_sc_);
        mg->setMaxAccelerationScalingFactor(acc_sc_);
        mg->setEndEffectorLink("link6_1");

        // 初始位姿: 末端垂直朝下
        geometry_msgs::msg::Pose init_pose;
        init_pose.position.x = 0.0;
        init_pose.position.y = 0.22;
        init_pose.position.z = 0.15;
        tf2::Quaternion q; q.setRPY(-M_PI_2, 0, 0); q.normalize();
        init_pose.orientation = tf2::toMsg(q);

        RCLCPP_INFO(log, "移动到初始位姿 (末端垂直)...");
        mg->setPoseTarget(init_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (mg->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(log, "规划失败!"); return;
        }
        mg->execute(plan);
        RCLCPP_INFO(log, "✓ 已到达初始位姿");

        auto model = mg->getRobotModel();
        auto jmg   = model->getJointModelGroup("dummyx2_arm");
        auto* link  = model->getLinkModel("link6_1");

        while (rclcpp::ok() && !has_joints_) {
            rclcpp::spin_some(shared_from_this());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // ─── 后台spin线程 ──────────────────────────────────────────
        std::atomic<bool> spin_ok{true};
        std::thread spin_th([this, &spin_ok]() {
            while (spin_ok && rclcpp::ok()) {
                rclcpp::spin_some(shared_from_this());
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        });

        RCLCPP_INFO(log, "════════════════════════════════════════════");
        RCLCPP_INFO(log, "  Xbox手柄 末端笛卡尔控制 已就绪!");
        RCLCPP_INFO(log, "  左摇杆=XY平移  右摇杆Y=上下 X=Yaw");
        RCLCPP_INFO(log, "  LT/RT=Roll  LB/RB=Pitch  十字键=微调");
        RCLCPP_INFO(log, "  A=切换模式 Y=加速 Back=减速");
        RCLCPP_INFO(log, "  B=急停 X=回起点 Start=退出");
        RCLCPP_INFO(log, "  速度档位: 25%% 50%% [100%%] 150%% 200%% 300%%");
        RCLCPP_INFO(log, "════════════════════════════════════════════");

        // ─── 主控制循环 ────────────────────────────────────────────
        auto t_prev = std::chrono::steady_clock::now();
        bool prev_a = false, prev_y = false, prev_back = false;

        while (rclcpp::ok()) {
            auto t_now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(t_now - t_prev).count();
            t_prev = t_now;
            dt = std::clamp(dt, 0.001, 0.1);

            if (!has_joy_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            sensor_msgs::msg::Joy joy;
            { std::lock_guard<std::mutex> lk(joy_mu_); joy = joy_; }

            // ─── 按键处理 ─────────────────────────────────────────
            if (joy.buttons.size() > 7) {
                if (joy.buttons[7]) {
                    RCLCPP_INFO(log, "退出控制"); break;
                }
                if (joy.buttons[1]) {
                    RCLCPP_WARN(log, "B键: 急停回Home");
                    jog_active_ = false;
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    mg->setNamedTarget("home");
                    if (mg->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
                        mg->execute(plan);
                    RCLCPP_INFO(log, "已回到Home");
                    jog_active_ = true;
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    continue;
                }
                if (joy.buttons[2]) {
                    RCLCPP_INFO(log, "X键: 回初始位姿");
                    jog_active_ = false;
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    mg->setPoseTarget(init_pose);
                    if (mg->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
                        mg->execute(plan);
                    RCLCPP_INFO(log, "✓ 回到初始位姿");
                    jog_active_ = true;
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    continue;
                }
                bool a = joy.buttons[0];
                if (a && !prev_a) {
                    mode_trans_ = !mode_trans_;
                    RCLCPP_INFO(log, "模式: %s", mode_trans_ ? "平移优先" : "旋转优先");
                }
                prev_a = a;

                // Y键: 加速, Back键: 减速
                bool y = joy.buttons[3];
                if (y && !prev_y && speed_idx_ < (int)SPEED_SCALES.size() - 1) {
                    speed_idx_++;
                    RCLCPP_INFO(log, "▲ 速度: %.0f%% (%.1fmm/步)",
                        SPEED_SCALES[speed_idx_]*100, lin_step_*SPEED_SCALES[speed_idx_]*1000);
                }
                prev_y = y;
                bool back = joy.buttons[6];
                if (back && !prev_back && speed_idx_ > 0) {
                    speed_idx_--;
                    RCLCPP_INFO(log, "▼ 速度: %.0f%% (%.1fmm/步)",
                        SPEED_SCALES[speed_idx_]*100, lin_step_*SPEED_SCALES[speed_idx_]*1000);
                }
                prev_back = back;
            }

            if (!jog_active_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            // ─── 摇杆 → 任务空间速度 (归一化 -1~+1) ──────────────
            Eigen::VectorXd raw = Eigen::VectorXd::Zero(6);
            if (joy.axes.size() >= 6) {
                double lx = applyDeadzone(joy.axes[0], DEADZONE);
                double ly = applyDeadzone(joy.axes[1], DEADZONE);
                double rz = applyDeadzone(joy.axes[4], DEADZONE);
                double ry = applyDeadzone(joy.axes[3], DEADZONE);

                double lt = (1.0 - joy.axes[2]) / 2.0;
                double rt = (1.0 - joy.axes[5]) / 2.0;
                double roll_v = lt - rt;
                double pitch_v = 0;
                if (joy.buttons.size() > 5) {
                    if (joy.buttons[4]) pitch_v += 0.6;
                    if (joy.buttons[5]) pitch_v -= 0.6;
                }
                double dx_fine = 0, dz_fine = 0;
                if (joy.axes.size() > 7) {
                    dx_fine = applyDeadzone(joy.axes[6], 0.5) * 0.3;
                    dz_fine = applyDeadzone(joy.axes[7], 0.5) * 0.3;
                }

                if (mode_trans_) {
                    raw[0] = lx + dx_fine;
                    raw[1] = ly;
                    raw[2] = rz + dz_fine;
                    raw[3] = roll_v * 0.4;
                    raw[4] = pitch_v * 0.4;
                    raw[5] = ry * 0.3;
                } else {
                    raw[0] = (lx + dx_fine) * 0.3;
                    raw[1] = ly * 0.3;
                    raw[2] = (rz + dz_fine) * 0.5;
                    raw[3] = roll_v;
                    raw[4] = pitch_v;
                    raw[5] = ry;
                }
            }

            // ─── 指数平滑 ─────────────────────────────────────────
            sv_ = alpha_ * raw + (1.0 - alpha_) * sv_;

            double vnorm = sv_.lpNorm<Eigen::Infinity>();
            if (vnorm < 0.02) {
                if (was_moving_) {
                    std::array<double, 6> q_now;
                    { std::lock_guard<std::mutex> lk(js_mu_); q_now = cur_q_; }

                    trajectory_msgs::msg::JointTrajectory brake;
                    brake.header.stamp = now();
                    brake.joint_names = JOINT_NAMES;
                    trajectory_msgs::msg::JointTrajectoryPoint bp;
                    bp.positions.assign(q_now.begin(), q_now.end());
                    bp.velocities.assign(6, 0.0);
                    bp.time_from_start = rclcpp::Duration::from_seconds(2.0 / ctrl_rate_);
                    brake.points.push_back(bp);
                    traj_pub_->publish(brake);
                    was_moving_ = false;
                }
                sv_ *= 0.85;
                std::this_thread::sleep_for(
                    std::chrono::microseconds(int(1e6 / ctrl_rate_)));
                continue;
            }

            was_moving_ = true;

            // ─── 读取当前关节位置 ─────────────────────────────────
            std::array<double, 6> q_now;
            { std::lock_guard<std::mutex> lk(js_mu_); q_now = cur_q_; }

            // ─── Jacobian IK ──────────────────────────────────────
            auto state = std::make_shared<moveit::core::RobotState>(model);
            for (size_t i = 0; i < 6; ++i)
                state->setVariablePosition(JOINT_NAMES[i], q_now[i]);
            state->update(true);

            Eigen::MatrixXd J;
            state->getJacobian(jmg, link, Eigen::Vector3d::Zero(), J);

            double sc = SPEED_SCALES[speed_idx_];
            Eigen::VectorXd dx(6);
            dx[0] = sv_[0] * lin_speed_ * sc * dt;
            dx[1] = sv_[1] * lin_speed_ * sc * dt;
            dx[2] = sv_[2] * lin_speed_ * sc * dt;
            dx[3] = sv_[3] * ang_speed_ * sc * dt;
            dx[4] = sv_[4] * ang_speed_ * sc * dt;
            dx[5] = sv_[5] * ang_speed_ * sc * dt;

            // ─── 工作空间预检查 ──────────────────────────────────
            Eigen::Vector3d tcp_pos = state->getGlobalLinkTransform("link6_1").translation();
            Eigen::Vector3d tcp_new = tcp_pos + dx.head<3>();
            if (clampWorkspace(tcp_new)) {
                dx.head<3>() = tcp_new - tcp_pos;
                RCLCPP_WARN_THROTTLE(log, *get_clock(), 3000, "工作空间边界");
            }

            // DLS伪逆
            Eigen::Matrix<double, 6, 6> JJt = J * J.transpose()
                + DLS_LAMBDA * DLS_LAMBDA * Eigen::Matrix<double, 6, 6>::Identity();
            Eigen::VectorXd dq = J.transpose() * JJt.ldlt().solve(dx);

            double dq_max = dq.lpNorm<Eigen::Infinity>();
            if (dq_max > max_jnt_step_)
                dq *= max_jnt_step_ / dq_max;

            std::vector<double> q_target(6);
            for (int i = 0; i < 6; ++i)
                q_target[i] = q_now[i] + dq[i];

            state->setJointGroupPositions(jmg, q_target);
            state->enforceBounds(jmg);
            state->copyJointGroupPositions(jmg, q_target);

            std::vector<double> q_vel(6, 0.0);
            if (vnorm > 0.15) {
                for (int i = 0; i < 6; ++i)
                    q_vel[i] = dq[i] / dt;
            }

            // ─── 发布轨迹 ────────────────────────────────────────
            trajectory_msgs::msg::JointTrajectory traj;
            traj.header.stamp = now();
            traj.joint_names = JOINT_NAMES;

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions.assign(q_target.begin(), q_target.end());
            pt.velocities.assign(q_vel.begin(), q_vel.end());
            pt.time_from_start = rclcpp::Duration::from_seconds(1.0 / ctrl_rate_);
            traj.points.push_back(pt);

            traj_pub_->publish(traj);

            RCLCPP_INFO_THROTTLE(log, *get_clock(), 2000,
                "TCP(%.3f,%.3f,%.3f) |dq|=%.4f 速度=%.0f%% 模式=%s",
                tcp_new.x(), tcp_new.y(), tcp_new.z(),
                dq.norm(), SPEED_SCALES[speed_idx_]*100,
                mode_trans_ ? "平移" : "旋转");

            std::this_thread::sleep_for(
                std::chrono::microseconds(int(1e6 / ctrl_rate_)));
        }

        // ─── 清理 ──────────────────────────────────────────────────
        spin_ok = false;
        if (spin_th.joinable()) spin_th.join();

        RCLCPP_INFO(log, "3秒后回Home...");
        rclcpp::sleep_for(std::chrono::seconds(3));
        mg->setNamedTarget("home");
        if (mg->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
            mg->execute(plan);
        RCLCPP_INFO(log, "====== 手柄控制已退出 ======");
    }

private:
    // 速度倍率档位: Y键加速, Back键减速
    static constexpr std::array<double, 6> SPEED_SCALES = {0.25, 0.5, 1.0, 1.5, 2.0, 3.0};
    std::atomic<int> speed_idx_;  // 当前档位索引, 默认2 (1.0x)

    double lin_step_, ang_step_, max_jnt_vel_, max_jnt_step_;
    double lin_speed_, ang_speed_, alpha_, ctrl_rate_, vel_sc_, acc_sc_;
    std::atomic<bool> mode_trans_, has_joy_, has_joints_, jog_active_, was_moving_;

    std::mutex joy_mu_;
    sensor_msgs::msg::Joy joy_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    std::mutex js_mu_;
    std::array<double, 6> cur_q_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    Eigen::VectorXd sv_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyStickController>();
    RCLCPP_INFO(node->get_logger(), "====== Xbox手柄 末端笛卡尔控制节点启动 ======");
    node->run();
    rclcpp::shutdown();
    return 0;
}
