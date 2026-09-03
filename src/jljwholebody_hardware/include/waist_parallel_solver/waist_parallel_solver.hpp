#ifndef WAIST_PARALLEL_SOLVER_HPP_
#define WAIST_PARALLEL_SOLVER_HPP_

// 腰部并联机构解算模块（纯 C++，仅依赖 Eigen，不依赖 ROS）。
//
// 与原并联机构实现数学完全一致：
//   - 电机转轴平行于 y 轴（waist）；平台姿态约定 c_hat = Ry(pitch)*Rx(roll)*c
//   - b_hat(a, p, theta) = a + R(axis, -theta) * p
//   - 逆解：单腿杆长约束闭式解（aa*cos + bb*sin = -cc）
//   - 正解：两个杆长残差的 Newton 迭代（含回溯线搜索）
//   - 雅可比 J = d(pose)/d(motor)；速度 motor_vel = J^-1 pose_vel；
//     力矩 motor_torque = J^T pose_torque；功率守恒。
//
// 可直接调用：构造 Solver(MechanismParams)，调用 inverse/forward/jacobian/
// joint_to_motor_velocities/joint_to_motor_efforts/motor_to_joint_dynamics。
// 也可用 load_waist_yaml() 从 waist.yaml 加载参数（需 yaml-cpp，可选）。

#include <Eigen/Dense>

#include <string>

namespace waist_parallel
{

// 机构参数（单位：米 / 弧度）。
struct MechanismParams
{
  Eigen::Vector3d rotation_axis {Eigen::Vector3d::UnitY()};  // 腰部电机轴
  Eigen::Vector3d a1 {0, 0.0618, 0.075};   // 电机 1 转轴点
  Eigen::Vector3d a2 {0, -0.0618, 0.075};  // 电机 2 转轴点
  Eigen::Vector3d p1 {-0.08, 0.0033, -0.00018};   // 肘臂（零位 rb - ra）
  Eigen::Vector3d p2 {-0.08, -0.0033, -0.00018};
  Eigen::Vector3d c1 {-0.07493, 0.0651, 0.00329};  // 平台杆端
  Eigen::Vector3d c2 {-0.07493, -0.0651, 0.00329};
  double rod1 {0.07170945404896066};
  double rod2 {0.07170945404896066};
  double offset1 {0.0};
  double offset2 {0.0};
  double direction1 {1.0};   // motor_direction[0]
  double direction2 {-1.0};  // motor_direction[1]
  // 机构角（电机角经 direction/offset 变换后）钳位范围
  double theta_min1 {-0.6};
  double theta_max1 {0.8};
  double theta_min2 {-0.6};
  double theta_max2 {0.8};
  // 逻辑姿态限制
  double pitch_min {-0.5};
  double pitch_max {0.5};
  double roll_min {-0.2};
  double roll_max {0.2};
};

struct InverseResult
{
  bool ok {false};
  double motor1 {0.0};
  double motor2 {0.0};
  std::string message;
};

struct ForwardResult
{
  bool ok {false};
  double pitch {0.0};
  double roll {0.0};
  std::string message;
};

// 可调用子函数集合（模块化）。
class Solver
{
public:
  explicit Solver(const MechanismParams & params);

  const MechanismParams & params() const;

  // ── 逆解：逻辑姿态 (pitch, roll) -> 两个电机角度（量化 0.001 并钳位）──
  InverseResult inverse(double pitch, double roll) const;

  // ── 正解：两个电机角度 -> (pitch, roll)（Newton；pitch_seed/roll_seed 初值）──
  ForwardResult forward(
    double motor1,
    double motor2,
    double pitch_seed = 0.0,
    double roll_seed = 0.0) const;

  // ── 雅可比 J = d(pose)/d(motor)，在 (pitch, roll) 处计算 ──
  bool pose_jacobian(
    double motor1,
    double motor2,
    double pitch,
    double roll,
    Eigen::Matrix2d & jacobian,
    std::string & error_message) const;

  // ── 速度映射：逻辑关节速度 -> 电机速度（motor_vel = J^-1 pose_vel）──
  bool joint_to_motor_velocities(
    double pitch,
    double roll,
    double motor1,
    double motor2,
    double pitch_velocity,
    double roll_velocity,
    double & motor1_velocity,
    double & motor2_velocity,
    std::string & error_message) const;

  // ── 力矩映射：逻辑关节力矩 -> 电机力矩（motor_torque = J^T pose_torque）──
  bool joint_to_motor_efforts(
    double pitch,
    double roll,
    double motor1,
    double motor2,
    double pitch_effort,
    double roll_effort,
    double & motor1_effort,
    double & motor2_effort,
    std::string & error_message) const;

  // ── 动力学映射：电机速度/力矩 -> 逻辑关节速度/力矩（pose_vel = J motor_vel；
  //    pose_torque = J^-T motor_torque）──
  bool motor_to_joint_dynamics(
    double pitch,
    double roll,
    double motor1,
    double motor2,
    double motor1_velocity,
    double motor2_velocity,
    double motor1_effort,
    double motor2_effort,
    double & pitch_velocity,
    double & roll_velocity,
    double & pitch_effort,
    double & roll_effort,
    std::string & error_message) const;

private:
  MechanismParams params_;

  static Eigen::Vector3d rotate_about(
    const Eigen::Vector3d & value,
    const Eigen::Vector3d & axis,
    double angle);
  static Eigen::Vector3d c_hat(
    const Eigen::Vector3d & c,
    double pitch,
    double roll);
  static void c_hat_derivatives(
    const Eigen::Vector3d & c,
    double pitch,
    double roll,
    Eigen::Vector3d & rotated,
    Eigen::Vector3d & d_pitch,
    Eigen::Vector3d & d_roll);
  Eigen::Vector3d b_hat(
    const Eigen::Vector3d & a,
    const Eigen::Vector3d & p,
    double theta) const;
  Eigen::Vector3d db_hat_dtheta(
    const Eigen::Vector3d & p,
    double theta) const;
  static double wrap_to_pi(double value);
  double clamp_theta(double value, int index) const;
  double mechanism_theta(double motor, int index) const;
  bool solve_theta_one_leg(
    const Eigen::Vector3d & a,
    const Eigen::Vector3d & p,
    const Eigen::Vector3d & c,
    double rod_length,
    double pitch,
    double roll,
    double theta_prev,
    double theta_min,
    double theta_max,
    double & theta,
    std::string & error_message) const;
  bool solve_pose_from_thetas(
    double theta1,
    double theta2,
    double pitch_seed,
    double roll_seed,
    double & pitch,
    double & roll,
    std::string & error_message) const;
};

// 便捷：从 waist.yaml 加载机构参数（需要 yaml-cpp 链接）。
bool load_waist_yaml(const std::string & yaml_file, MechanismParams & params, std::string & error_message);

}  // namespace waist_parallel

#endif  // WAIST_PARALLEL_SOLVER_HPP_
