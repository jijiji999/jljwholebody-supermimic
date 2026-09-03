// 腰部并联机构解算实现。
#include "waist_parallel_solver/waist_parallel_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace waist_parallel
{

namespace
{
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kThetaStep = 1.0e-3;
constexpr double kSmallValue = 1.0e-12;
constexpr double kReachabilityTolerance = 1.0e-10;
constexpr double kResidualTolerance = 1.0e-11;
constexpr double kStepTolerance = 1.0e-11;
constexpr int kMaxNewtonIterations = 50;
constexpr int kLineSearchIterations = 14;

double clamp_value(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}
}  // namespace

Solver::Solver(const MechanismParams & params) : params_(params) {}

const MechanismParams & Solver::params() const { return params_; }

Eigen::Vector3d Solver::rotate_about(
  const Eigen::Vector3d & value,
  const Eigen::Vector3d & axis,
  const double angle)
{
  const double ca = std::cos(angle);
  const double sa = std::sin(angle);
  const Eigen::Vector3d axis_cross_value = axis.cross(value);
  const double axis_dot_value = axis.dot(value);
  return value * ca + axis_cross_value * sa + axis * (axis_dot_value * (1.0 - ca));
}

Eigen::Vector3d Solver::c_hat(const Eigen::Vector3d & c, const double pitch, const double roll)
{
  // 先绕 x 轴转 roll，再绕 y 轴转 pitch。
  const Eigen::Vector3d rx = rotate_about(c, Eigen::Vector3d::UnitX(), roll);
  return rotate_about(rx, Eigen::Vector3d::UnitY(), pitch);
}

void Solver::c_hat_derivatives(
  const Eigen::Vector3d & c,
  const double pitch,
  const double roll,
  Eigen::Vector3d & rotated,
  Eigen::Vector3d & d_pitch,
  Eigen::Vector3d & d_roll)
{
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double x = c.x();
  const double y = c.y();
  const double z = c.z();
  const double y_rot = y * cr - z * sr;
  const double z_rot = y * sr + z * cr;
  rotated = Eigen::Vector3d(cp * x + sp * z_rot, y_rot, -sp * x + cp * z_rot);
  d_pitch = Eigen::Vector3d(-sp * x + cp * z_rot, 0.0, -cp * x - sp * z_rot);
  d_roll = Eigen::Vector3d(sp * y_rot, -z_rot, cp * y_rot);
}

Eigen::Vector3d Solver::b_hat(
  const Eigen::Vector3d & a,
  const Eigen::Vector3d & p,
  const double theta) const
{
  return a + rotate_about(p, params_.rotation_axis, -theta);
}

Eigen::Vector3d Solver::db_hat_dtheta(
  const Eigen::Vector3d & p,
  const double theta) const
{
  return -params_.rotation_axis.cross(b_hat(Eigen::Vector3d::Zero(), p, theta));
}

double Solver::wrap_to_pi(double value)
{
  while (value > kPi) {
    value -= 2.0 * kPi;
  }
  while (value < -kPi) {
    value += 2.0 * kPi;
  }
  return value;
}

double Solver::clamp_theta(const double value, const int index) const
{
  const double min_value = index == 0 ? params_.theta_min1 : params_.theta_min2;
  const double max_value = index == 0 ? params_.theta_max1 : params_.theta_max2;
  const double quantized = std::round(value / kThetaStep) * kThetaStep;
  return clamp_value(quantized, min_value, max_value);
}

double Solver::mechanism_theta(const double motor, const int index) const
{
  return (index == 0 ? params_.direction1 : params_.direction2) * motor +
         (index == 0 ? params_.offset1 : params_.offset2);
}

bool Solver::solve_theta_one_leg(
  const Eigen::Vector3d & a,
  const Eigen::Vector3d & p,
  const Eigen::Vector3d & c,
  const double rod_length,
  const double pitch,
  const double roll,
  const double theta_prev,
  const double theta_min,
  const double theta_max,
  double & theta,
  std::string & error_message) const
{
  const Eigen::Vector3d chat = c_hat(c, pitch, roll);
  const Eigen::Vector3d d = a - chat;
  const Eigen::Vector3d axis = params_.rotation_axis;
  // 杆长约束 |a + R(axis,-theta)p - chat| = rod 展开为 aa*cos + bb*sin = -cc
  const double aa = d.dot(p) - d.dot(axis) * p.dot(axis);
  const double bb = -d.dot(axis.cross(p));
  const double cc = d.dot(axis) * p.dot(axis) +
    (d.squaredNorm() + p.squaredNorm() - rod_length * rod_length) * 0.5;
  const double radius = std::sqrt(aa * aa + bb * bb);
  if (radius < kSmallValue) {
    error_message = "Degenerate leg geometry";
    return false;
  }
  const double acos_argument = -cc / radius;
  if (acos_argument > 1.0 + kReachabilityTolerance ||
    acos_argument < -1.0 - kReachabilityTolerance)
  {
    error_message = "Unreachable pose (acos argument=" +
      std::to_string(acos_argument) + ")";
    return false;
  }
  const double alpha = std::acos(clamp_value(acos_argument, -1.0, 1.0));
  const double phase = std::atan2(bb, aa);
  const double candidates[2] = {
    wrap_to_pi(phase + alpha), wrap_to_pi(phase - alpha)};
  const double previous = wrap_to_pi(theta_prev);
  double selected = candidates[0];
  double selected_distance = std::numeric_limits<double>::infinity();
  bool have_in_range = false;
  for (const double candidate : candidates) {
    const bool in_range = candidate >= theta_min - kReachabilityTolerance &&
      candidate <= theta_max + kReachabilityTolerance;
    if (in_range && !have_in_range) {
      have_in_range = true;
      selected_distance = std::numeric_limits<double>::infinity();
    }
    if (have_in_range && !in_range) {
      continue;
    }
    const double distance = std::abs(wrap_to_pi(candidate - previous));
    if (distance < selected_distance) {
      selected = candidate;
      selected_distance = distance;
    }
  }
  theta = wrap_to_pi(selected);
  error_message.clear();
  return true;
}

InverseResult Solver::inverse(const double pitch, const double roll) const
{
  InverseResult result;
  const double limited_pitch = clamp_value(pitch, params_.pitch_min, params_.pitch_max);
  const double limited_roll = clamp_value(roll, params_.roll_min, params_.roll_max);
  double theta1 = 0.0;
  double theta2 = 0.0;
  if (!solve_theta_one_leg(
      params_.a1, params_.p1, params_.c1, params_.rod1,
      limited_pitch, limited_roll, 0.0,
      params_.theta_min1, params_.theta_max1, theta1, result.message) ||
    !solve_theta_one_leg(
      params_.a2, params_.p2, params_.c2, params_.rod2,
      limited_pitch, limited_roll, 0.0,
      params_.theta_min2, params_.theta_max2, theta2, result.message))
  {
    return result;
  }
  result.motor1 = clamp_theta(params_.direction1 * (theta1 - params_.offset1), 0);
  result.motor2 = clamp_theta(params_.direction2 * (theta2 - params_.offset2), 1);
  result.ok = true;
  result.message.clear();
  return result;
}

bool Solver::solve_pose_from_thetas(
  const double theta1,
  const double theta2,
  double pitch_seed,
  double roll_seed,
  double & pitch,
  double & roll,
  std::string & error_message) const
{
  pitch = clamp_value(pitch_seed, params_.pitch_min, params_.pitch_max);
  roll = clamp_value(roll_seed, params_.roll_min, params_.roll_max);
  const Eigen::Vector3d bhat1 = b_hat(params_.a1, params_.p1, mechanism_theta(theta1, 0));
  const Eigen::Vector3d bhat2 = b_hat(params_.a2, params_.p2, mechanism_theta(theta2, 1));
  const Eigen::Vector3d bhats[2] = {bhat1, bhat2};
  const Eigen::Vector3d cs[2] = {params_.c1, params_.c2};
  const double rods[2] = {params_.rod1, params_.rod2};

  for (int iteration = 0; iteration < kMaxNewtonIterations; ++iteration) {
    Eigen::Vector2d residual;
    Eigen::Matrix2d jacobian;
    for (int index = 0; index < 2; ++index) {
      Eigen::Vector3d chat;
      Eigen::Vector3d d_pitch;
      Eigen::Vector3d d_roll;
      c_hat_derivatives(cs[index], pitch, roll, chat, d_pitch, d_roll);
      const double k = (bhats[index].squaredNorm() + cs[index].squaredNorm() -
        rods[index] * rods[index]) * 0.5;
      residual[index] = bhats[index].dot(chat) - k;
      jacobian(index, 0) = bhats[index].dot(d_pitch);
      jacobian(index, 1) = bhats[index].dot(d_roll);
    }
    const double residual_norm = residual.norm();
    if (residual_norm < kResidualTolerance) {
      pitch = clamp_value(pitch, params_.pitch_min, params_.pitch_max);
      roll = clamp_value(roll, params_.roll_min, params_.roll_max);
      error_message.clear();
      return true;
    }
    const double determinant = jacobian.determinant();
    if (std::abs(determinant) < kSmallValue) {
      error_message = "Forward Jacobian is singular";
      return false;
    }
    const Eigen::Vector2d step = jacobian.inverse() * (-residual);
    double scale = 1.0;
    bool accepted = false;
    for (int line_search = 0; line_search < kLineSearchIterations; ++line_search) {
      const double next_pitch = pitch + scale * step[0];
      const double next_roll = roll + scale * step[1];
      Eigen::Vector2d next_residual;
      for (int index = 0; index < 2; ++index) {
        Eigen::Vector3d chat;
        Eigen::Vector3d unused_p;
        Eigen::Vector3d unused_r;
        c_hat_derivatives(cs[index], next_pitch, next_roll, chat, unused_p, unused_r);
        const double k = (bhats[index].squaredNorm() + cs[index].squaredNorm() -
          rods[index] * rods[index]) * 0.5;
        next_residual[index] = bhats[index].dot(chat) - k;
      }
      if (next_residual.norm() < residual_norm) {
        pitch = next_pitch;
        roll = next_roll;
        accepted = true;
        break;
      }
      scale *= 0.5;
    }
    if (!accepted) {
      error_message = "Forward Newton step could not reduce residual";
      return false;
    }
    if ((scale * step).norm() < kStepTolerance) {
      Eigen::Vector2d final_residual;
      Eigen::Matrix2d final_jacobian;
      for (int index = 0; index < 2; ++index) {
        Eigen::Vector3d chat;
        Eigen::Vector3d d_pitch;
        Eigen::Vector3d d_roll;
        c_hat_derivatives(cs[index], pitch, roll, chat, d_pitch, d_roll);
        const double k = (bhats[index].squaredNorm() + cs[index].squaredNorm() -
          rods[index] * rods[index]) * 0.5;
        final_residual[index] = bhats[index].dot(chat) - k;
      }
      if (final_residual.norm() >= kResidualTolerance) {
        error_message = "Forward Newton step stalled";
        return false;
      }
      pitch = clamp_value(pitch, params_.pitch_min, params_.pitch_max);
      roll = clamp_value(roll, params_.roll_min, params_.roll_max);
      error_message.clear();
      return true;
    }
  }
  error_message = "Forward Newton solve did not converge";
  return false;
}

ForwardResult Solver::forward(
  const double motor1,
  const double motor2,
  const double pitch_seed,
  const double roll_seed) const
{
  ForwardResult result;
  const double theta1 = clamp_theta(motor1, 0);
  const double theta2 = clamp_theta(motor2, 1);
  if (!solve_pose_from_thetas(
      theta1, theta2, pitch_seed, roll_seed, result.pitch, result.roll, result.message))
  {
    return result;
  }
  result.ok = true;
  result.message.clear();
  return result;
}

bool Solver::pose_jacobian(
  const double motor1,
  const double motor2,
  const double pitch,
  const double roll,
  Eigen::Matrix2d & jacobian,
  std::string & error_message) const
{
  const double theta1 = clamp_theta(motor1, 0);
  const double theta2 = clamp_theta(motor2, 1);
  const double motor_thetas[2] = {theta1, theta2};
  const Eigen::Vector3d a[2] = {params_.a1, params_.a2};
  const Eigen::Vector3d p[2] = {params_.p1, params_.p2};
  const Eigen::Vector3d c[2] = {params_.c1, params_.c2};
  const double directions[2] = {params_.direction1, params_.direction2};

  Eigen::Matrix2d pose_constraints;
  Eigen::Vector2d motor_constraints;
  for (int index = 0; index < 2; ++index) {
    const double mech_theta = mechanism_theta(motor_thetas[index], index);
    const Eigen::Vector3d bhat = b_hat(a[index], p[index], mech_theta);
    const Eigen::Vector3d chat = c_hat(c[index], pitch, roll);
    Eigen::Vector3d unused_pitch;
    Eigen::Vector3d d_pitch;
    Eigen::Vector3d d_roll;
    c_hat_derivatives(c[index], pitch, roll, unused_pitch, d_pitch, d_roll);
    const Eigen::Vector3d separation = bhat - chat;
    pose_constraints(index, 0) = -2.0 * separation.dot(d_pitch);
    pose_constraints(index, 1) = -2.0 * separation.dot(d_roll);
    const Eigen::Vector3d db = db_hat_dtheta(p[index], mech_theta);
    motor_constraints[index] = 2.0 * separation.dot(directions[index] * db);
  }
  const double determinant = pose_constraints.determinant();
  if (std::abs(determinant) < kSmallValue) {
    error_message = "Pose Jacobian is singular";
    return false;
  }
  Eigen::Matrix2d diagonal = Eigen::Matrix2d::Zero();
  diagonal(0, 0) = motor_constraints[0];
  diagonal(1, 1) = motor_constraints[1];
  jacobian = -pose_constraints.inverse() * diagonal;
  error_message.clear();
  return true;
}

bool Solver::joint_to_motor_velocities(
  const double pitch,
  const double roll,
  const double motor1,
  const double motor2,
  const double pitch_velocity,
  const double roll_velocity,
  double & motor1_velocity,
  double & motor2_velocity,
  std::string & error_message) const
{
  const double theta1 = clamp_theta(motor1, 0);
  const double theta2 = clamp_theta(motor2, 1);
  double solved_pitch = pitch;
  double solved_roll = roll;
  if (!solve_pose_from_thetas(
      theta1, theta2, pitch, roll, solved_pitch, solved_roll, error_message))
  {
    return false;
  }
  Eigen::Matrix2d jacobian;
  if (!pose_jacobian(theta1, theta2, solved_pitch, solved_roll, jacobian, error_message)) {
    return false;
  }
  const double determinant = jacobian.determinant();
  if (std::abs(determinant) < kSmallValue) {
    error_message = "Motor velocity Jacobian is singular";
    return false;
  }
  const Eigen::Vector2d result = jacobian.inverse() *
    Eigen::Vector2d(pitch_velocity, roll_velocity);
  motor1_velocity = result[0];
  motor2_velocity = result[1];
  error_message.clear();
  return true;
}

bool Solver::joint_to_motor_efforts(
  const double pitch,
  const double roll,
  const double motor1,
  const double motor2,
  const double pitch_effort,
  const double roll_effort,
  double & motor1_effort,
  double & motor2_effort,
  std::string & error_message) const
{
  const double theta1 = clamp_theta(motor1, 0);
  const double theta2 = clamp_theta(motor2, 1);
  double solved_pitch = pitch;
  double solved_roll = roll;
  if (!solve_pose_from_thetas(
      theta1, theta2, pitch, roll, solved_pitch, solved_roll, error_message))
  {
    return false;
  }
  Eigen::Matrix2d jacobian;
  if (!pose_jacobian(theta1, theta2, solved_pitch, solved_roll, jacobian, error_message)) {
    return false;
  }
  const Eigen::Vector2d result = jacobian.transpose() *
    Eigen::Vector2d(pitch_effort, roll_effort);
  motor1_effort = result[0];
  motor2_effort = result[1];
  error_message.clear();
  return true;
}

bool Solver::motor_to_joint_dynamics(
  const double pitch,
  const double roll,
  const double motor1,
  const double motor2,
  const double motor1_velocity,
  const double motor2_velocity,
  const double motor1_effort,
  const double motor2_effort,
  double & pitch_velocity,
  double & roll_velocity,
  double & pitch_effort,
  double & roll_effort,
  std::string & error_message) const
{
  const double theta1 = clamp_theta(motor1, 0);
  const double theta2 = clamp_theta(motor2, 1);
  double solved_pitch = pitch;
  double solved_roll = roll;
  if (!solve_pose_from_thetas(
      theta1, theta2, pitch, roll, solved_pitch, solved_roll, error_message))
  {
    return false;
  }
  Eigen::Matrix2d jacobian;
  if (!pose_jacobian(theta1, theta2, solved_pitch, solved_roll, jacobian, error_message)) {
    return false;
  }
  const Eigen::Vector2d velocity = jacobian *
    Eigen::Vector2d(motor1_velocity, motor2_velocity);
  const double determinant = jacobian.determinant();
  if (std::abs(determinant) < kSmallValue) {
    error_message = "Joint effort Jacobian is singular";
    return false;
  }
  const Eigen::Vector2d effort = jacobian.transpose().inverse() *
    Eigen::Vector2d(motor1_effort, motor2_effort);
  pitch_velocity = velocity[0];
  roll_velocity = velocity[1];
  pitch_effort = effort[0];
  roll_effort = effort[1];
  error_message.clear();
  return true;
}

}  // namespace waist_parallel
