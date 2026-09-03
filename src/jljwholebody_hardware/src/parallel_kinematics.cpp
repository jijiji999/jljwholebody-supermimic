#include "jljwholebody_hardware/parallel_kinematics.hpp"

#include "waist_parallel_solver/waist_parallel_yaml.hpp"

#include "yaml-cpp/yaml.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>

namespace jljwholebody_hardware
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

struct ThetaRange
{
  double min;
  double max;
};

ThetaRange theta_range(const bool right_side, const int index)
{
  if (!right_side) {
    return index == 0 ? ThetaRange {-1.107, 1.082} : ThetaRange {-1.064, 1.218};
  }
  return index == 0 ? ThetaRange {-1.082, 1.107} : ThetaRange {-1.218, 1.064};
}

Eigen::Vector3d read_vector3(const YAML::Node & node, const char * key)
{
  const auto values = node[key];
  if (!values || !values.IsSequence() || values.size() != 3U) {
    throw std::runtime_error(std::string("Expected 3-vector for key: ") + key);
  }
  return Eigen::Vector3d(
    values[0].as<double>(), values[1].as<double>(), values[2].as<double>());
}

double read_direction(const YAML::Node & node, const std::size_t index)
{
  if (!node) {
    return 1.0;
  }
  if (!node.IsSequence() || node.size() != 2U) {
    throw std::runtime_error("Expected 2-vector for key: motor_direction");
  }
  return node[index].as<double>() < 0.0 ? -1.0 : 1.0;
}

double read_optional_double(const YAML::Node & node, const char * key, const double fallback)
{
  return node[key] ? node[key].as<double>() : fallback;
}

bool finite_pose(const double pitch, const double roll)
{
  return std::isfinite(pitch) && std::isfinite(roll);
}

}  // namespace

bool ParallelKinematics::load_from_yaml(
  const std::string & yaml_file,
  std::string & error_message)
{
  try {
    const YAML::Node config = YAML::LoadFile(yaml_file);
    params_ = MechanismParams {};
    waist_solver_.reset();
    waist_loaded_ = false;

    const std::string type_name = config["joint_type"] ?
      config["joint_type"].as<std::string>() : "ankle";
    if (type_name == "waist") {
      params_.type = ParallelMechanismType::kWaist;
      waist_parallel::MechanismParams waist_params;
      if (!waist_parallel::load_waist_yaml(yaml_file, waist_params, error_message)) {
        loaded_ = false;
        return false;
      }
      waist_solver_ = std::make_unique<waist_parallel::Solver>(waist_params);
      pitch_limits_ = {waist_params.pitch_min, waist_params.pitch_max};
      roll_limits_ = {waist_params.roll_min, waist_params.roll_max};
      waist_loaded_ = true;
      loaded_ = true;
      error_message.clear();
      return true;
    }
    if (type_name != "ankle") {
      throw std::runtime_error(
              "Parallel solver supports joint_type=ankle or joint_type=waist");
    }
    params_.type = ParallelMechanismType::kAnkle;

    const Eigen::Vector3d a1 = read_vector3(config, "ra01");
    const Eigen::Vector3d a2 = read_vector3(config, "ra02");
    const Eigen::Vector3d b1 = read_vector3(config, "rb01");
    const Eigen::Vector3d b2 = read_vector3(config, "rb02");
    const Eigen::Vector3d c1 = read_vector3(config, "rc01");
    const Eigen::Vector3d c2 = read_vector3(config, "rc02");
    params_.a1 = a1;
    params_.a2 = a2;
    params_.b1 = b1;
    params_.b2 = b2;
    params_.c1 = c1;
    params_.c2 = c2;
    params_.p1 = b1 - a1;
    params_.p2 = b2 - a2;

    const auto rod_lengths = config["rod_length"];
    if (!rod_lengths || !rod_lengths.IsSequence() || rod_lengths.size() != 2U) {
      throw std::runtime_error("Expected 2-vector for key: rod_length");
    }
    params_.rod1 = rod_lengths[0].as<double>();
    params_.rod2 = rod_lengths[1].as<double>();
    params_.offset1 = read_optional_double(config, "m01", 0.0);
    params_.offset2 = read_optional_double(config, "m02", 0.0);
    params_.direction1 = read_direction(config["motor_direction"], 0U);
    params_.direction2 = read_direction(config["motor_direction"], 1U);

    const std::string path_lower = std::filesystem::path(yaml_file).filename().string();
    params_.right_side = path_lower.find("right") != std::string::npos;
    if (config["side"]) {
      const auto side = config["side"].as<std::string>();
      if (side == "right") {
        params_.right_side = true;
      } else if (side == "left") {
        params_.right_side = false;
      } else {
        throw std::runtime_error("side must be left or right");
      }
    }

    if (!(params_.rod1 > 0.0 && params_.rod2 > 0.0) ||
      !params_.p1.allFinite() || !params_.p2.allFinite())
    {
      throw std::runtime_error("Invalid measured ankle geometry");
    }

    loaded_ = true;
    pitch_limits_ = {-0.5, 0.3};
    roll_limits_ = {-0.2, 0.2};
    error_message.clear();
    return true;
  } catch (const std::exception & exception) {
    loaded_ = false;
    error_message = exception.what();
    return false;
  }
}

ParallelMechanismType ParallelKinematics::type() const
{
  return params_.type;
}

const ParallelAxisLimits & ParallelKinematics::pitch_limits() const
{
  return pitch_limits_;
}

const ParallelAxisLimits & ParallelKinematics::roll_limits() const
{
  return roll_limits_;
}

Eigen::Vector3d ParallelKinematics::rotate_x(const Eigen::Vector3d & value, const double angle)
{
  const double ca = std::cos(angle);
  const double sa = std::sin(angle);
  return Eigen::Vector3d(value.x(), ca * value.y() - sa * value.z(),
                         sa * value.y() + ca * value.z());
}

Eigen::Vector3d ParallelKinematics::rotate_y(const Eigen::Vector3d & value, const double angle)
{
  const double ca = std::cos(angle);
  const double sa = std::sin(angle);
  return Eigen::Vector3d(ca * value.x() + sa * value.z(), value.y(),
                         -sa * value.x() + ca * value.z());
}

Eigen::Vector3d ParallelKinematics::c_hat(
  const Eigen::Vector3d & value,
  const double pitch,
  const double roll)
{
  return rotate_y(rotate_x(value, roll), pitch);
}

void ParallelKinematics::c_hat_derivatives(
  const Eigen::Vector3d & value,
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
  const double x = value.x();
  const double y = value.y();
  const double z = value.z();
  const double y_rot = y * cr - z * sr;
  const double z_rot = y * sr + z * cr;

  rotated = Eigen::Vector3d(cp * x + sp * z_rot, y_rot, -sp * x + cp * z_rot);
  d_pitch = Eigen::Vector3d(-sp * x + cp * z_rot, 0.0, -cp * x - sp * z_rot);
  d_roll = Eigen::Vector3d(sp * y_rot, -z_rot, cp * y_rot);
}

Eigen::Vector3d ParallelKinematics::b_hat(
  const Eigen::Vector3d & a,
  const Eigen::Vector3d & p,
  const double theta)
{
  return a + rotate_x(p, -theta);
}

Eigen::Vector3d ParallelKinematics::db_hat_dtheta(
  const Eigen::Vector3d & p,
  const double theta)
{
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  return Eigen::Vector3d(0.0, -p.y() * st + p.z() * ct,
                         -p.y() * ct - p.z() * st);
}

double ParallelKinematics::wrap_to_pi(double value)
{
  while (value > kPi) {
    value -= 2.0 * kPi;
  }
  while (value < -kPi) {
    value += 2.0 * kPi;
  }
  return value;
}

double ParallelKinematics::quantize(const double value)
{
  return std::round(value / kThetaStep) * kThetaStep;
}

double ParallelKinematics::clamp_theta(const double value, const int index) const
{
  const ThetaRange range = theta_range(params_.right_side, index);
  return std::clamp(quantize(value), range.min, range.max);
}

double ParallelKinematics::mechanism_theta(const double motor, const int index) const
{
  return (index == 0 ? params_.direction1 : params_.direction2) * motor +
         (index == 0 ? params_.offset1 : params_.offset2);
}

bool ParallelKinematics::solve_theta_one_leg(
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
  const double aa = d.y() * p.y() + d.z() * p.z();
  const double bb = d.y() * p.z() - d.z() * p.y();
  const double cc = d.x() * p.x() +
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
    error_message = "Unreachable ankle pose (acos argument=" +
      std::to_string(acos_argument) + ")";
    return false;
  }

  const double alpha = std::acos(std::clamp(acos_argument, -1.0, 1.0));
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

bool ParallelKinematics::inverse(
  const double pitch,
  const double roll,
  double & motor_1,
  double & motor_2,
  std::string & error_message) const
{
  if (!is_loaded()) {
    error_message = "Parallel solver is not loaded";
    return false;
  }
  if (waist_loaded_) {
    const auto result = waist_solver_->inverse(pitch, roll);
    motor_1 = result.motor1;
    motor_2 = result.motor2;
    error_message = result.message;
    return result.ok;
  }
  if (!finite_pose(pitch, roll)) {
    error_message = "Non-finite parallel-joint pose";
    motor_1 = motor_2 = std::numeric_limits<double>::quiet_NaN();
    return false;
  }

  const double limited_pitch = std::clamp(pitch, pitch_limits_.min, pitch_limits_.max);
  const double limited_roll = std::clamp(roll, roll_limits_.min, roll_limits_.max);
  double theta1 = 0.0;
  double theta2 = 0.0;
  const ThetaRange range1 = theta_range(params_.right_side, 0);
  const ThetaRange range2 = theta_range(params_.right_side, 1);
  if (!solve_theta_one_leg(
      params_.a1, params_.p1, params_.c1, params_.rod1,
      limited_pitch, limited_roll, 0.0, range1.min, range1.max, theta1, error_message) ||
    !solve_theta_one_leg(
      params_.a2, params_.p2, params_.c2, params_.rod2,
      limited_pitch, limited_roll, 0.0, range2.min, range2.max, theta2, error_message))
  {
    motor_1 = motor_2 = std::numeric_limits<double>::quiet_NaN();
    return false;
  }

  motor_1 = clamp_theta(params_.direction1 * (theta1 - params_.offset1), 0);
  motor_2 = clamp_theta(params_.direction2 * (theta2 - params_.offset2), 1);
  error_message.clear();
  return true;
}

bool ParallelKinematics::residual_and_jacobian(
  const Eigen::Vector3d & bhat1,
  const Eigen::Vector3d & bhat2,
  const double pitch,
  const double roll,
  Eigen::Vector2d & residual,
  Eigen::Matrix2d & jacobian) const
{
  const Eigen::Vector3d bhats[2] = {bhat1, bhat2};
  const Eigen::Vector3d cs[2] = {params_.c1, params_.c2};
  const double lengths[2] = {params_.rod1, params_.rod2};
  for (int index = 0; index < 2; ++index) {
    Eigen::Vector3d chat;
    Eigen::Vector3d d_pitch;
    Eigen::Vector3d d_roll;
    c_hat_derivatives(cs[index], pitch, roll, chat, d_pitch, d_roll);
    const double k = (bhats[index].squaredNorm() + cs[index].squaredNorm() -
      lengths[index] * lengths[index]) * 0.5;
    residual[index] = bhats[index].dot(chat) - k;
    jacobian(index, 0) = bhats[index].dot(d_pitch);
    jacobian(index, 1) = bhats[index].dot(d_roll);
  }
  return residual.allFinite() && jacobian.allFinite();
}

bool ParallelKinematics::solve_pose_from_thetas(
  const double theta1,
  const double theta2,
  double pitch_seed,
  double roll_seed,
  double & pitch,
  double & roll,
  std::string & error_message) const
{
  if (!finite_pose(pitch_seed, roll_seed)) {
    pitch_seed = 0.0;
    roll_seed = 0.0;
  }
  pitch = std::clamp(pitch_seed, pitch_limits_.min, pitch_limits_.max);
  roll = std::clamp(roll_seed, roll_limits_.min, roll_limits_.max);
  const Eigen::Vector3d bhat1 = b_hat(
    params_.a1, params_.p1, mechanism_theta(theta1, 0));
  const Eigen::Vector3d bhat2 = b_hat(
    params_.a2, params_.p2, mechanism_theta(theta2, 1));

  for (int iteration = 0; iteration < kMaxNewtonIterations; ++iteration) {
    Eigen::Vector2d residual;
    Eigen::Matrix2d jacobian;
    if (!residual_and_jacobian(bhat1, bhat2, pitch, roll, residual, jacobian)) {
      error_message = "Non-finite forward residual";
      return false;
    }
    const double residual_norm = residual.norm();
    if (residual_norm < kResidualTolerance) {
      pitch = std::clamp(pitch, pitch_limits_.min, pitch_limits_.max);
      roll = std::clamp(roll, roll_limits_.min, roll_limits_.max);
      error_message.clear();
      return true;
    }

    const double determinant = jacobian.determinant();
    if (std::abs(determinant) < kSmallValue) {
      error_message = "Forward Jacobian is singular";
      return false;
    }
    const Eigen::Vector2d step = jacobian.inverse() * (-residual);
    if (!step.allFinite()) {
      error_message = "Forward Newton step is non-finite";
      return false;
    }
    double scale = 1.0;
    bool accepted = false;
    for (int line_search = 0; line_search < kLineSearchIterations; ++line_search) {
      const double next_pitch = pitch + scale * step[0];
      const double next_roll = roll + scale * step[1];
      Eigen::Vector2d next_residual;
      Eigen::Matrix2d next_jacobian;
      if (residual_and_jacobian(
          bhat1, bhat2, next_pitch, next_roll, next_residual, next_jacobian) &&
        next_residual.norm() < residual_norm)
      {
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
      if (!residual_and_jacobian(
          bhat1, bhat2, pitch, roll, final_residual, final_jacobian) ||
        final_residual.norm() >= kResidualTolerance)
      {
        error_message = "Forward Newton step stalled";
        return false;
      }
      pitch = std::clamp(pitch, pitch_limits_.min, pitch_limits_.max);
      roll = std::clamp(roll, roll_limits_.min, roll_limits_.max);
      error_message.clear();
      return true;
    }
  }
  error_message = "Forward Newton solve did not converge";
  return false;
}

bool ParallelKinematics::forward(
  const double motor_1,
  const double motor_2,
  double & pitch,
  double & roll,
  std::string & error_message,
  const double pitch_seed,
  const double roll_seed) const
{
  double limited_motor_1 = motor_1;
  double limited_motor_2 = motor_2;
  return forward(
    motor_1, motor_2, limited_motor_1, limited_motor_2,
    pitch, roll, error_message, pitch_seed, roll_seed);
}

bool ParallelKinematics::forward(
  const double motor_1,
  const double motor_2,
  double & limited_motor_1,
  double & limited_motor_2,
  double & pitch,
  double & roll,
  std::string & error_message,
  const double pitch_seed,
  const double roll_seed) const
{
  if (!is_loaded()) {
    error_message = "Parallel solver is not loaded";
    return false;
  }
  if (!std::isfinite(motor_1) || !std::isfinite(motor_2)) {
    error_message = "Non-finite motor angle";
    return false;
  }
  if (waist_loaded_) {
    const auto result = waist_solver_->forward(
      motor_1, motor_2, pitch_seed, roll_seed);
    pitch = result.pitch;
    roll = result.roll;
    error_message = result.message;
    limited_motor_1 = motor_1;
    limited_motor_2 = motor_2;
    return result.ok;
  }
  limited_motor_1 = clamp_theta(motor_1, 0);
  limited_motor_2 = clamp_theta(motor_2, 1);
  return solve_pose_from_thetas(
    limited_motor_1, limited_motor_2, pitch_seed, roll_seed,
    pitch, roll, error_message);
}

bool ParallelKinematics::pose_jacobian(
  const double theta1,
  const double theta2,
  const double pitch,
  const double roll,
  Eigen::Matrix2d & jacobian,
  std::string & error_message) const
{
  if (waist_loaded_) {
    return waist_solver_->pose_jacobian(
      theta1, theta2, pitch, roll, jacobian, error_message);
  }
  if (!finite_pose(pitch, roll)) {
    error_message = "Non-finite ankle pose for Jacobian";
    return false;
  }
  const double motor_theta[2] = {clamp_theta(theta1, 0), clamp_theta(theta2, 1)};
  const double mechanism_thetas[2] = {
    mechanism_theta(motor_theta[0], 0), mechanism_theta(motor_theta[1], 1)};
  const Eigen::Vector3d a[2] = {params_.a1, params_.a2};
  const Eigen::Vector3d p[2] = {params_.p1, params_.p2};
  const Eigen::Vector3d c[2] = {params_.c1, params_.c2};
  Eigen::Matrix2d pose_constraints;
  Eigen::Vector2d motor_constraints;
  for (int index = 0; index < 2; ++index) {
    const Eigen::Vector3d bhat = b_hat(a[index], p[index], mechanism_thetas[index]);
    const Eigen::Vector3d chat = c_hat(c[index], pitch, roll);
    Eigen::Vector3d unused_pitch;
    Eigen::Vector3d d_pitch;
    Eigen::Vector3d d_roll;
    c_hat_derivatives(c[index], pitch, roll, unused_pitch, d_pitch, d_roll);
    const Eigen::Vector3d separation = bhat - chat;
    pose_constraints(index, 0) = -2.0 * separation.dot(d_pitch);
    pose_constraints(index, 1) = -2.0 * separation.dot(d_roll);
    const Eigen::Vector3d db = db_hat_dtheta(p[index], mechanism_thetas[index]);
    const double direction = index == 0 ? params_.direction1 : params_.direction2;
    motor_constraints[index] = 2.0 * separation.dot(direction * db);
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
  if (!jacobian.allFinite()) {
    error_message = "Pose Jacobian is non-finite";
    return false;
  }
  error_message.clear();
  return true;
}

bool ParallelKinematics::joint_to_motor_velocities(
  const double pitch,
  const double roll,
  const double motor_1,
  const double motor_2,
  const double pitch_velocity,
  const double roll_velocity,
  double & motor_1_velocity,
  double & motor_2_velocity,
  std::string & error_message) const
{
  if (waist_loaded_) {
    return waist_solver_->joint_to_motor_velocities(
      pitch, roll, motor_1, motor_2, pitch_velocity, roll_velocity,
      motor_1_velocity, motor_2_velocity, error_message);
  }
  const double theta1 = clamp_theta(motor_1, 0);
  const double theta2 = clamp_theta(motor_2, 1);
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
  if (!result.allFinite()) {
    error_message = "Motor velocity mapping is non-finite";
    return false;
  }
  motor_1_velocity = result[0];
  motor_2_velocity = result[1];
  error_message.clear();
  return true;
}

bool ParallelKinematics::joint_to_motor_efforts(
  const double pitch,
  const double roll,
  const double motor_1,
  const double motor_2,
  const double pitch_effort,
  const double roll_effort,
  double & motor_1_effort,
  double & motor_2_effort,
  std::string & error_message) const
{
  if (waist_loaded_) {
    return waist_solver_->joint_to_motor_efforts(
      pitch, roll, motor_1, motor_2, pitch_effort, roll_effort,
      motor_1_effort, motor_2_effort, error_message);
  }
  const double theta1 = clamp_theta(motor_1, 0);
  const double theta2 = clamp_theta(motor_2, 1);
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
  if (!result.allFinite()) {
    error_message = "Motor torque mapping is non-finite";
    return false;
  }
  motor_1_effort = result[0];
  motor_2_effort = result[1];
  error_message.clear();
  return true;
}

bool ParallelKinematics::motor_to_joint_dynamics(
  const double pitch,
  const double roll,
  const double motor_1,
  const double motor_2,
  const double motor_1_velocity,
  const double motor_2_velocity,
  const double motor_1_effort,
  const double motor_2_effort,
  double & pitch_velocity,
  double & roll_velocity,
  double & pitch_effort,
  double & roll_effort,
  std::string & error_message) const
{
  if (waist_loaded_) {
    return waist_solver_->motor_to_joint_dynamics(
      pitch, roll, motor_1, motor_2, motor_1_velocity, motor_2_velocity,
      motor_1_effort, motor_2_effort, pitch_velocity, roll_velocity,
      pitch_effort, roll_effort, error_message);
  }
  const double theta1 = clamp_theta(motor_1, 0);
  const double theta2 = clamp_theta(motor_2, 1);
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
    Eigen::Vector2d(motor_1_velocity, motor_2_velocity);
  const double determinant = jacobian.determinant();
  if (std::abs(determinant) < kSmallValue) {
    error_message = "Joint effort Jacobian is singular";
    return false;
  }
  const Eigen::Vector2d effort = jacobian.transpose().inverse() *
    Eigen::Vector2d(motor_1_effort, motor_2_effort);
  if (!velocity.allFinite() || !effort.allFinite()) {
    error_message = "Joint dynamics mapping is non-finite";
    return false;
  }
  pitch_velocity = velocity[0];
  roll_velocity = velocity[1];
  pitch_effort = effort[0];
  roll_effort = effort[1];
  error_message.clear();
  return true;
}

bool ParallelKinematics::is_loaded() const
{
  return loaded_;
}

}  // namespace jljwholebody_hardware
