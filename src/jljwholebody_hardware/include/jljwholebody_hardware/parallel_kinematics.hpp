#ifndef JLJWHOLEBODY_HARDWARE__PARALLEL_KINEMATICS_HPP_
#define JLJWHOLEBODY_HARDWARE__PARALLEL_KINEMATICS_HPP_

#include <Eigen/Dense>

#include <memory>
#include <string>

#include "waist_parallel_solver/waist_parallel_solver.hpp"

namespace jljwholebody_hardware
{

enum class ParallelMechanismType
{
  kAnkle,
  kWaist,
};

struct ParallelAxisLimits
{
  double min {0.0};
  double max {0.0};
};

// Adapter for the measured ankle and waist parallel mechanisms.
// Angles are radians and geometry values in the YAML file are metres.
class ParallelKinematics
{
public:
  bool load_from_yaml(const std::string & yaml_file, std::string & error_message);

  ParallelMechanismType type() const;
  const ParallelAxisLimits & pitch_limits() const;
  const ParallelAxisLimits & roll_limits() const;

  bool inverse(
    double pitch,
    double roll,
    double & motor_1,
    double & motor_2,
    std::string & error_message) const;

  bool forward(
    double motor_1,
    double motor_2,
    double & pitch,
    double & roll,
    std::string & error_message,
    double pitch_seed = 0.0,
    double roll_seed = 0.0) const;

  bool forward(
    double motor_1,
    double motor_2,
    double & limited_motor_1,
    double & limited_motor_2,
    double & pitch,
    double & roll,
    std::string & error_message,
    double pitch_seed = 0.0,
    double roll_seed = 0.0) const;

  // Map logical pose velocity to motor velocity. Motor velocities are never
  // limited by this class.
  bool joint_to_motor_velocities(
    double pitch,
    double roll,
    double motor_1,
    double motor_2,
    double pitch_velocity,
    double roll_velocity,
    double & motor_1_velocity,
    double & motor_2_velocity,
    std::string & error_message) const;

  // Map logical pose torque using tau_theta = J^T tau_pose.
  bool joint_to_motor_efforts(
    double pitch,
    double roll,
    double motor_1,
    double motor_2,
    double pitch_effort,
    double roll_effort,
    double & motor_1_effort,
    double & motor_2_effort,
    std::string & error_message) const;

  bool motor_to_joint_dynamics(
    double pitch,
    double roll,
    double motor_1,
    double motor_2,
    double motor_1_velocity,
    double motor_2_velocity,
    double motor_1_effort,
    double motor_2_effort,
    double & pitch_velocity,
    double & roll_velocity,
    double & pitch_effort,
    double & roll_effort,
    std::string & error_message) const;

private:
  struct MechanismParams
  {
    ParallelMechanismType type {ParallelMechanismType::kAnkle};
    Eigen::Vector3d a1 {Eigen::Vector3d::Zero()};
    Eigen::Vector3d a2 {Eigen::Vector3d::Zero()};
    Eigen::Vector3d b1 {Eigen::Vector3d::Zero()};
    Eigen::Vector3d b2 {Eigen::Vector3d::Zero()};
    Eigen::Vector3d c1 {Eigen::Vector3d::Zero()};
    Eigen::Vector3d c2 {Eigen::Vector3d::Zero()};
    Eigen::Vector3d p1 {Eigen::Vector3d::Zero()};
    Eigen::Vector3d p2 {Eigen::Vector3d::Zero()};
    double rod1 {0.0};
    double rod2 {0.0};
    double offset1 {0.0};
    double offset2 {0.0};
    double direction1 {1.0};
    double direction2 {1.0};
    bool right_side {false};
  };

  static Eigen::Vector3d rotate_x(const Eigen::Vector3d & value, double angle);
  static Eigen::Vector3d rotate_y(const Eigen::Vector3d & value, double angle);
  static Eigen::Vector3d c_hat(const Eigen::Vector3d & value, double pitch, double roll);
  static void c_hat_derivatives(
    const Eigen::Vector3d & value,
    double pitch,
    double roll,
    Eigen::Vector3d & rotated,
    Eigen::Vector3d & d_pitch,
    Eigen::Vector3d & d_roll);
  static Eigen::Vector3d b_hat(
    const Eigen::Vector3d & a,
    const Eigen::Vector3d & p,
    double theta);
  static Eigen::Vector3d db_hat_dtheta(const Eigen::Vector3d & p, double theta);
  static double wrap_to_pi(double value);
  static double quantize(double value);

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
  bool residual_and_jacobian(
    const Eigen::Vector3d & bhat1,
    const Eigen::Vector3d & bhat2,
    double pitch,
    double roll,
    Eigen::Vector2d & residual,
    Eigen::Matrix2d & jacobian) const;
  bool solve_pose_from_thetas(
    double theta1,
    double theta2,
    double pitch_seed,
    double roll_seed,
    double & pitch,
    double & roll,
    std::string & error_message) const;
  bool pose_jacobian(
    double theta1,
    double theta2,
    double pitch,
    double roll,
    Eigen::Matrix2d & jacobian,
    std::string & error_message) const;
  bool is_loaded() const;

  MechanismParams params_ {};
  ParallelAxisLimits pitch_limits_ {-0.5, 0.3};
  ParallelAxisLimits roll_limits_ {-0.2, 0.2};
  // The waist mechanism uses the supplied y-axis solver. Keeping it behind
  // the existing adapter preserves the ankle API used by ros2_control and
  // deployment while making both mechanisms available through one binding.
  std::unique_ptr<waist_parallel::Solver> waist_solver_;
  bool waist_loaded_ {false};
  bool loaded_ {false};
};

}  // namespace jljwholebody_hardware

#endif  // JLJWHOLEBODY_HARDWARE__PARALLEL_KINEMATICS_HPP_
