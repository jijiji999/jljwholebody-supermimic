#ifndef JLJWHOLEBODY_DEPLOY__SUPERMIMIC_POLICY_HPP_
#define JLJWHOLEBODY_DEPLOY__SUPERMIMIC_POLICY_HPP_

#include "jljwholebody_deploy/wholebody_policy.hpp"

#include <filesystem>

namespace jljwholebody_deploy
{

class SuperMimicPolicy
{
public:
  static constexpr std::size_t kJointCount = 29;
  static constexpr std::size_t kObservationSize = 930;
  static constexpr std::size_t kGoalSize = 680;
  static constexpr double kRateHz = 50.0;

  struct Joint
  {
    std::size_t hardware_index;
    double default_position;
    double direction;
    double offset_rad;
    double lower;
    double upper;
    double kp;
    double kd;
    double effort_limit;
  };

  SuperMimicPolicy(
    const std::filesystem::path & bundle, const std::vector<std::string> & hardware_joints,
    int inference_threads, const std::filesystem::path & calibration = {});

  void reset();
  const std::vector<double> & step(
    const std::vector<double> & positions, const std::vector<double> & velocities,
    const std::array<float, 3> & angular_velocity, const std::array<float, 3> & gravity,
    std::size_t frame);

  const Joint & hardware_joint(std::size_t hardware_index) const;
  std::size_t frame_count() const;

private:
  std::vector<float> goals_;
  std::vector<float> goal_ = std::vector<float>(kGoalSize);
  std::vector<float> observation_ = std::vector<float>(kObservationSize);
  std::vector<double> targets_ = std::vector<double>(kJointCount);
  std::array<Joint, kJointCount> joints_;
  std::array<std::size_t, kJointCount> policy_index_;
  std::array<float, kJointCount> previous_action_ {};
  double action_scale_;
  float action_clip_;
  bool initialized_ {false};
  std::unique_ptr<OnnxPolicyRuntime> runtime_;
};

}  // namespace jljwholebody_deploy

#endif  // JLJWHOLEBODY_DEPLOY__SUPERMIMIC_POLICY_HPP_
