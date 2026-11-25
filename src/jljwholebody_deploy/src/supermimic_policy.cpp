#include "jljwholebody_deploy/supermimic_policy.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace jljwholebody_deploy
{
namespace
{

constexpr std::size_t kHistoryLength = 10;

bool is_finite(const double value)
{
  return std::isfinite(value);
}

}  // namespace

SuperMimicPolicy::SuperMimicPolicy(
  const std::filesystem::path & bundle, const std::vector<std::string> & hardware_joints,
  const int inference_threads, const std::filesystem::path & calibration)
{
  const auto profile = YAML::LoadFile((bundle / "profile.yaml").string());
  if (profile["schema_version"].as<int>() != 1 || profile["fps"].as<double>() != kRateHz) {
    throw std::runtime_error("unsupported SuperMimic profile version or frame rate");
  }
  const auto joints = profile["joints"];
  if (!joints.IsSequence() || joints.size() != kJointCount ||
    hardware_joints.size() != kJointCount)
  {
    throw std::runtime_error("SuperMimic requires 29 named logical joints");
  }
  action_scale_ = profile["action_scale"].as<double>();
  action_clip_ = profile["raw_action_clip"].as<float>();
  if (!is_finite(action_scale_) || action_scale_ <= 0.0 ||
    !is_finite(action_clip_) || action_clip_ <= 0.0F)
  {
    throw std::runtime_error("invalid SuperMimic action scale or clip");
  }
  const auto calibration_joints = calibration.empty() ?
    YAML::Node {} : YAML::LoadFile(calibration.string());
  if (!calibration.empty() &&
    (!calibration_joints.IsMap() || calibration_joints.size() != kJointCount))
  {
    throw std::runtime_error("SuperMimic calibration must contain all 29 named joints");
  }
  std::array<bool, kJointCount> mapped {};
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const auto joint = joints[i];
    const auto name = joint["name"].as<std::string>();
    const auto it = std::find(hardware_joints.begin(), hardware_joints.end(), name);
    if (it == hardware_joints.end()) {
      throw std::runtime_error("unknown SuperMimic joint: " + name);
    }
    const auto index = static_cast<std::size_t>(it - hardware_joints.begin());
    double direction = 1.0;
    double offset_rad = 0.0;
    if (!calibration.empty()) {
      const auto entry = calibration_joints[name];
      direction = entry["direction"].as<double>();
      offset_rad = entry["offset_rad"].as<double>();
      if ((direction != 1.0 && direction != -1.0) || !is_finite(offset_rad)) {
        throw std::runtime_error("invalid SuperMimic calibration for " + name);
      }
    }
    const double lower = joint["lower"].as<double>();
    const double upper = joint["upper"].as<double>();
    const double bound_a = offset_rad + direction * lower;
    const double bound_b = offset_rad + direction * upper;
    Joint contract {
      index, joint["default_position"].as<double>(), direction, offset_rad,
      std::min(bound_a, bound_b), std::max(bound_a, bound_b),
      joint["kp"].as<double>(), joint["kd"].as<double>(),
      joint["effort_limit"].as<double>()};
    if (mapped[index] || !is_finite(contract.default_position) ||
      !is_finite(lower) || !is_finite(upper) || lower >= upper ||
      !is_finite(contract.lower) || !is_finite(contract.upper) ||
      contract.lower >= contract.upper ||
      !is_finite(contract.kp) || contract.kp < 0.0 ||
      !is_finite(contract.kd) || contract.kd < 0.0 ||
      !is_finite(contract.effort_limit) || contract.effort_limit <= 0.0)
    {
      throw std::runtime_error("invalid SuperMimic joint contract: " + name);
    }
    mapped[index] = true;
    joints_[i] = contract;
    policy_index_[index] = i;
  }

  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
  const auto goal_path = bundle / "goals.f32";
  const auto bytes = std::filesystem::file_size(goal_path);
  if (bytes == 0 || bytes % (kGoalSize * sizeof(float)) != 0 ||
    bytes / (kGoalSize * sizeof(float)) != profile["frame_count"].as<std::size_t>() ||
    bytes > static_cast<std::uintmax_t>(std::numeric_limits<std::streamsize>::max()))
  {
    throw std::runtime_error("invalid SuperMimic goals.f32 length");
  }
  goals_.resize(bytes / sizeof(float));
  std::ifstream stream(goal_path, std::ios::binary);
  if (!stream.read(reinterpret_cast<char *>(goals_.data()), static_cast<std::streamsize>(bytes)) ||
    !std::all_of(goals_.begin(), goals_.end(), is_finite))
  {
    throw std::runtime_error("could not read finite SuperMimic goal frames");
  }
  runtime_ = std::make_unique<OnnxPolicyRuntime>(
    (bundle / "policy.onnx").string(), kObservationSize, kJointCount, inference_threads, kGoalSize);
}

void SuperMimicPolicy::reset()
{
  previous_action_.fill(0.0F);
  initialized_ = false;
}

const std::vector<double> & SuperMimicPolicy::step(
  const std::vector<double> & positions, const std::vector<double> & velocities,
  const std::array<float, 3> & angular_velocity, const std::array<float, 3> & gravity,
  const std::size_t frame)
{
  if (positions.size() != kJointCount || velocities.size() != kJointCount ||
    frame >= frame_count())
  {
    throw std::invalid_argument("invalid SuperMimic state or reference frame");
  }

  std::array<float, kJointCount> position_relative;
  std::array<float, kJointCount> velocity;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto & joint = joints_[i];
    position_relative[i] =
      static_cast<float>(joint.direction * (positions[joint.hardware_index] - joint.offset_rad)) -
      static_cast<float>(joint.default_position);
    velocity[i] = static_cast<float>(joint.direction * velocities[joint.hardware_index]);
  }

  // Pack all ten samples of each term, from the oldest sample to the newest.
  auto term_start = observation_.begin();
  const auto append_term = [&](const auto & term) {
      const auto term_end = term_start + kHistoryLength * term.size();
      if (initialized_) {
        std::move(term_start + term.size(), term_end, term_start);
        std::copy(term.begin(), term.end(), term_end - term.size());
      } else {
        for (auto sample = term_start; sample != term_end; sample += term.size()) {
          std::copy(term.begin(), term.end(), sample);
        }
      }
      term_start = term_end;
    };
  append_term(position_relative);
  append_term(velocity);
  append_term(angular_velocity);
  append_term(gravity);
  append_term(previous_action_);
  initialized_ = true;

  std::copy_n(goals_.begin() + frame * kGoalSize, kGoalSize, goal_.begin());
  const auto & action = runtime_->infer(observation_, goal_);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto & joint = joints_[i];
    previous_action_[i] = std::clamp(action[i], -action_clip_, action_clip_);
    const double policy_target = joint.default_position + action_scale_ * previous_action_[i];
    targets_[joint.hardware_index] = std::clamp(
      joint.offset_rad + joint.direction * policy_target, joint.lower, joint.upper);
  }
  return targets_;
}

const SuperMimicPolicy::Joint & SuperMimicPolicy::hardware_joint(
  const std::size_t hardware_index) const
{
  return joints_.at(policy_index_.at(hardware_index));
}

std::size_t SuperMimicPolicy::frame_count() const
{
  return goals_.size() / kGoalSize;
}

}  // namespace jljwholebody_deploy
