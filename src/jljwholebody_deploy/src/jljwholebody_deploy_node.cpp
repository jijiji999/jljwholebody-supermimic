#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pthread.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sched.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sys/mman.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "jljwholebody_deploy/supermimic_policy.hpp"
#include "jljwholebody_deploy/wholebody_policy.hpp"
#include "jljwholebody_hardware/dm_motor_manager.hpp"
#include "jljwholebody_hardware/parallel_kinematics.hpp"

namespace
{

using jljwholebody_hardware::ParallelAxisLimits;
using jljwholebody_hardware::ParallelKinematics;
using jljwholebody_hardware::dm_motor::MotorCommandResult;
using jljwholebody_hardware::dm_motor::MotorConfig;
using jljwholebody_hardware::dm_motor::MotorManager;
using jljwholebody_hardware::dm_motor::MotorState;
using jljwholebody_hardware::dm_motor::NamedMitCommand;
using jljwholebody_deploy::WholebodyObservationHistory;
using jljwholebody_deploy::OnnxPolicyRuntime;
using jljwholebody_deploy::SuperMimicPolicy;
using SteadyClock = std::chrono::steady_clock;

constexpr char kNodeName[] = "jljwholebody_deploy";
constexpr char kParallelControlMode[] = "fixed_motor_position_pd_loco_logical_torque";
constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kGaitPhaseStandCommandThreshold = 0.1;

std::string expand_user_path(const std::string & raw_path)
{
  if (raw_path.empty() || raw_path[0] != '~') {
    return raw_path;
  }

  const char * home = std::getenv("HOME");
  if (home == nullptr) {
    return raw_path;
  }

  if (raw_path.size() == 1U) {
    return std::string(home);
  }
  if (raw_path[1] == '/') {
    return std::string(home) + raw_path.substr(1);
  }
  return raw_path;
}

std::string default_config_path()
{
  return ament_index_cpp::get_package_share_directory("jljwholebody_deploy") +
         "/config/deploy.yaml";
}

std::string default_motor_config_path()
{
  return ament_index_cpp::get_package_share_directory("jljwholebody_hardware") +
         "/config/dm_motor_whole_body.yaml";
}

std::string default_parallel_mechanisms_path()
{
  return ament_index_cpp::get_package_share_directory("jljwholebody_hardware") +
         "/config/parallel_mechanisms.yaml";
}

SteadyClock::duration to_steady_duration(const double seconds)
{
  return std::chrono::duration_cast<SteadyClock::duration>(
    std::chrono::duration<double>(seconds));
}

tf2::Quaternion quaternion_from_rpy_degrees(
  const double roll_deg,
  const double pitch_deg,
  const double yaw_deg)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(
    roll_deg * kDegreesToRadians,
    pitch_deg * kDegreesToRadians,
    yaw_deg * kDegreesToRadians);
  quaternion.normalize();
  return quaternion;
}

std::string format_pose(const double pitch, const double roll)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4)
      << "(pitch=" << pitch << ", roll=" << roll << ")";
  return oss.str();
}

std::string join_names(const std::vector<std::string> & names)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (i > 0U) {
      oss << ", ";
    }
    oss << names[i];
  }
  return oss.str();
}

std::vector<std::string> compute_missing_motors(
  const std::vector<std::string> & expected_names,
  const std::vector<MotorState> & states)
{
  std::vector<std::string> missing = expected_names;
  for (const auto & state : states) {
    missing.erase(
      std::remove(missing.begin(), missing.end(), state.name),
      missing.end());
  }
  return missing;
}

bool has_failures(const std::vector<MotorCommandResult> & results)
{
  return std::any_of(
    results.begin(), results.end(),
    [](const auto & result) {return !result.success;});
}

bool is_parallel_joint_name(const std::string & joint_name)
{
  return joint_name.find("ankle_pitch") != std::string::npos ||
         joint_name.find("ankle_roll") != std::string::npos ||
         joint_name == "waist_pitch" ||
         joint_name == "waist_roll";
}

std::string summarize_failures(const std::vector<MotorCommandResult> & results)
{
  std::ostringstream oss;
  std::size_t failure_count = 0U;
  for (const auto & result : results) {
    if (result.success) {
      continue;
    }
    if (failure_count > 0U) {
      oss << ", ";
    }
    oss << result.name << " [" << result.message << "]";
    ++failure_count;
    if (failure_count >= 6U) {
      break;
    }
  }
  if (failure_count == 0U) {
    return "none";
  }
  return oss.str();
}

std::string read_required_yaml_string(const YAML::Node & node, const char * key)
{
  if (!node[key]) {
    throw std::runtime_error(std::string("Missing required key: ") + key);
  }
  return node[key].as<std::string>();
}

double read_required_yaml_double(const YAML::Node & node, const char * key)
{
  if (!node[key]) {
    throw std::runtime_error(std::string("Missing required key: ") + key);
  }
  return node[key].as<double>();
}

int read_required_yaml_int(const YAML::Node & node, const char * key)
{
  if (!node[key]) {
    throw std::runtime_error(std::string("Missing required key: ") + key);
  }
  return node[key].as<int>();
}

std::string resolve_config_path(
  const std::string & parent_file_path,
  const std::string & config_path)
{
  namespace fs = std::filesystem;

  if (config_path.empty()) {
    return config_path;
  }

  const fs::path path(expand_user_path(config_path));
  if (path.is_absolute()) {
    return path.lexically_normal().string();
  }

  return (fs::path(parent_file_path).parent_path() / path).lexically_normal().string();
}

double clamp_with_limits(
  const double value,
  const ParallelAxisLimits & limits,
  bool & clamped)
{
  if (value < limits.min) {
    clamped = true;
    return limits.min;
  }
  if (value > limits.max) {
    clamped = true;
    return limits.max;
  }
  return value;
}

double clamp_motor_position(const double value, const MotorConfig & config, bool & clamped)
{
  if (!config.has_position_limit) {
    return value;
  }

  if (value < config.lower_position_limit) {
    clamped = true;
    return static_cast<double>(config.lower_position_limit);
  }
  if (value > config.upper_position_limit) {
    clamped = true;
    return static_cast<double>(config.upper_position_limit);
  }
  return value;
}

enum class Mode
{
  kPassive,
  kFixed,
  kLoco,
  kDanc,
};

enum class PolicyBackend
{
  kOnnx,
  kTrajectory,
};

std::string to_string(const Mode mode)
{
  switch (mode) {
    case Mode::kPassive:
      return "passive";
    case Mode::kFixed:
      return "fixed";
    case Mode::kLoco:
      return "loco";
    case Mode::kDanc:
      return "danc";
    default:
      return "unknown";
  }
}

Mode mode_from_string(const std::string & raw_value)
{
  if (raw_value == "passive") {
    return Mode::kPassive;
  }
  if (raw_value == "fixed") {
    return Mode::kFixed;
  }
  if (raw_value == "loco") {
    return Mode::kLoco;
  }
  throw std::runtime_error("Unsupported startup mode: " + raw_value);
}

std::string to_string(const PolicyBackend backend)
{
  switch (backend) {
    case PolicyBackend::kOnnx:
      return "onnx";
    case PolicyBackend::kTrajectory:
      return "trajectory";
    default:
      return "unknown";
  }
}

PolicyBackend policy_backend_from_string(const std::string & raw_value)
{
  if (raw_value == "onnx") {
    return PolicyBackend::kOnnx;
  }
  if (raw_value == "trajectory") {
    return PolicyBackend::kTrajectory;
  }
  throw std::runtime_error("Unsupported policy backend: " + raw_value);
}

enum class TrajectorySource
{
  kProfiles,
  kCsvPlayback,
  kSquat,
};

std::string to_string(const TrajectorySource source)
{
  switch (source) {
    case TrajectorySource::kProfiles:
      return "profiles";
    case TrajectorySource::kCsvPlayback:
      return "csv_playback";
    case TrajectorySource::kSquat:
      return "squat";
    default:
      return "unknown";
  }
}

TrajectorySource trajectory_source_from_string(const std::string & raw_value)
{
  if (raw_value == "profiles") {
    return TrajectorySource::kProfiles;
  }
  if (raw_value == "csv_playback") {
    return TrajectorySource::kCsvPlayback;
  }
  if (raw_value == "squat") {
    return TrajectorySource::kSquat;
  }
  throw std::runtime_error("Unsupported trajectory source: " + raw_value);
}

enum class TrajectoryCommandSource
{
  kSimulationActual,
  kSimulationTarget,
};

std::string to_string(const TrajectoryCommandSource source)
{
  switch (source) {
    case TrajectoryCommandSource::kSimulationActual:
      return "simulation_actual";
    case TrajectoryCommandSource::kSimulationTarget:
      return "simulation_target";
    default:
      return "unknown";
  }
}

TrajectoryCommandSource trajectory_command_source_from_string(const std::string & raw_value)
{
  if (raw_value == "simulation_actual") {
    return TrajectoryCommandSource::kSimulationActual;
  }
  if (raw_value == "simulation_target") {
    return TrajectoryCommandSource::kSimulationTarget;
  }
  throw std::runtime_error("Unsupported trajectory playback command_source: " + raw_value);
}

double smoothstep01(const double value)
{
  const double x = std::clamp(value, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

std::vector<double> add_vectors(
  const std::vector<double> & lhs,
  const std::vector<double> & rhs)
{
  std::vector<double> result(lhs.size(), 0.0);
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    result[i] = lhs[i] + rhs[i];
  }
  return result;
}

std::vector<double> blend_vectors(
  const std::vector<double> & from,
  const std::vector<double> & to,
  const double alpha)
{
  const double clamped_alpha = std::clamp(alpha, 0.0, 1.0);
  std::vector<double> result(from.size(), 0.0);
  for (std::size_t i = 0; i < from.size(); ++i) {
    result[i] = from[i] + (to[i] - from[i]) * clamped_alpha;
  }
  return result;
}

std::string trim_copy(const std::string & value)
{
  const auto first = std::find_if_not(
    value.begin(),
    value.end(),
    [](const unsigned char ch) {return std::isspace(ch) != 0;});
  const auto last = std::find_if_not(
    value.rbegin(),
    value.rend(),
    [](const unsigned char ch) {return std::isspace(ch) != 0;}).base();

  if (first >= last) {
    return {};
  }
  return std::string(first, last);
}

std::vector<std::string> split_csv_line(const std::string & line)
{
  std::vector<std::string> fields;
  std::string field;
  bool in_quotes = false;

  for (std::size_t i = 0; i < line.size(); ++i) {
    const char ch = line[i];
    if (ch == '"') {
      if (in_quotes && i + 1U < line.size() && line[i + 1U] == '"') {
        field.push_back('"');
        ++i;
      } else {
        in_quotes = !in_quotes;
      }
      continue;
    }

    if (ch == ',' && !in_quotes) {
      fields.push_back(trim_copy(field));
      field.clear();
      continue;
    }

    field.push_back(ch);
  }

  fields.push_back(trim_copy(field));
  return fields;
}

double parse_csv_double(
  const std::string & text,
  const std::size_t row_number,
  const std::string & column_name)
{
  const std::string trimmed = trim_copy(text);
  if (trimmed.empty()) {
    throw std::runtime_error(
            "CSV row " + std::to_string(row_number) + " has an empty value for " + column_name);
  }

  std::size_t parsed_chars = 0U;
  const double value = std::stod(trimmed, &parsed_chars);
  if (parsed_chars != trimmed.size()) {
    throw std::runtime_error(
            "CSV row " + std::to_string(row_number) + " has a non-numeric value for " +
            column_name + ": " + trimmed);
  }
  if (!std::isfinite(value)) {
    throw std::runtime_error(
            "CSV row " + std::to_string(row_number) + " has a non-finite value for " +
            column_name);
  }
  return value;
}

bool all_near_zero(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(),
    values.end(),
    [](const double value) {return std::abs(value) < 1e-12;});
}

std::vector<std::string> csv_target_column_candidates(
  const std::string & target_prefix,
  const std::string & joint_name,
  const std::string & joint_suffix)
{
  std::vector<std::string> candidates;
  candidates.push_back(target_prefix + joint_name);

  if (!joint_suffix.empty()) {
    candidates.push_back(target_prefix + joint_name + joint_suffix);
    if (joint_name.size() > joint_suffix.size() &&
      joint_name.compare(
        joint_name.size() - joint_suffix.size(), joint_suffix.size(),
        joint_suffix) == 0)
    {
      candidates.push_back(
        target_prefix + joint_name.substr(0, joint_name.size() - joint_suffix.size()));
    }
  }

  return candidates;
}

std::string make_timestamp_string()
{
  const auto now = std::chrono::system_clock::now();
  const auto now_time_t = std::chrono::system_clock::to_time_t(now);
  std::tm now_tm {};
  localtime_r(&now_time_t, &now_tm);

  std::ostringstream stream;
  stream << std::put_time(&now_tm, "%Y-%m-%d_%H-%M-%S");
  return stream.str();
}

std::string make_wall_time_string()
{
  const auto now = std::chrono::system_clock::now();
  const auto now_time_t = std::chrono::system_clock::to_time_t(now);
  std::tm now_tm {};
  localtime_r(&now_time_t, &now_tm);

  std::ostringstream stream;
  stream << std::put_time(&now_tm, "%F %T");
  return stream.str();
}

std::string escape_json(const std::string & text)
{
  std::ostringstream stream;
  for (const char ch : text) {
    switch (ch) {
      case '\\':
        stream << "\\\\";
        break;
      case '"':
        stream << "\\\"";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        stream << ch;
        break;
    }
  }
  return stream.str();
}

std::string escape_html(const std::string & text)
{
  std::ostringstream stream;
  for (const char ch : text) {
    switch (ch) {
      case '&':
        stream << "&amp;";
        break;
      case '<':
        stream << "&lt;";
        break;
      case '>':
        stream << "&gt;";
        break;
      case '"':
        stream << "&quot;";
        break;
      default:
        stream << ch;
        break;
    }
  }
  return stream.str();
}

std::string escape_csv(const std::string & text)
{
  const bool needs_quotes = text.find_first_of(",\"\n\r") != std::string::npos;
  if (!needs_quotes) {
    return text;
  }

  std::string escaped;
  escaped.reserve(text.size() + 2U);
  escaped.push_back('"');
  for (const char ch : text) {
    if (ch == '"') {
      escaped.push_back('"');
    }
    escaped.push_back(ch);
  }
  escaped.push_back('"');
  return escaped;
}

std::string format_double(const double value, const int precision = 9)
{
  if (!std::isfinite(value)) {
    return "";
  }

  std::ostringstream stream;
  stream << std::setprecision(precision) << value;
  return stream.str();
}

void write_csv_row(std::ostream & stream, const std::vector<std::string> & fields)
{
  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (i > 0U) {
      stream << ',';
    }
    stream << escape_csv(fields[i]);
  }
  stream << '\n';
}

struct LogicalPose
{
  double pitch {0.0};
  double roll {0.0};
};

struct MotorGainOverride
{
  std::optional<double> kp;
  std::optional<double> kd;
};

struct ModeGains
{
  double direct_kp {0.0};
  double direct_kd {0.0};
  double ankle_kp {0.0};
  double ankle_kd {0.0};
  double transition_duration_s {0.0};
  std::unordered_map<std::string, MotorGainOverride> motor_gains;
  bool follow_feedback_target {false};
  bool hold_current_pose_on_entry {false};
};

struct LoopConfig
{
  double control_rate_hz {1000.0};
  double policy_rate_hz {50.0};
  double state_publish_rate_hz {100.0};
  double startup_state_timeout_s {3.0};
  double startup_feedback_warmup_s {0.5};
  double joy_timeout_s {0.5};
  double imu_timeout_s {0.2};
  double max_motor_feedback_age_s {0.03};
  double max_tilt_deg {35.0};
  int feedback_poll_timeout_ms {1};
  int command_timeout_ms {0};
  int lifecycle_timeout_ms {100};
  int enable_retries {3};
  int enable_retry_interval_ms {200};
  bool enable_on_start {true};
  bool disable_on_exit {true};
};

struct RealtimeConfig
{
  bool enabled {true};
  bool strict {false};
  bool lock_memory {true};
  int control_priority {45};
  int policy_priority {35};
  int control_cpu {-1};
  // -2 selects the last online CPU, -1 disables affinity.
  int policy_cpu {-2};
};

struct TopicsConfig
{
  std::string joy {"/joy"};
  std::string imu {"/imu/data"};
  std::string joint_states {"/deploy/joint_states"};
  std::string projected_gravity {"/deploy/projected_gravity"};
  std::string angular_velocity {"/deploy/base_ang_vel"};
  std::string mode {"/deploy/mode"};
  std::string errors {"/deploy/errors"};
  std::string status {"/deploy/status"};
};

struct ModeSwitchConfig
{
  int a_button_index {0};
  int danc_button_index {2};
  int lb_button_index {4};
  int rb_button_index {5};
  int lt_axis_index {4};
  double lt_pressed_threshold {0.5};
};

struct JoystickCommandConfig
{
  int lin_x_axis_index {1};
  double lin_x_sign {-1.0};
  double lin_x_scale {1.0};
  int lin_y_axis_index {0};
  double lin_y_sign {-1.0};
  double lin_y_scale {1.0};
  int yaw_axis_index {2};
  double yaw_sign {-1.0};
  double yaw_scale {0.5};
};

struct JoystickRecordingConfig
{
  int dpad_y_axis_index {7};
  double up_pressed_threshold {-0.5};
  double down_pressed_threshold {0.5};
};

struct JoystickConfig
{
  ModeSwitchConfig mode_switch {};
  JoystickCommandConfig command {};
  JoystickRecordingConfig recording {};
};

struct PolicyConfig
{
  PolicyBackend backend {PolicyBackend::kOnnx};
  std::string model_path;
  int expected_observation_dim {490};
  int expected_action_dim {29};
  int observation_history_length {5};
  int inference_threads {1};
  double raw_action_clip {5.0};
  double gait_period_s {0.8};
  double takeover_duration_s {2.0};
  bool shadow_mode {false};
  std::string profile_path;
  std::vector<std::string> joint_names;
  std::vector<double> default_joint_positions;
  std::vector<double> action_scales;
  std::vector<std::pair<double, double>> joint_position_limits;
  struct ActuatorContract
  {
    double kp {0.0};
    double kd {0.0};
    double effort_limit {std::numeric_limits<double>::infinity()};
    double velocity_limit {std::numeric_limits<double>::infinity()};
    bool valid {false};
  };
  std::vector<ActuatorContract> actuators;
};

struct ImuMountConfig
{
  double roll_offset_deg {0.0};
  double pitch_offset_deg {0.0};
  double yaw_offset_deg {0.0};
};

struct ImuFilterConfig
{
  bool enabled {false};
  double cutoff_hz {20.0};
  double reset_timeout_s {0.1};
};

struct ParallelControlConfig
{
  double pitch_kp {20.0};
  double pitch_kd {5.0};
  double roll_kp {20.0};
  double roll_kd {5.0};
};

struct TrajectoryJointProfile
{
  std::string joint_name;
  std::size_t joint_index {0U};
  double offset {0.0};
  double amplitude {0.0};
  double frequency_hz {1.0};
  double phase_rad {0.0};
};

struct TrajectoryPlaybackSample
{
  double time_s {0.0};
  std::vector<double> positions;
  std::vector<double> reference_positions;
};

struct TrajectoryPlaybackConfig
{
  std::string csv_path;
  std::string time_column {"time_s"};
  std::string target_prefix {"joint_pos_target/"};
  std::string reference_prefix {"joint_pos/"};
  std::string joint_suffix {"_joint"};
  std::array<double, 3> command {0.0, 0.0, 0.0};
  bool command_valid {false};
  bool loop {false};
  bool interpolate {false};
  bool skip_initial_zero_targets {true};
  bool reference_valid {false};
  TrajectoryCommandSource command_source {TrajectoryCommandSource::kSimulationActual};
  double speed_scale {1.0};
  double duration_s {0.0};
  std::vector<TrajectoryPlaybackSample> samples;
};

struct TrajectorySquatConfig
{
  double prepare_duration_s {5.0};
  double prep_hold_s {1.0};
  double cycle_period_s {5.0};
  int cycle_count {3};
  double return_duration_s {5.0};
  bool return_to_default {true};
  std::vector<double> preparation_offsets;
  std::vector<double> squat_offsets;
};

struct TrajectoryConfig
{
  TrajectorySource source {TrajectorySource::kProfiles};
  double startup_hold_s {2.0};
  std::vector<TrajectoryJointProfile> joint_profiles;
  TrajectoryPlaybackConfig playback {};
  TrajectorySquatConfig squat {};
};

struct TrackingRecordingConfig
{
  bool enabled {false};
  std::string output_root {"log/jljwholebody_deploy"};
  double sample_rate_hz {50.0};
  bool record_only_loco {true};
  bool write_html {true};
  bool write_csv {true};
  bool write_readme {true};
  bool write_summary {true};
};

struct DeploymentConfig
{
  std::string config_path;
  std::string motor_config_path;
  std::string parallel_mechanisms_path;
  std::string error_log_path;
  TopicsConfig topics {};
  LoopConfig loop {};
  RealtimeConfig realtime {};
  ImuMountConfig imu_mount {};
  ImuFilterConfig imu_filter {};
  ParallelControlConfig parallel_control {};
  JoystickConfig joystick {};
  Mode startup_mode {Mode::kPassive};
  ModeGains passive_gains {};
  ModeGains fixed_gains {};
  ModeGains loco_gains {};
  PolicyConfig policy {};
  std::string danc_bundle_path;
  std::string danc_joint_calibration_path;
  double danc_takeover_duration_s {1.0};
  bool danc_shadow_mode {true};
  TrajectoryConfig trajectory {};
  TrackingRecordingConfig recording {};
};

struct JoySnapshot
{
  std::vector<float> axes;
  std::vector<int32_t> buttons;
  SteadyClock::time_point stamp {};
  bool valid {false};
};

struct ImuSnapshot
{
  tf2::Quaternion orientation;
  std::array<double, 3> angular_velocity {0.0, 0.0, 0.0};
  SteadyClock::time_point stamp {};
  bool valid {false};
};

struct LogicalJointState
{
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  SteadyClock::time_point stamp {};
  bool valid {false};
};

struct TrackingSample
{
  std::size_t sample_index {0U};
  double time_s {0.0};
  std::array<double, 3> command {0.0, 0.0, 0.0};
  std::array<double, 3> projected_gravity {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()
  };
  std::array<double, 3> angular_velocity {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()
  };
  std::vector<float> policy_observation;
  std::vector<float> raw_action;
  std::vector<double> joint_positions;
  std::vector<double> joint_position_targets;
  std::vector<double> simulation_joint_positions;
  std::vector<double> joint_velocities;
  std::vector<double> joint_velocity_targets;
  std::vector<double> joint_torques;
  std::vector<double> joint_torque_targets;
};

struct TrackingRun
{
  std::size_t run_index {0U};
  std::string run_id;
  bool shadow_mode {false};
  std::string start_wall_time;
  std::string stop_wall_time;
  std::string start_reason;
  std::string stop_reason;
  SteadyClock::time_point start_time {};
  SteadyClock::time_point stop_time {};
  std::vector<std::string> joint_names;
  std::vector<TrackingSample> samples;
};

enum class TrackingPlotKind
{
  kPositionCompare,
  kVelocityCompare,
  kTorqueCompare,
  kPositionError,
  kSimulationPositionError,
};

struct DirectJointBinding
{
  std::size_t joint_index {0U};
  std::string joint_name;
  std::string motor_name;
  MotorConfig motor_config {};
};

struct ParallelBinding
{
  std::string name;
  std::string pitch_joint_name;
  std::string roll_joint_name;
  std::size_t pitch_joint_index {0U};
  std::size_t roll_joint_index {0U};
  std::string motor_1_name;
  std::string motor_2_name;
  MotorConfig motor_1_config {};
  MotorConfig motor_2_config {};
  ParallelKinematics kinematics {};
  LogicalPose last_forward_pose {};
  bool last_forward_pose_valid {false};
};

struct RuntimeStats
{
  SteadyClock::time_point start_time {SteadyClock::now()};
  uint64_t control_iterations {0U};
  uint64_t control_deadline_misses {0U};
  uint64_t control_cycles_with_any_fresh_feedback {0U};
  uint64_t control_cycles_with_full_fresh_feedback {0U};
  uint64_t control_fresh_feedback_count_sum {0U};
  double control_compute_ms_sum {0.0};
  double control_compute_ms_max {0.0};
  uint64_t policy_iterations {0U};
  uint64_t policy_deadline_misses {0U};
  double policy_compute_ms_sum {0.0};
  double policy_compute_ms_max {0.0};
  uint64_t joy_messages {0U};
  uint64_t imu_messages {0U};
  uint64_t fault_count {0U};
  double joy_interval_ms_max {0.0};
  double imu_interval_ms_max {0.0};
  SteadyClock::time_point last_joy_callback_time {};
  SteadyClock::time_point last_imu_callback_time {};
};

class ErrorLog
{
public:
  explicit ErrorLog(const std::string & log_path)
  {
    if (log_path.empty()) {
      return;
    }

    const std::filesystem::path path(expand_user_path(log_path));
    std::filesystem::create_directories(path.parent_path());
    stream_.open(path, std::ios::app);
  }

  void write(const std::string & level, const std::string & message)
  {
    if (!stream_.is_open()) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::system_clock::now();
    const auto now_time_t = std::chrono::system_clock::to_time_t(now);
    stream_ << std::put_time(std::localtime(&now_time_t), "%F %T")
            << " [" << level << "] " << message << '\n';
    stream_.flush();
  }

private:
  std::mutex mutex_;
  std::ofstream stream_;
};

class JljWholebodyDeployNode : public rclcpp::Node
{
public:
  JljWholebodyDeployNode()
  : Node(kNodeName)
  {
    declare_parameter<std::string>("config_path", "");
    declare_parameter<bool>("validate_config_only", false);
  }

  ~JljWholebodyDeployNode() override
  {
    shutdown_runtime();
  }

  void start()
  {
    load_config();
    initialize_danc_policy();
    if (get_parameter("validate_config_only").as_bool()) {
      initialize_policy_runtime(true);
      RCLCPP_INFO(
        get_logger(),
        "Configuration and native ONNX validation passed; hardware was not opened.");
      policy_runtime_.reset();
      observation_history_.reset();
      validation_complete_ = true;
      return;
    }
    imu_mount_offset_ = quaternion_from_rpy_degrees(
      config_.imu_mount.roll_offset_deg,
      config_.imu_mount.pitch_offset_deg,
      config_.imu_mount.yaw_offset_deg);
    error_log_ = std::make_unique<ErrorLog>(config_.error_log_path);

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      config_.topics.joint_states,
      rclcpp::QoS(10));
    mode_pub_ = create_publisher<std_msgs::msg::String>(
      config_.topics.mode,
      rclcpp::QoS(1).transient_local());
    error_pub_ = create_publisher<std_msgs::msg::String>(
      config_.topics.errors,
      rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(
      config_.topics.status,
      rclcpp::QoS(10));
    projected_gravity_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(
      config_.topics.projected_gravity,
      rclcpp::QoS(10));
    angular_velocity_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(
      config_.topics.angular_velocity,
      rclcpp::QoS(10));

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      config_.topics.joy,
      rclcpp::QoS(10),
      std::bind(&JljWholebodyDeployNode::on_joy, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      config_.topics.imu,
      rclcpp::SensorDataQoS(),
      std::bind(&JljWholebodyDeployNode::on_imu, this, std::placeholders::_1));

    motor_manager_ = std::make_unique<MotorManager>(config_.motor_config_path);
    motor_names_ = motor_manager_->motor_names();
    motor_configs_ = motor_manager_->motor_configs();
    if (motor_names_.empty()) {
      throw std::runtime_error("Motor config is empty, nothing to deploy.");
    }

    build_joint_index();
    load_parallel_bindings();
    load_direct_bindings();

    if (config_.loop.enable_on_start) {
      enable_motors_on_start();
    }

    if (!capture_initial_state()) {
      throw std::runtime_error("Failed to capture initial motor state.");
    }

    fixed_hold_positions_ = config_.policy.default_joint_positions;
    latest_policy_targets_ = fixed_hold_positions_;
    latest_commanded_targets_ = fixed_hold_positions_;
    latest_commanded_velocities_.assign(config_.policy.joint_names.size(), 0.0);
    latest_commanded_torques_.assign(config_.policy.joint_names.size(), 0.0);
    last_policy_raw_action_.assign(config_.policy.joint_names.size(), 0.0F);
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      runtime_stats_ = RuntimeStats {};
      runtime_stats_.start_time = SteadyClock::now();
      last_status_stats_ = runtime_stats_;
      last_status_publish_time_ = runtime_stats_.start_time;
    }
    transition_to_mode(config_.startup_mode, "startup");

    initialize_policy_runtime();
    lock_process_memory();

    stop_requested_ = false;
    if (config_.recording.enabled) {
      recording_writer_stop_requested_ = false;
      recording_writer_thread_ = std::thread(&JljWholebodyDeployNode::recording_writer_loop, this);
    }
    control_thread_ = std::thread(&JljWholebodyDeployNode::control_loop, this);
    policy_thread_ = std::thread(&JljWholebodyDeployNode::policy_loop, this);

    if (config_.loop.state_publish_rate_hz > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / config_.loop.state_publish_rate_hz);
      state_publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&JljWholebodyDeployNode::publish_joint_state, this));
    }
    status_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&JljWholebodyDeployNode::publish_status, this));

    if (config_.recording.enabled) {
      RCLCPP_INFO(
        get_logger(),
        "Tracking recording enabled: root=%s sample_rate=%.1f Hz record_only_loco=%s html=%s; "
        "D-pad Up starts and D-pad Down stops in loco mode.",
        config_.recording.output_root.c_str(),
        config_.recording.sample_rate_hz,
        config_.recording.record_only_loco ? "true" : "false",
        config_.recording.write_html ? "true" : "false");
    }

    RCLCPP_INFO(
      get_logger(),
      "Deployment node started. Control %.1f Hz, policy %.1f Hz, backend %s, shadow=%s, "
      "startup mode %s, parallel-joint control %s.",
      config_.loop.control_rate_hz,
      config_.loop.policy_rate_hz,
      to_string(config_.policy.backend).c_str(),
      config_.policy.shadow_mode ? "true" : "false",
      to_string(config_.startup_mode).c_str(),
      kParallelControlMode);
    RCLCPP_INFO(
      get_logger(),
      "IMU mount compensation applied: roll %.1f deg, pitch %.1f deg, yaw %.1f deg.",
      config_.imu_mount.roll_offset_deg,
      config_.imu_mount.pitch_offset_deg,
      config_.imu_mount.yaw_offset_deg);
    if (config_.imu_filter.enabled) {
      RCLCPP_INFO(
        get_logger(),
        "IMU low-pass filter enabled: cutoff %.2f Hz, reset timeout %.3f s.",
        config_.imu_filter.cutoff_hz,
        config_.imu_filter.reset_timeout_s);
    } else {
      RCLCPP_INFO(get_logger(), "IMU low-pass filter disabled.");
    }
    if (config_.policy.backend == PolicyBackend::kTrajectory) {
      RCLCPP_INFO(
        get_logger(),
        "Trajectory source: %s.",
        to_string(config_.trajectory.source).c_str());
    }
  }

  bool validation_complete() const
  {
    return validation_complete_;
  }

private:
  void load_config()
  {
    config_.config_path = get_parameter("config_path").as_string();
    if (config_.config_path.empty()) {
      config_.config_path = default_config_path();
    }
    config_.config_path = expand_user_path(config_.config_path);

    const YAML::Node root = YAML::LoadFile(config_.config_path);
    const YAML::Node deploy = root["deployment"];
    if (!deploy) {
      throw std::runtime_error("Config file must contain a 'deployment' root node.");
    }

    config_.motor_config_path = deploy["motor_config_path"] ?
      resolve_config_path(config_.config_path, deploy["motor_config_path"].as<std::string>()) :
      default_motor_config_path();
    if (config_.motor_config_path.empty()) {
      config_.motor_config_path = default_motor_config_path();
    }

    config_.parallel_mechanisms_path = deploy["parallel_mechanisms_path"] ?
      resolve_config_path(
      config_.config_path,
      deploy["parallel_mechanisms_path"].as<std::string>()) :
      default_parallel_mechanisms_path();
    if (config_.parallel_mechanisms_path.empty()) {
      config_.parallel_mechanisms_path = default_parallel_mechanisms_path();
    }

    config_.error_log_path = deploy["error_log_path"] ?
      deploy["error_log_path"].as<std::string>() :
      "~/.jljwholebody/deploy_errors.log";

    const auto topics = deploy["topics"];
    if (topics) {
      config_.topics.joy = topics["joy"] ? topics["joy"].as<std::string>() : config_.topics.joy;
      config_.topics.imu = topics["imu"] ? topics["imu"].as<std::string>() : config_.topics.imu;
      config_.topics.joint_states = topics["joint_states"] ?
        topics["joint_states"].as<std::string>() : config_.topics.joint_states;
      config_.topics.projected_gravity = topics["projected_gravity"] ?
        topics["projected_gravity"].as<std::string>() : config_.topics.projected_gravity;
      config_.topics.angular_velocity = topics["angular_velocity"] ?
        topics["angular_velocity"].as<std::string>() : config_.topics.angular_velocity;
      config_.topics.mode = topics["mode"] ? topics["mode"].as<std::string>() : config_.topics.mode;
      config_.topics.errors = topics["errors"] ?
        topics["errors"].as<std::string>() : config_.topics.errors;
      config_.topics.status = topics["status"] ?
        topics["status"].as<std::string>() : config_.topics.status;
    }

    const auto loop = deploy["loop"];
    if (loop) {
      config_.loop.control_rate_hz = loop["control_rate_hz"] ?
        loop["control_rate_hz"].as<double>() : config_.loop.control_rate_hz;
      config_.loop.policy_rate_hz = loop["policy_rate_hz"] ?
        loop["policy_rate_hz"].as<double>() : config_.loop.policy_rate_hz;
      config_.loop.state_publish_rate_hz = loop["state_publish_rate_hz"] ?
        loop["state_publish_rate_hz"].as<double>() : config_.loop.state_publish_rate_hz;
      config_.loop.startup_state_timeout_s = loop["startup_state_timeout_s"] ?
        loop["startup_state_timeout_s"].as<double>() : config_.loop.startup_state_timeout_s;
      config_.loop.startup_feedback_warmup_s = loop["startup_feedback_warmup_s"] ?
        loop["startup_feedback_warmup_s"].as<double>() : config_.loop.startup_feedback_warmup_s;
      config_.loop.joy_timeout_s = loop["joy_timeout_s"] ?
        loop["joy_timeout_s"].as<double>() : config_.loop.joy_timeout_s;
      config_.loop.imu_timeout_s = loop["imu_timeout_s"] ?
        loop["imu_timeout_s"].as<double>() : config_.loop.imu_timeout_s;
      config_.loop.max_motor_feedback_age_s = loop["max_motor_feedback_age_s"] ?
        loop["max_motor_feedback_age_s"].as<double>() : config_.loop.max_motor_feedback_age_s;
      config_.loop.max_tilt_deg = loop["max_tilt_deg"] ?
        loop["max_tilt_deg"].as<double>() : config_.loop.max_tilt_deg;
      config_.loop.feedback_poll_timeout_ms = loop["feedback_poll_timeout_ms"] ?
        loop["feedback_poll_timeout_ms"].as<int>() : config_.loop.feedback_poll_timeout_ms;
      config_.loop.command_timeout_ms = loop["command_timeout_ms"] ?
        loop["command_timeout_ms"].as<int>() : config_.loop.command_timeout_ms;
      config_.loop.lifecycle_timeout_ms = loop["lifecycle_timeout_ms"] ?
        loop["lifecycle_timeout_ms"].as<int>() : config_.loop.lifecycle_timeout_ms;
      config_.loop.enable_retries = loop["enable_retries"] ?
        loop["enable_retries"].as<int>() : config_.loop.enable_retries;
      config_.loop.enable_retry_interval_ms = loop["enable_retry_interval_ms"] ?
        loop["enable_retry_interval_ms"].as<int>() : config_.loop.enable_retry_interval_ms;
      config_.loop.enable_on_start = loop["enable_on_start"] ?
        loop["enable_on_start"].as<bool>() : config_.loop.enable_on_start;
      config_.loop.disable_on_exit = loop["disable_on_exit"] ?
        loop["disable_on_exit"].as<bool>() : config_.loop.disable_on_exit;
    }

    if (config_.loop.control_rate_hz <= 0.0 || config_.loop.policy_rate_hz <= 0.0 ||
      config_.loop.state_publish_rate_hz < 0.0)
    {
      throw std::runtime_error("loop rates must be positive, except state_publish_rate_hz may be 0");
    }
    if (config_.loop.startup_state_timeout_s < 0.0 ||
      config_.loop.startup_feedback_warmup_s < 0.0 ||
      config_.loop.joy_timeout_s < 0.0 ||
      config_.loop.imu_timeout_s < 0.0)
    {
      throw std::runtime_error("loop timeouts must be >= 0");
    }
    if (config_.loop.max_motor_feedback_age_s <= 0.0 ||
      config_.loop.max_tilt_deg <= 0.0 || config_.loop.max_tilt_deg >= 90.0)
    {
      throw std::runtime_error(
              "loop.max_motor_feedback_age_s must be > 0 and max_tilt_deg must be in (0, 90)");
    }
    if (config_.loop.feedback_poll_timeout_ms < 0 ||
      config_.loop.command_timeout_ms < 0 ||
      config_.loop.lifecycle_timeout_ms < 0 ||
      config_.loop.enable_retries < 1 ||
      config_.loop.enable_retry_interval_ms < 0)
    {
      throw std::runtime_error(
              "loop millisecond timeouts must be >= 0 and enable_retries must be >= 1");
    }

    const auto realtime = deploy["realtime"];
    if (realtime) {
      config_.realtime.enabled = realtime["enabled"] ?
        realtime["enabled"].as<bool>() : config_.realtime.enabled;
      config_.realtime.strict = realtime["strict"] ?
        realtime["strict"].as<bool>() : config_.realtime.strict;
      config_.realtime.lock_memory = realtime["lock_memory"] ?
        realtime["lock_memory"].as<bool>() : config_.realtime.lock_memory;
      config_.realtime.control_priority = realtime["control_priority"] ?
        realtime["control_priority"].as<int>() : config_.realtime.control_priority;
      config_.realtime.policy_priority = realtime["policy_priority"] ?
        realtime["policy_priority"].as<int>() : config_.realtime.policy_priority;
      config_.realtime.control_cpu = realtime["control_cpu"] ?
        realtime["control_cpu"].as<int>() : config_.realtime.control_cpu;
      config_.realtime.policy_cpu = realtime["policy_cpu"] ?
        realtime["policy_cpu"].as<int>() : config_.realtime.policy_cpu;
    }
    if (config_.realtime.control_cpu < -1 || config_.realtime.policy_cpu < -2) {
      throw std::runtime_error(
              "realtime.control_cpu must be >= -1 and policy_cpu must be >= -2");
    }
    if (config_.realtime.enabled &&
      (config_.realtime.control_priority < 1 || config_.realtime.control_priority > 99 ||
      config_.realtime.policy_priority < 1 || config_.realtime.policy_priority > 99))
    {
      throw std::runtime_error("realtime FIFO priorities must be in [1, 99]");
    }

    const auto imu_mount = deploy["imu_mount"];
    if (imu_mount) {
      config_.imu_mount.roll_offset_deg = imu_mount["roll_offset_deg"] ?
        imu_mount["roll_offset_deg"].as<double>() : config_.imu_mount.roll_offset_deg;
      config_.imu_mount.pitch_offset_deg = imu_mount["pitch_offset_deg"] ?
        imu_mount["pitch_offset_deg"].as<double>() : config_.imu_mount.pitch_offset_deg;
      config_.imu_mount.yaw_offset_deg = imu_mount["yaw_offset_deg"] ?
        imu_mount["yaw_offset_deg"].as<double>() : config_.imu_mount.yaw_offset_deg;
    }

    const auto imu_filter = deploy["imu_filter"];
    if (imu_filter) {
      config_.imu_filter.enabled = imu_filter["enabled"] ?
        imu_filter["enabled"].as<bool>() : config_.imu_filter.enabled;
      config_.imu_filter.cutoff_hz = imu_filter["cutoff_hz"] ?
        imu_filter["cutoff_hz"].as<double>() : config_.imu_filter.cutoff_hz;
      config_.imu_filter.reset_timeout_s = imu_filter["reset_timeout_s"] ?
        imu_filter["reset_timeout_s"].as<double>() : config_.imu_filter.reset_timeout_s;
    }
    if (config_.imu_filter.enabled && config_.imu_filter.cutoff_hz <= 0.0) {
      throw std::runtime_error("imu_filter.cutoff_hz must be > 0 when imu_filter.enabled is true");
    }
    if (config_.imu_filter.reset_timeout_s < 0.0) {
      throw std::runtime_error("imu_filter.reset_timeout_s must be >= 0");
    }

    const auto parallel_control = deploy["parallel_control"];
    if (parallel_control) {
      config_.parallel_control.pitch_kp = parallel_control["pitch_kp"] ?
        parallel_control["pitch_kp"].as<double>() : config_.parallel_control.pitch_kp;
      config_.parallel_control.pitch_kd = parallel_control["pitch_kd"] ?
        parallel_control["pitch_kd"].as<double>() : config_.parallel_control.pitch_kd;
      config_.parallel_control.roll_kp = parallel_control["roll_kp"] ?
        parallel_control["roll_kp"].as<double>() : config_.parallel_control.roll_kp;
      config_.parallel_control.roll_kd = parallel_control["roll_kd"] ?
        parallel_control["roll_kd"].as<double>() : config_.parallel_control.roll_kd;
    }
    if (config_.parallel_control.pitch_kp < 0.0 || config_.parallel_control.pitch_kd < 0.0 ||
      config_.parallel_control.roll_kp < 0.0 || config_.parallel_control.roll_kd < 0.0)
    {
      throw std::runtime_error("parallel_control gains must be >= 0");
    }

    const auto joystick = deploy["joystick"];
    if (joystick) {
      const auto mode_switch = joystick["mode_switch"];
      if (mode_switch) {
        config_.joystick.mode_switch.a_button_index = read_required_yaml_int(
          mode_switch, "a_button_index");
        if (mode_switch["danc_button_index"]) {
          config_.joystick.mode_switch.danc_button_index =
            read_required_yaml_int(mode_switch, "danc_button_index");
        }
        config_.joystick.mode_switch.lb_button_index = read_required_yaml_int(
          mode_switch, "lb_button_index");
        config_.joystick.mode_switch.rb_button_index = read_required_yaml_int(
          mode_switch, "rb_button_index");
        config_.joystick.mode_switch.lt_axis_index = read_required_yaml_int(
          mode_switch, "lt_axis_index");
        config_.joystick.mode_switch.lt_pressed_threshold = read_required_yaml_double(
          mode_switch, "lt_pressed_threshold");
      }

      const auto command = joystick["command"];
      if (command) {
        config_.joystick.command.lin_x_axis_index = read_required_yaml_int(
          command,
          "lin_x_axis_index");
        config_.joystick.command.lin_x_sign = read_required_yaml_double(command, "lin_x_sign");
        config_.joystick.command.lin_x_scale = read_required_yaml_double(command, "lin_x_scale");
        config_.joystick.command.lin_y_axis_index = read_required_yaml_int(
          command,
          "lin_y_axis_index");
        config_.joystick.command.lin_y_sign = read_required_yaml_double(command, "lin_y_sign");
        config_.joystick.command.lin_y_scale = read_required_yaml_double(command, "lin_y_scale");
        config_.joystick.command.yaw_axis_index = read_required_yaml_int(command, "yaw_axis_index");
        config_.joystick.command.yaw_sign = read_required_yaml_double(command, "yaw_sign");
        config_.joystick.command.yaw_scale = read_required_yaml_double(command, "yaw_scale");
      }

      const auto recording = joystick["recording"];
      if (recording) {
        config_.joystick.recording.dpad_y_axis_index = read_required_yaml_int(
          recording, "dpad_y_axis_index");
        config_.joystick.recording.up_pressed_threshold = read_required_yaml_double(
          recording, "up_pressed_threshold");
        config_.joystick.recording.down_pressed_threshold = read_required_yaml_double(
          recording, "down_pressed_threshold");
      }
    }
    if (config_.joystick.recording.up_pressed_threshold >= 0.0 ||
      config_.joystick.recording.down_pressed_threshold <= 0.0)
    {
      throw std::runtime_error(
              "joystick.recording up threshold must be negative and down threshold positive");
    }

    const auto modes = deploy["modes"];
    if (!modes) {
      throw std::runtime_error("Config is missing the 'modes' section.");
    }
    config_.startup_mode = mode_from_string(read_required_yaml_string(modes, "startup_mode"));
    config_.passive_gains = read_mode_gains(modes["passive"], true, false, 0.0);
    config_.fixed_gains = read_mode_gains(modes["fixed"], false, true, 5.0);
    config_.loco_gains = read_mode_gains(modes["loco"], false, false, 0.0);

    const auto policy = deploy["policy"];
    if (!policy) {
      throw std::runtime_error("Config is missing the 'policy' section.");
    }
    YAML::Node profile_root;
    YAML::Node policy_contract;
    policy_contract.reset(policy);
    std::string policy_contract_path = config_.config_path;
    if (policy["profile_path"] && !policy["profile_path"].as<std::string>().empty()) {
      const std::string profile_selector_path = resolve_config_path(
        config_.config_path, policy["profile_path"].as<std::string>());
      profile_root = YAML::LoadFile(profile_selector_path);
      if (profile_root["active_profile"]) {
        if (!profile_root["schema_version"] || profile_root["schema_version"].as<int>() != 1) {
          throw std::runtime_error("Unsupported or missing active policy selector schema_version");
        }
        const std::string selected_profile = profile_root["active_profile"].as<std::string>();
        config_.policy.profile_path = resolve_config_path(profile_selector_path, selected_profile);
        profile_root = YAML::LoadFile(config_.policy.profile_path);
      } else {
        config_.policy.profile_path = profile_selector_path;
      }
      policy_contract.reset(profile_root["policy_profile"]);
      policy_contract_path = config_.policy.profile_path;
      if (!policy_contract) {
        throw std::runtime_error("Policy profile must contain a 'policy_profile' root node.");
      }
      if (!policy_contract["schema_version"] ||
        (policy_contract["schema_version"].as<int>() != 1 &&
        policy_contract["schema_version"].as<int>() != 2))
      {
        throw std::runtime_error("Unsupported or missing policy profile schema_version");
      }
    }
    config_.policy.backend = policy["backend"] ?
      policy_backend_from_string(policy["backend"].as<std::string>()) :
      PolicyBackend::kOnnx;
    config_.policy.model_path = policy_contract["model_path"] ?
      resolve_config_path(
      policy_contract_path, policy_contract["model_path"].as<std::string>()) :
      std::string();
    config_.policy.expected_observation_dim = policy_contract["expected_observation_dim"] ?
      policy_contract["expected_observation_dim"].as<int>() : config_.policy.
      expected_observation_dim;
    config_.policy.expected_action_dim = policy_contract["expected_action_dim"] ?
      policy_contract["expected_action_dim"].as<int>() : config_.policy.expected_action_dim;
    config_.policy.observation_history_length = policy_contract["observation_history_length"] ?
      policy_contract["observation_history_length"].as<int>() :
      config_.policy.observation_history_length;
    config_.policy.inference_threads = policy["inference_threads"] ?
      policy["inference_threads"].as<int>() : config_.policy.inference_threads;
    config_.policy.raw_action_clip = policy["raw_action_clip"] ?
      policy["raw_action_clip"].as<double>() : config_.policy.raw_action_clip;
    config_.policy.gait_period_s = policy_contract["gait_period_s"] ?
      policy_contract["gait_period_s"].as<double>() : config_.policy.gait_period_s;
    config_.policy.takeover_duration_s = policy["takeover_duration_s"] ?
      policy["takeover_duration_s"].as<double>() : config_.policy.takeover_duration_s;
    config_.policy.shadow_mode = policy["shadow_mode"] ?
      policy["shadow_mode"].as<bool>() : config_.policy.shadow_mode;
    config_.policy.joint_names =
      policy_contract["joint_names"].as<std::vector<std::string>>();
    config_.policy.default_joint_positions =
      policy_contract["default_joint_positions"].as<std::vector<double>>();
    if (config_.policy.joint_names.empty()) {
      throw std::runtime_error("policy.joint_names must not be empty");
    }
    if (config_.policy.default_joint_positions.size() != config_.policy.joint_names.size()) {
      throw std::runtime_error("policy.default_joint_positions size must match joint_names size");
    }
    const auto action_scale = policy_contract["action_scale"];
    if (!action_scale) {
      throw std::runtime_error("policy.action_scale is required");
    }
    if (action_scale.IsSequence()) {
      config_.policy.action_scales = action_scale.as<std::vector<double>>();
    } else if (action_scale.IsMap()) {
      config_.policy.action_scales.clear();
      config_.policy.action_scales.reserve(config_.policy.joint_names.size());
      for (const auto & joint_name : config_.policy.joint_names) {
        const auto joint_scale = action_scale[joint_name];
        if (!joint_scale) {
          throw std::runtime_error(
                  "policy.action_scale map is missing joint '" + joint_name + "'");
        }
        config_.policy.action_scales.push_back(joint_scale.as<double>());
      }
    } else {
      config_.policy.action_scales.assign(
        config_.policy.joint_names.size(),
        read_required_yaml_double(policy, "action_scale"));
    }
    if (config_.policy.action_scales.size() != config_.policy.joint_names.size()) {
      throw std::runtime_error("policy.action_scale size must match joint_names size");
    }

    config_.policy.joint_position_limits.assign(
      config_.policy.joint_names.size(),
      {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()});
    const auto joint_limits = policy_contract["joint_limits"];
    if (joint_limits) {
      for (std::size_t index = 0; index < config_.policy.joint_names.size(); ++index) {
        const auto & joint_name = config_.policy.joint_names[index];
        if (is_parallel_joint_name(joint_name)) {
          continue;
        }
        const auto limits = joint_limits[joint_name];
        if (!limits || !limits.IsSequence() || limits.size() != 2U) {
          throw std::runtime_error(
                  "policy joint_limits is missing a [lower, upper] pair for '" + joint_name + "'");
        }
        const double lower = limits[0].as<double>();
        const double upper = limits[1].as<double>();
        if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper) {
          throw std::runtime_error("invalid policy joint limit for '" + joint_name + "'");
        }
        config_.policy.joint_position_limits[index] = {lower, upper};
      }
    } else if (!config_.policy.profile_path.empty()) {
      throw std::runtime_error("policy profile is missing XML-derived joint_limits");
    }

    config_.policy.actuators.assign(
      config_.policy.joint_names.size(), PolicyConfig::ActuatorContract {});
    const auto actuators = policy_contract["actuators"];
    if (actuators) {
      for (std::size_t index = 0; index < config_.policy.joint_names.size(); ++index) {
        const auto & joint_name = config_.policy.joint_names[index];
        const auto actuator = actuators[joint_name];
        if (!actuator) {
          throw std::runtime_error("policy actuator contract is missing '" + joint_name + "'");
        }
        auto & contract = config_.policy.actuators[index];
        contract.kp = read_required_yaml_double(actuator, "kp");
        contract.kd = read_required_yaml_double(actuator, "kd");
        contract.effort_limit = read_required_yaml_double(actuator, "effort_limit");
        contract.velocity_limit = read_required_yaml_double(actuator, "velocity_limit");
        contract.valid = true;
        if (contract.kp < 0.0 || contract.kd < 0.0 || contract.effort_limit <= 0.0 ||
          contract.velocity_limit <= 0.0)
        {
          throw std::runtime_error("invalid policy actuator contract for '" + joint_name + "'");
        }
        if (!is_parallel_joint_name(joint_name)) {
          const std::string motor_name = joint_name == "waist_yaw" ? "waistyaw" : joint_name;
          config_.loco_gains.motor_gains[motor_name] = {contract.kp, contract.kd};
        }
      }
    } else if (!config_.policy.profile_path.empty()) {
      throw std::runtime_error("policy profile is missing saved-run actuator parameters");
    }
    if (config_.policy.expected_action_dim <= 0) {
      config_.policy.expected_action_dim = static_cast<int>(config_.policy.joint_names.size());
    }
    if (config_.policy.gait_period_s <= 0.0 || config_.policy.takeover_duration_s < 0.0) {
      throw std::runtime_error(
              "policy.gait_period_s must be > 0 and takeover_duration_s must be >= 0");
    }
    if (config_.policy.expected_action_dim != static_cast<int>(config_.policy.joint_names.size())) {
      throw std::runtime_error("policy.expected_action_dim does not match joint_names size");
    }
    if (config_.policy.backend == PolicyBackend::kOnnx && config_.policy.model_path.empty()) {
      throw std::runtime_error("policy.model_path must not be empty when backend=onnx");
    }
    if (config_.policy.backend == PolicyBackend::kOnnx) {
      const int expected_single_frame_dim =
        11 + 3 * static_cast<int>(config_.policy.joint_names.size());
      if (config_.policy.observation_history_length < 1 ||
        config_.policy.expected_observation_dim !=
        expected_single_frame_dim * config_.policy.observation_history_length)
      {
        throw std::runtime_error(
                "policy observation dimension does not match whole-body term-major history contract");
      }
      if (config_.policy.inference_threads < 1) {
        throw std::runtime_error("policy.inference_threads must be at least one");
      }
    }

    const auto danc = deploy["danc"];
    if (danc) {
      config_.danc_bundle_path = resolve_config_path(
        config_.config_path, read_required_yaml_string(danc, "bundle_path"));
      if (danc["joint_calibration_path"]) {
        config_.danc_joint_calibration_path = resolve_config_path(
          config_.config_path, danc["joint_calibration_path"].as<std::string>());
      }
      config_.danc_takeover_duration_s = read_required_yaml_double(danc, "takeover_duration_s");
      config_.danc_shadow_mode = danc["shadow_mode"].as<bool>();
    }

    load_trajectory_config(deploy["trajectory"]);
    load_recording_config(deploy["recording"]);
  }

  ModeGains read_mode_gains(
    const YAML::Node & node,
    const bool default_follow_feedback_target,
    const bool default_hold_current_pose_on_entry,
    const double default_transition_duration_s) const
  {
    if (!node) {
      throw std::runtime_error("Missing mode gains config.");
    }

    ModeGains gains;
    gains.direct_kp = read_required_yaml_double(node, "direct_kp");
    gains.direct_kd = read_required_yaml_double(node, "direct_kd");
    gains.ankle_kp = read_required_yaml_double(node, "ankle_kp");
    gains.ankle_kd = read_required_yaml_double(node, "ankle_kd");
    const auto motor_gains = node["motor_gains"];
    if (motor_gains) {
      if (!motor_gains.IsMap()) {
        throw std::runtime_error("mode motor_gains must be a map of motor_name -> {kp, kd}.");
      }
      for (const auto & entry : motor_gains) {
        const auto motor_name = entry.first.as<std::string>();
        const auto override_node = entry.second;
        MotorGainOverride override;
        if (override_node["kp"]) {
          override.kp = override_node["kp"].as<double>();
        }
        if (override_node["kd"]) {
          override.kd = override_node["kd"].as<double>();
        }
        if (!override.kp.has_value() && !override.kd.has_value()) {
          throw std::runtime_error(
                  "mode motor_gains entry for '" + motor_name + "' must define kp and/or kd.");
        }
        gains.motor_gains[motor_name] = override;
      }
    }
    gains.follow_feedback_target = node["follow_feedback_target"] ?
      node["follow_feedback_target"].as<bool>() : default_follow_feedback_target;
    gains.hold_current_pose_on_entry = node["hold_current_pose_on_entry"] ?
      node["hold_current_pose_on_entry"].as<bool>() : default_hold_current_pose_on_entry;
    gains.transition_duration_s = node["transition_duration_s"] ?
      node["transition_duration_s"].as<double>() : default_transition_duration_s;
    if (gains.transition_duration_s < 0.0) {
      throw std::runtime_error("mode transition_duration_s must be >= 0");
    }
    return gains;
  }

  void load_trajectory_config(const YAML::Node & node)
  {
    config_.trajectory = TrajectoryConfig {};
    if (config_.policy.backend != PolicyBackend::kTrajectory) {
      return;
    }

    if (!node) {
      throw std::runtime_error("Config is missing the 'trajectory' section for backend=trajectory.");
    }

    config_.trajectory.startup_hold_s = node["startup_hold_s"] ?
      node["startup_hold_s"].as<double>() : config_.trajectory.startup_hold_s;
    config_.trajectory.source = node["source"] ?
      trajectory_source_from_string(node["source"].as<std::string>()) :
      config_.trajectory.source;

    if (config_.trajectory.source == TrajectorySource::kCsvPlayback) {
      load_trajectory_playback_config(node["playback"]);
      return;
    }
    if (config_.trajectory.source == TrajectorySource::kSquat) {
      load_trajectory_squat_config(node["squat"]);
      return;
    }

    const auto profiles = node["joint_profiles"];
    if (!profiles || !profiles.IsSequence()) {
      throw std::runtime_error("trajectory.joint_profiles must be a sequence.");
    }

    for (const auto & profile_node : profiles) {
      TrajectoryJointProfile profile;
      profile.joint_name = read_required_yaml_string(profile_node, "joint_name");
      const auto joint_it = std::find(
        config_.policy.joint_names.begin(),
        config_.policy.joint_names.end(),
        profile.joint_name);
      if (joint_it == config_.policy.joint_names.end()) {
        throw std::runtime_error(
                "trajectory profile references unknown joint: " + profile.joint_name);
      }

      profile.joint_index = static_cast<std::size_t>(
        std::distance(config_.policy.joint_names.begin(), joint_it));
      profile.offset = profile_node["offset"] ? profile_node["offset"].as<double>() : 0.0;
      profile.amplitude = profile_node["amplitude"] ? profile_node["amplitude"].as<double>() : 0.0;
      profile.frequency_hz = profile_node["frequency_hz"] ?
        profile_node["frequency_hz"].as<double>() : 1.0;
      profile.phase_rad = profile_node["phase_rad"] ? profile_node["phase_rad"].as<double>() : 0.0;
      config_.trajectory.joint_profiles.push_back(std::move(profile));
    }
  }

  std::vector<double> read_joint_offsets(
    const YAML::Node & node,
    const char * key) const
  {
    std::vector<double> offsets(config_.policy.joint_names.size(), 0.0);
    const auto offsets_node = node[key];
    if (!offsets_node) {
      return offsets;
    }

    if (offsets_node.IsSequence()) {
      offsets = offsets_node.as<std::vector<double>>();
      if (offsets.size() != config_.policy.joint_names.size()) {
        throw std::runtime_error(
                std::string("trajectory.squat.") + key +
                " size must match policy.joint_names size");
      }
      return offsets;
    }

    if (!offsets_node.IsMap()) {
      throw std::runtime_error(
              std::string("trajectory.squat.") + key +
              " must be either a sequence or a joint_name -> offset map");
    }

    for (const auto & entry : offsets_node) {
      const auto joint_name = entry.first.as<std::string>();
      const auto joint_it = std::find(
        config_.policy.joint_names.begin(),
        config_.policy.joint_names.end(),
        joint_name);
      if (joint_it == config_.policy.joint_names.end()) {
        throw std::runtime_error(
                std::string("trajectory.squat.") + key +
                " references unknown joint: " + joint_name);
      }

      const auto joint_index = static_cast<std::size_t>(
        std::distance(config_.policy.joint_names.begin(), joint_it));
      offsets[joint_index] = entry.second.as<double>();
    }

    return offsets;
  }

  void load_trajectory_squat_config(const YAML::Node & node)
  {
    if (!node) {
      throw std::runtime_error(
              "trajectory.squat section is required when trajectory.source=squat.");
    }

    auto & squat = config_.trajectory.squat;
    squat.prepare_duration_s = node["prepare_duration_s"] ?
      node["prepare_duration_s"].as<double>() : squat.prepare_duration_s;
    squat.prep_hold_s = node["prep_hold_s"] ?
      node["prep_hold_s"].as<double>() : squat.prep_hold_s;
    squat.cycle_period_s = node["cycle_period_s"] ?
      node["cycle_period_s"].as<double>() : squat.cycle_period_s;
    squat.cycle_count = node["cycle_count"] ?
      node["cycle_count"].as<int>() : squat.cycle_count;
    squat.return_duration_s = node["return_duration_s"] ?
      node["return_duration_s"].as<double>() : squat.return_duration_s;
    squat.return_to_default = node["return_to_default"] ?
      node["return_to_default"].as<bool>() : squat.return_to_default;
    squat.preparation_offsets = read_joint_offsets(node, "preparation_offsets");
    squat.squat_offsets = read_joint_offsets(node, "squat_offsets");

    if (squat.prepare_duration_s < 0.0) {
      throw std::runtime_error("trajectory.squat.prepare_duration_s must be >= 0");
    }
    if (squat.prep_hold_s < 0.0) {
      throw std::runtime_error("trajectory.squat.prep_hold_s must be >= 0");
    }
    if (squat.cycle_period_s <= 0.0) {
      throw std::runtime_error("trajectory.squat.cycle_period_s must be > 0");
    }
    if (squat.cycle_count < 0) {
      throw std::runtime_error("trajectory.squat.cycle_count must be >= 0");
    }
    if (squat.return_duration_s < 0.0) {
      throw std::runtime_error("trajectory.squat.return_duration_s must be >= 0");
    }
  }

  void load_recording_config(const YAML::Node & node)
  {
    config_.recording = TrackingRecordingConfig {};
    if (!node) {
      return;
    }

    config_.recording.enabled = node["enabled"] ?
      node["enabled"].as<bool>() : config_.recording.enabled;
    config_.recording.output_root = node["output_root"] ?
      node["output_root"].as<std::string>() : config_.recording.output_root;
    config_.recording.sample_rate_hz = node["sample_rate_hz"] ?
      node["sample_rate_hz"].as<double>() : config_.recording.sample_rate_hz;
    config_.recording.record_only_loco = node["record_only_loco"] ?
      node["record_only_loco"].as<bool>() : config_.recording.record_only_loco;
    config_.recording.write_html = node["write_html"] ?
      node["write_html"].as<bool>() : config_.recording.write_html;
    config_.recording.write_csv = node["write_csv"] ?
      node["write_csv"].as<bool>() : config_.recording.write_csv;
    config_.recording.write_readme = node["write_readme"] ?
      node["write_readme"].as<bool>() : config_.recording.write_readme;
    config_.recording.write_summary = node["write_summary"] ?
      node["write_summary"].as<bool>() : config_.recording.write_summary;

    if (config_.recording.sample_rate_hz <= 0.0) {
      throw std::runtime_error("recording.sample_rate_hz must be > 0");
    }
  }

  void load_trajectory_playback_config(const YAML::Node & node)
  {
    if (!node) {
      throw std::runtime_error(
              "trajectory.playback section is required when trajectory.source=csv_playback.");
    }

    auto & playback = config_.trajectory.playback;
    playback.csv_path = resolve_config_path(
      config_.config_path,
      read_required_yaml_string(node, "csv_path"));
    if (std::filesystem::is_directory(playback.csv_path)) {
      playback.csv_path =
        (std::filesystem::path(playback.csv_path) / "joint_timeseries.csv").string();
    }

    playback.time_column = node["time_column"] ?
      node["time_column"].as<std::string>() : playback.time_column;
    playback.target_prefix = node["target_prefix"] ?
      node["target_prefix"].as<std::string>() : playback.target_prefix;
    playback.reference_prefix = node["reference_prefix"] ?
      node["reference_prefix"].as<std::string>() : playback.reference_prefix;
    playback.joint_suffix = node["joint_suffix"] ?
      node["joint_suffix"].as<std::string>() : playback.joint_suffix;
    playback.command_source = node["command_source"] ?
      trajectory_command_source_from_string(node["command_source"].as<std::string>()) :
      playback.command_source;
    playback.speed_scale = node["speed_scale"] ?
      node["speed_scale"].as<double>() : playback.speed_scale;
    playback.loop = node["loop"] ? node["loop"].as<bool>() : playback.loop;
    playback.interpolate = node["interpolate"] ?
      node["interpolate"].as<bool>() : playback.interpolate;
    playback.skip_initial_zero_targets = node["skip_initial_zero_targets"] ?
      node["skip_initial_zero_targets"].as<bool>() : playback.skip_initial_zero_targets;
    if (!std::isfinite(playback.speed_scale) || playback.speed_scale <= 0.0) {
      throw std::runtime_error("trajectory.playback.speed_scale must be finite and > 0");
    }

    load_trajectory_playback_csv(playback);
    if (playback.command_source == TrajectoryCommandSource::kSimulationActual &&
      !playback.reference_valid)
    {
      throw std::runtime_error(
              "trajectory.playback.command_source=simulation_actual requires all reference columns");
    }

    RCLCPP_INFO(
      get_logger(),
      "Loaded trajectory CSV playback from %s: %zu samples, duration %.3f s, loop=%s, "
      "interpolate=%s, simulation_reference=%s, command_source=%s, speed_scale=%.3f, "
      "playback_duration=%.3f s.",
      playback.csv_path.c_str(),
      playback.samples.size(),
      playback.duration_s,
      playback.loop ? "true" : "false",
      playback.interpolate ? "true" : "false",
      playback.reference_valid ? "yes" : "no",
      to_string(playback.command_source).c_str(),
      playback.speed_scale,
      playback.duration_s / playback.speed_scale);
  }

  void load_trajectory_playback_csv(TrajectoryPlaybackConfig & playback) const
  {
    std::ifstream stream(playback.csv_path);
    if (!stream.is_open()) {
      throw std::runtime_error("Failed to open trajectory CSV: " + playback.csv_path);
    }

    std::string line;
    if (!std::getline(stream, line)) {
      throw std::runtime_error("Trajectory CSV is empty: " + playback.csv_path);
    }

    const auto header = split_csv_line(line);
    std::unordered_map<std::string, std::size_t> column_index;
    column_index.reserve(header.size());
    for (std::size_t i = 0; i < header.size(); ++i) {
      column_index.emplace(header[i], i);
    }

    const auto time_it = column_index.find(playback.time_column);
    if (time_it == column_index.end()) {
      throw std::runtime_error(
              "Trajectory CSV is missing time column: " + playback.time_column);
    }
    const std::size_t time_column_index = time_it->second;
    const auto command_vx_it = column_index.find("command_vx");
    const auto command_vy_it = column_index.find("command_vy");
    const auto command_wz_it = column_index.find("command_wz");
    const bool has_command_columns =
      command_vx_it != column_index.end() &&
      command_vy_it != column_index.end() &&
      command_wz_it != column_index.end();

    std::vector<std::size_t> target_column_indices;
    target_column_indices.reserve(config_.policy.joint_names.size());
    for (const auto & joint_name : config_.policy.joint_names) {
      std::optional<std::size_t> target_column_index;
      const auto candidates = csv_target_column_candidates(
        playback.target_prefix,
        joint_name,
        playback.joint_suffix);
      for (const auto & candidate : candidates) {
        const auto column_it = column_index.find(candidate);
        if (column_it != column_index.end()) {
          target_column_index = column_it->second;
          break;
        }
      }
      if (!target_column_index.has_value()) {
        throw std::runtime_error(
                "Trajectory CSV is missing target column for joint '" + joint_name +
                "' with prefix '" + playback.target_prefix + "'.");
      }
      target_column_indices.push_back(*target_column_index);
    }

    std::vector<std::size_t> reference_column_indices;
    reference_column_indices.reserve(config_.policy.joint_names.size());
    std::size_t reference_columns_found = 0U;
    for (const auto & joint_name : config_.policy.joint_names) {
      std::optional<std::size_t> reference_column_index;
      const auto candidates = csv_target_column_candidates(
        playback.reference_prefix,
        joint_name,
        playback.joint_suffix);
      for (const auto & candidate : candidates) {
        const auto column_it = column_index.find(candidate);
        if (column_it != column_index.end()) {
          reference_column_index = column_it->second;
          break;
        }
      }
      if (reference_column_index.has_value()) {
        ++reference_columns_found;
        reference_column_indices.push_back(*reference_column_index);
      } else {
        reference_column_indices.push_back(std::numeric_limits<std::size_t>::max());
      }
    }
    if (reference_columns_found != 0U &&
      reference_columns_found != config_.policy.joint_names.size())
    {
      throw std::runtime_error(
              "Trajectory CSV contains only " + std::to_string(reference_columns_found) + "/" +
              std::to_string(config_.policy.joint_names.size()) +
              " simulation reference columns with prefix '" + playback.reference_prefix + "'.");
    }
    playback.reference_valid =
      reference_columns_found == config_.policy.joint_names.size();

    playback.samples.clear();
    playback.duration_s = 0.0;
    std::optional<double> first_time_s;
    double previous_time_s = -std::numeric_limits<double>::infinity();
    std::size_t row_number = 1U;

    while (std::getline(stream, line)) {
      ++row_number;
      if (trim_copy(line).empty()) {
        continue;
      }

      auto fields = split_csv_line(line);
      if (fields.size() < header.size()) {
        fields.resize(header.size());
      }

      const double raw_time_s = parse_csv_double(
        fields[time_column_index],
        row_number,
        playback.time_column);

      if (has_command_columns && !playback.command_valid) {
        playback.command = {
          parse_csv_double(fields[command_vx_it->second], row_number, "command_vx"),
          parse_csv_double(fields[command_vy_it->second], row_number, "command_vy"),
          parse_csv_double(fields[command_wz_it->second], row_number, "command_wz")
        };
        playback.command_valid = true;
      }

      std::vector<double> positions(config_.policy.joint_names.size(), 0.0);
      for (std::size_t i = 0; i < target_column_indices.size(); ++i) {
        const std::size_t column = target_column_indices[i];
        if (column >= fields.size()) {
          throw std::runtime_error(
                  "CSV row " + std::to_string(row_number) +
                  " is shorter than the target column index.");
        }
        positions[i] = parse_csv_double(
          fields[column],
          row_number,
          header[column]);
      }

      std::vector<double> reference_positions;
      if (playback.reference_valid) {
        reference_positions.resize(config_.policy.joint_names.size(), 0.0);
        for (std::size_t i = 0; i < reference_column_indices.size(); ++i) {
          const std::size_t column = reference_column_indices[i];
          if (column >= fields.size()) {
            throw std::runtime_error(
                    "CSV row " + std::to_string(row_number) +
                    " is shorter than the simulation reference column index.");
          }
          reference_positions[i] = parse_csv_double(
            fields[column],
            row_number,
            header[column]);
        }
      }

      if (playback.skip_initial_zero_targets && playback.samples.empty() &&
        all_near_zero(positions))
      {
        continue;
      }

      if (!first_time_s.has_value()) {
        first_time_s = raw_time_s;
      }

      const double rebased_time_s = raw_time_s - *first_time_s;
      if (rebased_time_s <= previous_time_s) {
        throw std::runtime_error(
                "Trajectory CSV times must be strictly increasing after skipped rows.");
      }
      previous_time_s = rebased_time_s;

      TrajectoryPlaybackSample sample;
      sample.time_s = rebased_time_s;
      sample.positions = std::move(positions);
      sample.reference_positions = std::move(reference_positions);
      playback.samples.push_back(std::move(sample));
    }

    if (playback.samples.empty()) {
      throw std::runtime_error(
              "Trajectory CSV did not contain any usable target samples: " + playback.csv_path);
    }
    playback.duration_s = playback.samples.back().time_s;
  }

  void build_joint_index()
  {
    joint_name_to_index_.clear();
    for (std::size_t i = 0; i < config_.policy.joint_names.size(); ++i) {
      const auto inserted = joint_name_to_index_.emplace(config_.policy.joint_names[i], i);
      if (!inserted.second) {
        throw std::runtime_error(
                "Duplicate joint name in policy config: " + config_.policy.joint_names[i]);
      }
    }
  }

  void load_parallel_bindings()
  {
    parallel_bindings_.clear();
    const YAML::Node root = YAML::LoadFile(config_.parallel_mechanisms_path);
    const auto mechanisms = root["mechanisms"];
    if (!mechanisms || !mechanisms.IsSequence()) {
      throw std::runtime_error("parallel_mechanisms config must contain a sequence at 'mechanisms'");
    }

    std::unordered_set<std::string> claimed_motors;
    for (const auto & mechanism_node : mechanisms) {
      ParallelBinding binding;
      binding.name = read_required_yaml_string(mechanism_node, "name");
      binding.pitch_joint_name = read_required_yaml_string(mechanism_node, "pitch_joint");
      binding.roll_joint_name = read_required_yaml_string(mechanism_node, "roll_joint");
      binding.motor_1_name = read_required_yaml_string(mechanism_node, "motor_1");
      binding.motor_2_name = read_required_yaml_string(mechanism_node, "motor_2");
      const std::string kinematics_path = resolve_config_path(
        config_.parallel_mechanisms_path,
        read_required_yaml_string(mechanism_node, "config_path"));

      const auto pitch_joint_it = joint_name_to_index_.find(binding.pitch_joint_name);
      const auto roll_joint_it = joint_name_to_index_.find(binding.roll_joint_name);
      if (pitch_joint_it == joint_name_to_index_.end() ||
        roll_joint_it == joint_name_to_index_.end())
      {
        throw std::runtime_error(
                "Parallel mechanism references a logical joint that is missing from policy.joint_names");
      }

      if (!claimed_motors.insert(binding.motor_1_name).second ||
        !claimed_motors.insert(binding.motor_2_name).second)
      {
        throw std::runtime_error("Parallel motor name is claimed more than once.");
      }

      const auto motor_1_config = motor_manager_->motor_config(binding.motor_1_name);
      const auto motor_2_config = motor_manager_->motor_config(binding.motor_2_name);
      if (!motor_1_config.has_value() || !motor_2_config.has_value()) {
        throw std::runtime_error("Failed to find motor config for parallel motor binding.");
      }

      binding.pitch_joint_index = pitch_joint_it->second;
      binding.roll_joint_index = roll_joint_it->second;
      binding.motor_1_config = *motor_1_config;
      binding.motor_2_config = *motor_2_config;

      std::string error_message;
      if (!binding.kinematics.load_from_yaml(kinematics_path, error_message)) {
        throw std::runtime_error(
                "Failed to load kinematics for '" + binding.name + "': " + error_message);
      }

      parallel_bindings_.push_back(std::move(binding));
    }
  }

  void load_direct_bindings()
  {
    direct_joint_bindings_.clear();

    std::unordered_set<std::string> parallel_joint_names;
    std::unordered_set<std::string> parallel_motor_names;
    for (const auto & binding : parallel_bindings_) {
      parallel_joint_names.insert(binding.pitch_joint_name);
      parallel_joint_names.insert(binding.roll_joint_name);
      parallel_motor_names.insert(binding.motor_1_name);
      parallel_motor_names.insert(binding.motor_2_name);
    }

    for (std::size_t joint_index = 0; joint_index < config_.policy.joint_names.size();
      ++joint_index)
    {
      const std::string & joint_name = config_.policy.joint_names[joint_index];
      if (parallel_joint_names.count(joint_name) > 0U) {
        continue;
      }

      if (parallel_motor_names.count(joint_name) > 0U) {
        throw std::runtime_error(
                "Direct joint name collides with a parallel motor name: " + joint_name);
      }

      // The yaw actuator is named without an underscore on the physical bus.
      const std::string motor_name = joint_name == "waist_yaw" ? "waistyaw" : joint_name;
      const auto mapped_motor_config = motor_manager_->motor_config(motor_name);
      if (!mapped_motor_config.has_value()) {
        throw std::runtime_error(
                "Failed to find motor config for direct joint '" + joint_name +
                "' mapped to '" + motor_name + "'");
      }

      direct_joint_bindings_.push_back(
        DirectJointBinding {
          joint_index,
          joint_name,
          motor_name,
          *mapped_motor_config
        });
    }
  }

  void initialize_danc_policy()
  {
    if (config_.danc_bundle_path.empty()) {
      return;
    }
    if (!config_.danc_shadow_mode && config_.danc_joint_calibration_path.empty()) {
      throw std::runtime_error("danc motor output requires a measured joint calibration");
    }
    if (!std::filesystem::exists(config_.danc_bundle_path) && config_.danc_shadow_mode) {
      RCLCPP_WARN(
        get_logger(), "danc is unavailable until the SuperMimic bundle is installed at %s.",
        config_.danc_bundle_path.c_str());
      return;
    }
    const auto & switches = config_.joystick.mode_switch;
    if (switches.danc_button_index < 0 || switches.danc_button_index == switches.a_button_index ||
      switches.danc_button_index == switches.lb_button_index ||
      switches.danc_button_index == switches.rb_button_index ||
      !std::isfinite(config_.danc_takeover_duration_s) || config_.danc_takeover_duration_s < 0.0 ||
      config_.loop.policy_rate_hz != SuperMimicPolicy::kRateHz)
    {
      throw std::runtime_error("danc requires a dedicated button, a valid takeover, and 50 Hz");
    }
    danc_policy_ = std::make_unique<SuperMimicPolicy>(
      config_.danc_bundle_path, config_.policy.joint_names, config_.policy.inference_threads,
      config_.danc_joint_calibration_path);
    const std::vector<double> zero_velocity(config_.policy.joint_names.size(), 0.0);
    (void)danc_policy_->step(
      config_.policy.default_joint_positions, zero_velocity, {0.0F, 0.0F, 0.0F},
      {0.0F, 0.0F, -1.0F}, 0);
    danc_policy_->reset();
    RCLCPP_INFO(
      get_logger(), "Native SuperMimic ready: %zu frames at 50 Hz, shadow=%s, bundle=%s",
      danc_policy_->frame_count(), config_.danc_shadow_mode ? "true" : "false",
      config_.danc_bundle_path.c_str());
  }

  void initialize_policy_runtime(const bool fail_fast = false)
  {
    if (config_.policy.backend == PolicyBackend::kTrajectory) {
      RCLCPP_INFO(get_logger(), "Trajectory backend enabled. ONNX Runtime is skipped.");
      return;
    }

    if (config_.policy.model_path.empty()) {
      RCLCPP_WARN(get_logger(), "Policy model path is empty, loco mode will not be available.");
      return;
    }

    try {
      observation_history_ = std::make_unique<WholebodyObservationHistory>(
        config_.policy.joint_names.size(),
        static_cast<std::size_t>(config_.policy.observation_history_length));
      policy_runtime_ = std::make_unique<OnnxPolicyRuntime>(
        config_.policy.model_path,
        static_cast<std::size_t>(config_.policy.expected_observation_dim),
        static_cast<std::size_t>(config_.policy.expected_action_dim),
        config_.policy.inference_threads);
    } catch (const std::exception & ex) {
      policy_runtime_.reset();
      observation_history_.reset();
      if (fail_fast) {
        throw std::runtime_error(
                std::string(
                  "failed to initialize native ONNX policy: ") + ex.what());
      }
      report_fault(std::string("failed to initialize native ONNX policy: ") + ex.what(), false);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Native ONNX policy ready: model=%s input=%zu output=%zu history=%d threads=%d",
      config_.policy.model_path.c_str(),
      policy_runtime_->input_size(),
      policy_runtime_->output_size(),
      config_.policy.observation_history_length,
      config_.policy.inference_threads);
  }

  std::string summarize_feedback_snapshot(const std::vector<MotorState> & states) const
  {
    std::ostringstream oss;
    oss << states.size() << "/" << motor_names_.size() << " motors";
    const auto missing = compute_missing_motors(motor_names_, states);
    if (!missing.empty()) {
      oss << ", missing: " << join_names(missing);

      std::unordered_map<std::string, std::vector<std::string>> missing_by_interface;
      std::vector<std::string> interface_order;
      for (const auto & name : missing) {
        auto interface_name = std::string("unknown");
        const auto config_it = std::find_if(
          motor_configs_.begin(),
          motor_configs_.end(),
          [&name](const auto & config) {return config.name == name;});
        if (config_it != motor_configs_.end()) {
          interface_name = config_it->interface_name;
        }
        if (missing_by_interface.find(interface_name) == missing_by_interface.end()) {
          interface_order.push_back(interface_name);
        }
        missing_by_interface[interface_name].push_back(name);
      }
      for (const auto & interface_name : interface_order) {
        oss << ", " << interface_name << " missing: " <<
          join_names(missing_by_interface[interface_name]);
      }
    }

    std::vector<std::string> previews;
    previews.reserve(std::min<std::size_t>(states.size(), 6U));
    for (const auto & state : states) {
      if (previews.size() >= 6U) {
        break;
      }
      previews.push_back(
        state.name + "(" + state.interface_name + ")=" +
        jljwholebody_hardware::dm_motor::to_string(state.status));
    }
    if (!previews.empty()) {
      oss << ", states: " << join_names(previews);
      if (states.size() > previews.size()) {
        oss << ", ...";
      }
    }
    return oss.str();
  }

  void warmup_startup_feedback()
  {
    if (config_.loop.startup_feedback_warmup_s <= 0.0) {
      return;
    }

    const auto timeout = to_steady_duration(config_.loop.startup_feedback_warmup_s);
    const auto deadline = SteadyClock::now() + timeout;
    const auto poll_timeout = std::chrono::milliseconds(config_.loop.feedback_poll_timeout_ms);
    std::size_t polls_with_feedback = 0U;
    std::size_t polls = 0U;

    while (rclcpp::ok() && SteadyClock::now() < deadline) {
      if (motor_manager_->poll_once(poll_timeout)) {
        ++polls_with_feedback;
      }
      ++polls;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    const auto states = motor_manager_->snapshot_states();
    const auto message = summarize_feedback_snapshot(states);
    if (states.size() == motor_names_.size()) {
      RCLCPP_INFO(
        get_logger(),
        "Startup feedback warmup complete after %.2fs: %s; polls_with_feedback=%zu/%zu.",
        config_.loop.startup_feedback_warmup_s,
        message.c_str(),
        polls_with_feedback,
        polls);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Startup feedback warmup incomplete after %.2fs: %s; polls_with_feedback=%zu/%zu.",
        config_.loop.startup_feedback_warmup_s,
        message.c_str(),
        polls_with_feedback,
        polls);
    }
  }

  void enable_motors_on_start()
  {
    enabled_by_node_ = true;
    warmup_startup_feedback();

    std::vector<MotorCommandResult> last_results;
    for (int attempt = 1; attempt <= config_.loop.enable_retries; ++attempt) {
      last_results =
        motor_manager_->enable_all(std::chrono::milliseconds(config_.loop.lifecycle_timeout_ms));
      if (!has_failures(last_results)) {
        RCLCPP_INFO(
          get_logger(),
          "Enabled all motors on startup after %d/%d attempt(s).",
          attempt,
          config_.loop.enable_retries);
        return;
      }

      RCLCPP_WARN(
        get_logger(),
        "Enable attempt %d/%d failed: %s. Current feedback: %s",
        attempt,
        config_.loop.enable_retries,
        summarize_failures(last_results).c_str(),
        summarize_feedback_snapshot(motor_manager_->snapshot_states()).c_str());

      if (attempt < config_.loop.enable_retries) {
        std::this_thread::sleep_for(
          std::chrono::milliseconds(config_.loop.enable_retry_interval_ms));
      }
    }

    throw std::runtime_error(
            "Failed to enable one or more motors after " +
            std::to_string(config_.loop.enable_retries) +
            " attempt(s): " + summarize_failures(last_results));
  }

  bool capture_initial_state()
  {
    const auto timeout = to_steady_duration(config_.loop.startup_state_timeout_s);
    const auto deadline = SteadyClock::now() + timeout;
    const auto probe_timeout_ms =
      std::chrono::milliseconds(std::max(config_.loop.command_timeout_ms, 1));

    while (rclcpp::ok() && SteadyClock::now() < deadline) {
      (void)motor_manager_->poll_once(
        std::chrono::milliseconds(
          config_.loop.
          feedback_poll_timeout_ms));
      auto states = motor_manager_->snapshot_states();
      if (states.size() == motor_names_.size() && update_logical_state(states, true)) {
        RCLCPP_INFO(
          get_logger(),
          "Captured initial state from all %zu motors.",
          states.size());
        return true;
      }

      (void)motor_manager_->transmit_group_mit(build_probe_commands(), probe_timeout_ms);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    const auto missing = compute_missing_motors(motor_names_, motor_manager_->snapshot_states());
    RCLCPP_ERROR(
      get_logger(),
      "Timed out waiting for initial motor state. Missing feedback from: %s",
      join_names(missing).c_str());
    return false;
  }

  std::vector<NamedMitCommand> build_probe_commands() const
  {
    std::vector<NamedMitCommand> commands;
    commands.reserve(motor_configs_.size());
    for (const auto & config : motor_configs_) {
      NamedMitCommand command;
      command.name = config.name;
      command.command.position = 0.0F;
      command.command.velocity = 0.0F;
      command.command.kp = 0.0F;
      command.command.kd = 0.0F;
      command.command.torque = 0.0F;
      commands.push_back(std::move(command));
    }
    return commands;
  }

  bool update_logical_state(
    const std::vector<MotorState> & states,
    const bool strict)
  {
    std::unordered_map<std::string, const MotorState *> state_lookup;
    state_lookup.reserve(states.size());
    for (const auto & state : states) {
      state_lookup[state.name] = &state;
    }

    LogicalJointState next_state;
    next_state.positions.assign(config_.policy.joint_names.size(), 0.0);
    next_state.velocities.assign(config_.policy.joint_names.size(), 0.0);
    next_state.efforts.assign(config_.policy.joint_names.size(), 0.0);
    next_state.stamp = SteadyClock::now();
    next_state.valid = true;

    for (const auto & binding : direct_joint_bindings_) {
      const auto state_it = state_lookup.find(binding.motor_name);
      if (state_it == state_lookup.end()) {
        if (strict) {
          return false;
        }
        next_state.valid = false;
        continue;
      }

      next_state.positions[binding.joint_index] = state_it->second->position;
      next_state.velocities[binding.joint_index] = state_it->second->velocity;
      next_state.efforts[binding.joint_index] = state_it->second->torque;
    }

    for (auto & binding : parallel_bindings_) {
      const auto motor_1_it = state_lookup.find(binding.motor_1_name);
      const auto motor_2_it = state_lookup.find(binding.motor_2_name);
      if (motor_1_it == state_lookup.end() || motor_2_it == state_lookup.end()) {
        if (strict) {
          return false;
        }
        next_state.valid = false;
        continue;
      }

      const auto * motor_1_state = motor_1_it->second;
      const auto * motor_2_state = motor_2_it->second;

      double pitch = binding.last_forward_pose_valid ? binding.last_forward_pose.pitch : 0.0;
      double roll = binding.last_forward_pose_valid ? binding.last_forward_pose.roll : 0.0;
      double limited_motor_1 = motor_1_state->position;
      double limited_motor_2 = motor_2_state->position;
      std::string error_message;
      if (!binding.kinematics.forward(
          motor_1_state->position,
          motor_2_state->position,
          limited_motor_1,
          limited_motor_2,
          pitch,
          roll,
          error_message,
          pitch,
          roll))
      {
        if (strict) {
          return false;
        }
        next_state.valid = false;
        continue;
      }

      double pitch_velocity = 0.0;
      double roll_velocity = 0.0;
      double pitch_effort = 0.0;
      double roll_effort = 0.0;
      std::string dynamics_error;
      if (!binding.kinematics.motor_to_joint_dynamics(
          pitch,
          roll,
          motor_1_state->position,
          motor_2_state->position,
          motor_1_state->velocity,
          motor_2_state->velocity,
          motor_1_state->torque,
          motor_2_state->torque,
          pitch_velocity,
          roll_velocity,
          pitch_effort,
          roll_effort,
          dynamics_error))
      {
        if (strict) {
          return false;
        }
        next_state.valid = false;
        continue;
      }

      next_state.positions[binding.pitch_joint_index] = pitch;
      next_state.positions[binding.roll_joint_index] = roll;
      next_state.velocities[binding.pitch_joint_index] = pitch_velocity;
      next_state.velocities[binding.roll_joint_index] = roll_velocity;
      next_state.efforts[binding.pitch_joint_index] = pitch_effort;
      next_state.efforts[binding.roll_joint_index] = roll_effort;
      binding.last_forward_pose.pitch = pitch;
      binding.last_forward_pose.roll = roll;
      binding.last_forward_pose_valid = true;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    current_joint_state_ = std::move(next_state);
    return true;
  }

  void on_joy(const sensor_msgs::msg::Joy::SharedPtr message)
  {
    JoySnapshot snapshot;
    snapshot.axes = message->axes;
    snapshot.buttons.assign(message->buttons.begin(), message->buttons.end());
    snapshot.stamp = SteadyClock::now();
    snapshot.valid = true;

    std::lock_guard<std::mutex> lock(joy_mutex_);
    latest_joy_ = std::move(snapshot);

    const auto callback_time = SteadyClock::now();
    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    if (runtime_stats_.joy_messages > 0U) {
      runtime_stats_.joy_interval_ms_max = std::max(
        runtime_stats_.joy_interval_ms_max,
        std::chrono::duration<double, std::milli>(
          callback_time - runtime_stats_.last_joy_callback_time).count());
    }
    runtime_stats_.last_joy_callback_time = callback_time;
    ++runtime_stats_.joy_messages;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    ImuSnapshot snapshot;
    tf2::Quaternion raw_orientation(
      message->orientation.x,
      message->orientation.y,
      message->orientation.z,
      message->orientation.w);
    if (raw_orientation.length2() < 1e-12) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Received near-zero IMU quaternion, skip this frame.");
      return;
    }
    raw_orientation.normalize();

    snapshot.orientation = raw_orientation * imu_mount_offset_;
    snapshot.orientation.normalize();

    const tf2::Matrix3x3 imu_to_base_rotation(imu_mount_offset_.inverse());
    const tf2::Vector3 raw_angular_velocity(
      message->angular_velocity.x,
      message->angular_velocity.y,
      message->angular_velocity.z);
    const tf2::Vector3 corrected_angular_velocity =
      imu_to_base_rotation * raw_angular_velocity;
    snapshot.angular_velocity = {
      corrected_angular_velocity.x(),
      corrected_angular_velocity.y(),
      corrected_angular_velocity.z()
    };
    snapshot.stamp = SteadyClock::now();
    snapshot.valid = true;

    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      latest_imu_ = apply_imu_filter_locked(snapshot);
    }

    const auto callback_time = SteadyClock::now();
    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    if (runtime_stats_.imu_messages > 0U) {
      runtime_stats_.imu_interval_ms_max = std::max(
        runtime_stats_.imu_interval_ms_max,
        std::chrono::duration<double, std::milli>(
          callback_time - runtime_stats_.last_imu_callback_time).count());
    }
    runtime_stats_.last_imu_callback_time = callback_time;
    ++runtime_stats_.imu_messages;
  }

  ImuSnapshot apply_imu_filter_locked(const ImuSnapshot & input)
  {
    if (!config_.imu_filter.enabled) {
      filtered_imu_valid_ = false;
      return input;
    }

    const bool should_reset =
      !filtered_imu_valid_ ||
      !filtered_imu_.valid ||
      input.stamp <= filtered_imu_.stamp ||
      (config_.imu_filter.reset_timeout_s > 0.0 &&
      input.stamp - filtered_imu_.stamp > to_steady_duration(config_.imu_filter.reset_timeout_s));
    if (should_reset) {
      filtered_imu_ = input;
      filtered_imu_valid_ = true;
      return filtered_imu_;
    }

    const double dt_s = std::chrono::duration<double>(input.stamp - filtered_imu_.stamp).count();
    const double tau_s = 1.0 / (2.0 * M_PI * config_.imu_filter.cutoff_hz);
    const double alpha = std::clamp(1.0 - std::exp(-dt_s / tau_s), 0.0, 1.0);

    ImuSnapshot output = input;
    output.orientation =
      filtered_imu_.orientation.slerp(input.orientation, static_cast<tf2Scalar>(alpha));
    if (output.orientation.length2() < 1e-12) {
      output.orientation = input.orientation;
    } else {
      output.orientation.normalize();
    }
    for (std::size_t i = 0U; i < output.angular_velocity.size(); ++i) {
      output.angular_velocity[i] =
        filtered_imu_.angular_velocity[i] +
        alpha * (input.angular_velocity[i] - filtered_imu_.angular_velocity[i]);
    }
    output.stamp = input.stamp;
    output.valid = input.valid;

    filtered_imu_ = output;
    filtered_imu_valid_ = true;
    return output;
  }

  void lock_process_memory()
  {
    if (!config_.realtime.enabled || !config_.realtime.lock_memory) {
      return;
    }
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
      RCLCPP_INFO(get_logger(), "Locked current and future process memory for realtime control.");
      return;
    }

    const std::string message =
      std::string("mlockall failed: ") + std::strerror(errno) +
      ". Configure memlock limits for deterministic deployment.";
    if (config_.realtime.strict) {
      throw std::runtime_error(message);
    }
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }

  bool configure_realtime_thread(
    const char * thread_name,
    const int priority,
    int cpu_id)
  {
    (void)pthread_setname_np(pthread_self(), thread_name);
    if (!config_.realtime.enabled) {
      return true;
    }

    if (cpu_id == -2) {
      const unsigned int cpu_count = std::thread::hardware_concurrency();
      cpu_id = cpu_count > 0U ? static_cast<int>(cpu_count - 1U) : -1;
    }
    if (cpu_id >= 0) {
      cpu_set_t cpu_set;
      CPU_ZERO(&cpu_set);
      CPU_SET(cpu_id, &cpu_set);
      const int affinity_result =
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set), &cpu_set);
      if (affinity_result != 0) {
        const std::string message =
          std::string("failed to set ") + thread_name + " CPU affinity to " +
          std::to_string(cpu_id) + ": " + std::strerror(affinity_result);
        if (config_.realtime.strict) {
          report_fault(message, false);
          stop_requested_ = true;
          return false;
        }
        RCLCPP_WARN(get_logger(), "%s", message.c_str());
      }
    }

    sched_param scheduling {};
    scheduling.sched_priority = priority;
    const int scheduling_result =
      pthread_setschedparam(pthread_self(), SCHED_FIFO, &scheduling);
    if (scheduling_result != 0) {
      const std::string message =
        std::string("failed to set ") + thread_name + " SCHED_FIFO priority " +
        std::to_string(priority) + ": " + std::strerror(scheduling_result);
      if (config_.realtime.strict) {
        report_fault(message, false);
        stop_requested_ = true;
        return false;
      }
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    }

    volatile unsigned char stack_prefault[64U * 1024U] {};
    for (std::size_t i = 0U; i < sizeof(stack_prefault); i += 4096U) {
      stack_prefault[i] = 0U;
    }
    return true;
  }

  void control_loop()
  {
    if (!configure_realtime_thread(
        "wholebody_ctrl", config_.realtime.control_priority, config_.realtime.control_cpu))
    {
      return;
    }
    const auto period = to_steady_duration(1.0 / config_.loop.control_rate_hz);
    auto next_tick = SteadyClock::now() + period;

    bool last_passive_combo = false;
    bool last_fixed_combo = false;
    bool last_loco_combo = false;
    bool last_danc_combo = false;
    bool last_recording_start_pressed = false;
    bool last_recording_stop_pressed = false;

    while (!stop_requested_.load() && rclcpp::ok()) {
      const auto loop_start = SteadyClock::now();
      std::size_t fresh_feedback_count = 0U;
      try {
        (void)motor_manager_->poll_once(std::chrono::milliseconds(0));
        const auto states = motor_manager_->snapshot_states();
        fresh_feedback_count = count_fresh_feedback(states, loop_start);
        warn_if_feedback_freshness_collapsed(states, loop_start, fresh_feedback_count);
        if (!states.empty()) {
          (void)update_logical_state(states, false);
        }

        JoySnapshot joy;
        {
          std::lock_guard<std::mutex> lock(joy_mutex_);
          joy = latest_joy_;
        }
        const bool passive_combo = combo_pressed_passive(joy);
        const bool fixed_combo = combo_pressed_fixed(joy);
        const bool loco_combo = combo_pressed_loco(joy);
        const bool joy_fresh = joy.valid &&
          SteadyClock::now() - joy.stamp <= to_steady_duration(config_.loop.joy_timeout_s);
        const bool danc_combo = read_button(joy, config_.joystick.mode_switch.rb_button_index) &&
          read_button(joy, config_.joystick.mode_switch.danc_button_index);
        const bool recording_start_pressed = dpad_recording_start_pressed(joy);
        const bool recording_stop_pressed = dpad_recording_stop_pressed(joy);

        if (passive_combo && !last_passive_combo) {
          transition_to_mode(Mode::kPassive, "joystick");
        } else if (fixed_combo && !last_fixed_combo) {
          transition_to_mode(Mode::kFixed, "joystick");
        } else if (loco_combo && !last_loco_combo) {
          if (config_.policy.backend == PolicyBackend::kOnnx && !policy_runtime_) {
            report_fault("loco requested but native ONNX Runtime is not ready", false);
          } else {
            transition_to_mode(Mode::kLoco, "joystick");
          }
        } else if (joy_fresh && danc_combo && !last_danc_combo &&
          !passive_combo && !fixed_combo && !loco_combo)
        {
          if (!danc_policy_) {
            RCLCPP_WARN(get_logger(), "danc requires an installed SuperMimic bundle.");
          } else if (current_mode_.load() != Mode::kFixed ||
            SteadyClock::now() - fixed_transition_start_time_ <
            to_steady_duration(config_.fixed_gains.transition_duration_s))
          {
            RCLCPP_WARN(get_logger(), "Enter fixed mode and wait for it to settle before danc.");
          } else {
            transition_to_mode(Mode::kDanc, "joystick", Mode::kFixed);
          }
        }
        last_passive_combo = passive_combo;
        last_fixed_combo = fixed_combo;
        last_loco_combo = loco_combo;
        if (joy_fresh) {
          last_danc_combo = danc_combo;
        }

        ImuSnapshot imu;
        {
          std::lock_guard<std::mutex> lock(imu_mutex_);
          imu = latest_imu_;
        }

        auto mode = current_mode_.load();
        if (mode == Mode::kLoco) {
          if (recording_start_pressed && !last_recording_start_pressed) {
            start_tracking_recording("joystick_dpad_up");
          } else if (recording_stop_pressed && !last_recording_stop_pressed) {
            stop_tracking_recording("joystick_dpad_down");
          }
        }
        last_recording_start_pressed = recording_start_pressed;
        last_recording_stop_pressed = recording_stop_pressed;

        if (mode == Mode::kLoco || mode == Mode::kDanc) {
          validate_policy_safety(mode, states, imu);
        }
        if (mode == Mode::kDanc) {
          if (!joy_fresh) {
            throw std::runtime_error("danc joystick data is stale");
          }
          bool complete;
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!current_joint_state_.valid ||
              SteadyClock::now() - danc_last_policy_time_ >
              to_steady_duration(3.0 / SuperMimicPolicy::kRateHz))
            {
              throw std::runtime_error("danc state or policy targets are stale");
            }
            complete = danc_complete_;
          }
          if (complete) {
            transition_to_mode(Mode::kFixed, "danc_complete", Mode::kDanc);
            mode = current_mode_.load();
          }
        }
        const auto commands = build_commands_for_mode(mode, states);
        maybe_record_tracking_sample(mode, loop_start, imu);
        if (mode == current_mode_.load()) {
          if (config_.loop.command_timeout_ms == 0) {
            motor_manager_->transmit_group_mit_fast(commands);
          } else {
            (void)motor_manager_->transmit_group_mit(
              commands,
              std::chrono::milliseconds(config_.loop.command_timeout_ms));
          }
        }
      } catch (const std::exception & ex) {
        report_fault(std::string("control loop exception: ") + ex.what(), true);
      }

      const auto after_work = SteadyClock::now();
      const bool deadline_missed = after_work > next_tick;
      record_control_iteration(
        std::chrono::duration<double, std::milli>(after_work - loop_start).count(),
        deadline_missed,
        fresh_feedback_count);

      if (deadline_missed && after_work - next_tick >= period) {
        next_tick = after_work + period;
      } else {
        std::this_thread::sleep_until(next_tick);
        next_tick += period;
      }
    }
  }

  void policy_loop()
  {
    if (!configure_realtime_thread(
        "wholebody_policy", config_.realtime.policy_priority, config_.realtime.policy_cpu))
    {
      return;
    }
    const auto period = to_steady_duration(1.0 / config_.loop.policy_rate_hz);
    auto next_tick = SteadyClock::now() + period;

    while (!stop_requested_.load() && rclcpp::ok()) {
      const auto loop_start = SteadyClock::now();
      try {
        const auto mode = current_mode_.load();
        if (mode == Mode::kLoco) {
          step_policy_once();
        } else if (mode == Mode::kDanc) {
          step_danc_once(loop_start);
        }
      } catch (const std::exception & ex) {
        report_fault(std::string("policy loop exception: ") + ex.what(), true);
      }

      const auto after_work = SteadyClock::now();
      const bool deadline_missed = after_work > next_tick;
      record_policy_iteration(
        std::chrono::duration<double, std::milli>(after_work - loop_start).count(),
        deadline_missed);

      if (deadline_missed && after_work - next_tick >= period) {
        next_tick = after_work + period;
      } else {
        std::this_thread::sleep_until(next_tick);
        next_tick += period;
      }
    }
  }

  void step_danc_once(const SteadyClock::time_point step_start)
  {
    LogicalJointState state;
    std::size_t frame;
    uint64_t generation;
    bool preparing;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (current_mode_.load() != Mode::kDanc || danc_complete_) {
        return;
      }
      frame = danc_frame_;
      if (frame >= danc_policy_->frame_count()) {
        danc_complete_ = true;
        return;
      }
      generation = danc_generation_;
      state = current_joint_state_;
      preparing = step_start - danc_start_time_ <
        to_steady_duration(config_.danc_takeover_duration_s);
    }
    ImuSnapshot imu;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      imu = latest_imu_;
    }
    if (!state.valid || !imu.valid ||
      SteadyClock::now() - imu.stamp > to_steady_duration(config_.loop.imu_timeout_s))
    {
      throw std::runtime_error("danc requires valid joint state and a fresh IMU");
    }
    if (generation != danc_applied_generation_) {
      danc_policy_->reset();
      danc_applied_generation_ = generation;
    }
    const std::array<float, 3> angular_velocity {
      static_cast<float>(imu.angular_velocity[0]),
      static_cast<float>(imu.angular_velocity[1]),
      static_cast<float>(imu.angular_velocity[2])};
    const auto & targets = danc_policy_->step(
      state.positions, state.velocities, angular_velocity, compute_projected_gravity(imu.orientation),
      frame);
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (current_mode_.load() == Mode::kDanc && generation == danc_generation_) {
      danc_targets_ = targets;
      danc_last_policy_time_ = step_start;
      danc_frame_ += preparing ? 0U : 1U;
    }
  }

  void step_policy_once()
  {
    if (config_.policy.backend == PolicyBackend::kTrajectory) {
      step_trajectory_once();
      return;
    }

    if (!policy_runtime_ || !observation_history_) {
      report_fault("loco mode requires initialized native ONNX Runtime", false);
      return;
    }

    JoySnapshot joy;
    {
      std::lock_guard<std::mutex> lock(joy_mutex_);
      joy = latest_joy_;
    }
    if (!joy.valid ||
      SteadyClock::now() - joy.stamp > to_steady_duration(config_.loop.joy_timeout_s))
    {
      report_fault("joystick data is stale in loco mode", false);
      return;
    }

    ImuSnapshot imu;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      imu = latest_imu_;
    }
    if (!imu.valid ||
      SteadyClock::now() - imu.stamp > to_steady_duration(config_.loop.imu_timeout_s))
    {
      report_fault("imu data is stale in loco mode", false);
      return;
    }

    LogicalJointState joint_state;
    std::vector<float> previous_raw_action;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      joint_state = current_joint_state_;
      previous_raw_action = last_policy_raw_action_;
    }
    if (!joint_state.valid) {
      report_fault("logical joint state is not valid in loco mode", false);
      return;
    }

    const auto command = command_from_joy(joy);
    const uint64_t policy_step = loco_policy_step_count_.load(std::memory_order_relaxed);
    const uint64_t reset_generation =
      policy_reset_generation_.load(std::memory_order_acquire);
    if (reset_generation != applied_policy_reset_generation_) {
      observation_history_->reset();
      applied_policy_reset_generation_ = reset_generation;
    }

    const std::array<float, 3> angular_velocity {
      static_cast<float>(imu.angular_velocity[0]),
      static_cast<float>(imu.angular_velocity[1]),
      static_cast<float>(imu.angular_velocity[2])
    };
    const auto projected_gravity = compute_projected_gravity(imu.orientation);
    const auto phase = compute_gait_phase(command, policy_step);
    observation_history_->update(
      angular_velocity,
      projected_gravity,
      command,
      phase,
      joint_state.positions,
      config_.policy.default_joint_positions,
      joint_state.velocities,
      previous_raw_action);
    geometry_msgs::msg::Vector3Stamped projected_gravity_message;
    projected_gravity_message.header.stamp = now();
    projected_gravity_message.header.frame_id = "base_link";
    projected_gravity_message.vector.x = projected_gravity[0];
    projected_gravity_message.vector.y = projected_gravity[1];
    projected_gravity_message.vector.z = projected_gravity[2];
    projected_gravity_pub_->publish(projected_gravity_message);
    geometry_msgs::msg::Vector3Stamped angular_velocity_message;
    angular_velocity_message.header.stamp = projected_gravity_message.header.stamp;
    angular_velocity_message.header.frame_id = "base_link";
    angular_velocity_message.vector.x = imu.angular_velocity[0];
    angular_velocity_message.vector.y = imu.angular_velocity[1];
    angular_velocity_message.vector.z = imu.angular_velocity[2];
    angular_velocity_pub_->publish(angular_velocity_message);
    const auto & observation = observation_history_->data();
    if (observation.size() != static_cast<std::size_t>(config_.policy.expected_observation_dim)) {
      std::ostringstream oss;
      oss << "observation dimension mismatch, expected " <<
        config_.policy.expected_observation_dim <<
        " but built " << observation.size();
      report_fault(oss.str(), true);
      return;
    }

    const auto & inference_output = policy_runtime_->infer(observation);
    std::vector<float> raw_action(inference_output.begin(), inference_output.end());
    if (raw_action.size() != config_.policy.joint_names.size()) {
      std::ostringstream oss;
      oss << "policy action size mismatch, expected " << config_.policy.joint_names.size() <<
        " but received " << raw_action.size();
      report_fault(oss.str(), true);
      return;
    }

    std::vector<double> policy_targets(config_.policy.joint_names.size(), 0.0);
    bool model_limit_clamped = false;
    for (std::size_t i = 0; i < raw_action.size(); ++i) {
      raw_action[i] = static_cast<float>(std::clamp(
          raw_action[i],
          static_cast<float>(-config_.policy.raw_action_clip),
          static_cast<float>(config_.policy.raw_action_clip)));
      policy_targets[i] =
        config_.policy.default_joint_positions[i] +
        config_.policy.action_scales[i] * static_cast<double>(raw_action[i]);
      if (!is_parallel_joint_name(config_.policy.joint_names[i])) {
        const auto [lower, upper] = config_.policy.joint_position_limits[i];
        const double limited_target = std::clamp(policy_targets[i], lower, upper);
        model_limit_clamped = model_limit_clamped || limited_target != policy_targets[i];
        policy_targets[i] = limited_target;
      }
    }
    if (model_limit_clamped) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Policy targets were clamped to the whole-body policy joint limits");
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_policy_targets_ = std::move(policy_targets);
    latest_policy_observation_ = observation;
    last_policy_raw_action_ = std::move(raw_action);
    loco_policy_step_count_.fetch_add(1U, std::memory_order_relaxed);
  }

  void step_trajectory_once()
  {
    JoySnapshot joy;
    {
      std::lock_guard<std::mutex> lock(joy_mutex_);
      joy = latest_joy_;
    }
    if (!joy.valid ||
      SteadyClock::now() - joy.stamp > to_steady_duration(config_.loop.joy_timeout_s))
    {
      report_fault("joystick data is stale in trajectory loco mode", false);
      return;
    }

    ImuSnapshot imu;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      imu = latest_imu_;
    }
    if (!imu.valid ||
      SteadyClock::now() - imu.stamp > to_steady_duration(config_.loop.imu_timeout_s))
    {
      report_fault("imu data is stale in trajectory loco mode", false);
      return;
    }

    const double elapsed_s = std::chrono::duration<double>(
      SteadyClock::now() - loco_mode_start_time_).count();
    std::vector<double> logical_targets = config_.policy.default_joint_positions;
    if (elapsed_s >= config_.trajectory.startup_hold_s) {
      const double motion_time_s = elapsed_s - config_.trajectory.startup_hold_s;
      logical_targets = trajectory_targets_at(motion_time_s);
    }

    std::vector<float> raw_action = logical_targets_to_raw_action(logical_targets);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_policy_targets_ = std::move(logical_targets);
      last_policy_raw_action_ = std::move(raw_action);
    }
  }

  std::vector<double> trajectory_targets_at(const double motion_time_s) const
  {
    if (config_.trajectory.source == TrajectorySource::kCsvPlayback) {
      return playback_targets_at(motion_time_s);
    }
    if (config_.trajectory.source == TrajectorySource::kSquat) {
      return squat_targets_at(motion_time_s);
    }

    std::vector<double> logical_targets = config_.policy.default_joint_positions;
    for (const auto & profile : config_.trajectory.joint_profiles) {
      logical_targets[profile.joint_index] =
        config_.policy.default_joint_positions[profile.joint_index] +
        profile.offset +
        profile.amplitude * std::sin(
        2.0 * M_PI * profile.frequency_hz * motion_time_s + profile.phase_rad);
    }
    return logical_targets;
  }

  std::vector<double> squat_targets_at(double motion_time_s) const
  {
    const auto & squat = config_.trajectory.squat;
    const auto stand_targets = config_.policy.default_joint_positions;
    const auto prep_targets = add_vectors(stand_targets, squat.preparation_offsets);
    const auto bottom_targets = add_vectors(stand_targets, squat.squat_offsets);

    if (squat.prepare_duration_s > 1e-9 && motion_time_s < squat.prepare_duration_s) {
      return blend_vectors(
        stand_targets,
        prep_targets,
        smoothstep01(motion_time_s / squat.prepare_duration_s));
    }
    motion_time_s -= squat.prepare_duration_s;

    if (motion_time_s < squat.prep_hold_s) {
      return prep_targets;
    }
    motion_time_s -= squat.prep_hold_s;

    const double cycle_duration_s =
      static_cast<double>(squat.cycle_count) * squat.cycle_period_s;
    if (squat.cycle_count > 0 && motion_time_s < cycle_duration_s) {
      const double phase = std::fmod(motion_time_s, squat.cycle_period_s) / squat.cycle_period_s;
      const double squat_depth = 0.5 * (1.0 - std::cos(2.0 * M_PI * phase));
      return blend_vectors(prep_targets, bottom_targets, squat_depth);
    }
    motion_time_s -= cycle_duration_s;

    if (squat.return_to_default) {
      if (squat.return_duration_s > 1e-9 && motion_time_s < squat.return_duration_s) {
        return blend_vectors(
          prep_targets,
          stand_targets,
          smoothstep01(motion_time_s / squat.return_duration_s));
      }
      return stand_targets;
    }

    return prep_targets;
  }

  std::vector<double> playback_targets_at(double motion_time_s) const
  {
    const auto & playback = config_.trajectory.playback;
    const bool use_simulation_actual =
      playback.command_source == TrajectoryCommandSource::kSimulationActual;
    return playback_values_at(motion_time_s * playback.speed_scale, use_simulation_actual);
  }

  std::vector<double> playback_reference_at(double motion_time_s) const
  {
    const auto & playback = config_.trajectory.playback;
    if (!playback.reference_valid) {
      return {};
    }
    return playback_values_at(motion_time_s * playback.speed_scale, true);
  }

  std::vector<double> playback_values_at(
    double motion_time_s,
    const bool use_reference) const
  {
    const auto & playback = config_.trajectory.playback;
    const auto & samples = playback.samples;
    if (samples.empty()) {
      return use_reference ? std::vector<double> {} : config_.policy.default_joint_positions;
    }

    const auto & first_values = use_reference ?
      samples.front().reference_positions : samples.front().positions;
    const auto & last_values = use_reference ?
      samples.back().reference_positions : samples.back().positions;

    if (playback.loop && playback.duration_s > 1e-9) {
      motion_time_s = std::fmod(std::max(0.0, motion_time_s), playback.duration_s);
    }

    if (motion_time_s <= samples.front().time_s) {
      return first_values;
    }
    if (motion_time_s >= samples.back().time_s) {
      return last_values;
    }

    const auto upper = std::upper_bound(
      samples.begin(),
      samples.end(),
      motion_time_s,
      [](const double time_s, const TrajectoryPlaybackSample & sample) {
        return time_s < sample.time_s;
      });
    const auto next = upper;
    const auto previous = std::prev(upper);
    const auto & previous_values = use_reference ?
      previous->reference_positions : previous->positions;

    if (!playback.interpolate || next == samples.end()) {
      return previous_values;
    }

    const double dt = next->time_s - previous->time_s;
    if (dt <= 1e-12) {
      return previous_values;
    }

    const double alpha = std::clamp((motion_time_s - previous->time_s) / dt, 0.0, 1.0);
    const auto & next_values = use_reference ? next->reference_positions : next->positions;
    std::vector<double> interpolated(previous_values.size(), 0.0);
    for (std::size_t i = 0; i < interpolated.size(); ++i) {
      interpolated[i] =
        previous_values[i] + alpha * (next_values[i] - previous_values[i]);
    }
    return interpolated;
  }

  std::vector<float> logical_targets_to_raw_action(
    const std::vector<double> & logical_targets) const
  {
    std::vector<float> raw_action(logical_targets.size(), 0.0F);
    for (std::size_t i = 0; i < logical_targets.size(); ++i) {
      const double scale = config_.policy.action_scales[i];
      if (std::abs(scale) < 1e-9) {
        raw_action[i] = 0.0F;
        continue;
      }

      const double unclamped_value =
        (logical_targets[i] - config_.policy.default_joint_positions[i]) / scale;
      raw_action[i] = static_cast<float>(std::clamp(
          unclamped_value,
          -config_.policy.raw_action_clip,
          config_.policy.raw_action_clip));
    }
    return raw_action;
  }

  std::size_t count_fresh_feedback(
    const std::vector<MotorState> & states,
    const SteadyClock::time_point cycle_start) const
  {
    return static_cast<std::size_t>(std::count_if(
             states.begin(), states.end(),
             [cycle_start](const auto & state) {
               return state.valid && state.stamp >= cycle_start;
             }));
  }

  void warn_if_feedback_freshness_collapsed(
    const std::vector<MotorState> & states,
    const SteadyClock::time_point cycle_start,
    const std::size_t fresh_feedback_count)
  {
    if (motor_names_.empty() ||
      fresh_feedback_count >= std::max<std::size_t>(1U, motor_names_.size() / 2U))
    {
      return;
    }

    std::unordered_map<std::string, const MotorState *> state_lookup;
    state_lookup.reserve(states.size());
    for (const auto & state : states) {
      state_lookup[state.name] = &state;
    }

    std::vector<std::string> stale_or_missing;
    stale_or_missing.reserve(motor_names_.size());
    for (const auto & name : motor_names_) {
      const auto state_it = state_lookup.find(name);
      if (state_it == state_lookup.end()) {
        stale_or_missing.push_back(name + "(missing)");
        continue;
      }

      const auto & state = *state_it->second;
      if (!state.valid || state.stamp < cycle_start) {
        const double age_ms = state.valid ?
          std::chrono::duration<double, std::milli>(cycle_start - state.stamp).count() :
          0.0;
        std::ostringstream item;
        item << name << "(" << state.interface_name << ", stale";
        if (state.valid) {
          item << " " << std::fixed << std::setprecision(1) << age_ms << "ms";
        }
        item << ")";
        stale_or_missing.push_back(item.str());
      }
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Fresh motor feedback collapsed: %zu/%zu motors fresh this cycle; stale/missing: %s",
      fresh_feedback_count,
      motor_names_.size(),
      join_names(stale_or_missing).c_str());
  }

  void record_control_iteration(
    const double compute_ms,
    const bool deadline_missed,
    const std::size_t fresh_feedback_count)
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++runtime_stats_.control_iterations;
    runtime_stats_.control_compute_ms_sum += compute_ms;
    runtime_stats_.control_compute_ms_max = std::max(
      runtime_stats_.control_compute_ms_max,
      compute_ms);
    if (deadline_missed) {
      ++runtime_stats_.control_deadline_misses;
    }
    runtime_stats_.control_fresh_feedback_count_sum +=
      static_cast<uint64_t>(fresh_feedback_count);
    if (fresh_feedback_count > 0U) {
      ++runtime_stats_.control_cycles_with_any_fresh_feedback;
    }
    if (fresh_feedback_count == motor_names_.size()) {
      ++runtime_stats_.control_cycles_with_full_fresh_feedback;
    }
  }

  void record_policy_iteration(const double compute_ms, const bool deadline_missed)
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++runtime_stats_.policy_iterations;
    runtime_stats_.policy_compute_ms_sum += compute_ms;
    runtime_stats_.policy_compute_ms_max =
      std::max(runtime_stats_.policy_compute_ms_max, compute_ms);
    if (deadline_missed) {
      ++runtime_stats_.policy_deadline_misses;
    }
  }

  void publish_status()
  {
    RuntimeStats current_stats;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      current_stats = runtime_stats_;
    }

    const auto now_time = SteadyClock::now();
    const double elapsed_s = std::max(
      1e-6,
      std::chrono::duration<double>(now_time - last_status_publish_time_).count());

    const uint64_t control_delta = current_stats.control_iterations -
      last_status_stats_.control_iterations;
    const uint64_t control_miss_delta =
      current_stats.control_deadline_misses - last_status_stats_.control_deadline_misses;
    const uint64_t control_any_fresh_delta =
      current_stats.control_cycles_with_any_fresh_feedback -
      last_status_stats_.control_cycles_with_any_fresh_feedback;
    const uint64_t control_full_fresh_delta =
      current_stats.control_cycles_with_full_fresh_feedback -
      last_status_stats_.control_cycles_with_full_fresh_feedback;
    const uint64_t control_fresh_feedback_sum_delta =
      current_stats.control_fresh_feedback_count_sum -
      last_status_stats_.control_fresh_feedback_count_sum;
    const uint64_t policy_delta = current_stats.policy_iterations -
      last_status_stats_.policy_iterations;
    const uint64_t policy_miss_delta =
      current_stats.policy_deadline_misses - last_status_stats_.policy_deadline_misses;
    const uint64_t imu_delta = current_stats.imu_messages - last_status_stats_.imu_messages;
    const uint64_t joy_delta = current_stats.joy_messages - last_status_stats_.joy_messages;

    JoySnapshot joy;
    ImuSnapshot imu;
    {
      std::lock_guard<std::mutex> joy_lock(joy_mutex_);
      joy = latest_joy_;
    }
    {
      std::lock_guard<std::mutex> imu_lock(imu_mutex_);
      imu = latest_imu_;
    }

    const bool joy_stale = !joy.valid ||
      (SteadyClock::now() - joy.stamp > to_steady_duration(config_.loop.joy_timeout_s));
    const bool imu_stale = !imu.valid ||
      (SteadyClock::now() - imu.stamp > to_steady_duration(config_.loop.imu_timeout_s));
    const auto joy_command = command_from_joy(joy);
    bool recording_active = false;
    std::size_t recording_samples = 0U;
    {
      std::lock_guard<std::mutex> recording_lock(recording_mutex_);
      recording_active = active_recording_.has_value();
      if (recording_active) {
        recording_samples = active_recording_->samples.size();
      }
    }

    const double any_fresh_feedback_rate = control_delta > 0U ?
      100.0 * static_cast<double>(control_any_fresh_delta) / static_cast<double>(control_delta) :
      0.0;
    const double full_fresh_feedback_rate = control_delta > 0U ?
      100.0 * static_cast<double>(control_full_fresh_delta) / static_cast<double>(control_delta) :
      0.0;
    const double average_fresh_feedback_count = control_delta > 0U ?
      static_cast<double>(control_fresh_feedback_sum_delta) / static_cast<double>(control_delta) :
      0.0;

    Mode mode;
    std::size_t danc_frame;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      mode = current_mode_.load();
      danc_frame = danc_frame_;
    }
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "mode=" << to_string(mode)
        << " backend=" << to_string(config_.policy.backend)
        << " shadow=" << (config_.policy.shadow_mode ? "true" : "false")
        << " parallel_control=" << kParallelControlMode
        << " trajectory=" << to_string(config_.trajectory.source)
        << " control=" << static_cast<double>(control_delta) / elapsed_s << "Hz"
        << " control_miss=" <<
      (control_delta > 0U ? 100.0 * static_cast<double>(control_miss_delta) /
    static_cast<double>(control_delta) : 0.0) << "%"
        << " policy=" << static_cast<double>(policy_delta) / elapsed_s << "Hz"
        << " policy_miss=" <<
      (policy_delta > 0U ? 100.0 * static_cast<double>(policy_miss_delta) /
    static_cast<double>(policy_delta) : 0.0) << "%"
        << " fresh_any=" << any_fresh_feedback_rate << "%"
        << " fresh_full=" << full_fresh_feedback_rate << "%"
        << " fresh_avg=" << average_fresh_feedback_count << "/" << motor_names_.size()
        << " imu=" << static_cast<double>(imu_delta) / elapsed_s << "Hz"
        << (imu_stale ? " stale" : " ok")
        << " imu_filter=" << (config_.imu_filter.enabled ? "on" : "off")
        << " joy=" << static_cast<double>(joy_delta) / elapsed_s << "Hz"
        << (joy_stale ? " stale" : " ok")
        << " joy_cmd=("
        << static_cast<double>(joy_command[0]) << ", "
        << static_cast<double>(joy_command[1]) << ", "
        << static_cast<double>(joy_command[2]) << ")"
        << " recording=" << (recording_active ? "on" : "off")
        << " recording_samples=" << recording_samples
        << " control_max=" << current_stats.control_compute_ms_max << "ms"
        << " policy_max=" << current_stats.policy_compute_ms_max << "ms"
        << " imu_gap_max=" << current_stats.imu_interval_ms_max << "ms"
        << " joy_gap_max=" << current_stats.joy_interval_ms_max << "ms"
        << " faults=" << current_stats.fault_count;
    if (mode == Mode::kDanc) {
      oss << " danc_shadow=" << (config_.danc_shadow_mode ? "true" : "false")
          << " danc_calibrated=" << (config_.danc_joint_calibration_path.empty() ? "false" : "true")
          << " danc_frame=" << danc_frame << "/" << danc_policy_->frame_count();
    }

    std_msgs::msg::String status_message;
    status_message.data = oss.str();
    status_pub_->publish(status_message);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", status_message.data.c_str());

    last_status_stats_ = current_stats;
    last_status_publish_time_ = now_time;
  }

  std::array<float, 2> compute_gait_phase(
    const std::array<float, 3> & command,
    const uint64_t policy_step) const
  {
    const double command_norm = std::sqrt(
      static_cast<double>(command[0]) * static_cast<double>(command[0]) +
      static_cast<double>(command[1]) * static_cast<double>(command[1]) +
      static_cast<double>(command[2]) * static_cast<double>(command[2]));
    if (command_norm < kGaitPhaseStandCommandThreshold) {
      return {0.0F, 0.0F};
    }

    const double policy_dt = 1.0 / config_.loop.policy_rate_hz;
    const double phase =
      std::fmod(static_cast<double>(policy_step) * policy_dt, config_.policy.gait_period_s) /
      config_.policy.gait_period_s;
    const double angle = phase * 2.0 * M_PI;
    return {
      static_cast<float>(std::sin(angle)),
      static_cast<float>(std::cos(angle))
    };
  }

  std::array<float, 3> compute_projected_gravity(const tf2::Quaternion & orientation) const
  {
    const tf2::Matrix3x3 rotation(orientation.normalized());
    const tf2::Vector3 world_gravity(0.0, 0.0, -1.0);
    const tf2::Vector3 body_gravity = rotation.transpose() * world_gravity;
    return {
      static_cast<float>(body_gravity.x()),
      static_cast<float>(body_gravity.y()),
      static_cast<float>(body_gravity.z())
    };
  }

  void validate_policy_safety(
    const Mode mode,
    const std::vector<MotorState> & states,
    const ImuSnapshot & imu) const
  {
    if (states.size() != motor_names_.size()) {
      throw std::runtime_error(
              to_string(mode) + " safety: feedback available for " + std::to_string(states.size()) + "/" +
              std::to_string(motor_names_.size()) + " motors");
    }

    const auto now = SteadyClock::now();
    for (const auto & state : states) {
      const double age_s = std::chrono::duration<double>(now - state.stamp).count();
      if (!state.valid || age_s > config_.loop.max_motor_feedback_age_s) {
        throw std::runtime_error(
                to_string(mode) + " safety: stale feedback for " + state.name + " age=" +
                format_double(age_s, 4) + "s");
      }
      if (state.status != jljwholebody_hardware::dm_motor::MotorStatus::kEnabled) {
        throw std::runtime_error(
                to_string(mode) + " safety: motor " + state.name + " status=" +
                jljwholebody_hardware::dm_motor::to_string(state.status));
      }
    }

    if (!imu.valid || now - imu.stamp > to_steady_duration(config_.loop.imu_timeout_s)) {
      throw std::runtime_error(to_string(mode) + " safety: IMU is stale");
    }
    const auto gravity = compute_projected_gravity(imu.orientation);
    const double tilt_deg = std::acos(std::clamp(-static_cast<double>(gravity[2]), -1.0, 1.0)) *
      180.0 / M_PI;
    if (tilt_deg > config_.loop.max_tilt_deg) {
      throw std::runtime_error(
              to_string(mode) + " safety: body tilt " + format_double(tilt_deg, 1) +
              "deg exceeds " + format_double(config_.loop.max_tilt_deg, 1) + "deg");
    }
  }

  std::array<float, 3> command_from_joy(const JoySnapshot & joy) const
  {
    return {
      read_axis(joy, config_.joystick.command.lin_x_axis_index) *
      static_cast<float>(config_.joystick.command.lin_x_sign *
      config_.joystick.command.lin_x_scale),
      read_axis(joy, config_.joystick.command.lin_y_axis_index) *
      static_cast<float>(config_.joystick.command.lin_y_sign *
      config_.joystick.command.lin_y_scale),
      read_axis(joy, config_.joystick.command.yaw_axis_index) *
      static_cast<float>(config_.joystick.command.yaw_sign * config_.joystick.command.yaw_scale)
    };
  }

  std::vector<NamedMitCommand> build_commands_for_mode(
    const Mode mode,
    const std::vector<MotorState> & states)
  {
    const bool danc_driving = mode == Mode::kDanc && !config_.danc_shadow_mode;
    const bool fixed_position_mode = mode == Mode::kFixed ||
      (mode == Mode::kDanc && config_.danc_shadow_mode);
    const ModeGains & gains =
      (mode == Mode::kLoco &&
      config_.policy.shadow_mode) ? config_.passive_gains : mode_gains(mode);
    std::unordered_map<std::string, const MotorState *> state_lookup;
    state_lookup.reserve(states.size());
    for (const auto & state : states) {
      state_lookup[state.name] = &state;
    }

    LogicalJointState logical_state;
    std::vector<double> policy_targets;
    std::vector<double> fixed_transition_targets;
    std::vector<double> fixed_transition_start_positions;
    std::vector<double> policy_transition_start_positions;
    SteadyClock::time_point fixed_transition_start_time {};
    double fixed_transition_duration_s {0.0};
    SteadyClock::time_point policy_transition_start_time {};
    double takeover_duration_s = config_.policy.takeover_duration_s;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      logical_state = current_joint_state_;
      policy_targets = latest_policy_targets_;
      if (fixed_position_mode) {
        fixed_transition_start_positions = fixed_transition_start_positions_;
        fixed_transition_start_time = fixed_transition_start_time_;
        fixed_transition_duration_s = config_.fixed_gains.transition_duration_s;
      } else if (mode == Mode::kLoco) {
        policy_transition_start_positions = loco_transition_start_positions_;
        policy_transition_start_time = loco_mode_start_time_;
      } else if (danc_driving) {
        policy_targets = danc_targets_;
        policy_transition_start_positions = danc_start_positions_;
        policy_transition_start_time = danc_start_time_;
        takeover_duration_s = config_.danc_takeover_duration_s;
      }
    }

    const std::vector<double> * desired_logical_targets = nullptr;
    if (mode == Mode::kPassive && gains.follow_feedback_target) {
      desired_logical_targets = &logical_state.positions;
    } else if (fixed_position_mode) {
      if (fixed_transition_duration_s > 1e-9) {
        const double elapsed_s = std::chrono::duration<double>(
          SteadyClock::now() - fixed_transition_start_time).count();
        if (elapsed_s < fixed_transition_duration_s) {
          fixed_transition_targets = blend_vectors(
            fixed_transition_start_positions,
            fixed_hold_positions_,
            smoothstep01(elapsed_s / fixed_transition_duration_s));
          desired_logical_targets = &fixed_transition_targets;
        } else {
          desired_logical_targets = &fixed_hold_positions_;
        }
      } else {
        desired_logical_targets = &fixed_hold_positions_;
      }
    } else if (mode == Mode::kLoco && config_.policy.shadow_mode) {
      desired_logical_targets = &logical_state.positions;
    } else if (mode == Mode::kLoco || danc_driving) {
      const double elapsed_s = std::chrono::duration<double>(
        SteadyClock::now() - policy_transition_start_time).count();
      if (takeover_duration_s > 1e-9 && elapsed_s < takeover_duration_s)
      {
        fixed_transition_targets = blend_vectors(
          policy_transition_start_positions,
          policy_targets,
          smoothstep01(elapsed_s / takeover_duration_s));
        desired_logical_targets = &fixed_transition_targets;
      } else {
        desired_logical_targets = &policy_targets;
      }
    } else {
      desired_logical_targets = &fixed_hold_positions_;
    }

    std::vector<NamedMitCommand> commands;
    commands.reserve(direct_joint_bindings_.size() + parallel_bindings_.size() * 2U);
    std::vector<double> commanded_logical_targets = *desired_logical_targets;
    std::vector<double> commanded_logical_velocities(
      config_.policy.joint_names.size(), 0.0);
    std::vector<double> commanded_logical_torques(
      config_.policy.joint_names.size(), 0.0);

    for (const auto & binding : direct_joint_bindings_) {
      double target_position = logical_state.valid && gains.follow_feedback_target &&
        mode == Mode::kPassive ?
        logical_state.positions[binding.joint_index] :
        (*desired_logical_targets)[binding.joint_index];
      bool clamped = false;
      auto [model_lower, model_upper] = config_.policy.joint_position_limits[binding.joint_index];
      if (danc_driving) {
        const auto & joint = danc_policy_->hardware_joint(binding.joint_index);
        model_lower = joint.lower;
        model_upper = joint.upper;
      }
      const double model_limited_position = std::clamp(
        target_position, model_lower, model_upper);
      clamped = model_limited_position != target_position;
      target_position = model_limited_position;
      target_position = clamp_motor_position(target_position, binding.motor_config, clamped);
      commanded_logical_targets[binding.joint_index] = target_position;

      NamedMitCommand command;
      command.name = binding.motor_name;
      command.command.position = static_cast<float>(target_position);
      command.command.velocity = 0.0F;
      if (danc_driving) {
        const double effort = danc_effort(binding.joint_index, target_position, logical_state);
        command.command.torque = static_cast<float>(effort);
        commanded_logical_torques[binding.joint_index] = effort;
        commands.push_back(std::move(command));
        continue;
      }
      const auto [kp, kd] = resolve_motor_gains(
        gains,
        binding.motor_name,
        gains.direct_kp,
        gains.direct_kd,
        binding.motor_config.default_kp,
        binding.motor_config.default_kd);
      command.command.kp = static_cast<float>(kp);
      command.command.kd = static_cast<float>(kd);
      command.command.torque = 0.0F;
      if (logical_state.valid &&
        binding.joint_index < logical_state.positions.size() &&
        binding.joint_index < logical_state.velocities.size())
      {
        commanded_logical_torques[binding.joint_index] =
          kp * (target_position - logical_state.positions[binding.joint_index]) -
          kd * logical_state.velocities[binding.joint_index] +
          static_cast<double>(command.command.torque);
      }
      commands.push_back(std::move(command));
    }

    for (const auto & binding : parallel_bindings_) {
      bool clamped = false;
      const double requested_pitch = (*desired_logical_targets)[binding.pitch_joint_index];
      const double requested_roll = (*desired_logical_targets)[binding.roll_joint_index];
      double pitch_target = requested_pitch;
      double roll_target = requested_roll;
      pitch_target = clamp_with_limits(pitch_target, binding.kinematics.pitch_limits(), clamped);
      roll_target = clamp_with_limits(roll_target, binding.kinematics.roll_limits(), clamped);
      commanded_logical_targets[binding.pitch_joint_index] = pitch_target;
      commanded_logical_targets[binding.roll_joint_index] = roll_target;
      if (clamped) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Parallel target for %s was clamped to %s",
          binding.name.c_str(),
          format_pose(pitch_target, roll_target).c_str());
      }

      double motor_1_position = 0.0;
      double motor_2_position = 0.0;
      std::string error_message;
      const bool inverse_solved = binding.kinematics.inverse(
        pitch_target,
        roll_target,
        motor_1_position,
        motor_2_position,
        error_message);
      if (!inverse_solved) {
        if (danc_driving) {
          throw std::runtime_error("danc parallel inverse failed for " + binding.name + ": " +
                  error_message);
        }
        report_fault(
          "parallel inverse solve failed for " + binding.name + ": " + error_message,
          false);
        const auto motor_1_it = state_lookup.find(binding.motor_1_name);
        const auto motor_2_it = state_lookup.find(binding.motor_2_name);
        motor_1_position = motor_1_it != state_lookup.end() ? motor_1_it->second->position : 0.0;
        motor_2_position = motor_2_it != state_lookup.end() ? motor_2_it->second->position : 0.0;
      }

      const bool active_policy =
        (danc_driving || (mode == Mode::kLoco && !config_.policy.shadow_mode)) && inverse_solved;
      const auto & pitch_contract = config_.policy.actuators[binding.pitch_joint_index];
      const auto & roll_contract = config_.policy.actuators[binding.roll_joint_index];
      const double pitch_kp = pitch_contract.valid ?
        pitch_contract.kp : config_.parallel_control.pitch_kp;
      const double pitch_kd = pitch_contract.valid ?
        pitch_contract.kd : config_.parallel_control.pitch_kd;
      const double roll_kp = roll_contract.valid ?
        roll_contract.kp : config_.parallel_control.roll_kp;
      const double roll_kd = roll_contract.valid ?
        roll_contract.kd : config_.parallel_control.roll_kd;

      double motor_1_torque = 0.0;
      double motor_2_torque = 0.0;
      if (active_policy) {
        const auto motor_1_it = state_lookup.find(binding.motor_1_name);
        const auto motor_2_it = state_lookup.find(binding.motor_2_name);
        if (motor_1_it == state_lookup.end() || motor_2_it == state_lookup.end()) {
          throw std::runtime_error("parallel logical torque requires fresh motor feedback");
        }

        const double current_pitch = logical_state.positions[binding.pitch_joint_index];
        const double current_roll = logical_state.positions[binding.roll_joint_index];
        const double pitch_effort = danc_driving ?
          danc_effort(binding.pitch_joint_index, pitch_target, logical_state) :
          pitch_kp * (pitch_target - current_pitch) -
          pitch_kd * logical_state.velocities[binding.pitch_joint_index];
        const double roll_effort = danc_driving ?
          danc_effort(binding.roll_joint_index, roll_target, logical_state) :
          roll_kp * (roll_target - current_roll) -
          roll_kd * logical_state.velocities[binding.roll_joint_index];
        commanded_logical_torques[binding.pitch_joint_index] = pitch_effort;
        commanded_logical_torques[binding.roll_joint_index] = roll_effort;
        if (!binding.kinematics.joint_to_motor_efforts(
            current_pitch,
            current_roll,
            motor_1_it->second->position,
            motor_2_it->second->position,
            pitch_effort,
            roll_effort,
            motor_1_torque,
            motor_2_torque,
            error_message))
        {
          throw std::runtime_error(
                  "parallel effort mapping failed for " + binding.name + ": " + error_message);
        }
        // Position is ignored when Kp is zero; keeping feedback avoids a latent target jump.
        motor_1_position = motor_1_it->second->position;
        motor_2_position = motor_2_it->second->position;
      }

      NamedMitCommand motor_1_command;
      motor_1_command.name = binding.motor_1_name;
      motor_1_command.command.position = static_cast<float>(motor_1_position);
      motor_1_command.command.velocity = 0.0F;
      std::pair<double, double> motor_1_gains;
      if (active_policy) {
        motor_1_gains = {0.0, 0.0};
      } else {
        motor_1_gains = resolve_motor_gains(
          gains,
          binding.motor_1_name,
          gains.ankle_kp,
          gains.ankle_kd,
          binding.motor_1_config.default_kp,
          binding.motor_1_config.default_kd);
      }
      const auto [motor_1_kp, motor_1_kd] = motor_1_gains;
      motor_1_command.command.kp = static_cast<float>(motor_1_kp);
      motor_1_command.command.kd = static_cast<float>(motor_1_kd);
      motor_1_command.command.torque = static_cast<float>(motor_1_torque);
      commands.push_back(std::move(motor_1_command));

      NamedMitCommand motor_2_command;
      motor_2_command.name = binding.motor_2_name;
      motor_2_command.command.position = static_cast<float>(motor_2_position);
      motor_2_command.command.velocity = 0.0F;
      std::pair<double, double> motor_2_gains;
      if (active_policy) {
        motor_2_gains = {0.0, 0.0};
      } else {
        motor_2_gains = resolve_motor_gains(
          gains,
          binding.motor_2_name,
          gains.ankle_kp,
          gains.ankle_kd,
          binding.motor_2_config.default_kp,
          binding.motor_2_config.default_kd);
      }
      const auto [motor_2_kp, motor_2_kd] = motor_2_gains;
      motor_2_command.command.kp = static_cast<float>(motor_2_kp);
      motor_2_command.command.kd = static_cast<float>(motor_2_kd);
      motor_2_command.command.torque = static_cast<float>(motor_2_torque);
      commands.push_back(std::move(motor_2_command));
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_commanded_targets_ = std::move(commanded_logical_targets);
      latest_commanded_velocities_ = std::move(commanded_logical_velocities);
      latest_commanded_torques_ = std::move(commanded_logical_torques);
    }

    return commands;
  }

  double danc_effort(
    const std::size_t index, const double target, const LogicalJointState & state) const
  {
    const auto & joint = danc_policy_->hardware_joint(index);
    const double effort =
      joint.kp * (target - state.positions[index]) - joint.kd * state.velocities[index];
    if (!std::isfinite(effort)) {
      throw std::runtime_error("danc joint effort is not finite");
    }
    return std::clamp(effort, -joint.effort_limit, joint.effort_limit);
  }

  double resolve_gain(const double configured_gain, const double default_gain) const
  {
    return configured_gain < 0.0 ? default_gain : configured_gain;
  }

  std::pair<double, double> resolve_motor_gains(
    const ModeGains & gains,
    const std::string & motor_name,
    const double mode_kp,
    const double mode_kd,
    const double default_kp,
    const double default_kd) const
  {
    double kp = resolve_gain(mode_kp, default_kp);
    double kd = resolve_gain(mode_kd, default_kd);

    const auto override_it = gains.motor_gains.find(motor_name);
    if (override_it != gains.motor_gains.end()) {
      const auto & override = override_it->second;
      if (override.kp.has_value()) {
        kp = resolve_gain(*override.kp, default_kp);
      }
      if (override.kd.has_value()) {
        kd = resolve_gain(*override.kd, default_kd);
      }
    }

    return {kp, kd};
  }

  const ModeGains & mode_gains(const Mode mode) const
  {
    switch (mode) {
      case Mode::kPassive:
        return config_.passive_gains;
      case Mode::kFixed:
      case Mode::kDanc:
        return config_.fixed_gains;
      case Mode::kLoco:
      default:
        return config_.loco_gains;
    }
  }

  bool combo_pressed_passive(const JoySnapshot & joy) const
  {
    return read_button(joy, config_.joystick.mode_switch.a_button_index) &&
           read_button(joy, config_.joystick.mode_switch.lb_button_index);
  }

  bool combo_pressed_fixed(const JoySnapshot & joy) const
  {
    return read_button(joy, config_.joystick.mode_switch.a_button_index) &&
           read_axis(joy, config_.joystick.mode_switch.lt_axis_index) >=
           static_cast<float>(config_.joystick.mode_switch.lt_pressed_threshold);
  }

  bool combo_pressed_loco(const JoySnapshot & joy) const
  {
    return read_button(joy, config_.joystick.mode_switch.a_button_index) &&
           read_button(joy, config_.joystick.mode_switch.rb_button_index);
  }

  bool dpad_recording_start_pressed(const JoySnapshot & joy) const
  {
    return read_axis(joy, config_.joystick.recording.dpad_y_axis_index) <=
           static_cast<float>(config_.joystick.recording.up_pressed_threshold);
  }

  bool dpad_recording_stop_pressed(const JoySnapshot & joy) const
  {
    return read_axis(joy, config_.joystick.recording.dpad_y_axis_index) >=
           static_cast<float>(config_.joystick.recording.down_pressed_threshold);
  }

  bool read_button(const JoySnapshot & joy, const int index) const
  {
    if (!joy.valid || index < 0 || static_cast<std::size_t>(index) >= joy.buttons.size()) {
      return false;
    }
    return joy.buttons[static_cast<std::size_t>(index)] != 0;
  }

  float read_axis(const JoySnapshot & joy, const int index) const
  {
    if (!joy.valid || index < 0 || static_cast<std::size_t>(index) >= joy.axes.size()) {
      return 0.0F;
    }
    return joy.axes[static_cast<std::size_t>(index)];
  }

  void maybe_record_tracking_sample(
    const Mode mode,
    const SteadyClock::time_point timestamp,
    const ImuSnapshot & imu)
  {
    if (!config_.recording.enabled) {
      return;
    }
    if (config_.recording.record_only_loco && mode != Mode::kLoco) {
      return;
    }

    std::lock_guard<std::mutex> recording_lock(recording_mutex_);
    if (!active_recording_.has_value()) {
      if (config_.recording.record_only_loco) {
        return;
      }
      start_tracking_recording_locked("auto");
    }

    const auto period = to_steady_duration(1.0 / config_.recording.sample_rate_hz);
    if (active_recording_->samples.size() > 0U && timestamp < next_recording_sample_time_) {
      return;
    }
    next_recording_sample_time_ = timestamp + period;

    LogicalJointState joint_state;
    std::vector<double> targets;
    std::vector<double> velocity_targets;
    std::vector<double> torque_targets;
    std::vector<double> simulation_positions;
    std::vector<float> observation;
    std::vector<float> raw_action;
    SteadyClock::time_point loco_start_time;
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      joint_state = current_joint_state_;
      targets = latest_commanded_targets_;
      velocity_targets = latest_commanded_velocities_;
      torque_targets = latest_commanded_torques_;
      observation = latest_policy_observation_;
      raw_action = last_policy_raw_action_;
      loco_start_time = loco_mode_start_time_;
    }
    if (!joint_state.valid || joint_state.positions.empty()) {
      return;
    }
    if (targets.size() != joint_state.positions.size()) {
      targets = config_.policy.default_joint_positions;
    }
    if (velocity_targets.size() != joint_state.positions.size()) {
      velocity_targets.assign(joint_state.positions.size(), 0.0);
    }
    if (torque_targets.size() != joint_state.positions.size()) {
      torque_targets.assign(joint_state.positions.size(), 0.0);
    }
    if (mode == Mode::kLoco &&
      config_.policy.backend == PolicyBackend::kTrajectory &&
      config_.trajectory.source == TrajectorySource::kCsvPlayback)
    {
      const double elapsed_s = std::chrono::duration<double>(timestamp - loco_start_time).count();
      if (elapsed_s >= config_.trajectory.startup_hold_s) {
        simulation_positions = playback_reference_at(
          elapsed_s - config_.trajectory.startup_hold_s);
      }
    }
    if (simulation_positions.size() != joint_state.positions.size()) {
      simulation_positions.assign(
        joint_state.positions.size(), std::numeric_limits<double>::quiet_NaN());
    }
    if (active_recording_->joint_names.empty()) {
      active_recording_->joint_names = config_.policy.joint_names;
    }

    TrackingSample sample;
    sample.sample_index = active_recording_->samples.size();
    sample.time_s =
      std::chrono::duration<double>(timestamp - active_recording_->start_time).count();
    sample.command = current_recording_command();
    if (imu.valid) {
      const auto projected_gravity = compute_projected_gravity(imu.orientation);
      sample.projected_gravity = {
        static_cast<double>(projected_gravity[0]),
        static_cast<double>(projected_gravity[1]),
        static_cast<double>(projected_gravity[2])
      };
      sample.angular_velocity = imu.angular_velocity;
    }
    sample.policy_observation = std::move(observation);
    sample.raw_action = std::move(raw_action);
    sample.joint_positions = std::move(joint_state.positions);
    sample.joint_position_targets = std::move(targets);
    sample.simulation_joint_positions = std::move(simulation_positions);
    sample.joint_velocities = std::move(joint_state.velocities);
    sample.joint_velocity_targets = std::move(velocity_targets);
    sample.joint_torques = std::move(joint_state.efforts);
    sample.joint_torque_targets = std::move(torque_targets);
    active_recording_->samples.push_back(std::move(sample));
  }

  std::array<double, 3> current_recording_command()
  {
    if (config_.policy.backend == PolicyBackend::kTrajectory &&
      config_.trajectory.source == TrajectorySource::kCsvPlayback &&
      config_.trajectory.playback.command_valid)
    {
      return config_.trajectory.playback.command;
    }

    JoySnapshot joy;
    {
      std::lock_guard<std::mutex> lock(joy_mutex_);
      joy = latest_joy_;
    }
    const auto command = command_from_joy(joy);
    return {
      static_cast<double>(command[0]),
      static_cast<double>(command[1]),
      static_cast<double>(command[2])
    };
  }

  void start_tracking_recording(const std::string & reason)
  {
    if (!config_.recording.enabled) {
      return;
    }

    std::lock_guard<std::mutex> lock(recording_mutex_);
    start_tracking_recording_locked(reason);
  }

  void start_tracking_recording_locked(const std::string & reason)
  {
    if (active_recording_.has_value()) {
      return;
    }

    TrackingRun run;
    run.run_index = ++recording_run_counter_;
    run.shadow_mode = config_.policy.shadow_mode;
    run.run_id = make_timestamp_string() + (run.shadow_mode ? "_shadow" : "") + "_parallel_" +
      kParallelControlMode +
      "_run_" + std::to_string(run.run_index);
    run.start_wall_time = make_wall_time_string();
    run.start_reason = reason;
    run.start_time = SteadyClock::now();
    run.samples.reserve(
      static_cast<std::size_t>(
        std::max(1024.0, config_.recording.sample_rate_hz * 20.0)));

    active_recording_ = std::move(run);
    next_recording_sample_time_ = active_recording_->start_time;

    RCLCPP_INFO(
      get_logger(),
      "Started tracking recording: %s (%s)",
      active_recording_->run_id.c_str(),
      reason.c_str());
  }

  void stop_tracking_recording(const std::string & reason)
  {
    if (!config_.recording.enabled) {
      return;
    }

    std::optional<TrackingRun> completed_run;
    {
      std::lock_guard<std::mutex> lock(recording_mutex_);
      if (!active_recording_.has_value()) {
        return;
      }

      active_recording_->stop_reason = reason;
      active_recording_->stop_wall_time = make_wall_time_string();
      active_recording_->stop_time = SteadyClock::now();
      completed_run = std::move(active_recording_);
      active_recording_.reset();
    }

    if (completed_run.has_value()) {
      RCLCPP_INFO(
        get_logger(),
        "Stopped tracking recording: %s with %zu samples (%s).",
        completed_run->run_id.c_str(),
        completed_run->samples.size(),
        reason.c_str());

      {
        std::lock_guard<std::mutex> queue_lock(recording_queue_mutex_);
        completed_recording_runs_.push_back(std::move(*completed_run));
      }
      recording_queue_cv_.notify_one();
    }
  }

  void recording_writer_loop()
  {
    while (true) {
      TrackingRun run;
      {
        std::unique_lock<std::mutex> lock(recording_queue_mutex_);
        recording_queue_cv_.wait(
          lock,
          [this]() {
            return recording_writer_stop_requested_ || !completed_recording_runs_.empty();
          });

        if (completed_recording_runs_.empty()) {
          if (recording_writer_stop_requested_) {
            return;
          }
          continue;
        }

        run = std::move(completed_recording_runs_.front());
        completed_recording_runs_.pop_front();
      }

      try {
        write_tracking_run(run);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to write tracking recording %s: %s",
          run.run_id.c_str(),
          ex.what());
      }
    }
  }

  void write_tracking_run(const TrackingRun & run) const
  {
    if (run.samples.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Tracking recording %s has no samples; no report written.",
        run.run_id.c_str());
      return;
    }

    const std::filesystem::path output_root(expand_user_path(config_.recording.output_root));
    const std::filesystem::path output_dir = output_root / run.run_id;
    std::filesystem::create_directories(output_dir);

    if (config_.recording.write_csv) {
      write_tracking_csv(run, output_dir / "joint_timeseries.csv");
    }
    if (config_.recording.write_summary) {
      write_tracking_summary(run, output_dir / "summary.json");
    }
    if (config_.recording.write_readme) {
      write_tracking_readme(run, output_dir / "README.md");
    }
    if (config_.recording.write_html) {
      write_tracking_dashboard_html(run, output_dir / "index.html");
      write_tracking_plot_html(
        run,
        output_dir / "joint_position_compare.html",
        TrackingPlotKind::kPositionCompare,
        "Joint Position Tracking",
        "rad or m");
      write_tracking_plot_html(
        run,
        output_dir / "joint_velocities.html",
        TrackingPlotKind::kVelocityCompare,
        "Joint Velocity Tracking",
        "rad/s or m/s");
      write_tracking_plot_html(
        run,
        output_dir / "joint_torques.html",
        TrackingPlotKind::kTorqueCompare,
        "Joint Torque Tracking",
        "N*m or N");
      write_tracking_plot_html(
        run,
        output_dir / "joint_position_errors.html",
        TrackingPlotKind::kPositionError,
        "Joint Position Tracking Error",
        "target - actual");
      write_tracking_plot_html(
        run,
        output_dir / "simulation_position_errors.html",
        TrackingPlotKind::kSimulationPositionError,
        "Hardware vs Simulation Position Error",
        "simulation - hardware");
    }

    RCLCPP_INFO(
      get_logger(),
      "Tracking recording written: %s",
      output_dir.string().c_str());
  }

  void write_tracking_csv(const TrackingRun & run, const std::filesystem::path & path) const
  {
    std::ofstream stream(path);
    if (!stream.is_open()) {
      throw std::runtime_error("failed to open CSV for writing: " + path.string());
    }

    std::vector<std::string> header {
      "sample",
      "time_s",
      "shadow_mode",
      "command_vx",
      "command_vy",
      "command_wz",
      "imu_projected_gravity_x",
      "imu_projected_gravity_y",
      "imu_projected_gravity_z"
    };
    header.push_back("imu_angular_velocity_x");
    header.push_back("imu_angular_velocity_y");
    header.push_back("imu_angular_velocity_z");
    for (const auto & joint_name : run.joint_names) {
      header.push_back("joint_pos/" + joint_name);
    }
    for (const auto & joint_name : run.joint_names) {
      header.push_back("joint_pos_target/" + joint_name);
    }
    for (const auto & joint_name : run.joint_names) {
      header.push_back("sim_joint_pos/" + joint_name);
    }
    for (const auto & joint_name : run.joint_names) {
      header.push_back("joint_pos_error/" + joint_name);
    }
    for (const auto & joint_name : run.joint_names) {
      header.push_back("sim_hardware_pos_error/" + joint_name);
    }
    for (const auto & joint_name : run.joint_names) {
      header.push_back("joint_vel/" + joint_name);
    }
    for (const auto & joint_name : run.joint_names) {
      header.push_back("joint_vel_target/" + joint_name);
    }
    for (const auto & joint_name : run.joint_names) {
      header.push_back("joint_torque/" + joint_name);
    }
    for (const auto & joint_name : run.joint_names) {
      header.push_back("joint_torque_target/" + joint_name);
    }
    for (int i = 0; i < config_.policy.expected_observation_dim; ++i) {
      header.push_back("policy_observation/" + std::to_string(i));
    }
    for (const auto & joint_name : config_.policy.joint_names) {
      header.push_back("raw_action/" + joint_name);
    }
    write_csv_row(stream, header);

    for (const auto & sample : run.samples) {
      std::vector<std::string> row;
      row.reserve(header.size());
      row.push_back(std::to_string(sample.sample_index));
      row.push_back(format_double(sample.time_s));
      row.push_back(run.shadow_mode ? "true" : "false");
      row.push_back(format_double(sample.command[0]));
      row.push_back(format_double(sample.command[1]));
      row.push_back(format_double(sample.command[2]));
      row.push_back(format_double(sample.projected_gravity[0]));
      row.push_back(format_double(sample.projected_gravity[1]));
      row.push_back(format_double(sample.projected_gravity[2]));
      row.push_back(format_double(sample.angular_velocity[0]));
      row.push_back(format_double(sample.angular_velocity[1]));
      row.push_back(format_double(sample.angular_velocity[2]));

      append_csv_values(row, sample.joint_positions);
      append_csv_values(row, sample.joint_position_targets);
      append_csv_values(row, sample.simulation_joint_positions);
      for (std::size_t i = 0; i < run.joint_names.size(); ++i) {
        row.push_back(format_double(sample.joint_position_targets[i] - sample.joint_positions[i]));
      }
      for (std::size_t i = 0; i < run.joint_names.size(); ++i) {
        row.push_back(
          format_double(
            sample.simulation_joint_positions[i] - sample.joint_positions[i]));
      }
      append_csv_values(row, sample.joint_velocities);
      append_csv_values(row, sample.joint_velocity_targets);
      append_csv_values(row, sample.joint_torques);
      append_csv_values(row, sample.joint_torque_targets);
      for (int i = 0; i < config_.policy.expected_observation_dim; ++i) {
        row.push_back(
          i < static_cast<int>(sample.policy_observation.size()) ?
          format_double(sample.policy_observation[static_cast<std::size_t>(i)]) : "nan");
      }
      for (std::size_t i = 0; i < config_.policy.joint_names.size(); ++i) {
        row.push_back(i < sample.raw_action.size() ? format_double(sample.raw_action[i]) : "nan");
      }
      write_csv_row(stream, row);
    }
  }

  void append_csv_values(
    std::vector<std::string> & row,
    const std::vector<double> & values) const
  {
    for (const double value : values) {
      row.push_back(format_double(value));
    }
  }

  void write_tracking_summary(const TrackingRun & run, const std::filesystem::path & path) const
  {
    const double duration_s = run.samples.empty() ? 0.0 : run.samples.back().time_s;
    const double sample_rate_hz = duration_s > 1e-9 && run.samples.size() > 1U ?
      static_cast<double>(run.samples.size() - 1U) / duration_s :
      0.0;

    std::ofstream stream(path);
    if (!stream.is_open()) {
      throw std::runtime_error("failed to open summary for writing: " + path.string());
    }

    stream << "{\n";
    stream << "  \"run_id\": \"" << escape_json(run.run_id) << "\",\n";
    stream << "  \"shadow_mode\": " << (run.shadow_mode ? "true" : "false") << ",\n";
    stream << "  \"parallel_control_mode\": \"" <<
      kParallelControlMode << "\",\n";
    stream << "  \"start_wall_time\": \"" << escape_json(run.start_wall_time) << "\",\n";
    stream << "  \"stop_wall_time\": \"" << escape_json(run.stop_wall_time) << "\",\n";
    stream << "  \"start_reason\": \"" << escape_json(run.start_reason) << "\",\n";
    stream << "  \"stop_reason\": \"" << escape_json(run.stop_reason) << "\",\n";
    stream << "  \"backend\": \"" << escape_json(to_string(config_.policy.backend)) << "\",\n";
    stream << "  \"trajectory_source\": \"" << escape_json(to_string(config_.trajectory.source)) <<
      "\",\n";
    if (config_.trajectory.source == TrajectorySource::kCsvPlayback) {
      stream << "  \"trajectory_command_source\": \"" <<
        escape_json(to_string(config_.trajectory.playback.command_source)) << "\",\n";
      stream << "  \"playback_speed_scale\": " <<
        format_double(config_.trajectory.playback.speed_scale) << ",\n";
      stream << "  \"source_trajectory_duration_s\": " <<
        format_double(config_.trajectory.playback.duration_s) << ",\n";
    }
    stream << "  \"duration_s\": " << format_double(duration_s) << ",\n";
    stream << "  \"requested_sample_rate_hz\": " <<
      format_double(config_.recording.sample_rate_hz) << ",\n";
    stream << "  \"sample_rate_hz\": " << format_double(sample_rate_hz) << ",\n";
    stream << "  \"num_samples\": " << run.samples.size() << ",\n";
    stream << "  \"joint_names\": [";
    for (std::size_t i = 0; i < run.joint_names.size(); ++i) {
      if (i > 0U) {
        stream << ", ";
      }
      stream << "\"" << escape_json(run.joint_names[i]) << "\"";
    }
    stream << "],\n";
    stream << "  \"tracking_metrics\": {\n";
    for (std::size_t joint_index = 0; joint_index < run.joint_names.size(); ++joint_index) {
      double target_squared_error_sum = 0.0;
      double target_max_abs_error = 0.0;
      std::size_t target_count = 0U;
      double simulation_squared_error_sum = 0.0;
      double simulation_max_abs_error = 0.0;
      std::size_t simulation_count = 0U;
      for (const auto & sample : run.samples) {
        const double actual = sample.joint_positions[joint_index];
        const double target = sample.joint_position_targets[joint_index];
        if (std::isfinite(actual) && std::isfinite(target)) {
          const double error = target - actual;
          target_squared_error_sum += error * error;
          target_max_abs_error = std::max(target_max_abs_error, std::abs(error));
          ++target_count;
        }
        const double simulation = sample.simulation_joint_positions[joint_index];
        if (std::isfinite(actual) && std::isfinite(simulation)) {
          const double error = simulation - actual;
          simulation_squared_error_sum += error * error;
          simulation_max_abs_error = std::max(simulation_max_abs_error, std::abs(error));
          ++simulation_count;
        }
      }
      stream << "    \"" << escape_json(run.joint_names[joint_index]) << "\": {";
      stream << "\"target_rmse\": " <<
        (target_count > 0U ?
      format_double(std::sqrt(target_squared_error_sum / static_cast<double>(target_count))) :
      "null");
      stream << ", \"target_max_abs_error\": " <<
        (target_count > 0U ? format_double(target_max_abs_error) : "null");
      stream << ", \"simulation_rmse\": " <<
        (simulation_count > 0U ?
      format_double(
        std::sqrt(simulation_squared_error_sum / static_cast<double>(simulation_count))) :
      "null");
      stream << ", \"simulation_max_abs_error\": " <<
        (simulation_count > 0U ? format_double(simulation_max_abs_error) : "null");
      stream << ", \"simulation_samples\": " << simulation_count << "}";
      stream << (joint_index + 1U < run.joint_names.size() ? ",\n" : "\n");
    }
    stream << "  },\n";
    stream << "  \"units\": {\n";
    stream << "    \"joint_pos\": \"logical joint actual position; rad or m\",\n";
    stream <<
      "    \"joint_pos_target\": \"logical joint commanded position; rad or m\",\n";
    stream <<
      "    \"sim_joint_pos\": \"simulated logical joint position when available\",\n";
    stream << "    \"joint_pos_error\": \"joint_pos_target - joint_pos\",\n";
    stream << "    \"sim_hardware_pos_error\": \"sim_joint_pos - joint_pos\",\n";
    stream << "    \"joint_vel\": \"logical joint actual velocity; rad/s or m/s\",\n";
    stream << "    \"joint_vel_target\": \"logical joint commanded velocity; rad/s or m/s\",\n";
    stream <<
      "    \"joint_torque\": \"logical joint actual effort; N*m or N\",\n";
    stream <<
      "    \"joint_torque_target\": \"expected logical actuator effort from the command (PD plus feedforward); N*m or N\",\n";
    stream <<
      "    \"imu_projected_gravity\": \"unit gravity vector in body frame from IMU orientation\"\n";
    stream << "  },\n";
    stream << "  \"artifacts\": {\n";
    stream << "    \"joint_csv\": \"joint_timeseries.csv\",\n";
    stream << "    \"interactive_html\": \"index.html\",\n";
    stream << "    \"joint_position_compare_html\": \"joint_position_compare.html\",\n";
    stream << "    \"joint_velocities_html\": \"joint_velocities.html\",\n";
    stream << "    \"joint_torques_html\": \"joint_torques.html\",\n";
    stream << "    \"joint_position_errors_html\": \"joint_position_errors.html\",\n";
    stream << "    \"simulation_position_errors_html\": \"simulation_position_errors.html\",\n";
    stream << "    \"readme\": \"README.md\"\n";
    stream << "  }\n";
    stream << "}\n";
  }

  void write_tracking_readme(const TrackingRun & run, const std::filesystem::path & path) const
  {
    std::ofstream stream(path);
    if (!stream.is_open()) {
      throw std::runtime_error("failed to open README for writing: " + path.string());
    }

    const double duration_s = run.samples.empty() ? 0.0 : run.samples.back().time_s;
    stream << "# Hardware Tracking Report" << (run.shadow_mode ? " (SHADOW)" : "") << "\n\n";
    stream << "Generated by `jljwholebody_deploy_node` while tracking on hardware.\n\n";
    if (run.shadow_mode) {
      stream << "> **SHADOW RUN:** Policy observations and actions were recorded, but policy "
             << "targets were not applied to the motors.\n\n";
    }
    stream << "## Run\n\n";
    stream << "- `run_id`: " << run.run_id << "\n";
    stream << "- `shadow_mode`: " << (run.shadow_mode ? "true" : "false") << "\n";
    stream << "- `parallel_control_mode`: " << kParallelControlMode << "\n";
    stream << "- `start_wall_time`: " << run.start_wall_time << "\n";
    stream << "- `stop_wall_time`: " << run.stop_wall_time << "\n";
    stream << "- `duration_s`: " << format_double(duration_s, 6) << "\n";
    stream << "- `samples`: " << run.samples.size() << "\n";
    stream << "- `backend`: " << to_string(config_.policy.backend) << "\n";
    stream << "- `trajectory_source`: " << to_string(config_.trajectory.source) << "\n";
    if (config_.trajectory.source == TrajectorySource::kCsvPlayback) {
      stream << "- `trajectory_command_source`: " <<
        to_string(config_.trajectory.playback.command_source) << "\n";
      stream << "- `playback_speed_scale`: " <<
        format_double(config_.trajectory.playback.speed_scale) << "\n";
    }
    stream << "\n";
    stream << "## Output Files\n\n";
    stream <<
      "- `index.html`: interactive joint/metric selector comparing commanded and actual values.\n";
    stream <<
      "- `joint_timeseries.csv`: commanded and actual position, velocity, and effort in logical-joint space.\n";
    stream <<
      "- `joint_timeseries.csv` also includes `imu_projected_gravity_x/y/z` from the IMU orientation used by deploy.\n";
    stream <<
      "- `joint_position_compare.html`: logical actual, command, and optional simulation position curves.\n";
    stream <<
      "- Parallel mechanisms are recorded as virtual pitch/roll joints, not their two motor coordinates.\n";
    stream << "- `joint_velocities.html`: commanded and actual logical velocity curves.\n";
    stream << "- `joint_torques.html`: commanded and actual logical effort curves.\n";
    stream << "- `joint_position_errors.html`: target minus actual position curves.\n";
    stream <<
      "- `simulation_position_errors.html`: simulated actual minus hardware actual position curves.\n";
    stream << "- `summary.json`: metadata, per-joint RMSE/max error, and artifact list.\n";
  }

  void write_tracking_dashboard_html(
    const TrackingRun & run,
    const std::filesystem::path & path) const
  {
    std::ofstream stream(path);
    if (!stream.is_open()) {
      throw std::runtime_error("failed to open interactive HTML for writing: " + path.string());
    }

    const auto write_number = [&stream](const double value) {
        if (std::isfinite(value)) {
          stream << format_double(value);
        } else {
          stream << "null";
        }
      };
    const auto write_matrix = [&stream, &run, &write_number](const auto & getter) {
        stream << '[';
        for (std::size_t sample_index = 0; sample_index < run.samples.size(); ++sample_index) {
          if (sample_index > 0U) {
            stream << ',';
          }
          stream << '[';
          const auto & values = getter(run.samples[sample_index]);
          for (std::size_t joint_index = 0; joint_index < run.joint_names.size(); ++joint_index) {
            if (joint_index > 0U) {
              stream << ',';
            }
            write_number(values[joint_index]);
          }
          stream << ']';
        }
        stream << ']';
      };

    stream << R"HTML(<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>关节期望/实际数据对比</title>
<style>
:root{color-scheme:light;font-family:system-ui,-apple-system,"Segoe UI",sans-serif;color:#172033;background:#f3f6fa}
*{box-sizing:border-box}body{margin:0}.page{max-width:1280px;margin:auto;padding:22px}
h1{font-size:24px;margin:0 0 5px}.meta{color:#667085;font-size:13px;margin-bottom:18px}
.controls,.card{background:#fff;border:1px solid #d8dee9;border-radius:10px;box-shadow:0 2px 10px #24324a0a}
.controls{padding:14px;display:flex;gap:14px;flex-wrap:wrap;align-items:end;margin-bottom:14px}
label{display:grid;gap:5px;font-size:12px;color:#526071}select{min-width:210px;padding:8px;border:1px solid #bcc6d6;border-radius:6px;background:#fff;color:#172033}
.checks{display:flex;gap:14px;padding:7px 0;font-size:13px}.checks label{display:flex;align-items:center;gap:5px}
.card{padding:15px}.legend{display:flex;gap:18px;flex-wrap:wrap;font-size:12px;color:#526071;margin:2px 0 10px}.swatch{display:inline-block;width:18px;height:3px;margin-right:6px;vertical-align:middle}
.canvas-wrap{position:relative;width:100%;height:min(62vh,620px);min-height:360px}canvas{width:100%;height:100%;display:block}.hint{font-size:12px;color:#667085;margin-top:10px}
@media(max-width:620px){.page{padding:12px}.controls{gap:10px}select{min-width:160px}.canvas-wrap{min-height:300px}}
</style></head><body><div class="page">
<h1>关节期望/实际数据对比</h1><div class="meta" id="meta"></div>
<div class="controls">
<label>指标<select id="metric"><option value="position">位置 (rad / m)</option><option value="velocity">速度 (rad/s / m/s)</option><option value="torque">力矩/力 (N·m / N)</option></select></label>
<label>关节 A<select id="jointA"></select></label><label>对比关节 B<select id="jointB"><option value="">不选择</option></select></label>
<div class="checks"><label><input id="showActual" type="checkbox" checked>实际</label><label><input id="showTarget" type="checkbox" checked>期望</label></div>
</div><div class="card"><div class="legend" id="legend"></div><div class="canvas-wrap"><canvas id="chart"></canvas></div>
<div class="hint">数据在本文件内，可离线打开。并联机构显示虚拟 pitch/roll 逻辑关节。</div></div></div>
<script>
const DATA=)HTML";
    stream << "{\"runId\":\"" << escape_json(run.run_id) << "\",\"rate\":";
    write_number(config_.recording.sample_rate_hz);
    stream << ",\"joints\":[";
    for (std::size_t i = 0; i < run.joint_names.size(); ++i) {
      if (i > 0U) {
        stream << ',';
      }
      stream << '"' << escape_json(run.joint_names[i]) << '"';
    }
    stream << "],\"time\":[";
    for (std::size_t i = 0; i < run.samples.size(); ++i) {
      if (i > 0U) {
        stream << ',';
      }
      write_number(run.samples[i].time_s);
    }
    stream << "],\"position\":{\"actual\":";
    write_matrix([](const TrackingSample & sample) -> const auto & {
      return sample.joint_positions;
    });
    stream << ",\"target\":";
    write_matrix([](const TrackingSample & sample) -> const auto & {
      return sample.joint_position_targets;
    });
    stream << "},\"velocity\":{\"actual\":";
    write_matrix([](const TrackingSample & sample) -> const auto & {
      return sample.joint_velocities;
    });
    stream << ",\"target\":";
    write_matrix([](const TrackingSample & sample) -> const auto & {
      return sample.joint_velocity_targets;
    });
    stream << "},\"torque\":{\"actual\":";
    write_matrix([](const TrackingSample & sample) -> const auto & {
      return sample.joint_torques;
    });
    stream << ",\"target\":";
    write_matrix([](const TrackingSample & sample) -> const auto & {
      return sample.joint_torque_targets;
    });
    stream << R"HTML(}};
const metric=document.getElementById('metric'),jointA=document.getElementById('jointA'),jointB=document.getElementById('jointB');
const actualBox=document.getElementById('showActual'),targetBox=document.getElementById('showTarget'),canvas=document.getElementById('chart'),ctx=canvas.getContext('2d');
DATA.joints.forEach((name,i)=>{for(const select of [jointA,jointB]){const o=document.createElement('option');o.value=i;o.textContent=name;select.appendChild(o)}});
if(DATA.joints.length>1)jointB.value='';
document.getElementById('meta').textContent=`${DATA.runId} · ${DATA.time.length} 个样本 · 目标采样率 ${DATA.rate} Hz`;
const palette={aActual:'#2563eb',aTarget:'#dc2626',bActual:'#059669',bTarget:'#d97706'};
function finite(v){return typeof v==='number'&&Number.isFinite(v)}
function selectedSeries(){const m=DATA[metric.value],out=[],a=Number(jointA.value),b=jointB.value===''?null:Number(jointB.value);if(actualBox.checked)out.push({name:`${DATA.joints[a]} 实际`,values:m.actual.map(r=>r[a]),color:palette.aActual,dash:[]});if(targetBox.checked)out.push({name:`${DATA.joints[a]} 期望`,values:m.target.map(r=>r[a]),color:palette.aTarget,dash:[7,5]});if(b!==null&&actualBox.checked)out.push({name:`${DATA.joints[b]} 实际`,values:m.actual.map(r=>r[b]),color:palette.bActual,dash:[]});if(b!==null&&targetBox.checked)out.push({name:`${DATA.joints[b]} 期望`,values:m.target.map(r=>r[b]),color:palette.bTarget,dash:[7,5]});return out}
function render(){const box=canvas.getBoundingClientRect(),dpr=window.devicePixelRatio||1;canvas.width=Math.max(1,Math.round(box.width*dpr));canvas.height=Math.max(1,Math.round(box.height*dpr));ctx.setTransform(dpr,0,0,dpr,0,0);const w=box.width,h=box.height,p={l:68,r:22,t:20,b:42},pw=w-p.l-p.r,ph=h-p.t-p.b,series=selectedSeries();let lo=Infinity,hi=-Infinity;for(const s of series)for(const v of s.values)if(finite(v)){lo=Math.min(lo,v);hi=Math.max(hi,v)}if(!finite(lo)){lo=-1;hi=1}if(Math.abs(hi-lo)<1e-9){lo-=.5;hi+=.5}else{const pad=(hi-lo)*.08;lo-=pad;hi+=pad}const t0=DATA.time[0]||0,t1=Math.max(DATA.time.at(-1)||0,t0+1e-6),x=t=>p.l+(t-t0)/(t1-t0)*pw,y=v=>p.t+(hi-v)/(hi-lo)*ph;ctx.clearRect(0,0,w,h);ctx.font='12px system-ui';ctx.fillStyle='#667085';ctx.strokeStyle='#e3e8ef';ctx.lineWidth=1;for(let i=0;i<=5;i++){const gy=p.t+i*ph/5;ctx.beginPath();ctx.moveTo(p.l,gy);ctx.lineTo(w-p.r,gy);ctx.stroke();const value=hi-i*(hi-lo)/5;ctx.fillText(value.toPrecision(4),5,gy+4)}for(let i=0;i<=5;i++){const gx=p.l+i*pw/5;ctx.beginPath();ctx.moveTo(gx,p.t);ctx.lineTo(gx,h-p.b);ctx.stroke();ctx.fillText((t0+i*(t1-t0)/5).toFixed(2)+'s',gx-13,h-14)}for(const s of series){ctx.beginPath();ctx.strokeStyle=s.color;ctx.lineWidth=1.8;ctx.setLineDash(s.dash);let open=false;s.values.forEach((v,i)=>{if(!finite(v)){open=false;return}const px=x(DATA.time[i]),py=y(v);if(open)ctx.lineTo(px,py);else{ctx.moveTo(px,py);open=true}});ctx.stroke()}ctx.setLineDash([]);document.getElementById('legend').innerHTML=series.map(s=>`<span><i class="swatch" style="background:${s.color}"></i>${s.name}</span>`).join('')}
for(const el of [metric,jointA,jointB,actualBox,targetBox])el.addEventListener('change',render);window.addEventListener('resize',render);render();
</script></body></html>)HTML";
  }

  void write_tracking_plot_html(
    const TrackingRun & run,
    const std::filesystem::path & path,
    const TrackingPlotKind kind,
    const std::string & title,
    const std::string & ylabel) const
  {
    std::ofstream stream(path);
    if (!stream.is_open()) {
      throw std::runtime_error("failed to open HTML for writing: " + path.string());
    }

    const std::size_t joint_count = run.joint_names.size();
    const std::size_t max_points_per_series = 1600U;
    const std::size_t stride = std::max<std::size_t>(
      1U,
      (run.samples.size() + max_points_per_series - 1U) / max_points_per_series);

    stream << "<!doctype html>\n<html lang=\"en\">\n<head>\n";
    stream << "<meta charset=\"utf-8\">\n";
    stream << "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n";
    stream << "<title>" << escape_html(title) << "</title>\n";
    stream << "<style>\n";
    stream <<
      "body{font-family:system-ui,-apple-system,BlinkMacSystemFont,\"Segoe UI\",sans-serif;margin:0;background:#f6f7f9;color:#20242a;}\n";
    stream <<
      "header{position:sticky;top:0;background:#ffffffee;border-bottom:1px solid #d8dde6;padding:14px 20px;z-index:2;}\n";
    stream <<
      "h1{font-size:20px;margin:0 0 4px 0;font-weight:650;} .meta{font-size:13px;color:#59616d;}\n";
    stream <<
      "main{padding:16px 20px 28px 20px;display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:14px;}\n";
    stream <<
      ".plot{background:#fff;border:1px solid #d8dde6;border-radius:6px;padding:10px 12px 12px 12px;}\n";
    stream << ".plot h2{font-size:14px;margin:0 0 6px 0;font-weight:620;color:#20242a;}\n";
    stream << ".legend{font-size:12px;color:#59616d;margin-top:6px;display:flex;gap:14px;}\n";
    stream <<
      ".line-actual{stroke:#2563eb}.line-target{stroke:#dc2626}.line-simulation{stroke:#15803d}.line-single{stroke:#2563eb}.line-error{stroke:#7c3aed}\n";
    stream <<
      "svg{display:block;width:100%;height:260px;} .axis{stroke:#9aa3af;stroke-width:1}.grid{stroke:#e6e9ee;stroke-width:1}.label{fill:#59616d;font-size:11px}.zero{stroke:#adb5c2;stroke-width:1;stroke-dasharray:4 4}\n";
    stream <<
      "@media (max-width: 520px){main{grid-template-columns:1fr;padding:12px}.plot{padding:8px}svg{height:220px}}\n";
    stream << "</style>\n</head>\n<body>\n<header>\n";
    stream << "<h1>" << escape_html(title) << "</h1>\n";
    stream << "<div class=\"meta\">run " << escape_html(run.run_id)
           << " | samples " << run.samples.size()
           << " | shown every " << stride << " sample(s)"
           << " | y: " << escape_html(ylabel) << "</div>\n";
    stream << "</header>\n<main>\n";

    for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
      write_joint_svg(stream, run, kind, joint_index, stride);
    }

    stream << "</main>\n</body>\n</html>\n";
  }

  void write_joint_svg(
    std::ostream & stream,
    const TrackingRun & run,
    const TrackingPlotKind kind,
    const std::size_t joint_index,
    const std::size_t stride) const
  {
    constexpr double width = 720.0;
    constexpr double height = 260.0;
    constexpr double left = 54.0;
    constexpr double right = 14.0;
    constexpr double top = 18.0;
    constexpr double bottom = 32.0;
    const double plot_width = width - left - right;
    const double plot_height = height - top - bottom;

    const double t_min = run.samples.front().time_s;
    const double t_max = std::max(run.samples.back().time_s, t_min + 1e-6);
    double y_min = std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();

    const auto update_range = [&](const double value) {
        if (!std::isfinite(value)) {
          return;
        }
        y_min = std::min(y_min, value);
        y_max = std::max(y_max, value);
      };

    for (std::size_t i = 0; i < run.samples.size(); i += stride) {
      const auto & sample = run.samples[i];
      if (kind == TrackingPlotKind::kPositionCompare) {
        update_range(sample.joint_positions[joint_index]);
        update_range(sample.joint_position_targets[joint_index]);
        update_range(sample.simulation_joint_positions[joint_index]);
      } else if (kind == TrackingPlotKind::kVelocityCompare ||
        kind == TrackingPlotKind::kTorqueCompare)
      {
        update_range(tracking_plot_value(sample, kind, joint_index));
        update_range(tracking_target_value(sample, kind, joint_index));
      } else {
        update_range(tracking_plot_value(sample, kind, joint_index));
      }
    }

    if (!std::isfinite(y_min) || !std::isfinite(y_max)) {
      y_min = -1.0;
      y_max = 1.0;
    }
    if (std::abs(y_max - y_min) < 1e-9) {
      y_min -= 0.5;
      y_max += 0.5;
    } else {
      const double pad = 0.08 * (y_max - y_min);
      y_min -= pad;
      y_max += pad;
    }

    const auto point = [&](const double time_s, const double value) {
        const double x = left + (time_s - t_min) / (t_max - t_min) * plot_width;
        const double y = top + (y_max - value) / (y_max - y_min) * plot_height;
        std::ostringstream point_stream;
        point_stream << std::fixed << std::setprecision(2) << x << "," << y;
        return point_stream.str();
      };

    stream << "<section class=\"plot\">\n";
    stream << "<h2>" << escape_html(run.joint_names[joint_index]) << "</h2>\n";
    stream << "<svg viewBox=\"0 0 " << width << " " << height << "\" role=\"img\">\n";

    for (int grid = 0; grid <= 4; ++grid) {
      const double y = top + static_cast<double>(grid) * plot_height / 4.0;
      stream << "<line class=\"grid\" x1=\"" << left << "\" y1=\"" << y
             << "\" x2=\"" << (width - right) << "\" y2=\"" << y << "\"/>\n";
    }
    for (int grid = 0; grid <= 4; ++grid) {
      const double x = left + static_cast<double>(grid) * plot_width / 4.0;
      stream << "<line class=\"grid\" x1=\"" << x << "\" y1=\"" << top
             << "\" x2=\"" << x << "\" y2=\"" << (height - bottom) << "\"/>\n";
    }
    if (y_min < 0.0 && y_max > 0.0) {
      const double y_zero = top + (y_max / (y_max - y_min)) * plot_height;
      stream << "<line class=\"zero\" x1=\"" << left << "\" y1=\"" << y_zero
             << "\" x2=\"" << (width - right) << "\" y2=\"" << y_zero << "\"/>\n";
    }
    stream << "<line class=\"axis\" x1=\"" << left << "\" y1=\"" << (height - bottom)
           << "\" x2=\"" << (width - right) << "\" y2=\"" << (height - bottom) << "\"/>\n";
    stream << "<line class=\"axis\" x1=\"" << left << "\" y1=\"" << top
           << "\" x2=\"" << left << "\" y2=\"" << (height - bottom) << "\"/>\n";
    stream << "<text class=\"label\" x=\"4\" y=\"" << (top + 10.0) << "\">"
           << escape_html(format_double(y_max, 4)) << "</text>\n";
    stream << "<text class=\"label\" x=\"4\" y=\"" << (height - bottom) << "\">"
           << escape_html(format_double(y_min, 4)) << "</text>\n";
    stream << "<text class=\"label\" x=\"" << left << "\" y=\"" << (height - 8.0)
           << "\">" << escape_html(format_double(t_min, 3)) << "s</text>\n";
    stream << "<text class=\"label\" x=\"" << (width - right - 48.0) << "\" y=\""
           << (height - 8.0) << "\">" << escape_html(format_double(t_max, 3)) << "s</text>\n";

    if (kind == TrackingPlotKind::kPositionCompare) {
      stream << "<polyline class=\"line-actual\" fill=\"none\" stroke-width=\"1.8\" points=\"";
      write_polyline_points(stream, run, stride, joint_index, point, false);
      stream << "\"/>\n";
      stream <<
        "<polyline class=\"line-target\" fill=\"none\" stroke-width=\"1.5\" stroke-dasharray=\"5 4\" points=\"";
      write_polyline_points(stream, run, stride, joint_index, point, true);
      stream << "\"/>\n";
      stream << "<polyline class=\"line-simulation\" fill=\"none\" stroke-width=\"1.5\" points=\"";
      write_simulation_polyline_points(stream, run, stride, joint_index, point);
      stream << "\"/>\n";
    } else if (kind == TrackingPlotKind::kVelocityCompare ||
      kind == TrackingPlotKind::kTorqueCompare)
    {
      stream << "<polyline class=\"line-actual\" fill=\"none\" stroke-width=\"1.8\" points=\"";
      write_polyline_points(stream, run, stride, joint_index, point, kind);
      stream << "\"/>\n";
      stream <<
        "<polyline class=\"line-target\" fill=\"none\" stroke-width=\"1.5\" stroke-dasharray=\"5 4\" points=\"";
      write_target_polyline_points(stream, run, stride, joint_index, point, kind);
      stream << "\"/>\n";
    } else {
      const char * cls =
        (kind == TrackingPlotKind::kPositionError ||
        kind == TrackingPlotKind::kSimulationPositionError) ? "line-error" : "line-single";
      stream << "<polyline class=\"" << cls << "\" fill=\"none\" stroke-width=\"1.7\" points=\"";
      write_polyline_points(stream, run, stride, joint_index, point, kind);
      stream << "\"/>\n";
    }

    stream << "</svg>\n";
    if (kind == TrackingPlotKind::kPositionCompare) {
      stream <<
        "<div class=\"legend\"><span style=\"color:#2563eb\">hardware actual</span><span style=\"color:#dc2626\">command target</span><span style=\"color:#15803d\">simulation actual</span></div>\n";
    } else if (kind == TrackingPlotKind::kVelocityCompare ||
      kind == TrackingPlotKind::kTorqueCompare)
    {
      stream <<
        "<div class=\"legend\"><span style=\"color:#2563eb\">actual</span><span style=\"color:#dc2626\">command target</span></div>\n";
    } else if (kind == TrackingPlotKind::kPositionError) {
      stream <<
        "<div class=\"legend\"><span style=\"color:#7c3aed\">target - actual</span></div>\n";
    } else if (kind == TrackingPlotKind::kSimulationPositionError) {
      stream <<
        "<div class=\"legend\"><span style=\"color:#7c3aed\">simulation - hardware</span></div>\n";
    }
    stream << "</section>\n";
  }

  template<typename PointFn>
  void write_target_polyline_points(
    std::ostream & stream,
    const TrackingRun & run,
    const std::size_t stride,
    const std::size_t joint_index,
    const PointFn & point,
    const TrackingPlotKind kind) const
  {
    bool first = true;
    for (std::size_t i = 0; i < run.samples.size(); i += stride) {
      const auto & sample = run.samples[i];
      const double value = tracking_target_value(sample, kind, joint_index);
      if (!std::isfinite(value)) {
        continue;
      }
      if (!first) {
        stream << ' ';
      }
      stream << point(sample.time_s, value);
      first = false;
    }
  }

  template<typename PointFn>
  void write_polyline_points(
    std::ostream & stream,
    const TrackingRun & run,
    const std::size_t stride,
    const std::size_t joint_index,
    const PointFn & point,
    const bool use_target) const
  {
    bool first = true;
    for (std::size_t i = 0; i < run.samples.size(); i += stride) {
      const auto & sample = run.samples[i];
      const double value = use_target ?
        sample.joint_position_targets[joint_index] :
        sample.joint_positions[joint_index];
      if (!std::isfinite(value)) {
        continue;
      }
      if (!first) {
        stream << ' ';
      }
      stream << point(sample.time_s, value);
      first = false;
    }
  }

  template<typename PointFn>
  void write_simulation_polyline_points(
    std::ostream & stream,
    const TrackingRun & run,
    const std::size_t stride,
    const std::size_t joint_index,
    const PointFn & point) const
  {
    bool first = true;
    for (std::size_t i = 0; i < run.samples.size(); i += stride) {
      const auto & sample = run.samples[i];
      const double value = sample.simulation_joint_positions[joint_index];
      if (!std::isfinite(value)) {
        continue;
      }
      if (!first) {
        stream << ' ';
      }
      stream << point(sample.time_s, value);
      first = false;
    }
  }

  template<typename PointFn>
  void write_polyline_points(
    std::ostream & stream,
    const TrackingRun & run,
    const std::size_t stride,
    const std::size_t joint_index,
    const PointFn & point,
    const TrackingPlotKind kind) const
  {
    bool first = true;
    for (std::size_t i = 0; i < run.samples.size(); i += stride) {
      const auto & sample = run.samples[i];
      const double value = tracking_plot_value(sample, kind, joint_index);
      if (!std::isfinite(value)) {
        continue;
      }
      if (!first) {
        stream << ' ';
      }
      stream << point(sample.time_s, value);
      first = false;
    }
  }

  double tracking_plot_value(
    const TrackingSample & sample,
    const TrackingPlotKind kind,
    const std::size_t joint_index) const
  {
    switch (kind) {
      case TrackingPlotKind::kVelocityCompare:
        return sample.joint_velocities[joint_index];
      case TrackingPlotKind::kTorqueCompare:
        return sample.joint_torques[joint_index];
      case TrackingPlotKind::kPositionError:
        return sample.joint_position_targets[joint_index] - sample.joint_positions[joint_index];
      case TrackingPlotKind::kSimulationPositionError:
        return sample.simulation_joint_positions[joint_index] - sample.joint_positions[joint_index];
      case TrackingPlotKind::kPositionCompare:
      default:
        return sample.joint_positions[joint_index];
    }
  }

  double tracking_target_value(
    const TrackingSample & sample,
    const TrackingPlotKind kind,
    const std::size_t joint_index) const
  {
    switch (kind) {
      case TrackingPlotKind::kVelocityCompare:
        return sample.joint_velocity_targets[joint_index];
      case TrackingPlotKind::kTorqueCompare:
        return sample.joint_torque_targets[joint_index];
      case TrackingPlotKind::kPositionCompare:
      default:
        return sample.joint_position_targets[joint_index];
    }
  }

  void transition_to_mode(
    const Mode next_mode, const std::string & reason,
    const std::optional<Mode> expected_mode = std::nullopt)
  {
    const auto transition_time = SteadyClock::now();

    Mode previous_mode;
    LogicalJointState joint_state;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      previous_mode = current_mode_.load();
      if (previous_mode == next_mode || (expected_mode && previous_mode != *expected_mode)) {
        return;
      }
      joint_state = current_joint_state_;
      if ((next_mode == Mode::kFixed && config_.fixed_gains.hold_current_pose_on_entry) ||
        (next_mode == Mode::kLoco && config_.loco_gains.hold_current_pose_on_entry))
      {
        fixed_hold_positions_ = current_joint_state_.positions;
      }
      if (next_mode == Mode::kFixed) {
        if (joint_state.valid &&
          joint_state.positions.size() == config_.policy.default_joint_positions.size())
        {
          fixed_transition_start_positions_ = joint_state.positions;
        } else {
          fixed_transition_start_positions_ = config_.policy.default_joint_positions;
        }
        fixed_transition_start_time_ = transition_time;
      }
      if (next_mode == Mode::kLoco) {
        loco_policy_step_count_.store(0U, std::memory_order_relaxed);
        policy_reset_generation_.fetch_add(1U, std::memory_order_release);
        loco_mode_start_time_ = transition_time;
        if (current_joint_state_.valid) {
          latest_policy_targets_ = current_joint_state_.positions;
          loco_transition_start_positions_ = current_joint_state_.positions;
        }
      }
      if (next_mode != Mode::kLoco) {
        last_policy_raw_action_.assign(config_.policy.joint_names.size(), 0.0F);
      }
      if (next_mode == Mode::kDanc) {
        danc_start_positions_ = danc_targets_ = joint_state.positions;
        danc_start_time_ = danc_last_policy_time_ = transition_time;
        danc_frame_ = 0U;
        danc_complete_ = false;
        ++danc_generation_;
      }
      current_mode_.store(next_mode);
    }
    if (previous_mode == Mode::kLoco) {
      stop_tracking_recording("mode_" + to_string(next_mode));
    }

    std_msgs::msg::String mode_message;
    mode_message.data = to_string(next_mode);
    mode_pub_->publish(mode_message);

    RCLCPP_INFO(
      get_logger(),
      "Mode transition: %s -> %s (%s)",
      to_string(previous_mode).c_str(),
      to_string(next_mode).c_str(),
      reason.c_str());

    if (next_mode == Mode::kFixed) {
      if (config_.fixed_gains.hold_current_pose_on_entry && joint_state.valid) {
        RCLCPP_INFO(get_logger(), "Fixed mode is holding the current logical joint pose.");
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Fixed mode target is policy.default_joint_positions; transitioning over %.1f s.",
          config_.fixed_gains.transition_duration_s);
      }
    }
  }

  void report_fault(const std::string & message, const bool throttle_repeated)
  {
    const auto now = SteadyClock::now();
    {
      std::lock_guard<std::mutex> lock(error_mutex_);
      if (throttle_repeated &&
        message == last_error_message_ &&
        now - last_error_time_ < std::chrono::seconds(1))
      {
        return;
      }
      last_error_message_ = message;
      last_error_time_ = now;
    }

    if (error_log_) {
      error_log_->write("ERROR", message);
    }

    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      ++runtime_stats_.fault_count;
    }

    std_msgs::msg::String error_message;
    error_message.data = message;
    error_pub_->publish(error_message);

    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    transition_to_mode(Mode::kPassive, "fault");
  }

  void publish_joint_state()
  {
    LogicalJointState state;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state = current_joint_state_;
    }

    if (!state.valid) {
      return;
    }

    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = config_.policy.joint_names;
    message.position = state.positions;
    message.velocity = state.velocities;
    message.effort = state.efforts;
    joint_state_pub_->publish(message);
  }

  void disable_motors_on_exit()
  {
    if (!motor_manager_ || !config_.loop.disable_on_exit || !enabled_by_node_) {
      return;
    }

    try {
      const auto results =
        motor_manager_->disable_all(std::chrono::milliseconds(config_.loop.lifecycle_timeout_ms));
      enabled_by_node_ = false;
      if (has_failures(results)) {
        RCLCPP_WARN(
          get_logger(),
          "One or more motors failed to disable cleanly: %s",
          summarize_failures(results).c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Disabled all motors on exit.");
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "Failed to disable motors on exit: %s", ex.what());
    }
  }

  void shutdown_runtime()
  {
    stop_requested_ = true;

    if (control_thread_.joinable()) {
      control_thread_.join();
    }

    disable_motors_on_exit();

    if (policy_thread_.joinable()) {
      policy_thread_.join();
    }

    policy_runtime_.reset();
    observation_history_.reset();
    stop_tracking_recording("shutdown");
    {
      std::lock_guard<std::mutex> lock(recording_queue_mutex_);
      recording_writer_stop_requested_ = true;
    }
    recording_queue_cv_.notify_all();
    if (recording_writer_thread_.joinable()) {
      recording_writer_thread_.join();
    }
  }

  DeploymentConfig config_ {};
  std::unique_ptr<ErrorLog> error_log_;
  std::unique_ptr<MotorManager> motor_manager_;
  std::unique_ptr<OnnxPolicyRuntime> policy_runtime_;
  std::unique_ptr<WholebodyObservationHistory> observation_history_;
  std::unique_ptr<SuperMimicPolicy> danc_policy_;
  std::vector<std::string> motor_names_;
  std::vector<MotorConfig> motor_configs_;
  std::unordered_map<std::string, std::size_t> joint_name_to_index_;
  std::vector<DirectJointBinding> direct_joint_bindings_;
  std::vector<ParallelBinding> parallel_bindings_;

  std::mutex joy_mutex_;
  JoySnapshot latest_joy_;

  std::mutex imu_mutex_;
  ImuSnapshot latest_imu_;
  ImuSnapshot filtered_imu_;
  bool filtered_imu_valid_ {false};
  tf2::Quaternion imu_mount_offset_ {0.0, 0.0, 0.0, 1.0};

  std::mutex state_mutex_;
  LogicalJointState current_joint_state_;
  std::vector<double> fixed_hold_positions_;
  std::vector<double> fixed_transition_start_positions_;
  std::vector<double> loco_transition_start_positions_;
  std::vector<double> latest_policy_targets_;
  std::vector<double> latest_commanded_targets_;
  std::vector<double> latest_commanded_velocities_;
  std::vector<double> latest_commanded_torques_;
  std::vector<float> latest_policy_observation_;
  std::vector<float> last_policy_raw_action_;
  std::vector<double> danc_targets_;
  std::vector<double> danc_start_positions_;
  SteadyClock::time_point danc_start_time_ {};
  SteadyClock::time_point danc_last_policy_time_ {};
  std::size_t danc_frame_ {0U};
  bool danc_complete_ {false};
  uint64_t danc_generation_ {0U};
  uint64_t danc_applied_generation_ {0U};

  std::mutex error_mutex_;
  std::string last_error_message_;
  SteadyClock::time_point last_error_time_ {};

  std::mutex stats_mutex_;
  RuntimeStats runtime_stats_ {};
  RuntimeStats last_status_stats_ {};
  SteadyClock::time_point last_status_publish_time_ {SteadyClock::now()};
  SteadyClock::time_point loco_mode_start_time_ {SteadyClock::now()};
  SteadyClock::time_point fixed_transition_start_time_ {SteadyClock::now()};
  std::atomic<uint64_t> loco_policy_step_count_ {0U};
  std::atomic<uint64_t> policy_reset_generation_ {1U};
  uint64_t applied_policy_reset_generation_ {0U};

  std::mutex recording_mutex_;
  std::optional<TrackingRun> active_recording_;
  SteadyClock::time_point next_recording_sample_time_ {};
  std::size_t recording_run_counter_ {0U};

  std::mutex recording_queue_mutex_;
  std::condition_variable recording_queue_cv_;
  std::deque<TrackingRun> completed_recording_runs_;
  bool recording_writer_stop_requested_ {false};

  std::atomic<Mode> current_mode_ {Mode::kPassive};
  std::atomic<bool> stop_requested_ {true};
  bool validation_complete_ {false};
  bool enabled_by_node_ {false};

  std::thread control_thread_;
  std::thread policy_thread_;
  std::thread recording_writer_thread_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr projected_gravity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr angular_velocity_pub_;
  rclcpp::TimerBase::SharedPtr state_publish_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = 0;
  try {
    auto node = std::make_shared<JljWholebodyDeployNode>();
    node->start();
    if (!node->validation_complete()) {
      rclcpp::spin(node);
    }
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger(kNodeName), "Deployment node failed: %s", ex.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
