#include "jljwholebody_hardware/jljwholebody_system.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace jljwholebody_hardware
{

namespace
{

constexpr const char * kLoggerName = "JljWholebodySystem";

std::string get_hardware_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  const std::string & default_value)
{
  const auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? default_value : it->second;
}

std::string lowercase_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char ch) {return static_cast<char>(std::tolower(ch));});
  return value;
}

bool parse_bool_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  const bool default_value)
{
  const auto raw_value = get_hardware_parameter(info, key, default_value ? "true" : "false");
  const auto value = lowercase_copy(raw_value);
  if (value == "true" || value == "1" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "false" || value == "0" || value == "no" || value == "off") {
    return false;
  }

  throw std::runtime_error("Invalid boolean hardware parameter '" + key + "': " + raw_value);
}

int parse_int_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  const int default_value)
{
  const auto raw_value = get_hardware_parameter(info, key, std::to_string(default_value));
  const int value = std::stoi(raw_value);
  if (value < 0) {
    throw std::runtime_error("Hardware parameter '" + key + "' must be non-negative");
  }
  return value;
}

double parse_double_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  const double default_value)
{
  const auto raw_value = get_hardware_parameter(info, key, std::to_string(default_value));
  const double value = std::stod(raw_value);
  if (value < 0.0) {
    throw std::runtime_error("Hardware parameter '" + key + "' must be non-negative");
  }
  return value;
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

std::string read_required_yaml_string(const YAML::Node & node, const char * key)
{
  if (!node[key]) {
    throw std::runtime_error(std::string("Missing required config key: ") + key);
  }
  return node[key].as<std::string>();
}

std::string resolve_config_path(
  const std::string & parent_file_path,
  const std::string & config_path)
{
  namespace fs = std::filesystem;

  if (config_path.empty()) {
    return config_path;
  }

  const fs::path path(config_path);
  if (path.is_absolute()) {
    return path.lexically_normal().string();
  }

  return (fs::path(parent_file_path).parent_path() / path).lexically_normal().string();
}

std::vector<std::string> compute_missing_motors(
  const std::vector<std::string> & expected_names,
  const std::vector<dm_motor::MotorState> & states)
{
  std::vector<std::string> missing = expected_names;
  for (const auto & state : states) {
    missing.erase(
      std::remove(missing.begin(), missing.end(), state.name),
      missing.end());
  }
  return missing;
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

bool has_failed_results(const std::vector<dm_motor::MotorCommandResult> & results)
{
  return std::any_of(
    results.begin(), results.end(),
    [](const auto & result) {return !result.success;});
}

}  // namespace

hardware_interface::CallbackReturn JljWholebodySystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto joint_count = info_.joints.size();
  hw_positions_.assign(joint_count, 0.0);
  hw_velocities_.assign(joint_count, 0.0);
  hw_efforts_.assign(joint_count, 0.0);
  hw_commands_.assign(joint_count, 0.0);
  joint_index_by_name_.clear();
  direct_joint_bindings_.clear();
  parallel_mechanism_bindings_.clear();

  try {
    motor_config_path_ = get_hardware_parameter(info_, "motor_config_path", "");
    if (motor_config_path_.empty()) {
      motor_config_path_ = default_motor_config_path();
    }
    parallel_mechanisms_path_ = get_hardware_parameter(info_, "parallel_mechanisms_path", "");
    if (parallel_mechanisms_path_.empty()) {
      parallel_mechanisms_path_ = default_parallel_mechanisms_path();
    }
    enable_on_activate_ = parse_bool_parameter(info_, "enable_on_activate", false);
    disable_on_deactivate_ = parse_bool_parameter(info_, "disable_on_deactivate", true);
    command_timeout_ms_ = parse_int_parameter(info_, "command_timeout_ms", 0);
    feedback_poll_timeout_ms_ = parse_int_parameter(info_, "feedback_poll_timeout_ms", 0);
    lifecycle_timeout_ms_ = parse_int_parameter(info_, "lifecycle_timeout_ms", 500);
    startup_feedback_warmup_s_ =
      parse_double_parameter(info_, "startup_feedback_warmup_s", 0.5);
    enable_retries_ = parse_int_parameter(info_, "enable_retries", 3);
    if (enable_retries_ < 1) {
      throw std::runtime_error("Hardware parameter 'enable_retries' must be at least 1");
    }
    enable_retry_interval_ms_ =
      parse_int_parameter(info_, "enable_retry_interval_ms", 200);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to parse hardware parameters: %s",
      ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const auto & joint = info_.joints[index];
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must expose exactly one position command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const bool has_position_state = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & interface) {
        return interface.name == hardware_interface::HW_IF_POSITION;
      });
    const bool has_velocity_state = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & interface) {
        return interface.name == hardware_interface::HW_IF_VELOCITY;
      });
    const bool has_effort_state = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & interface) {
        return interface.name == hardware_interface::HW_IF_EFFORT;
      });

    if (!has_position_state || !has_velocity_state || !has_effort_state) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must expose position, velocity, and effort state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!joint_index_by_name_.emplace(joint.name, index).second) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Duplicate joint '%s' detected in ros2_control definition.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  try {
    const YAML::Node root = YAML::LoadFile(parallel_mechanisms_path_);
    std::unordered_set<std::string> parallel_joint_names;
    std::unordered_set<std::string> claimed_motor_names;

    const auto mechanisms = root["mechanisms"];
    if (mechanisms && !mechanisms.IsSequence()) {
      throw std::runtime_error("Parallel mechanisms config must contain a sequence at 'mechanisms'");
    }

    if (mechanisms) {
      parallel_mechanism_bindings_.reserve(mechanisms.size());
      for (const auto & mechanism_node : mechanisms) {
        ParallelMechanismBinding binding;
        binding.name = read_required_yaml_string(mechanism_node, "name");
        binding.pitch_joint_name = read_required_yaml_string(mechanism_node, "pitch_joint");
        binding.roll_joint_name = read_required_yaml_string(mechanism_node, "roll_joint");
        binding.motor_1_name = read_required_yaml_string(mechanism_node, "motor_1");
        binding.motor_2_name = read_required_yaml_string(mechanism_node, "motor_2");
        binding.config_path = resolve_config_path(
          parallel_mechanisms_path_,
          read_required_yaml_string(mechanism_node, "config_path"));

        if (binding.pitch_joint_name == binding.roll_joint_name) {
          throw std::runtime_error(
                  "Parallel mechanism '" + binding.name + "' reuses the same logical joint twice");
        }
        if (binding.motor_1_name == binding.motor_2_name) {
          throw std::runtime_error(
                  "Parallel mechanism '" + binding.name + "' reuses the same motor twice");
        }

        const auto pitch_joint_it = joint_index_by_name_.find(binding.pitch_joint_name);
        const auto roll_joint_it = joint_index_by_name_.find(binding.roll_joint_name);
        if (pitch_joint_it == joint_index_by_name_.end() ||
          roll_joint_it == joint_index_by_name_.end())
        {
          throw std::runtime_error(
                  "Parallel mechanism '" + binding.name + "' references unknown logical joints");
        }

        if (!parallel_joint_names.insert(binding.pitch_joint_name).second ||
          !parallel_joint_names.insert(binding.roll_joint_name).second)
        {
          throw std::runtime_error(
                  "Logical joint is already claimed by another parallel mechanism");
        }

        if (!claimed_motor_names.insert(binding.motor_1_name).second ||
          !claimed_motor_names.insert(binding.motor_2_name).second)
        {
          throw std::runtime_error(
                  "Motor is already claimed by another parallel mechanism");
        }

        binding.pitch_joint_index = pitch_joint_it->second;
        binding.roll_joint_index = roll_joint_it->second;

        std::string error_message;
        if (!binding.kinematics.load_from_yaml(binding.config_path, error_message)) {
          throw std::runtime_error(
                  "Failed to load kinematics for '" + binding.name + "': " + error_message);
        }

        parallel_mechanism_bindings_.push_back(std::move(binding));
      }
    }

    direct_joint_bindings_.reserve(info_.joints.size() - parallel_joint_names.size());
    for (std::size_t index = 0; index < info_.joints.size(); ++index) {
      const auto & joint = info_.joints[index];
      if (parallel_joint_names.count(joint.name) > 0U) {
        continue;
      }

      const auto motor_name_it = joint.parameters.find("motor_name");
      const std::string motor_name = motor_name_it == joint.parameters.end() ?
        joint.name :
        motor_name_it->second;

      if (!claimed_motor_names.insert(motor_name).second) {
        throw std::runtime_error(
                "Duplicate motor mapping '" + motor_name + "' detected in ros2_control joints");
      }

      direct_joint_bindings_.push_back(DirectJointBinding {
        index,
        joint.name,
        motor_name,
        dm_motor::MotorConfig {}
      });
    }
  } catch (const std::exception & ex) {
    direct_joint_bindings_.clear();
    parallel_mechanism_bindings_.clear();
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to load parallel mechanism config: %s",
      ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Initialized DM motor hardware interface for %zu joints (%zu direct, %zu parallel mechanisms).",
    joint_count,
    direct_joint_bindings_.size(),
    parallel_mechanism_bindings_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JljWholebodySystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 3);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_EFFORT,
      &hw_efforts_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JljWholebodySystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }

  return command_interfaces;
}

std::string JljWholebodySystem::summarize_feedback_snapshot(
  const std::vector<dm_motor::MotorState> & states) const
{
  const auto expected_names = motor_manager_ ?
    motor_manager_->motor_names() :
    std::vector<std::string> {};
  const auto motor_configs = motor_manager_ ?
    motor_manager_->motor_configs() :
    std::vector<dm_motor::MotorConfig> {};

  std::ostringstream oss;
  oss << states.size() << "/" << expected_names.size() << " motors";
  const auto missing = compute_missing_motors(expected_names, states);
  if (!missing.empty()) {
    oss << ", missing: " << join_names(missing);

    std::unordered_map<std::string, std::vector<std::string>> missing_by_interface;
    std::vector<std::string> interface_order;
    for (const auto & name : missing) {
      auto interface_name = std::string("unknown");
      const auto config_it = std::find_if(
        motor_configs.begin(),
        motor_configs.end(),
        [&name](const auto & config) {return config.name == name;});
      if (config_it != motor_configs.end()) {
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
      state.name + "(" + state.interface_name + ")=" + dm_motor::to_string(state.status));
  }
  if (!previews.empty()) {
    oss << ", states: " << join_names(previews);
    if (states.size() > previews.size()) {
      oss << ", ...";
    }
  }
  return oss.str();
}

void JljWholebodySystem::warmup_activation_feedback()
{
  if (!motor_manager_ || startup_feedback_warmup_s_ <= 0.0) {
    return;
  }

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(startup_feedback_warmup_s_));
  const auto poll_timeout = std::chrono::milliseconds(feedback_poll_timeout_ms_);
  std::size_t polls = 0U;
  std::size_t polls_with_feedback = 0U;

  while (std::chrono::steady_clock::now() < deadline) {
    if (motor_manager_->poll_once(poll_timeout)) {
      ++polls_with_feedback;
    }
    ++polls;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const auto states = motor_manager_->snapshot_states();
  const auto summary = summarize_feedback_snapshot(states);
  const auto expected_count = motor_manager_->motor_names().size();
  if (states.size() == expected_count) {
    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName),
      "Activation feedback warmup complete after %.2fs: %s; polls_with_feedback=%zu/%zu.",
      startup_feedback_warmup_s_,
      summary.c_str(),
      polls_with_feedback,
      polls);
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger(kLoggerName),
      "Activation feedback warmup incomplete after %.2fs: %s; polls_with_feedback=%zu/%zu.",
      startup_feedback_warmup_s_,
      summary.c_str(),
      polls_with_feedback,
      polls);
  }
}

void JljWholebodySystem::enable_motors_on_activate()
{
  if (!motor_manager_) {
    throw std::runtime_error("Motor manager is not initialized");
  }

  warmup_activation_feedback();

  std::vector<dm_motor::MotorCommandResult> last_results;
  for (int attempt = 1; attempt <= enable_retries_; ++attempt) {
    last_results = motor_manager_->enable_all(std::chrono::milliseconds(lifecycle_timeout_ms_));
    if (!has_failed_results(last_results)) {
      RCLCPP_INFO(
        rclcpp::get_logger(kLoggerName),
        "Enabled all motors during activation after %d/%d attempt(s).",
        attempt,
        enable_retries_);
      return;
    }

    log_failed_results(last_results, "enable");
    RCLCPP_WARN(
      rclcpp::get_logger(kLoggerName),
      "Enable attempt %d/%d failed. Current feedback: %s",
      attempt,
      enable_retries_,
      summarize_feedback_snapshot(motor_manager_->snapshot_states()).c_str());

    if (attempt < enable_retries_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(enable_retry_interval_ms_));
    }
  }

  throw std::runtime_error("Failed to enable one or more motors during activation");
}

hardware_interface::CallbackReturn JljWholebodySystem::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  try {
    motor_manager_ = std::make_unique<dm_motor::MotorManager>(motor_config_path_);

    for (auto & binding : direct_joint_bindings_) {
      const auto config = motor_manager_->motor_config(binding.motor_name);
      if (!config.has_value()) {
        throw std::runtime_error("Motor config not found for direct joint: " + binding.motor_name);
      }
      binding.motor_config = *config;
    }

    for (auto & binding : parallel_mechanism_bindings_) {
      const auto motor_1_config = motor_manager_->motor_config(binding.motor_1_name);
      if (!motor_1_config.has_value()) {
        throw std::runtime_error(
                "Motor config not found for parallel motor: " + binding.motor_1_name);
      }
      const auto motor_2_config = motor_manager_->motor_config(binding.motor_2_name);
      if (!motor_2_config.has_value()) {
        throw std::runtime_error(
                "Motor config not found for parallel motor: " + binding.motor_2_name);
      }
      binding.motor_1_config = *motor_1_config;
      binding.motor_2_config = *motor_2_config;
    }

    if (enable_on_activate_) {
      enable_motors_on_activate();
    }

    sync_state_from_feedback(motor_manager_->snapshot_states());
    for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
      hw_commands_[i] = hw_positions_[i];
    }
  } catch (const std::exception & ex) {
    if (motor_manager_ && enable_on_activate_) {
      try {
        RCLCPP_WARN(
          rclcpp::get_logger(kLoggerName),
          "Activation failed; disabling motors before releasing hardware interface.");
        const auto results = motor_manager_->disable_all(
          std::chrono::milliseconds(lifecycle_timeout_ms_));
        log_failed_results(results, "disable_after_activation_failure");
        if (!has_failed_results(results)) {
          RCLCPP_INFO(
            rclcpp::get_logger(kLoggerName),
            "Disabled all motors after activation failure.");
        }
      } catch (const std::exception & cleanup_ex) {
        RCLCPP_WARN(
          rclcpp::get_logger(kLoggerName),
          "Failed to disable motors after activation failure: %s",
          cleanup_ex.what());
      }
    }
    motor_manager_.reset();
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to activate hardware interface: %s",
      ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  active_ = true;
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Hardware interface activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JljWholebodySystem::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  if (motor_manager_ && disable_on_deactivate_) {
    try {
      const auto results = motor_manager_->disable_all(
        std::chrono::milliseconds(lifecycle_timeout_ms_));
      log_failed_results(results, "disable");
      if (!has_failed_results(results)) {
        RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Disabled all motors during deactivation.");
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        rclcpp::get_logger(kLoggerName),
        "Failed to disable motors during deactivation: %s",
        ex.what());
    }
  }

  active_ = false;
  motor_manager_.reset();
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  std::fill(hw_efforts_.begin(), hw_efforts_.end(), 0.0);
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Hardware interface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type JljWholebodySystem::read(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (!active_ || !motor_manager_) {
    return hardware_interface::return_type::OK;
  }

  try {
    (void)motor_manager_->poll_once(std::chrono::milliseconds(feedback_poll_timeout_ms_));
    sync_state_from_feedback(motor_manager_->snapshot_states());
    return hardware_interface::return_type::OK;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger(kLoggerName),
      steady_clock_,
      2000,
      "Motor feedback read failed: %s",
      ex.what());
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type JljWholebodySystem::write(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (!active_ || !motor_manager_) {
    return hardware_interface::return_type::OK;
  }

  std::vector<dm_motor::NamedMitCommand> commands;
  commands.reserve(direct_joint_bindings_.size() + parallel_mechanism_bindings_.size() * 2U);

  for (const auto & binding : direct_joint_bindings_) {
    dm_motor::NamedMitCommand command;
    command.name = binding.motor_name;
    command.command.position = static_cast<float>(hw_commands_[binding.joint_index]);
    command.command.velocity = 0.0F;
    command.command.kp = binding.motor_config.default_kp;
    command.command.kd = binding.motor_config.default_kd;
    command.command.torque = 0.0F;
    commands.push_back(std::move(command));
  }

  for (const auto & binding : parallel_mechanism_bindings_) {
    const double requested_pitch = hw_commands_[binding.pitch_joint_index];
    const double requested_roll = hw_commands_[binding.roll_joint_index];
    double motor_1_position = 0.0;
    double motor_2_position = 0.0;
    std::string error_message;
    if (!binding.kinematics.inverse(
        requested_pitch,
        requested_roll,
        motor_1_position,
        motor_2_position,
        error_message))
    {
      RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger(kLoggerName),
        steady_clock_,
        2000,
        "Failed to solve parallel-joint command for %s: %s",
        binding.name.c_str(),
        error_message.c_str());
      return hardware_interface::return_type::ERROR;
    }

    dm_motor::NamedMitCommand motor_1_command;
    motor_1_command.name = binding.motor_1_name;
    motor_1_command.command.position = static_cast<float>(motor_1_position);
    motor_1_command.command.velocity = 0.0F;
    motor_1_command.command.kp = 0.0F;
    motor_1_command.command.kd = 0.0F;
    motor_1_command.command.torque = 0.0F;
    commands.push_back(std::move(motor_1_command));

    dm_motor::NamedMitCommand motor_2_command;
    motor_2_command.name = binding.motor_2_name;
    motor_2_command.command.position = static_cast<float>(motor_2_position);
    motor_2_command.command.velocity = 0.0F;
    motor_2_command.command.kp = 0.0F;
    motor_2_command.command.kd = 0.0F;
    motor_2_command.command.torque = 0.0F;
    commands.push_back(std::move(motor_2_command));
  }

  try {
    const auto results = motor_manager_->transmit_group_mit(
      commands,
      std::chrono::milliseconds(command_timeout_ms_));
    sync_state_from_feedback(motor_manager_->snapshot_states());
    log_failed_results(results, "mit_tx");
    return hardware_interface::return_type::OK;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger(kLoggerName),
      steady_clock_,
      2000,
      "Motor command write failed: %s",
      ex.what());
    return hardware_interface::return_type::ERROR;
  }
}

void JljWholebodySystem::sync_state_from_feedback(const std::vector<dm_motor::MotorState> & states)
{
  std::unordered_map<std::string, const dm_motor::MotorState *> motor_lookup;
  motor_lookup.reserve(states.size());
  for (const auto & state : states) {
    motor_lookup[state.name] = &state;
  }

  for (const auto & binding : direct_joint_bindings_) {
    const auto state_it = motor_lookup.find(binding.motor_name);
    if (state_it == motor_lookup.end()) {
      continue;
    }

    hw_positions_[binding.joint_index] = state_it->second->position;
    hw_velocities_[binding.joint_index] = state_it->second->velocity;
    hw_efforts_[binding.joint_index] = state_it->second->torque;
  }

  for (const auto & binding : parallel_mechanism_bindings_) {
    const auto motor_1_it = motor_lookup.find(binding.motor_1_name);
    const auto motor_2_it = motor_lookup.find(binding.motor_2_name);
    if (motor_1_it == motor_lookup.end() || motor_2_it == motor_lookup.end()) {
      hw_velocities_[binding.pitch_joint_index] = 0.0;
      hw_velocities_[binding.roll_joint_index] = 0.0;
      hw_efforts_[binding.pitch_joint_index] = 0.0;
      hw_efforts_[binding.roll_joint_index] = 0.0;
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger(kLoggerName),
        steady_clock_,
        2000,
        "Missing motor feedback while rebuilding parallel joint state for %s",
        binding.name.c_str());
      continue;
    }

    double pitch = hw_positions_[binding.pitch_joint_index];
    double roll = hw_positions_[binding.roll_joint_index];
    double limited_motor_1 = motor_1_it->second->position;
    double limited_motor_2 = motor_2_it->second->position;
    std::string solve_error;
    if (!binding.kinematics.forward(
        motor_1_it->second->position,
        motor_2_it->second->position,
        limited_motor_1,
        limited_motor_2,
        pitch,
        roll,
        solve_error,
        hw_positions_[binding.pitch_joint_index],
        hw_positions_[binding.roll_joint_index]))
    {
      hw_velocities_[binding.pitch_joint_index] = 0.0;
      hw_velocities_[binding.roll_joint_index] = 0.0;
      hw_efforts_[binding.pitch_joint_index] = 0.0;
      hw_efforts_[binding.roll_joint_index] = 0.0;
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger(kLoggerName),
        steady_clock_,
        2000,
        "Parallel forward solve failed for %s: %s",
        binding.name.c_str(),
        solve_error.c_str());
      continue;
    }

    hw_positions_[binding.pitch_joint_index] = pitch;
    hw_positions_[binding.roll_joint_index] = roll;

    double pitch_velocity = 0.0;
    double roll_velocity = 0.0;
    double pitch_effort = 0.0;
    double roll_effort = 0.0;
    std::string dynamics_error;
    if (!binding.kinematics.motor_to_joint_dynamics(
        pitch,
        roll,
        motor_1_it->second->position,
        motor_2_it->second->position,
        motor_1_it->second->velocity,
        motor_2_it->second->velocity,
        motor_1_it->second->torque,
        motor_2_it->second->torque,
        pitch_velocity,
        roll_velocity,
        pitch_effort,
        roll_effort,
        dynamics_error))
    {
      hw_velocities_[binding.pitch_joint_index] = 0.0;
      hw_velocities_[binding.roll_joint_index] = 0.0;
      hw_efforts_[binding.pitch_joint_index] = 0.0;
      hw_efforts_[binding.roll_joint_index] = 0.0;
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger(kLoggerName),
        steady_clock_,
        2000,
        "Parallel dynamics mapping failed for %s: %s",
        binding.name.c_str(),
        dynamics_error.c_str());
      continue;
    }

    hw_velocities_[binding.pitch_joint_index] = pitch_velocity;
    hw_velocities_[binding.roll_joint_index] = roll_velocity;
    hw_efforts_[binding.pitch_joint_index] = pitch_effort;
    hw_efforts_[binding.roll_joint_index] = roll_effort;
  }
}

void JljWholebodySystem::log_failed_results(
  const std::vector<dm_motor::MotorCommandResult> & results,
  const std::string & action)
{
  std::vector<std::string> failures;
  failures.reserve(results.size());

  for (const auto & result : results) {
    if (result.success) {
      continue;
    }

    if (action == "mit_tx" && command_timeout_ms_ == 0 &&
      result.message == "mit_tx: no feedback received")
    {
      continue;
    }

    failures.push_back(result.name + " [" + result.message + "]");
  }

  if (failures.empty()) {
    return;
  }

  std::ostringstream oss;
  oss << action << " reported " << failures.size() << " motor issue(s)";
  const std::size_t preview_count = std::min<std::size_t>(failures.size(), 3U);
  for (std::size_t i = 0; i < preview_count; ++i) {
    oss << (i == 0 ? ": " : ", ") << failures[i];
  }
  if (failures.size() > preview_count) {
    oss << ", ...";
  }

  RCLCPP_WARN_THROTTLE(
    rclcpp::get_logger(kLoggerName),
    steady_clock_,
    2000,
    "%s",
    oss.str().c_str());
}

}  // namespace jljwholebody_hardware

PLUGINLIB_EXPORT_CLASS(
  jljwholebody_hardware::JljWholebodySystem,
  hardware_interface::SystemInterface)
