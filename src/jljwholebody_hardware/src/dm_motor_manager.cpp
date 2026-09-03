#include "jljwholebody_hardware/dm_motor_manager.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace jljwholebody_hardware::dm_motor
{

namespace
{

std::string read_required_string(const YAML::Node & node, const char * key)
{
  if (!node[key]) {
    throw std::runtime_error(std::string("Missing required config key: ") + key);
  }
  return node[key].as<std::string>();
}

template<typename T>
T read_optional_value(const YAML::Node & node, const char * key, const T & default_value)
{
  return node[key] ? node[key].as<T>() : default_value;
}

MitLimits parse_limits(const YAML::Node & node)
{
  MitLimits limits;
  limits.position_min = read_optional_value<float>(node, "position_min", limits.position_min);
  limits.position_max = read_optional_value<float>(node, "position_max", limits.position_max);
  limits.velocity_min = read_optional_value<float>(node, "velocity_min", limits.velocity_min);
  limits.velocity_max = read_optional_value<float>(node, "velocity_max", limits.velocity_max);
  limits.torque_min = read_optional_value<float>(node, "torque_min", limits.torque_min);
  limits.torque_max = read_optional_value<float>(node, "torque_max", limits.torque_max);
  limits.kp_min = read_optional_value<float>(node, "kp_min", limits.kp_min);
  limits.kp_max = read_optional_value<float>(node, "kp_max", limits.kp_max);
  limits.kd_min = read_optional_value<float>(node, "kd_min", limits.kd_min);
  limits.kd_max = read_optional_value<float>(node, "kd_max", limits.kd_max);
  return limits;
}

bool has_limits_node(const YAML::Node & node)
{
  return node["position_min"] || node["position_max"] ||
         node["velocity_min"] || node["velocity_max"] ||
         node["torque_min"] || node["torque_max"] ||
         node["kp_min"] || node["kp_max"] ||
         node["kd_min"] || node["kd_max"];
}

void parse_joint_safety_limits(const YAML::Node & node, MotorConfig & config)
{
  if (node["upper_position_limit"]) {
    config.upper_position_limit = node["upper_position_limit"].as<float>();
    config.has_position_limit = true;
  }
  if (node["lower_position_limit"]) {
    config.lower_position_limit = node["lower_position_limit"].as<float>();
    config.has_position_limit = true;
  }
  if (node["max_output_torque"]) {
    config.max_output_torque = node["max_output_torque"].as<float>();
  }
  if (node["max_output_speed"]) {
    config.max_output_speed = node["max_output_speed"].as<float>();
  }
}

}  // namespace

MotorManager::MotorManager(const std::string & config_path)
: config_path_(config_path)
{
  load_config(config_path_);
  validate_config();
  open_transports();
}

const std::string & MotorManager::config_path() const
{
  return config_path_;
}

const std::vector<std::string> & MotorManager::motor_names() const
{
  return motor_order_;
}

std::vector<MotorConfig> MotorManager::motor_configs() const
{
  return configs_;
}

std::optional<MotorConfig> MotorManager::motor_config(const std::string & name) const
{
  const auto * driver = find_driver(name);
  if (driver == nullptr) {
    return std::nullopt;
  }
  return driver->config();
}

MotorCommandResult MotorManager::enable_motor(
  const std::string & name,
  const std::chrono::milliseconds timeout)
{
  auto * driver = find_driver(name);
  if (driver == nullptr) {
    return {name, false, "enable: motor not found", std::nullopt};
  }
  return driver->enable(timeout);
}

MotorCommandResult MotorManager::disable_motor(
  const std::string & name,
  const std::chrono::milliseconds timeout)
{
  auto * driver = find_driver(name);
  if (driver == nullptr) {
    return {name, false, "disable: motor not found", std::nullopt};
  }
  return driver->disable(timeout);
}

std::vector<MotorCommandResult> MotorManager::enable_all(const std::chrono::milliseconds timeout)
{
  std::vector<MotorCommandResult> results;
  results.reserve(motor_order_.size());
  for (const auto & name : motor_order_) {
    results.emplace_back(enable_motor(name, timeout));
  }
  return results;
}

std::vector<MotorCommandResult> MotorManager::disable_all(const std::chrono::milliseconds timeout)
{
  std::vector<MotorCommandResult> results;
  results.reserve(motor_order_.size());
  for (const auto & name : motor_order_) {
    results.emplace_back(disable_motor(name, timeout));
  }
  return results;
}

std::vector<MotorCommandResult> MotorManager::command_group_mit(
  const std::vector<NamedMitCommand> & commands,
  const std::chrono::milliseconds timeout)
{
  std::vector<std::pair<DmMotorDriver *, CanFrame>> batch;
  batch.reserve(commands.size());
  for (const auto & item : commands) {
    auto * driver = find_driver(item.name);
    if (driver != nullptr) {
      batch.emplace_back(driver, driver->make_mit_frame(item.command));
    }
  }
  auto results = send_batch_and_collect(batch, "mit_command", MotorStatus::kEnabled, timeout);
  for (const auto & item : commands) {
    if (find_driver(item.name) == nullptr) {
      results.push_back({item.name, false, "mit_command: motor not found", std::nullopt});
    }
  }
  return results;
}

std::vector<MotorCommandResult> MotorManager::transmit_group_mit(
  const std::vector<NamedMitCommand> & commands,
  const std::chrono::milliseconds timeout)
{
  std::vector<std::pair<DmMotorDriver *, CanFrame>> batch;
  batch.reserve(commands.size());
  for (const auto & item : commands) {
    auto * driver = find_driver(item.name);
    if (driver != nullptr) {
      batch.emplace_back(driver, driver->make_mit_frame(item.command));
    }
  }
  auto results = send_batch_and_collect(batch, "mit_tx", std::nullopt, timeout);
  for (const auto & item : commands) {
    if (find_driver(item.name) == nullptr) {
      results.push_back({item.name, false, "mit_tx: motor not found", std::nullopt});
    }
  }
  return results;
}

std::vector<MotorCommandResult> MotorManager::transmit_group_mit_unclamped(
  const std::vector<NamedMitCommand> & commands,
  const std::chrono::milliseconds timeout)
{
  std::vector<std::pair<DmMotorDriver *, CanFrame>> batch;
  batch.reserve(commands.size());
  for (const auto & item : commands) {
    auto * driver = find_driver(item.name);
    if (driver != nullptr) {
      batch.emplace_back(driver, driver->make_mit_frame_unclamped(item.command));
    }
  }
  auto results = send_batch_and_collect(batch, "mit_tx_unclamped", std::nullopt, timeout);
  for (const auto & item : commands) {
    if (find_driver(item.name) == nullptr) {
      results.push_back({item.name, false, "mit_tx_unclamped: motor not found", std::nullopt});
    }
  }
  return results;
}

void MotorManager::transmit_group_mit_fast(const std::vector<NamedMitCommand> & commands)
{
  for (const auto & item : commands) {
    auto * driver = find_driver(item.name);
    if (driver == nullptr) {
      throw std::invalid_argument("MIT command references unknown motor: " + item.name);
    }
    driver->transport()->send(driver->make_mit_frame(item.command));
  }
}

std::vector<MotorState> MotorManager::snapshot_states() const
{
  std::vector<MotorState> states;
  states.reserve(motor_order_.size());
  for (const auto & name : motor_order_) {
    const auto * driver = find_driver(name);
    if (driver != nullptr && driver->last_state().has_value()) {
      states.push_back(*driver->last_state());
    }
  }
  return states;
}

bool MotorManager::poll_once(const std::chrono::milliseconds timeout)
{
  bool updated = false;
  for (const auto & [interface_name, transport] : transports_) {
    const auto frames = transport->receive_available(timeout, 128U);
    for (const auto & frame : frames) {
      if (handle_feedback_frame(interface_name, frame).has_value()) {
        updated = true;
      }
    }
  }
  return updated;
}

void MotorManager::load_config(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  MotorConfig defaults;

  if (root["defaults"]) {
    const auto defaults_node = root["defaults"];
    defaults.model = read_optional_value<std::string>(defaults_node, "model", defaults.model);
    defaults.master_id = read_optional_value<uint32_t>(defaults_node, "master_id", defaults.master_id);
    if (defaults_node["use_can_fd"]) {
      defaults.use_can_fd = defaults_node["use_can_fd"].as<bool>();
    }
    if (defaults_node["bitrate_switch"]) {
      defaults.bitrate_switch = defaults_node["bitrate_switch"].as<bool>();
    }
    if (defaults_node["limits"] && has_limits_node(defaults_node["limits"])) {
      defaults.limits = parse_limits(defaults_node["limits"]);
    }
    parse_joint_safety_limits(defaults_node, defaults);
  }

  if (root["interfaces"]) {
    for (const auto & entry : root["interfaces"]) {
      const auto interface_name = read_required_string(entry, "name");
      interface_options_[interface_name] = InterfaceOptions {
        read_optional_value<bool>(entry, "use_can_fd", true),
        read_optional_value<bool>(entry, "bitrate_switch", true)
      };
    }
  }

  if (!root["motors"] || !root["motors"].IsSequence()) {
    throw std::runtime_error("Config must contain a motors sequence");
  }

  for (const auto & motor_node : root["motors"]) {
    MotorConfig config = defaults;
    config.name = read_required_string(motor_node, "name");
    config.interface_name = read_required_string(motor_node, "interface");
    config.can_id = motor_node["can_id"].as<uint32_t>();
    config.master_id = read_optional_value<uint32_t>(motor_node, "master_id", config.master_id);
    config.model = read_optional_value<std::string>(motor_node, "model", config.model);
    config.notes = read_optional_value<std::string>(motor_node, "notes", std::string());
    config.limits = make_model_mit_limits(config.model);
    {
      const auto [default_kp, default_kd] = make_model_default_pd_gains(config.model);
      config.default_kp = default_kp;
      config.default_kd = default_kd;
    }

    const auto interface_it = interface_options_.find(config.interface_name);
    if (interface_it != interface_options_.end()) {
      config.use_can_fd = interface_it->second.use_can_fd;
      config.bitrate_switch = interface_it->second.bitrate_switch;
    }

    if (motor_node["use_can_fd"]) {
      config.use_can_fd = motor_node["use_can_fd"].as<bool>();
    }
    if (motor_node["bitrate_switch"]) {
      config.bitrate_switch = motor_node["bitrate_switch"].as<bool>();
    }
    if (motor_node["limits"] && has_limits_node(motor_node["limits"])) {
      config.limits = parse_limits(motor_node["limits"]);
    }
    parse_joint_safety_limits(motor_node, config);
    config.default_kp = read_optional_value<float>(motor_node, "default_kp", config.default_kp);
    config.default_kd = read_optional_value<float>(motor_node, "default_kd", config.default_kd);

    configs_.push_back(config);
    motor_order_.push_back(config.name);
  }
}

void MotorManager::validate_config() const
{
  std::set<std::string> names;
  std::set<std::string> route_keys;
  std::set<std::pair<std::string, uint8_t>> low_byte_keys;

  for (const auto & config : configs_) {
    if (!names.insert(config.name).second) {
      throw std::runtime_error("Duplicate motor name in config: " + config.name);
    }

    if (config.can_id >= 0x10U) {
      throw std::runtime_error(
              "CAN ID must stay below 0x10 for Damiao feedback compatibility: " + config.name);
    }

    if (config.master_id != config.can_id + 0x10U) {
      throw std::runtime_error("master_id must equal can_id + 0x10: " + config.name);
    }

    const auto route = route_key(config.interface_name, feedback_nibble(config.can_id));
    if (!route_keys.insert(route).second) {
      throw std::runtime_error(
              "Duplicate low-4-bit CAN ID route on " + config.interface_name +
              ", which would make feedback ambiguous");
    }

    const auto low_byte = std::make_pair(config.interface_name, can_id_low_byte(config.can_id));
    if (!low_byte_keys.insert(low_byte).second) {
      throw std::runtime_error(
              "Duplicate low-8-bit CAN ID on " + config.interface_name +
              ", motor protocol would conflict");
    }
  }
}

void MotorManager::open_transports()
{
  for (const auto & config : configs_) {
    if (transports_.find(config.interface_name) == transports_.end()) {
      transports_[config.interface_name] = std::make_shared<SocketCanTransport>(config.interface_name);
    }
  }

  for (const auto & config : configs_) {
    auto driver = std::make_unique<DmMotorDriver>(config, transports_.at(config.interface_name));
    feedback_routes_[route_key(config.interface_name, feedback_nibble(config.can_id))] = driver.get();
    drivers_[config.name] = std::move(driver);
  }
}

DmMotorDriver * MotorManager::find_driver(const std::string & name)
{
  const auto it = drivers_.find(name);
  return it == drivers_.end() ? nullptr : it->second.get();
}

const DmMotorDriver * MotorManager::find_driver(const std::string & name) const
{
  const auto it = drivers_.find(name);
  return it == drivers_.end() ? nullptr : it->second.get();
}

std::optional<MotorManager::FeedbackMatch> MotorManager::handle_feedback_frame(
  const std::string & interface_name,
  const CanFrame & frame)
{
  if (frame.size < 1U) {
    return std::nullopt;
  }

  const uint8_t nibble = static_cast<uint8_t>(frame.data[0] & 0x0FU);
  const auto route_it = feedback_routes_.find(route_key(interface_name, nibble));
  if (route_it == feedback_routes_.end()) {
    return std::nullopt;
  }

  auto * driver = route_it->second;
  const auto state = protocol::decode_feedback_frame(frame, driver->config());
  if (!state.has_value()) {
    return std::nullopt;
  }

  driver->remember_state(*state);
  return FeedbackMatch {driver->config().name, *state};
}

std::string MotorManager::route_key(const std::string & interface_name, const uint8_t nibble) const
{
  return interface_name + "#" + std::to_string(nibble);
}

std::vector<MotorCommandResult> MotorManager::send_batch_and_collect(
  const std::vector<std::pair<DmMotorDriver *, CanFrame>> & batch,
  const std::string & action,
  const std::optional<MotorStatus> expected_status,
  const std::chrono::milliseconds timeout)
{
  for (const auto & [driver, frame] : batch) {
    driver->transport()->send(frame);
  }

  if (timeout.count() == 0 && !expected_status.has_value()) {
    std::vector<MotorCommandResult> results;
    results.reserve(batch.size());
    for (const auto & [driver, unused_frame] : batch) {
      (void)unused_frame;
      results.push_back({
        driver->config().name,
        true,
        action + ": transmitted",
        std::nullopt
      });
    }
    return results;
  }

  std::unordered_map<std::string, MotorState> received_states;
  for (const auto & [interface_name, transport] : transports_) {
    const auto frames = transport->receive_available(timeout, 256U);
    for (const auto & frame : frames) {
      const auto match = handle_feedback_frame(interface_name, frame);
      if (match.has_value()) {
        received_states[match->name] = match->state;
      }
    }
  }

  std::vector<MotorCommandResult> results;
  results.reserve(batch.size());
  for (const auto & [driver, unused_frame] : batch) {
    (void)unused_frame;
    std::optional<MotorState> state;
    const auto state_it = received_states.find(driver->config().name);
    if (state_it != received_states.end()) {
      state = state_it->second;
    }
    results.emplace_back(build_result_from_feedback(*driver, action, expected_status, state));
  }
  return results;
}

MotorCommandResult MotorManager::build_result_from_feedback(
  const DmMotorDriver & driver,
  const std::string & action,
  const std::optional<MotorStatus> expected_status,
  const std::optional<MotorState> & state) const
{
  MotorCommandResult result;
  result.name = driver.config().name;
  result.state = state;

  if (!state.has_value()) {
    result.success = false;
    result.message = action + ": no feedback received";
    return result;
  }

  if (expected_status.has_value()) {
    if (state->status == *expected_status) {
      result.success = true;
      result.message = action + ": success";
      return result;
    }

    if (is_fault_status(state->status)) {
      result.success = false;
      result.message = action + ": motor fault " + to_string(state->status);
      return result;
    }

    result.success = false;
    result.message = action + ": received state " + to_string(state->status);
    return result;
  }

  result.success = !is_fault_status(state->status);
  result.message = result.success ?
    action + ": feedback received" :
    action + ": motor fault " + to_string(state->status);
  return result;
}

}  // namespace jljwholebody_hardware::dm_motor
