#ifndef JLJWHOLEBODY_HARDWARE__DM_MOTOR_MANAGER_HPP_
#define JLJWHOLEBODY_HARDWARE__DM_MOTOR_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "jljwholebody_hardware/dm_motor_driver.hpp"

namespace jljwholebody_hardware::dm_motor
{

class MotorManager
{
public:
  explicit MotorManager(const std::string & config_path);

  const std::string & config_path() const;
  const std::vector<std::string> & motor_names() const;
  std::vector<MotorConfig> motor_configs() const;
  std::optional<MotorConfig> motor_config(const std::string & name) const;

  MotorCommandResult enable_motor(
    const std::string & name,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  MotorCommandResult disable_motor(
    const std::string & name,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  std::vector<MotorCommandResult> enable_all(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  std::vector<MotorCommandResult> disable_all(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  std::vector<MotorCommandResult> command_group_mit(
    const std::vector<NamedMitCommand> & commands,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  std::vector<MotorCommandResult> transmit_group_mit(
    const std::vector<NamedMitCommand> & commands,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(0));
  std::vector<MotorCommandResult> transmit_group_mit_unclamped(
    const std::vector<NamedMitCommand> & commands,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(0));
  void transmit_group_mit_fast(const std::vector<NamedMitCommand> & commands);

  std::vector<MotorState> snapshot_states() const;
  bool poll_once(std::chrono::milliseconds timeout = std::chrono::milliseconds(0));

private:
  struct InterfaceOptions
  {
    bool use_can_fd {true};
    bool bitrate_switch {true};
  };

  struct FeedbackMatch
  {
    std::string name;
    MotorState state;
  };

  void load_config(const std::string & path);
  void validate_config() const;
  void open_transports();

  DmMotorDriver * find_driver(const std::string & name);
  const DmMotorDriver * find_driver(const std::string & name) const;
  std::optional<FeedbackMatch> handle_feedback_frame(
    const std::string & interface_name,
    const CanFrame & frame);
  std::string route_key(const std::string & interface_name, uint8_t nibble) const;
  std::vector<MotorCommandResult> send_batch_and_collect(
    const std::vector<std::pair<DmMotorDriver *, CanFrame>> & batch,
    const std::string & action,
    std::optional<MotorStatus> expected_status,
    std::chrono::milliseconds timeout);
  MotorCommandResult build_result_from_feedback(
    const DmMotorDriver & driver,
    const std::string & action,
    std::optional<MotorStatus> expected_status,
    const std::optional<MotorState> & state) const;

  std::string config_path_;
  std::vector<std::string> motor_order_;
  std::vector<MotorConfig> configs_;
  std::unordered_map<std::string, InterfaceOptions> interface_options_;
  std::unordered_map<std::string, std::shared_ptr<SocketCanTransport>> transports_;
  std::unordered_map<std::string, std::unique_ptr<DmMotorDriver>> drivers_;
  std::unordered_map<std::string, DmMotorDriver *> feedback_routes_;
};

}  // namespace jljwholebody_hardware::dm_motor

#endif  // JLJWHOLEBODY_HARDWARE__DM_MOTOR_MANAGER_HPP_
