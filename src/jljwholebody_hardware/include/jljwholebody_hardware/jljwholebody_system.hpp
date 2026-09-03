#ifndef JLJWHOLEBODY_HARDWARE__JLJWHOLEBODY_SYSTEM_HPP_
#define JLJWHOLEBODY_HARDWARE__JLJWHOLEBODY_SYSTEM_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "jljwholebody_hardware/dm_motor_manager.hpp"
#include "jljwholebody_hardware/dm_motor_types.hpp"
#include "jljwholebody_hardware/parallel_kinematics.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace jljwholebody_hardware
{

class JljWholebodySystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(JljWholebodySystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  struct DirectJointBinding
  {
    std::size_t joint_index {0U};
    std::string joint_name;
    std::string motor_name;
    dm_motor::MotorConfig motor_config {};
  };

  struct ParallelMechanismBinding
  {
    std::string name;
    std::string pitch_joint_name;
    std::string roll_joint_name;
    std::string motor_1_name;
    std::string motor_2_name;
    std::string config_path;
    std::size_t pitch_joint_index {0U};
    std::size_t roll_joint_index {0U};
    dm_motor::MotorConfig motor_1_config {};
    dm_motor::MotorConfig motor_2_config {};
    ParallelKinematics kinematics {};
  };

  void sync_state_from_feedback(const std::vector<dm_motor::MotorState> & states);
  std::string summarize_feedback_snapshot(const std::vector<dm_motor::MotorState> & states) const;
  void warmup_activation_feedback();
  void enable_motors_on_activate();
  void log_failed_results(
    const std::vector<dm_motor::MotorCommandResult> & results,
    const std::string & action);

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_;

  std::string motor_config_path_;
  std::string parallel_mechanisms_path_;
  std::unordered_map<std::string, std::size_t> joint_index_by_name_;
  std::vector<DirectJointBinding> direct_joint_bindings_;
  std::vector<ParallelMechanismBinding> parallel_mechanism_bindings_;
  std::unique_ptr<dm_motor::MotorManager> motor_manager_;

  bool enable_on_activate_{false};
  bool disable_on_deactivate_{true};
  int command_timeout_ms_{0};
  int feedback_poll_timeout_ms_{0};
  int lifecycle_timeout_ms_{500};
  double startup_feedback_warmup_s_{0.5};
  int enable_retries_{3};
  int enable_retry_interval_ms_{200};
  bool active_{false};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace jljwholebody_hardware

#endif  // JLJWHOLEBODY_HARDWARE__JLJWHOLEBODY_SYSTEM_HPP_
