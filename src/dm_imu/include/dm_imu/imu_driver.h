#ifndef DM_IMU__IMU_DRIVER_H_
#define DM_IMU__IMU_DRIVER_H_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <serial/serial.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

#include "dm_imu/imu_stream_parser.hpp"

namespace dmbot_serial
{

class DmImu : public rclcpp::Node
{
public:
  DmImu();
  ~DmImu() override;

private:
  using SteadyClock = std::chrono::steady_clock;

  void open_serial();
  void configure_device();
  void send_command(const uint8_t * data, std::size_t size);
  void receive_loop();
  void handle_frame(const dm_imu::ImuFrame & frame);
  void publish_sample();
  void publish_status_if_due();

  std::string port_;
  int baud_ {921600};
  std::string frame_id_;
  std::string imu_topic_;
  std::string pose_topic_;
  std::string status_topic_;
  bool publish_pose_ {false};
  bool configure_on_start_ {true};
  bool save_configuration_ {false};
  int output_interval_ms_ {1};
  int command_repetitions_ {2};

  serial::Serial serial_;
  std::thread receive_thread_;
  std::atomic<bool> stop_requested_ {false};
  std::unique_ptr<dm_imu::ImuStreamParser> parser_;

  std::array<float, 3> acceleration_ {};
  std::array<float, 3> angular_velocity_ {};
  std::array<float, 3> euler_deg_ {};
  bool have_acceleration_ {false};
  bool have_angular_velocity_ {false};
  bool have_euler_ {false};

  uint64_t published_samples_ {0};
  uint64_t last_status_samples_ {0};
  dm_imu::ImuParserStats last_parser_stats_ {};
  SteadyClock::time_point last_status_time_ {};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

}  // namespace dmbot_serial

#endif  // DM_IMU__IMU_DRIVER_H_
