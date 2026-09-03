#include "dm_imu/imu_driver.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

namespace dmbot_serial
{

DmImu::DmImu()
: Node("dm_imu_node")
{
  declare_parameter<std::string>("port", "/dev/dm_imu");
  declare_parameter<int>("baud", 921600);
  declare_parameter<std::string>("frame_id", "imu_link");
  declare_parameter<std::string>("imu_topic", "/imu/data");
  declare_parameter<std::string>("pose_topic", "/imu/pose");
  declare_parameter<std::string>("status_topic", "/imu/status");
  declare_parameter<bool>("publish_pose", false);
  declare_parameter<bool>("configure_on_start", true);
  declare_parameter<bool>("save_configuration", false);
  declare_parameter<int>("output_interval_ms", 1);
  declare_parameter<int>("command_repetitions", 2);

  port_ = get_parameter("port").as_string();
  baud_ = get_parameter("baud").as_int();
  frame_id_ = get_parameter("frame_id").as_string();
  imu_topic_ = get_parameter("imu_topic").as_string();
  pose_topic_ = get_parameter("pose_topic").as_string();
  status_topic_ = get_parameter("status_topic").as_string();
  publish_pose_ = get_parameter("publish_pose").as_bool();
  configure_on_start_ = get_parameter("configure_on_start").as_bool();
  save_configuration_ = get_parameter("save_configuration").as_bool();
  output_interval_ms_ = get_parameter("output_interval_ms").as_int();
  command_repetitions_ = get_parameter("command_repetitions").as_int();

  if (output_interval_ms_ < 1 || output_interval_ms_ > 10) {
    throw std::invalid_argument("output_interval_ms must be in [1, 10] (1000 to 100 Hz)");
  }
  if (command_repetitions_ < 1 || command_repetitions_ > 5) {
    throw std::invalid_argument("command_repetitions must be in [1, 5]");
  }

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
  status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
  if (publish_pose_) {
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
  }

  parser_ = std::make_unique<dm_imu::ImuStreamParser>(
    [this](const dm_imu::ImuFrame & frame) {handle_frame(frame);});
  open_serial();
  if (configure_on_start_) {
    configure_device();
  }

  last_status_time_ = SteadyClock::now();
  receive_thread_ = std::thread(&DmImu::receive_loop, this);
  RCLCPP_INFO(
    get_logger(),
    "DM IMU started: port=%s baud=%d interval=%dms configure=%s save=%s",
    port_.c_str(), baud_, output_interval_ms_, configure_on_start_ ? "yes" : "no",
    save_configuration_ ? "yes" : "no");
}

DmImu::~DmImu()
{
  stop_requested_.store(true);
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  if (serial_.isOpen()) {
    serial_.close();
  }
}

void DmImu::open_serial()
{
  try {
    serial_.setPort(port_);
    serial_.setBaudrate(static_cast<uint32_t>(baud_));
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    auto timeout = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const serial::IOException & ex) {
    throw std::runtime_error("failed to open IMU serial port " + port_ + ": " + ex.what());
  }
  if (!serial_.isOpen()) {
    throw std::runtime_error("failed to open IMU serial port " + port_);
  }
  serial_.flushInput();
}

void DmImu::send_command(const uint8_t * data, const std::size_t size)
{
  for (int attempt = 0; attempt < command_repetitions_; ++attempt) {
    if (serial_.write(data, size) != size) {
      throw std::runtime_error("short write while configuring IMU");
    }
    std::this_thread::sleep_for(10ms);
  }
}

void DmImu::configure_device()
{
  const std::array<uint8_t, 4> enter {0xAA, 0x06, 0x01, 0x0D};
  const std::array<uint8_t, 4> accel_on {0xAA, 0x01, 0x14, 0x0D};
  const std::array<uint8_t, 4> gyro_on {0xAA, 0x01, 0x15, 0x0D};
  const std::array<uint8_t, 4> euler_on {0xAA, 0x01, 0x16, 0x0D};
  const std::array<uint8_t, 4> quaternion_off {0xAA, 0x01, 0x07, 0x0D};
  const std::array<uint8_t, 5> interval {
    0xAA, 0x02,
    static_cast<uint8_t>(output_interval_ms_ & 0xFF),
    static_cast<uint8_t>((output_interval_ms_ >> 8) & 0xFF),
    0x0D};
  const std::array<uint8_t, 4> save {0xAA, 0x03, 0x01, 0x0D};
  const std::array<uint8_t, 4> exit {0xAA, 0x06, 0x00, 0x0D};

  send_command(enter.data(), enter.size());
  send_command(accel_on.data(), accel_on.size());
  send_command(gyro_on.data(), gyro_on.size());
  send_command(euler_on.data(), euler_on.size());
  send_command(quaternion_off.data(), quaternion_off.size());
  send_command(interval.data(), interval.size());
  if (save_configuration_) {
    send_command(save.data(), save.size());
  }
  send_command(exit.data(), exit.size());
  serial_.flushInput();
  std::this_thread::sleep_for(100ms);
}

void DmImu::receive_loop()
{
  std::array<uint8_t, 512> bytes {};
  while (rclcpp::ok() && !stop_requested_.load()) {
    try {
      const std::size_t available = serial_.available();
      if (available == 0U) {
        std::this_thread::sleep_for(200us);
        publish_status_if_due();
        continue;
      }
      const std::size_t received = serial_.read(bytes.data(), std::min(available, bytes.size()));
      parser_->push(bytes.data(), received);
      publish_status_if_due();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "IMU serial error: %s", ex.what());
      std::this_thread::sleep_for(10ms);
    }
  }
}

void DmImu::handle_frame(const dm_imu::ImuFrame & frame)
{
  switch (frame.type) {
    case 0x01:
      acceleration_ = frame.values;
      have_acceleration_ = true;
      have_angular_velocity_ = false;
      have_euler_ = false;
      break;
    case 0x02:
      angular_velocity_ = frame.values;
      have_angular_velocity_ = true;
      break;
    case 0x03:
      euler_deg_ = frame.values;
      have_euler_ = true;
      break;
    default:
      return;
  }

  if (have_acceleration_ && have_angular_velocity_ && have_euler_) {
    publish_sample();
    have_acceleration_ = false;
    have_angular_velocity_ = false;
    have_euler_ = false;
  }
}

void DmImu::publish_sample()
{
  sensor_msgs::msg::Imu message;
  message.header.stamp = get_clock()->now();
  message.header.frame_id = frame_id_;

  tf2::Quaternion orientation;
  orientation.setRPY(
    static_cast<double>(euler_deg_[0]) * M_PI / 180.0,
    static_cast<double>(euler_deg_[1]) * M_PI / 180.0,
    static_cast<double>(euler_deg_[2]) * M_PI / 180.0);
  orientation.normalize();
  message.orientation.w = orientation.w();
  message.orientation.x = orientation.x();
  message.orientation.y = orientation.y();
  message.orientation.z = orientation.z();
  message.angular_velocity.x = angular_velocity_[0];
  message.angular_velocity.y = angular_velocity_[1];
  message.angular_velocity.z = angular_velocity_[2];
  message.linear_acceleration.x = acceleration_[0];
  message.linear_acceleration.y = acceleration_[1];
  message.linear_acceleration.z = acceleration_[2];
  imu_pub_->publish(message);
  ++published_samples_;

  if (pose_pub_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = message.header;
    pose.pose.orientation = message.orientation;
    pose_pub_->publish(pose);
  }
}

void DmImu::publish_status_if_due()
{
  const auto now = SteadyClock::now();
  const double elapsed = std::chrono::duration<double>(now - last_status_time_).count();
  if (elapsed < 1.0) {
    return;
  }
  const auto stats = parser_->stats();
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(1)
         << "samples=" << static_cast<double>(published_samples_ - last_status_samples_) / elapsed << "Hz"
         << " frames=" << static_cast<double>(stats.valid_frames - last_parser_stats_.valid_frames) / elapsed << "Hz"
         << " crc_errors=" << (stats.crc_errors - last_parser_stats_.crc_errors)
         << " format_errors=" << (stats.format_errors - last_parser_stats_.format_errors)
         << " discarded_bytes=" << (stats.discarded_bytes - last_parser_stats_.discarded_bytes)
         << " total_samples=" << published_samples_;
  std_msgs::msg::String status;
  status.data = stream.str();
  status_pub_->publish(status);
  RCLCPP_INFO(get_logger(), "%s", status.data.c_str());

  last_status_time_ = now;
  last_status_samples_ = published_samples_;
  last_parser_stats_ = stats;
}

}  // namespace dmbot_serial
