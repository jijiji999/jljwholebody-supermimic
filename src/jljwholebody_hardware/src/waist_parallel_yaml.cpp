#include "waist_parallel_solver/waist_parallel_yaml.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace waist_parallel
{

namespace
{
Eigen::Vector3d read_vector3(const YAML::Node & node, const char * key)
{
  const auto values = node[key];
  if (!values || !values.IsSequence() || values.size() != 3U) {
    throw std::runtime_error(std::string("Expected 3-vector for key: ") + key);
  }
  return Eigen::Vector3d(
    values[0].as<double>(), values[1].as<double>(), values[2].as<double>());
}

double read_direction(const YAML::Node & node, const std::size_t index)
{
  if (!node) {
    return 1.0;
  }
  if (!node.IsSequence() || node.size() != 2U) {
    throw std::runtime_error("Expected 2-vector for key: motor_direction");
  }
  return node[index].as<double>() < 0.0 ? -1.0 : 1.0;
}

double read_optional_double(const YAML::Node & node, const char * key, const double fallback)
{
  return node[key] ? node[key].as<double>() : fallback;
}

void read_theta_range(
  const YAML::Node & node,
  const char * key,
  const double fallback_min,
  const double fallback_max,
  double & min_value,
  double & max_value)
{
  min_value = fallback_min;
  max_value = fallback_max;
  const auto values = node[key];
  if (!values || !values.IsSequence() || values.size() != 2U) {
    return;
  }
  min_value = values[0].as<double>();
  max_value = values[1].as<double>();
}
}  // namespace

bool load_waist_yaml(
  const std::string & yaml_file,
  MechanismParams & params,
  std::string & error_message)
{
  try {
    const YAML::Node config = YAML::LoadFile(yaml_file);

    const std::string type_name = config["joint_type"] ?
      config["joint_type"].as<std::string>() : "ankle";
    if (type_name != "waist") {
      throw std::runtime_error("Expected joint_type=waist, got: " + type_name);
    }

    params = MechanismParams {};
    params.a1 = read_vector3(config, "ra01");
    params.a2 = read_vector3(config, "ra02");
    const Eigen::Vector3d rb1 = read_vector3(config, "rb01");
    const Eigen::Vector3d rb2 = read_vector3(config, "rb02");
    params.c1 = read_vector3(config, "rc01");
    params.c2 = read_vector3(config, "rc02");
    params.p1 = rb1 - params.a1;
    params.p2 = rb2 - params.a2;

    const auto rod_lengths = config["rod_length"];
    if (rod_lengths && rod_lengths.IsSequence() && rod_lengths.size() == 2U) {
      params.rod1 = rod_lengths[0].as<double>();
      params.rod2 = rod_lengths[1].as<double>();
    } else {
      params.rod1 = (rb1 - params.c1).norm();
      params.rod2 = (rb2 - params.c2).norm();
    }

    params.offset1 = read_optional_double(config, "m01", 0.0);
    params.offset2 = read_optional_double(config, "m02", 0.0);
    params.direction1 = read_direction(config["motor_direction"], 0U);
    params.direction2 = read_direction(config["motor_direction"], 1U);

    read_theta_range(
      config, "theta_range_1", -0.6, 0.8, params.theta_min1, params.theta_max1);
    read_theta_range(
      config, "theta_range_2", -0.6, 0.8, params.theta_min2, params.theta_max2);

    // 姿态限制按部署约定使用代码默认值，避免未验证的 YAML 占位范围。
    params.pitch_min = -0.5;
    params.pitch_max = 0.5;
    params.roll_min = -0.2;
    params.roll_max = 0.2;

    error_message.clear();
    return true;
  } catch (const std::exception & ex) {
    error_message = ex.what();
    return false;
  }
}

}  // namespace waist_parallel
