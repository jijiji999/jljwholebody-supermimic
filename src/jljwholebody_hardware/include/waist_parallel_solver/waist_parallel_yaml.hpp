#ifndef JLJWHOLEBODY_PARALLEL_SOLVER__WAIST_PARALLEL_YAML_HPP_
#define JLJWHOLEBODY_PARALLEL_SOLVER__WAIST_PARALLEL_YAML_HPP_

#include "waist_parallel_solver/waist_parallel_solver.hpp"

#include <string>

namespace waist_parallel
{

// 从 waist.yaml 加载机构参数（几何、rod_length、motor_direction、theta_range、
// pitch/roll 范围）。注意：解算器本身不读取 YAML 的 pitch_range/roll_range，
// 此加载器按部署约定填充默认姿态限制（pitch [-0.5,0.5]、
// roll [-0.2,0.2]），theta_range 若 YAML 有则读取。
bool load_waist_yaml(
  const std::string & yaml_file,
  MechanismParams & params,
  std::string & error_message);

}  // namespace waist_parallel

#endif  // JLJWHOLEBODY_PARALLEL_SOLVER__WAIST_PARALLEL_YAML_HPP_
