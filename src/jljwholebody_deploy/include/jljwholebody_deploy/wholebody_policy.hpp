#ifndef JLJWHOLEBODY_DEPLOY__WHOLEBODY_POLICY_HPP_
#define JLJWHOLEBODY_DEPLOY__WHOLEBODY_POLICY_HPP_

#include <onnxruntime_cxx_api.h>

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace jljwholebody_deploy
{

class WholebodyObservationHistory
{
public:
  WholebodyObservationHistory(std::size_t joint_count, std::size_t history_length);

  void reset();
  void update(
    const std::array<float, 3> & base_angular_velocity,
    const std::array<float, 3> & projected_gravity,
    const std::array<float, 3> & command,
    const std::array<float, 2> & phase,
    const std::vector<double> & joint_positions,
    const std::vector<double> & default_joint_positions,
    const std::vector<double> & joint_velocities,
    const std::vector<float> & previous_action);

  const std::vector<float> & data() const;
  std::size_t single_frame_size() const;
  std::size_t history_length() const;
  bool initialized() const;

private:
  void update_term(std::size_t term_index, const float * values, std::size_t value_count);

  std::size_t joint_count_;
  std::size_t history_length_;
  std::array<std::size_t, 7> term_sizes_;
  std::array<std::size_t, 7> term_offsets_;
  std::vector<float> observation_;
  std::vector<float> joint_position_relative_;
  std::vector<float> joint_velocity_;
  bool initialized_ {false};
};

class OnnxPolicyRuntime
{
public:
  OnnxPolicyRuntime(
    const std::string & model_path,
    std::size_t expected_input_size,
    std::size_t expected_output_size,
    int intra_op_threads = 1,
    std::size_t goal_input_size = 0U);

  const std::vector<float> & infer(
    const std::vector<float> & observation, const std::vector<float> & goal = {});
  std::size_t input_size() const;
  std::size_t output_size() const;
  const std::string & input_name() const;
  const std::string & output_name() const;

private:
  static std::size_t tensor_element_count(
    std::vector<int64_t> & shape,
    const std::string & tensor_name);

  Ort::Env env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  Ort::AllocatorWithDefaultOptions allocator_;
  Ort::MemoryInfo memory_info_;
  std::string input_name_;
  std::string output_name_;
  std::string goal_name_;
  std::array<const char *, 2> input_names_ {};
  std::array<const char *, 1> output_names_;
  std::vector<int64_t> input_shape_;
  std::vector<int64_t> output_shape_;
  std::vector<float> input_buffer_;
  std::vector<float> output_buffer_;
  std::vector<float> goal_buffer_;
  std::array<Ort::Value, 2> input_tensors_ {Ort::Value {nullptr}, Ort::Value {nullptr}};
  std::unique_ptr<Ort::Value> output_tensor_;
};

}  // namespace jljwholebody_deploy

#endif  // JLJWHOLEBODY_DEPLOY__WHOLEBODY_POLICY_HPP_
