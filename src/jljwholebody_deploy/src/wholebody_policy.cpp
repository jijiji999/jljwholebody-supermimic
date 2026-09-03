#include "jljwholebody_deploy/wholebody_policy.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace jljwholebody_deploy
{

WholebodyObservationHistory::WholebodyObservationHistory(
  const std::size_t joint_count,
  const std::size_t history_length)
: joint_count_(joint_count),
  history_length_(history_length),
  term_sizes_ {3U, 3U, 3U, 2U, joint_count, joint_count, joint_count},
  joint_position_relative_(joint_count),
  joint_velocity_(joint_count)
{
  if (joint_count_ == 0U) {
    throw std::invalid_argument("joint_count must be greater than zero");
  }
  if (history_length_ == 0U) {
    throw std::invalid_argument("history_length must be greater than zero");
  }

  std::size_t offset = 0U;
  for (std::size_t i = 0U; i < term_sizes_.size(); ++i) {
    term_offsets_[i] = offset;
    offset += term_sizes_[i] * history_length_;
  }
  observation_.assign(offset, 0.0F);
}

void WholebodyObservationHistory::reset()
{
  std::fill(observation_.begin(), observation_.end(), 0.0F);
  initialized_ = false;
}

void WholebodyObservationHistory::update(
  const std::array<float, 3> & base_angular_velocity,
  const std::array<float, 3> & projected_gravity,
  const std::array<float, 3> & command,
  const std::array<float, 2> & phase,
  const std::vector<double> & joint_positions,
  const std::vector<double> & default_joint_positions,
  const std::vector<double> & joint_velocities,
  const std::vector<float> & previous_action)
{
  if (joint_positions.size() != joint_count_ ||
    default_joint_positions.size() != joint_count_ ||
    joint_velocities.size() != joint_count_ ||
    previous_action.size() != joint_count_)
  {
    throw std::invalid_argument("wholebody observation joint vector size mismatch");
  }

  for (std::size_t i = 0U; i < joint_count_; ++i) {
    joint_position_relative_[i] = static_cast<float>(
      joint_positions[i] - default_joint_positions[i]);
    joint_velocity_[i] = static_cast<float>(joint_velocities[i]);
  }

  update_term(0U, base_angular_velocity.data(), base_angular_velocity.size());
  update_term(1U, projected_gravity.data(), projected_gravity.size());
  update_term(2U, command.data(), command.size());
  update_term(3U, phase.data(), phase.size());
  update_term(4U, joint_position_relative_.data(), joint_position_relative_.size());
  update_term(5U, joint_velocity_.data(), joint_velocity_.size());
  update_term(6U, previous_action.data(), previous_action.size());
  initialized_ = true;
}

const std::vector<float> & WholebodyObservationHistory::data() const
{
  return observation_;
}

std::size_t WholebodyObservationHistory::single_frame_size() const
{
  return std::accumulate(term_sizes_.begin(), term_sizes_.end(), std::size_t {0U});
}

std::size_t WholebodyObservationHistory::history_length() const
{
  return history_length_;
}

bool WholebodyObservationHistory::initialized() const
{
  return initialized_;
}

void WholebodyObservationHistory::update_term(
  const std::size_t term_index,
  const float * values,
  const std::size_t value_count)
{
  if (term_index >= term_sizes_.size() || value_count != term_sizes_[term_index]) {
    throw std::invalid_argument("wholebody observation term size mismatch");
  }

  const std::size_t offset = term_offsets_[term_index];
  const std::size_t term_size = term_sizes_[term_index];
  auto term_begin = observation_.begin() + static_cast<std::ptrdiff_t>(offset);

  if (!initialized_) {
    for (std::size_t frame = 0U; frame < history_length_; ++frame) {
      std::copy_n(
        values, term_size,
        term_begin + static_cast<std::ptrdiff_t>(frame * term_size));
    }
    return;
  }

  std::move(
    term_begin + static_cast<std::ptrdiff_t>(term_size),
    term_begin + static_cast<std::ptrdiff_t>(history_length_ * term_size),
    term_begin);
  std::copy_n(
    values, term_size,
    term_begin + static_cast<std::ptrdiff_t>((history_length_ - 1U) * term_size));
}

OnnxPolicyRuntime::OnnxPolicyRuntime(
  const std::string & model_path,
  const std::size_t expected_input_size,
  const std::size_t expected_output_size,
  const int intra_op_threads)
: env_(ORT_LOGGING_LEVEL_WARNING, "wholebody_policy"),
  memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
  if (intra_op_threads < 1) {
    throw std::invalid_argument("ONNX intra_op_threads must be at least one");
  }

  session_options_.SetIntraOpNumThreads(intra_op_threads);
  session_options_.SetInterOpNumThreads(1);
  session_options_.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
  session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  session_options_.EnableCpuMemArena();
  session_options_.EnableMemPattern();
  session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);

  if (session_->GetInputCount() != 1U || session_->GetOutputCount() != 1U) {
    throw std::runtime_error("wholebody policy must have exactly one input and one output");
  }

  auto allocated_input_name = session_->GetInputNameAllocated(0U, allocator_);
  auto allocated_output_name = session_->GetOutputNameAllocated(0U, allocator_);
  input_name_ = allocated_input_name.get();
  output_name_ = allocated_output_name.get();
  input_names_[0] = input_name_.c_str();
  output_names_[0] = output_name_.c_str();

  input_shape_ = session_->GetInputTypeInfo(0U).GetTensorTypeAndShapeInfo().GetShape();
  output_shape_ = session_->GetOutputTypeInfo(0U).GetTensorTypeAndShapeInfo().GetShape();
  const std::size_t model_input_size = tensor_element_count(input_shape_, input_name_);
  const std::size_t model_output_size = tensor_element_count(output_shape_, output_name_);
  if (model_input_size != expected_input_size) {
    throw std::runtime_error(
            "ONNX input size mismatch: model=" + std::to_string(model_input_size) +
            ", profile=" + std::to_string(expected_input_size));
  }
  if (model_output_size != expected_output_size) {
    throw std::runtime_error(
            "ONNX output size mismatch: model=" + std::to_string(model_output_size) +
            ", profile=" + std::to_string(expected_output_size));
  }

  input_buffer_.assign(model_input_size, 0.0F);
  output_buffer_.assign(model_output_size, 0.0F);
  input_tensor_ = std::make_unique<Ort::Value>(Ort::Value::CreateTensor<float>(
      memory_info_, input_buffer_.data(), input_buffer_.size(),
      input_shape_.data(), input_shape_.size()));
  output_tensor_ = std::make_unique<Ort::Value>(Ort::Value::CreateTensor<float>(
      memory_info_, output_buffer_.data(), output_buffer_.size(),
      output_shape_.data(), output_shape_.size()));
}

const std::vector<float> & OnnxPolicyRuntime::infer(const std::vector<float> & observation)
{
  if (observation.size() != input_buffer_.size()) {
    throw std::invalid_argument(
            "observation size mismatch: expected " + std::to_string(input_buffer_.size()) +
            ", received " + std::to_string(observation.size()));
  }
  if (!std::all_of(observation.begin(), observation.end(), [](const float value) {
      return std::isfinite(value);
    }))
  {
    throw std::invalid_argument("observation contains NaN or Inf");
  }

  std::copy(observation.begin(), observation.end(), input_buffer_.begin());
  session_->Run(
    Ort::RunOptions {nullptr},
    input_names_.data(), input_tensor_.get(), input_names_.size(),
    output_names_.data(), output_tensor_.get(), output_names_.size());

  if (!std::all_of(output_buffer_.begin(), output_buffer_.end(), [](const float value) {
      return std::isfinite(value);
    }))
  {
    throw std::runtime_error("policy output contains NaN or Inf");
  }
  return output_buffer_;
}

std::size_t OnnxPolicyRuntime::input_size() const
{
  return input_buffer_.size();
}

std::size_t OnnxPolicyRuntime::output_size() const
{
  return output_buffer_.size();
}

const std::string & OnnxPolicyRuntime::input_name() const
{
  return input_name_;
}

const std::string & OnnxPolicyRuntime::output_name() const
{
  return output_name_;
}

std::size_t OnnxPolicyRuntime::tensor_element_count(
  std::vector<int64_t> & shape,
  const std::string & tensor_name)
{
  if (shape.empty()) {
    throw std::runtime_error("ONNX tensor has no dimensions: " + tensor_name);
  }
  std::size_t count = 1U;
  for (std::size_t i = 0U; i < shape.size(); ++i) {
    if (shape[i] == -1 && i == 0U) {
      shape[i] = 1;
    }
    if (shape[i] <= 0) {
      throw std::runtime_error(
              "ONNX tensor has unsupported dynamic dimension: " + tensor_name);
    }
    count *= static_cast<std::size_t>(shape[i]);
  }
  return count;
}

}  // namespace jljwholebody_deploy
