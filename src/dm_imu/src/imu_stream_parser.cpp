#include "dm_imu/imu_stream_parser.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

#include "dm_imu/bsp_crc.h"

namespace dm_imu
{
namespace
{
constexpr std::size_t kFrameSize = 19;
constexpr std::array<uint8_t, 2> kHeader {0x55U, 0xAAU};

float decode_float(const uint8_t * bytes)
{
  float value = 0.0F;
  std::memcpy(&value, bytes, sizeof(value));
  return value;
}
}  // namespace

ImuStreamParser::ImuStreamParser(FrameCallback callback)
: callback_(std::move(callback))
{
  buffer_.reserve(kFrameSize * 4U);
}

void ImuStreamParser::push(const uint8_t * data, const std::size_t size)
{
  if (data == nullptr || size == 0U) {
    return;
  }
  stats_.bytes += size;
  buffer_.insert(buffer_.end(), data, data + size);
  parse_available();
}

void ImuStreamParser::reset()
{
  buffer_.clear();
  stats_ = {};
}

const ImuParserStats & ImuStreamParser::stats() const noexcept
{
  return stats_;
}

void ImuStreamParser::parse_available()
{
  while (buffer_.size() >= 2U) {
    const auto header = std::search(
      buffer_.begin(), buffer_.end(),
      kHeader.begin(), kHeader.end());
    if (header != buffer_.begin()) {
      const std::size_t discarded = header == buffer_.end() ?
        buffer_.size() - 1U : static_cast<std::size_t>(header - buffer_.begin());
      stats_.discarded_bytes += discarded;
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(discarded));
    }
    if (buffer_.size() < kFrameSize) {
      return;
    }

    const bool known_type = buffer_[3] >= 0x01U && buffer_[3] <= 0x04U;
    if (!known_type || buffer_[18] != 0x0AU) {
      ++stats_.format_errors;
      ++stats_.discarded_bytes;
      buffer_.erase(buffer_.begin());
      continue;
    }

    const uint16_t expected_crc = static_cast<uint16_t>(buffer_[16]) |
      (static_cast<uint16_t>(buffer_[17]) << 8U);
    const uint16_t actual_crc = Get_CRC16(buffer_.data(), 16U);
    if (actual_crc != expected_crc) {
      ++stats_.crc_errors;
      ++stats_.discarded_bytes;
      buffer_.erase(buffer_.begin());
      continue;
    }

    ImuFrame frame;
    frame.device_id = buffer_[2];
    frame.type = buffer_[3];
    for (std::size_t i = 0; i < frame.values.size(); ++i) {
      frame.values[i] = decode_float(buffer_.data() + 4U + i * sizeof(float));
    }
    ++stats_.valid_frames;
    if (callback_) {
      callback_(frame);
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + kFrameSize);
  }
}

}  // namespace dm_imu
