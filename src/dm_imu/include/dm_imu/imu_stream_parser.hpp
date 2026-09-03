#ifndef DM_IMU__IMU_STREAM_PARSER_HPP_
#define DM_IMU__IMU_STREAM_PARSER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace dm_imu
{

struct ImuFrame
{
  uint8_t device_id {0};
  uint8_t type {0};
  std::array<float, 3> values {};
};

struct ImuParserStats
{
  uint64_t bytes {0};
  uint64_t valid_frames {0};
  uint64_t crc_errors {0};
  uint64_t format_errors {0};
  uint64_t discarded_bytes {0};
};

class ImuStreamParser
{
public:
  using FrameCallback = std::function<void(const ImuFrame &)>;

  explicit ImuStreamParser(FrameCallback callback);

  void push(const uint8_t * data, std::size_t size);
  void reset();
  const ImuParserStats & stats() const noexcept;

private:
  void parse_available();

  FrameCallback callback_;
  std::vector<uint8_t> buffer_;
  ImuParserStats stats_;
};

}  // namespace dm_imu

#endif  // DM_IMU__IMU_STREAM_PARSER_HPP_
