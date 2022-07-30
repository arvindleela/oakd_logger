#pragma once
#include <depthai/depthai.hpp>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace oakd_logger {
using MAX_SIZE_T = std::numeric_limits<size_t>();
static constexpr size_t MAX_IMU_UPDATE_RATE_HZ = 500;
static constexpr size_t MAX_IMU_BUFFER_SIZE = 1000;  // 10s at 100Hz

// Define various streams
enum class DataStream : uint8_t {
  IMU,
  LEFT_MONO,
  RIGHT_MONO,
  RGB,
  INVALID,
};

struct DataStreamHash {
  template <typename T>
  uint8_t operator()(T t) const {
    return static_cast<uint8_t>(t);
  }
};

static const std::unordered_set<DataStream> MONO_CAMERAS(
    {DataStream::LEFT_MONO, DataStream::RIGHT_MONO});
static const std::unordered_map<DataStream, dai::CameraBoardSocket>
    CAMERA_SOCKET = {{DataStream::LEFT_MONO, dai::CameraBoardSocket::LEFT},
                     {DataStream::RIGHT_MONO, dai::CameraBoardSocket::RIGHT},
                     {DataStream::RGB, dai::CameraBoardSocket::RGB}};

// Define buffer sizes
static const std::unordered_map<DataStream, size_t> QUEUE_BUFFER_SIZE = {
    {DataStream::IMU, 400 /* 2s at 200Hz */},
    {DataStream::LEFT_MONO, 60 /* 2s at 30Hz */},
    {DataStream::RIGHT_MONO, 60 /* 2s at 30Hz */},
    {DataStream::RGB, 60 /* 2s at 30Hz */}};

// Config parameters with a reasonable initialization
struct Config {
  /**
   * @brief:  Sanity check on config parameters
   */
  bool sanity_check() const {
    // Check if IMU update rate is valid
    if (imu_update_rate_hz > MAX_IMU_UPDATE_RATE_HZ) {
      return false;
    }

    if (imu_buffer_size() > MAX_IMU_BUFFER_SIZE) {
      return false;
    }

    return true;
  }

  size_t imu_buffer_size() const {
    return static_cast<size_t>(imu_update_rate_hz * imu_buffer_size_s);
  }

  // Pipeline config
  std::unordered_map<DataStream, bool, DataStreamHash> disable_stream = {
      {DataStream::IMU, false},
      {DataStream::LEFT_MONO, false},
      {DataStream::RIGHT_MONO, false},
      {DataStream::RGB, true}};

  // IMU config
  size_t imu_batch_report_threshold = 1;
  size_t imu_max_batch_reports = 20;
  size_t imu_update_rate_hz = 100;
  double imu_buffer_size_s = 10;
};
}  // namespace oakd_logger
