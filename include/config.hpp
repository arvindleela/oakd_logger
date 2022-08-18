#pragma once
#include <depthai/depthai.hpp>
#include <limits>
#include <unordered_set>

#include "logger_types.hpp"

namespace oakd_logger {
using MAX_SIZE_T = std::numeric_limits<size_t>();
static constexpr size_t MAX_IMU_UPDATE_RATE_HZ = 500;

static constexpr size_t RGB_PREVIEW_ROWS = 720;
static constexpr size_t RGB_PREVIEW_COLS = 1280;

// Define buffer sizes
static const std::unordered_map<DataStream, size_t> QUEUE_BUFFER_SIZE = {
    {DataStream::IMU, 20 /* 2s at 200Hz */},
    {DataStream::LEFT_MONO, 1 /* 2s at 30Hz */},
    {DataStream::RIGHT_MONO, 1 /* 2s at 30Hz */},
    {DataStream::RGB, 1 /* 2s at 30Hz */}};

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
    return true;
  }

  // Pipeline config
  std::unordered_map<DataStream, bool, DataStreamHash> disable_stream = {
      {DataStream::IMU, false},
      {DataStream::LEFT_MONO, false},
      {DataStream::RIGHT_MONO, false},
      {DataStream::RGB, false}};

  // IMU config
  size_t imu_batch_report_threshold = 20;
  size_t imu_max_batch_reports = 20;
  size_t imu_update_rate_hz = 100;

  // Duration of logging
  float logging_duration_s = 1.0;
};
}  // namespace oakd_logger
