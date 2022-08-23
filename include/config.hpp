#pragma once
#include <depthai/depthai.hpp>
#include <limits>
#include <unordered_set>

#include "logger_types.hpp"

namespace oakd_logger {
using MAX_SIZE_T = std::numeric_limits<size_t>();

static constexpr size_t IMU_UPDATE_RATE_HZ = 100;
static constexpr size_t CAM_FPS = 30;

static constexpr size_t STATISTICS_RECORD_TIME_S = 10;
static constexpr size_t IMU_STAT_BUFFER_SIZE =
    STATISTICS_RECORD_TIME_S * IMU_UPDATE_RATE_HZ;
static constexpr size_t CAM_STAT_BUFFER_SIZE =
    STATISTICS_RECORD_TIME_S * CAM_FPS;

static constexpr size_t RGB_PREVIEW_ROWS = 720;
static constexpr size_t RGB_PREVIEW_COLS = 1280;

// Some thresholds for packet statistics viz
static constexpr double MIN_IMU_DT_S = 0.01;
static constexpr double MAX_IMU_DT_S = 0.03;  // 3 samples
static constexpr double MIN_CAM_DT_S = 0.0;
static constexpr double MAX_CAM_DT_S = 0.1;  // 3 samples

// Define buffer sizes
static const std::unordered_map<DataStream, size_t> QUEUE_BUFFER_SIZE = {
    {DataStream::IMU, 100 /* 1s at 100Hz */},
    {DataStream::LEFT_MONO, 6 /* 2s at 30Hz */},
    {DataStream::RIGHT_MONO, 6 /* 2s at 30Hz */},
    {DataStream::RGB, 20 /* 2s at 30Hz */}};

// Config parameters with a reasonable initialization
struct Config {
  // Pipeline config
  std::unordered_map<DataStream, bool, DataStreamHash> disable_stream = {
      {DataStream::IMU, false},
      {DataStream::LEFT_MONO, false},
      {DataStream::RIGHT_MONO, false},
      {DataStream::RGB, true}};

  // IMU config
  size_t imu_batch_report_threshold = 20;
  size_t imu_max_batch_reports = 20;
};
}  // namespace oakd_logger
