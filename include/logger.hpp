#pragma once
#include <glog/logging.h>

#include <chrono>
#include <depthai/depthai.hpp>
#include <optional>

#include "config.hpp"

namespace oakd_logger {

using TimePoint = std::chrono::time_point<
    std::chrono::steady_clock,
    std::chrono::
        nanoseconds>;  //
                       //           std::chrono::steady_clock::duration>;
using QueueTypePtr = std::shared_ptr<dai::DataOutputQueue>;

class DataQueues {
 public:
  /**
   * @brief:    Add sensor stream queue
   * @param[in] device: Device
   * @param[in] type:   Stream type
   * @return:   True, if success
   */
  bool add(dai::Device* device, const DataStream& type);

  bool log_queue();

 private:
  double time_cast(const TimePoint& time) const;

  std::unordered_map<DataStream, QueueTypePtr, DataStreamHash> queues_;
  std::optional<TimePoint> start_ts_ = std::nullopt;
};

class Logger {
 public:
  Logger(const std::string_view logdir, const Config& config);

  bool initialize();

  /**
   * @brief:    Start logging
   */
  void start_logging();

 private:
  /**
   * @brief:    Configure IMU and add to pipeline
   * @return:   True, if success
   */
  bool configure_and_add_imu();

  /**
   * @brief:    Configure and add the stereo pair
   * @return:   True, if success
   */
  bool configure_and_add_mono_camera(const DataStream& type);

  /**
   * @brief:    Configure and add RGB camera
   * @return:   True, if success
   */
  bool configure_and_add_rgb_camera();

  // Logger configuration
  Config config_;

  // Data queues
  DataQueues data_queues_;

  // Sensor queue
  std::unique_ptr<dai::Pipeline> pipeline_ = nullptr;
  std::unique_ptr<dai::Device> device_ = nullptr;
  std::shared_ptr<dai::DataOutputQueue> left_cam_queue_ = nullptr;

  std::shared_ptr<dai::node::MonoCamera> left_cam_node_ = nullptr;
  std::shared_ptr<dai::node::MonoCamera> right_cam_node_ = nullptr;
  std::shared_ptr<dai::node::ColorCamera> rgb_cam_node_ = nullptr;

  // Initialized
  bool initialized_ = false;
};
}  // namespace oakd_logger
