#pragma once
#include <glog/logging.h>

#include <chrono>
#include <depthai/depthai.hpp>
#include <optional>

#include "config.hpp"
#include "logger_types.hpp"
#include "preview.hpp"
#include "serialization.hpp"

namespace oakd_logger {

using QueueTypePtr = std::shared_ptr<dai::DataOutputQueue>;

class Logger {
 public:
  Logger(const std::string_view logdir, const Config& config);

  bool initialize();

  /**
   * @brief:    Prepare logging at a given output path
   * @param[in] output_path:    Output path binary file
   * @return:   True, if file is successfully opened
   */
  bool prepare_output_stream(std::string_view output_path) {
    return serializer_.prepare_output_stream(output_path);
  };

  /**
   * @brief:    Start logging
   */
  void start_logging();

  /**
   * @brief:    Replay from file
   */
  void replay(std::string_view input_file);

 private:
  /**
   * @brief:    Add sensor stream queue
   * @param[in] type:   Stream type
   * @return:   True, if success
   */
  bool add_to_queue(const DataStream& type);

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

  /**
   * @brief:    Log data that is accumulated in all queues
   * @return:   Last sensor timestamp
   */
  std::optional<TimePoint> log_queues();

  /**
   * @brief:    Read cam packet
   * @param[in] type:   Type of image, i.e MONO or RGB
   * @param[in] packet: Pointer to unerlying ImgFrame
   * @return:   Last cam timestamp
   */
  TimePoint read_cam_packet(const DataStream& type,
                            const std::shared_ptr<dai::ImgFrame> packet);

  /**
   * @brief:    Read IMU packet
   * @param[in] packet: IMU packet to be parsed
   * @return    Last IMU timestamp
   */
  TimePoint read_imu_packet(const dai::IMUPacket& packet);

  /**
   * @brief:    Time since start
   * @return:   Time since start
   */
  double time_since_start(const TimePoint& time) const;

 private:
  // Starting timestamp
  std::optional<TimePoint> start_ts_ = std::nullopt;

  // Device queues
  std::unordered_map<DataStream, QueueTypePtr, DataStreamHash> queues_;

  OAKDSerializer serializer_;

  OAKDPreviewer preview_;

  // Logger configuration
  Config config_;

  // Sensor queue
  std::unique_ptr<dai::Pipeline> pipeline_ = nullptr;
  std::unique_ptr<dai::Device> device_ = nullptr;

  // Initialized
  bool device_initialized_ = false;
};
}  // namespace oakd_logger
