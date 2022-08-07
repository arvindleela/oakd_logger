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
using IMUQueue = std::queue<dai::IMUPacket>;

struct StereoImg {
  DataStream type;
  std::shared_ptr<dai::ImgFrame> img_frame;
};

using IMGQueue = std::queue<StereoImg>;
using IMGVector = std::vector<StereoImg>;

class DataQueues {
 public:
  /**
   * @brief:    Add sensor stream queue
   * @param[in] device: Device
   * @param[in] type:   Stream type
   * @return:   True, if success
   */
  bool add(dai::Device* device, const DataStream& type);

  bool log_queue(OAKDSerializer& serializer);

  double log_duration_s() const;

 private:
  std::optional<DataStream> get_next(
      IMUQueue& imu_queue, IMGQueue& img_queue, dai::IMUPacket& next_imu_packet,
      std::shared_ptr<dai::ImgFrame>& next_img_frame_ptr) const;

  double time_cast(const TimePoint& time) const;

  std::unordered_map<DataStream, QueueTypePtr, DataStreamHash> queues_;

  std::optional<TimePoint> start_ts_ = std::nullopt;
  std::optional<TimePoint> last_timestamp_ = std::nullopt;
};

class Logger {
 public:
  Logger(const std::string_view logdir, const Config& config);

  bool initialize();

  bool prepare_output_log(std::string_view output_path) {
    return serializer_.prepare_output_log(output_path);
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

  OAKDSerializer serializer_;
  // OAKDPreviewer preview_;

  // Logger configuration
  Config config_;

  // Data queues
  DataQueues data_queues_;

  // Sensor queue
  std::unique_ptr<dai::Pipeline> pipeline_ = nullptr;
  std::unique_ptr<dai::Device> device_ = nullptr;

  // Initialized
  bool initialized_ = false;
};
}  // namespace oakd_logger
