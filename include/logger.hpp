#pragma once
#include <glog/logging.h>

#include <chrono>
#include <depthai/depthai.hpp>
#include <optional>

#include "circular_buffer.hpp"
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

struct SensorPacketStatistics {
  static constexpr size_t WINDOW_LENGTH = 100;
  static constexpr size_t MAX_SEQUENCE_NUM = 255;

  SensorPacketStatistics(const double timestamp, const size_t sequence_num);

  void update(const double timestamp, const size_t sequence_num);

  void statistics(double& max_dt, size_t& max_ds);

  void info(std::string_view type);

  size_t size() const { return delta_timestamps.size(); }

  Utilities::Containers::CircularBuffer<double, WINDOW_LENGTH> delta_timestamps;
  Utilities::Containers::CircularBuffer<size_t, WINDOW_LENGTH>
      delta_sequence_num;
  double last_timestamp;
  size_t last_sequence_num;
};

class DataQueues {
 public:
  /**
   * @brief:    Add sensor stream queue
   * @param[in] device: Device
   * @param[in] type:   Stream type
   * @return:   True, if success
   */
  bool add(dai::Device* device, const DataStream& type);

  bool log_queue(OAKDSerializer &serializer, OAKDPreviewer *preview, int &key);

  double log_duration_s() const;

  double time_cast(const TimePoint &time) const;

private:
  std::optional<DataStream> get_next(
      IMUQueue& imu_queue, IMGQueue& img_queue, dai::IMUPacket& next_imu_packet,
      std::shared_ptr<dai::ImgFrame>& next_img_frame_ptr) const;

  std::unordered_map<DataStream, QueueTypePtr, DataStreamHash> queues_;

  std::optional<TimePoint> start_ts_ = std::nullopt;
  std::optional<TimePoint> last_timestamp_ = std::nullopt;
};

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
   * @brief:    Update packet satistics for a given sensor type
   */
  void update_packet_statistics(const DataStream& type, const double timestamp,
                                const size_t sequence_num);

  OAKDSerializer serializer_;
  OAKDPreviewer preview_;

  // Logger configuration
  Config config_;

  // Data queues
  DataQueues data_queues_;

  // Packet loss statistics
  std::unordered_map<DataStream, SensorPacketStatistics> statistics_;

  // Sensor queue
  std::unique_ptr<dai::Pipeline> pipeline_ = nullptr;
  std::unique_ptr<dai::Device> device_ = nullptr;

  // Initialized
  bool initialized_ = false;
};
}  // namespace oakd_logger
