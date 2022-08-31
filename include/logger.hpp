#pragma once
#include <glog/logging.h>

#include <chrono>
#include <depthai/depthai.hpp>
#include <optional>

#include "config.hpp"
#include "logger_types.hpp"
#include "packet_statistics.hpp"
#include "preview.hpp"
#include "serialization.hpp"
#include "Eigen/Dense"

namespace oakd_logger {

using QueueTypePtr = std::shared_ptr<dai::DataOutputQueue>;

class Logger {
 public:
  /**
   * @brief:    C'tor for the logger
   * @param[in] logdir:     Folder where logs are stored
   * @param[in] config:     Config parameters
   */
  Logger(const std::string_view logdir, const Config &config);

  /**
   * @brief:    Initialize logger
   * @return:   True, if success
   */
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
   * @brief[in] input_file: Raw binary input
   */
  void replay(std::string_view input_file);

  /**
   * @brief:    Sequentially read from file
   * @brief[in] input_file: Raw binary input
   * @brief[in] imu_packet: A placeholder for IMU data
   * @brief[in] cam_packet: A placeholder for image data
   * @return:   If success return packet type, else std::nullopt
   */
  std::optional<DataStream> sequential_read(std::string_view input_file,
                                            IMUPacket &imu_packet,
                                            CameraPacket &cam_packet);

 private:
  /**
   * @brief:    Add sensor stream queue
   * @param[in] type:   Stream type
   */
  void add_to_queue(const DataStream &type);

  /**
   * @brief:    Configure IMU and add to pipeline
   * @return:   True, if success
   */
  bool configure_and_add_imu();

  /**
   * @brief:    Configure and add the stereo pair
   * @return:   True, if success
   */
  bool configure_and_add_mono_camera(const DataStream &type);

  /**
   * @brief:    Configure and add RGB camera
   * @return:   True, if success
   */
  bool configure_and_add_rgb_camera();

  /**
   * @brief:    Log data that is accumulated in all queues
   * @return:   Last sensor timestamp or std::nullopt is not queue is ready
   */
  std::optional<TimePoint> log_queues();

  /**
   * @brief:    Read cam packet
   * @param[in] type:   Type of image, i.e MONO or RGB
   * @param[in] packet: Pointer to underlying ImgFrame
   * @return:   Last cam timestamp
   */
  TimePoint read_cam_packet(const DataStream &type,
                            const std::shared_ptr<dai::ImgFrame> packet);

  /**
   * @brief:    Read IMU packet
   * @param[in] packet: IMU packet to be parsed
   * @return    Last IMU timestamp
   */
  TimePoint read_imu_packet(const dai::IMUPacket &packet);

  /**
   * @brief:    Time since start
   * @param[in] time: Current TimePoint
   * @return:   Time since start
   */
  float time_since_start(const TimePoint &time) const;

  /**
   * @brief:    Proces keyboard input
   * @param[in] key: Return from cv::waitKey
   */
  void process_key(const int key);

 private:
  // Starting timestamp
  std::optional<TimePoint> start_ts_ = std::nullopt;

  // Device queues
  std::unordered_map<DataStream, QueueTypePtr, DataStreamHash> queues_;
  std::shared_ptr<IMUStat> imu_stat_;
  std::unordered_map<DataStream, std::shared_ptr<CAMStat>> cam_stats_;

  OAKDSerializer serializer_;

  // Some actions
  bool record_ = false;
  bool quit_ = false;

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
