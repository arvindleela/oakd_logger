#include "logger.hpp"

#include <opencv2/opencv.hpp>

#include "magic_enum.hpp"

namespace oakd_logger {

Logger::Logger(const std::string_view logdir, const Config& config)
    : config_(config) {
  google::InitGoogleLogging("OAK-D Logger");

  // Setup logging directories
  FLAGS_log_dir = logdir;

  LOG(INFO) << "Instantiated logger ...";

  // Prepare statistics
  imu_stat_ = std::make_shared<IMUStat>(MIN_IMU_DT_S, MAX_IMU_DT_S);

  for (const auto& type : ALL_SENSOR_TYPES) {
    if (!image_type(type)) {
      continue;
    }
    cam_stats_.insert(
        {type, std::make_shared<CAMStat>(MIN_CAM_DT_S, MAX_CAM_DT_S)});
  }
}

bool Logger::initialize() {
  // Pipeline
  pipeline_ = std::make_unique<dai::Pipeline>();

  const bool imu_success = configure_and_add_imu();
  const bool left_mono_success =
      configure_and_add_mono_camera(DataStream::LEFT_MONO);
  const bool right_mono_success =
      configure_and_add_mono_camera(DataStream::RIGHT_MONO);
  const bool rgb_mono_success = configure_and_add_rgb_camera();

  // Add pipeline to device
  device_ =
      std::make_unique<dai::Device>(*pipeline_, dai::UsbSpeed::SUPER_PLUS);

  // Populate IMU queue
  if (imu_success) {
    if (!add_to_queue(DataStream::IMU)) {
      LOG(ERROR) << "Error adding IMU data queue.";
    }
  }

  if (left_mono_success) {
    if (!add_to_queue(DataStream::LEFT_MONO)) {
      LOG(ERROR) << "Error adding LEFT MONO data queue.";
    }
  }

  if (right_mono_success) {
    if (!add_to_queue(DataStream::RIGHT_MONO)) {
      LOG(ERROR) << "Error adding RIGHT MONO data queue.";
    }
  }

  if (rgb_mono_success) {
    if (!add_to_queue(DataStream::RGB)) {
      LOG(ERROR) << "Error adding RGB data queue.";
    }
  }

  std::cout << "Initialized " << pipeline_->getAllNodes().size() << " nodes. "
            << std::endl;

  device_initialized_ = true;
  return true;
}

std::optional<TimePoint> Logger::log_queues() {
  std::optional<TimePoint> last_timestamp = std::nullopt;
  for (const auto& [type, queue] : queues_) {
    if (type == DataStream::IMU) {
      const auto IMUData_queue = queue->tryGetAll<dai::IMUData>();
      for (const auto& imu_data : IMUData_queue) {
        for (const auto& imu_packet : imu_data->packets) {
          last_timestamp = read_imu_packet(imu_packet);
        }
      }
    } else if (image_type(type)) {
      // All other types are images
      const auto ImgData_queue = queue->tryGetAll<dai::ImgFrame>();
      for (const auto& cam_packet : ImgData_queue) {
        last_timestamp = read_cam_packet(type, cam_packet);
      }
    }
  }
  return last_timestamp;
}

TimePoint Logger::read_imu_packet(const dai::IMUPacket& packet) {
  const auto timestamp = packet.acceleroMeter.timestamp.get();

  // Write IMU packet if needed
  if (record_ && serializer_.ok_to_log()) {
    serializer_.write_imu_packet(time_since_start(timestamp), packet);
  }

  // Update preview
  preview_.update_imu(time_since_start(timestamp));

  // Update sensor statistics
  imu_stat_->update_statistics(packet.acceleroMeter.sequence,
                               time_since_start(timestamp));

  return timestamp;
}

TimePoint Logger::read_cam_packet(const DataStream& type,
                                  const std::shared_ptr<dai::ImgFrame> packet) {
  const auto timestamp = packet->getTimestamp();

  // Write num_cam_packet if needed
  if (record_ && serializer_.ok_to_log()) {
    serializer_.write_camera_packet(type, time_since_start(timestamp), packet);
  } else {
    LOG_EVERY_N(WARNING, 10)
        << "Not logging any data since serializer is not ready to log.";
  }

  CameraPacket cam_packet(type, time_since_start(timestamp),
                          packet->getSequenceNum(), packet->getCvFrame());

  // Update preview
  preview_.update_image(cam_packet, imu_stat_, cam_stats_, record_);

  // Update sensor statistics
  cam_stats_.at(type)->update_statistics(packet->getSequenceNum(),
                                         time_since_start(timestamp));
  return timestamp;
}

void Logger::process_key(const int key) {
  if (key == 'q') {
    quit_ = true;
  }

  if (key == 'r') {
    record_ = !record_;
  }
}

void Logger::start_logging() {
  // Early exit if not initialized
  if (!device_initialized_) {
    LOG(ERROR) << "Start logging without initializing device.";
    return;
  }

  LOG(INFO) << "Queue size: " << device_->getOutputQueueNames().size();
  for (const auto& qn : device_->getOutputQueueNames()) {
    LOG(INFO) << qn;
  }

  int key = 0;
  TimePoint last_sensor_timestamp{};
  while (!quit_) {
    const auto last_sensor_timestamp = log_queues();
    if (last_sensor_timestamp) {
      if (!start_ts_) {
        start_ts_ = last_sensor_timestamp;
      }
      cv::imshow("Preview", *(preview_.preview_img()));
      process_key(cv::waitKey(10));
    }
  }

  LOG(ERROR) << "Finished logging after "
             << time_since_start(last_sensor_timestamp)
             << " s. with key == 'q': " << (key == 'q');
  LOG(ERROR) << serializer_.output_file_info();
}

void Logger::replay(std::string_view input_file) {
  if (!serializer_.prepare_input_stream(input_file)) {
    LOG(ERROR) << "Error reading from file " << input_file;
    return;
  }

  LOG(INFO) << "Read from file " << input_file << ", success. ";

  std::optional<DataStream> type = std::nullopt;

  IMUPacket imu_packet;
  CameraPacket cam_packet;

  int key = 0;
  do {
    type = serializer_.read_input_stream(imu_packet, cam_packet);

    // Update preview
    if (type) {
      if (*type == DataStream::IMU) {
        // Update sensor statistics
        imu_stat_->update_statistics(imu_packet.accelerometer.sequence_num,
                                     imu_packet.timestamp);

        preview_.update_imu(imu_packet.timestamp);
      } else if (image_type(*type)) {
        // Update sensor statistics
        cam_stats_.at(*type)->update_statistics(cam_packet.sequence_num,
                                                cam_packet.timestamp);

        preview_.update_image(cam_packet, imu_stat_, cam_stats_,
                              /* record */ false);

        cv::imshow("Preview", *preview_.preview_img());
        key = cv::waitKey(1);
      }
    }
  } while (type && key != 'q');

  LOG(INFO) << "Finished replay.";
  LOG(ERROR) << serializer_.input_file_info();
}

bool Logger::configure_and_add_imu() {
  if (config_.disable_stream.at(DataStream::IMU)) {
    LOG(ERROR) << "IMU is disabled by config.";
    return false;
  }

  if (!pipeline_) {
    LOG(ERROR) << "Pipeline is invalid.";
    return false;
  }

  // Instantiate nodes
  std::shared_ptr<dai::node::IMU> imu_node =
      pipeline_->create<dai::node::IMU>();
  if (!imu_node) {
    LOG(ERROR) << "Failed to instantiate IMU node.";
    return false;
  }

  auto link_imu_out = pipeline_->create<dai::node::XLinkOut>();
  link_imu_out->setStreamName(
      std::string(magic_enum::enum_name(DataStream::IMU)));

  imu_node->out.link(link_imu_out->input);

  // Generate synced IMU measurements at SYNCED_IMU_FREQUENCY_HZ
  imu_node->enableIMUSensor(
      {dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW},
      IMU_UPDATE_RATE_HZ);

  imu_node->setBatchReportThreshold(config_.imu_batch_report_threshold);
  imu_node->setMaxBatchReports(config_.imu_max_batch_reports);

  LOG(INFO) << "Successfully instantiated IMU node with update_rate_hz: "
            << IMU_UPDATE_RATE_HZ << ", batch report threshold: "
            << imu_node->getBatchReportThreshold()
            << ", max_batch_reports: " << imu_node->getMaxBatchReports();

  return true;
}

bool Logger::configure_and_add_mono_camera(const DataStream& type) {
  if (!mono_image_type(type)) {
    LOG(ERROR) << "Type " << magic_enum::enum_name(type)
               << " is not a mono camera.";
    return false;
  }

  if (config_.disable_stream.at(type)) {
    LOG(ERROR) << "Mono camera of type " << magic_enum::enum_name(type)
               << " is disable by config.";
    return false;
  }

  if (!pipeline_) {
    LOG(ERROR) << "Pipeline is invalid.";
    return false;
  }

  // Instantiate nodes
  auto camera_node = pipeline_->create<dai::node::MonoCamera>();

  if (!camera_node) {
    LOG(ERROR) << "Failed to instantiate Mono camera node of type "
               << magic_enum::enum_name(type);
    return false;
  }
  camera_node->setBoardSocket(CAMERA_SOCKET.at(type));
  camera_node->setResolution(
      dai::MonoCameraProperties::SensorResolution::THE_720_P);
  camera_node->setFps(CAM_FPS);

  auto link_camera_out = pipeline_->create<dai::node::XLinkOut>();
  link_camera_out->setStreamName(std::string(magic_enum::enum_name(type)));

  // Link camera node
  camera_node->out.link(link_camera_out->input);

  return true;
}

bool Logger::configure_and_add_rgb_camera() {
  if (config_.disable_stream.at(DataStream::RGB)) {
    LOG(ERROR) << "RGB camera is disable by config.";
    return false;
  }

  if (!pipeline_) {
    LOG(ERROR) << "Pipeline is invalid.";
    return false;
  }

  // Instantiate nodes
  auto camera_node = pipeline_->create<dai::node::ColorCamera>();

  if (!camera_node) {
    LOG(ERROR) << "Failed to instantiate RGB camera node.";
    return false;
  }

  camera_node->setBoardSocket(CAMERA_SOCKET.at(DataStream::RGB));
  camera_node->setResolution(
      dai::ColorCameraProperties::SensorResolution::THE_1080_P);
  camera_node->setFps(CAM_FPS);
  camera_node->setPreviewSize(RGB_PREVIEW_COLS, RGB_PREVIEW_ROWS);
  camera_node->setInterleaved(false);
  camera_node->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);

  auto link_camera_out = pipeline_->create<dai::node::XLinkOut>();
  link_camera_out->setStreamName(
      std::string(magic_enum::enum_name(DataStream::RGB)));

  // Link camera node
  camera_node->preview.link(link_camera_out->input);

  return true;
}

bool Logger::add_to_queue(const DataStream& type) {
  queues_[type] = device_.get()->getOutputQueue(
      std::string(magic_enum::enum_name(type)), QUEUE_BUFFER_SIZE.at(type),
      /* blocking */ false);
  LOG(INFO) << "Adding queue of type : " << magic_enum::enum_name(type)
            << ", valid: " << (queues_[type] != nullptr)
            << ", of size: " << QUEUE_BUFFER_SIZE.at(type);
  return true;
}

double Logger::time_since_start(const TimePoint& time) const {
  static constexpr bool VERBOSE = false;
  if (!start_ts_) {
    LOG_IF(INFO, VERBOSE) << "Start timestamp is not initialized";
    return -1.0;
  }
  using namespace std::chrono;
  return static_cast<double>(
             duration_cast<nanoseconds>(time - *start_ts_).count()) *
         1e-9;
}

}  // namespace oakd_logger
