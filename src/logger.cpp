#include "logger.hpp"

#include <opencv2/opencv.hpp>

#include "magic_enum.hpp"

namespace oakd_logger {

double DataQueues::time_cast(const TimePoint& time) const {
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

bool DataQueues::add(dai::Device* device, const DataStream& type) {
  if (!device) {
    LOG(ERROR) << "Invalid device while adding data queue.";
    return false;
  }
  queues_[type] = device->getOutputQueue(
      std::string(magic_enum::enum_name(type)), QUEUE_BUFFER_SIZE.at(type),
      /* blocking */ false);
  LOG(INFO) << "Adding queue of type : " << magic_enum::enum_name(type)
            << ", valid: " << (queues_[type] != nullptr)
            << ", of size: " << QUEUE_BUFFER_SIZE.at(type);
  return true;
}

bool DataQueues::log_queue(OAKDSerializer& serializer) {
  IMUQueue imu_packet_queue;
  IMGVector collated_img_vector;
  for (const auto& [type, queue] : queues_) {
    switch (type) {
      case DataStream::IMU: {
        const auto IMUData_queue = queue->tryGetAll<dai::IMUData>();
        for (const auto& imu_data : IMUData_queue) {
          for (const auto& imu_packet : imu_data->packets) {
            imu_packet_queue.push(imu_packet);
          }
        }
        break;
      }
      default: {
        // All other types are images
        const auto ImgData_queue = queue->tryGetAll<dai::ImgFrame>();
        for (const auto& img : ImgData_queue) {
          collated_img_vector.push_back({.type = type, .img_frame = img});
        }
      }
    }
  }

  // Sort collated_image_vector
  std::sort(collated_img_vector.begin(), collated_img_vector.end(),
            [](const auto& a, const auto& b) {
              return a.img_frame->getTimestamp() < b.img_frame->getTimestamp();
            });

  IMGQueue img_queue;
  for (const auto& img : collated_img_vector) {
    img_queue.push(img);
  }

  // Go over imu_packet_queue and img_queue
  dai::IMUPacket next_imu_packet;
  std::shared_ptr<dai::ImgFrame> next_img_frame_ptr = nullptr;

  auto next_type = get_next(imu_packet_queue, img_queue, next_imu_packet,
                            next_img_frame_ptr);
  TimePoint next_timestamp = TimePoint::max();
  while (next_type != std::nullopt) {
    int sequence_num = -1;
    if (*next_type == DataStream::IMU) {
      next_timestamp = next_imu_packet.acceleroMeter.timestamp.get();

      if (!start_ts_) {
        start_ts_ = next_timestamp;
      }

      sequence_num = next_imu_packet.acceleroMeter.sequence;

      // Write next_imu_packet if possible
      serializer.write_imu_packet(*next_type, time_cast(next_timestamp),
                                  next_imu_packet);
    } else {
      next_timestamp = next_img_frame_ptr->getTimestamp();

      if (!start_ts_) {
        start_ts_ = next_timestamp;
      }

      sequence_num = next_img_frame_ptr->getSequenceNum();

      // Write num_cam_packet if possible
      serializer.write_camera_packet(*next_type, time_cast(next_timestamp),
                                     next_img_frame_ptr);
    }

    LOG(INFO) << "Writing " << magic_enum::enum_name(*next_type)
              << " with ts: " << time_cast(next_timestamp)
              << ", sequence num: " << sequence_num << std::endl;

    *last_timestamp_ = next_timestamp;

    // Get the next chronologically ordered sensor
    next_type = get_next(imu_packet_queue, img_queue, next_imu_packet,
                         next_img_frame_ptr);
  }

  return true;
}

double DataQueues::log_duration_s() const {
  return time_cast(*last_timestamp_);
}

std::optional<DataStream> DataQueues::get_next(
    IMUQueue& imu_queue, IMGQueue& img_queue, dai::IMUPacket& next_imu_packet,
    std::shared_ptr<dai::ImgFrame>& next_img_frame_ptr) const {
  // If there is no data, then early exit
  if (imu_queue.empty() && img_queue.empty()) {
    return std::nullopt;
  }

  // Note that Gyroscope and Accelerometer are synchronized
  const TimePoint imu_time =
      imu_queue.empty() ? TimePoint::max()
                        : imu_queue.front().acceleroMeter.timestamp.get();
  const TimePoint img_time = img_queue.empty()
                                 ? TimePoint::max()
                                 : img_queue.front().img_frame->getTimestamp();
  const auto img_type =
      img_queue.empty() ? DataStream::INVALID : img_queue.front().type;

  auto ret_type = DataStream::INVALID;

  if (imu_time < img_time) {
    ret_type = DataStream::IMU;
    next_imu_packet = imu_queue.front();
    imu_queue.pop();
  } else {
    ret_type = img_type;
    next_img_frame_ptr = img_queue.front().img_frame;
    img_queue.pop();
  };
  return ret_type;
}

Logger::Logger(const std::string_view logdir, const Config& config)
    : config_(config) {
  google::InitGoogleLogging("OAK-D Logger");

  // Setup logging directories
  FLAGS_log_dir = logdir;

  LOG(INFO) << "Instantiated logger ...";

  const bool config_sanity_check_passed = config.sanity_check();
  if (!config_sanity_check_passed) {
    LOG(ERROR) << "Config sanity checks failed.";
    return;
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
  device_ = std::make_unique<dai::Device>(*pipeline_, dai::UsbSpeed::SUPER);

  // Populate IMU queue
  if (imu_success) {
    if (!data_queues_.add(device_.get(), DataStream::IMU)) {
      LOG(ERROR) << "Error adding IMU data queue.";
    }
  }

  if (left_mono_success) {
    if (!data_queues_.add(device_.get(), DataStream::LEFT_MONO)) {
      LOG(ERROR) << "Error adding LEFT MONO data queue.";
    }
  }

  if (right_mono_success) {
    if (!data_queues_.add(device_.get(), DataStream::RIGHT_MONO)) {
      LOG(ERROR) << "Error adding RIGHT MONO data queue.";
    }
  }

  if (rgb_mono_success) {
    if (!data_queues_.add(device_.get(), DataStream::RGB)) {
      LOG(ERROR) << "Error adding RGB data queue.";
    }
  }

  std::cout << "Initialized " << pipeline_->getAllNodes().size() << " nodes. "
            << std::endl;

  initialized_ = true;
  return true;
}

void Logger::start_logging() {
  // Early exit if not initialized
  if (!initialized_) {
    LOG(ERROR) << "Start logging without initializing logger";
    return;
  }

  LOG(INFO) << "Queue size: " << device_->getOutputQueueNames().size();
  for (const auto& qn : device_->getOutputQueueNames()) {
    LOG(INFO) << qn;
  }
  LOG(INFO) << "Logging duration " << config_.logging_duration_s << "s.";

  while (true) {
    data_queues_.log_queue(serializer_);

    if (data_queues_.log_duration_s() > config_.logging_duration_s) {
      break;
    }
  }

  LOG(INFO) << "Finished logging after " << data_queues_.log_duration_s()
            << " s. ";
  LOG(ERROR) << serializer_.info();
}

void Logger::replay(std::string_view input_file) {
  bool input_stream_success = serializer_.prepare_input_stream(input_file);

  LOG(INFO) << "Read from file " << input_file
            << ", success: " << input_stream_success << std::endl;

  bool done_parsing = input_stream_success;
  while (done_parsing) {
    done_parsing = serializer_.read_input_file();
  }

  LOG(INFO) << "Finished replay.";
  LOG(ERROR) << serializer_.info();
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
      config_.imu_update_rate_hz);

  imu_node->setBatchReportThreshold(config_.imu_batch_report_threshold);
  imu_node->setMaxBatchReports(config_.imu_max_batch_reports);

  LOG(INFO) << "Successfully instantiated IMU node with update_rate_hz: "
            << config_.imu_update_rate_hz << ", batch report threshold: "
            << imu_node->getBatchReportThreshold()
            << ", max_batch_reports: " << imu_node->getMaxBatchReports();

  return true;
}

bool Logger::configure_and_add_mono_camera(const DataStream& type) {
  if (MONO_CAMERAS.find(type) == MONO_CAMERAS.end()) {
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

}  // namespace oakd_logger
