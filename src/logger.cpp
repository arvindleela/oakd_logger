#include "logger.hpp"

#include <opencv2/opencv.hpp>

#include "magic_enum.hpp"

namespace oakd_logger {

double DataQueues::time_cast(const TimePoint& time) const {
  if (!start_ts_) {
    LOG(ERROR) << "Start timestamp is not initialized";
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

bool DataQueues::log_queue() {
  bool have_imu = false;
  for (const auto& [type, queue] : queues_) {
    switch (type) {
      case DataStream::IMU: {
        const auto& imu_data_vec = queue->getAll<dai::IMUData>();
        if (!imu_data_vec.empty()) {
          have_imu = true;
          std::cout << "Here with " << magic_enum::enum_name(type)
                    << ", packets.size: " << imu_data_vec.size() << std::endl;
        }
        break;
      }
      default: {
        const auto& image_data_vec = queue->getAll<dai::ImgFrame>();
        if (!image_data_vec.empty()) {
          const auto image_data = image_data_vec.front();
          // Set initial timestamp
          if (!start_ts_) {
            start_ts_ = image_data->getTimestamp();
          }
          std::cout << "Here with " << image_data_vec.size() << " images  "
                    << magic_enum::enum_name(type) << ", "
                    << time_cast(image_data->getTimestamp()) << std::endl;
          cv::imshow(std::string(magic_enum::enum_name(type)),
                     image_data->getCvFrame());
        }
      }
    }
  }
  return true;
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
  std::cout << "Queue size: " << device_->getOutputQueueNames().size()
            << std::endl;
  for (const auto& qn : device_->getOutputQueueNames()) {
    std::cout << qn << std::endl;
  }

  while (true) {
    data_queues_.log_queue();
    int key = cv::waitKey(1);
    if (key == 'q' || key == 'Q') {
      break;
    }
  }
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

  camera_node->setPreviewSize(300, 300);
  camera_node->setBoardSocket(CAMERA_SOCKET.at(DataStream::RGB));
  camera_node->setResolution(
      dai::ColorCameraProperties::SensorResolution::THE_720_P);
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
