#pragma once
#include <chrono>
#include <unordered_map>

#include "Eigen/Dense"
#include "basic_types.hpp"
#include "magic_enum.hpp"

namespace oakd_logger {

using TimePoint = std::chrono::time_point<
    std::chrono::steady_clock,
    std::chrono::
    nanoseconds>;  //           std::chrono::steady_clock::duration>;

// Define various streams
enum class DataStream : size_t {
  IMU,
  LEFT_MONO,
  RIGHT_MONO,
  RGB,
  INVALID,
};

/*
 * @brief:  Nominal image dimenssions
 */
static constexpr size_t CAM_ROWS = 720;
static constexpr size_t CAM_COLS = 1280;
using EigenImage = Eigen::Matrix<uint8_t, CAM_ROWS, CAM_COLS, Eigen::RowMajor>;

/**
 * @brief:  Return true if type is an image
 * @param[in] type: DataStream type
 * @return: True, if type is a image
 */
inline bool image_type(const DataStream &type) {
    return (type == DataStream::LEFT_MONO || type == DataStream::RIGHT_MONO ||
        type == DataStream::RGB);
}

/**
 * @brief:  Return true if type is a mono image
 * @param[in] type: DataStream type
 * @return: True, if type is a mono image
 */
inline bool mono_image_type(const DataStream &type) {
    return (type == DataStream::LEFT_MONO || type == DataStream::RIGHT_MONO);
}

// Define all valid sensors
constexpr std::array<DataStream, 4> ALL_SENSOR_TYPES = {
    DataStream::IMU, DataStream::LEFT_MONO, DataStream::RIGHT_MONO,
    DataStream::RGB};

struct DataStreamHash {
  template<typename T>
  uint8_t operator()(T t) const {
      return static_cast<uint8_t>(t);
  }
};

// Define CameraBoardSockets for each type
static const std::unordered_map<DataStream, dai::CameraBoardSocket>
    CAMERA_SOCKET = {{DataStream::LEFT_MONO, dai::CameraBoardSocket::LEFT},
                     {DataStream::RIGHT_MONO, dai::CameraBoardSocket::RIGHT},
                     {DataStream::RGB, dai::CameraBoardSocket::RGB}};

/**
 * This needs to be moved to utilities basic types
 */

constexpr float MAX_FLOAT = std::numeric_limits<float>::max();
constexpr int MAX_INT = std::numeric_limits<int>::max();

class IMUMeas {
 public:
  IMUMeas() : sequence_num_{MAX_INT}, data_{MAX_FLOAT, MAX_FLOAT, MAX_FLOAT} {}

  template<typename T>
  IMUMeas(const T &report)
      : sequence_num_(report.sequence), data_{report.x, report.y, report.z} {}

  const std::array<float, 3> &data() { return data_; }
  int sequence_num() const { return sequence_num_; }

 private:
  const int sequence_num_;
  const std::array<float, 3> data_;
};

struct IMUPacket {
  float timestamp;
  IMUMeas accelerometer;
  IMUMeas gyroscope;

  IMUPacket() : timestamp{MAX_FLOAT}, accelerometer{}, gyroscope{} {}

  IMUPacket(const float _timestamp, const dai::IMUPacket &packet)
      : timestamp(_timestamp),
        accelerometer{packet.acceleroMeter},
        gyroscope{packet.gyroscope} {}
};

struct CameraPacket {
 public:
  CameraPacket()
      : type_(DataStream::INVALID), timestamp_{MAX_FLOAT}, sequence_num_{MAX_INT} {
  }

  CameraPacket(DataStream packet_type, float timestamp, int sequence_num,
               const cv::Mat &image)
      : type_{packet_type}, timestamp_{timestamp},
        sequence_num_{sequence_num}, image_{image} {}

  const DataStream &type() const { return type_; }
  float timestamp() const { return timestamp_; }
  int sequence_num() const { return sequence_num_; }
  const cv::Mat &image() const { return image_; }

 private:
  DataStream type_;
  float timestamp_;
  int sequence_num_;
  cv::Mat image_;
};

}  // namespace oakd_logger
