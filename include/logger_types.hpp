#pragma once
#include <chrono>
#include <unordered_map>

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

/**
 * @brief:  Return true if type is an image
 */
inline bool image_type(const DataStream &type) {
  return (type == DataStream::LEFT_MONO || type == DataStream::RIGHT_MONO ||
          type == DataStream::RGB);
}

/**
 * @brief:  Return true if type is a mono image
 */
inline bool mono_image_type(const DataStream &type) {
  return (type == DataStream::LEFT_MONO || type == DataStream::RIGHT_MONO);
}

// Define all valid sensors
constexpr std::array<DataStream, 4> ALL_SENSOR_TYPES = {
    DataStream::IMU, DataStream::LEFT_MONO, DataStream::RIGHT_MONO,
    DataStream::RGB};

struct DataStreamHash {
  template <typename T>
  uint8_t operator()(T t) const {
    return static_cast<uint8_t>(t);
  }
};

static const std::unordered_set<DataStream> MONO_CAMERAS(
    {DataStream::LEFT_MONO, DataStream::RIGHT_MONO});
static const std::unordered_map<DataStream, dai::CameraBoardSocket>
    CAMERA_SOCKET = {{DataStream::LEFT_MONO, dai::CameraBoardSocket::LEFT},
                     {DataStream::RIGHT_MONO, dai::CameraBoardSocket::RIGHT},
                     {DataStream::RGB, dai::CameraBoardSocket::RGB}};

/**
 * This needs to be moved to utilities basic types
 */

constexpr float MAX_FLOAT = std::numeric_limits<float>::max();
constexpr int MAX_INT = std::numeric_limits<int>::max();

struct IMUMeas
{
  IMUMeas() : sequence_num{MAX_INT}, x{MAX_FLOAT}, y{MAX_FLOAT}, z{MAX_FLOAT} {}

  template <typename T>
  IMUMeas(const T& report)
      : sequence_num(report.sequence), x(report.x), y(report.y), z(report.z) {}

  int sequence_num = MAX_INT;
  float x = MAX_FLOAT;
  float y = MAX_FLOAT;
  float z = MAX_FLOAT;
};

struct IMUPacket
{
  float timestamp;
  IMUMeas accelerometer;
  IMUMeas gyroscope;

  IMUPacket() : timestamp{MAX_FLOAT}, accelerometer{}, gyroscope{} {}

  IMUPacket(const float _timestamp, const dai::IMUPacket& packet)
      : timestamp(_timestamp),
        accelerometer{packet.acceleroMeter},
        gyroscope{packet.gyroscope} {}
};

struct CameraPacket {
  CameraPacket()
      : type(DataStream::INVALID), timestamp{MAX_FLOAT}, sequence_num(MAX_INT) {
  }

  CameraPacket(DataStream packet_type, float timestamp, int sequence_num,
               const cv::Mat &image)
      : type{packet_type}, timestamp{timestamp},
        sequence_num{sequence_num}, image{image} {}

  DataStream type;
  float timestamp;
  int sequence_num;
  cv::Mat image;
};

}  // namespace oakd_logger
