#pragma once
#include <chrono>
#include <unordered_map>

#include "basic_types.hpp"

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
  CameraPacket() : timestamp{MAX_FLOAT} {}

  CameraPacket(int nrows, int ncols, int type)
      : timestamp{MAX_FLOAT}, image{cv::Mat(nrows, ncols, type)} {}

  float timestamp;
  cv::Mat image;
};

}  // namespace oakd_logger
