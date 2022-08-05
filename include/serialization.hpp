#pragma once
#include <fstream>
#include <sstream>

#include "config.hpp"
#include "magic_enum.hpp"

namespace oakd_logger {
class OAKDSerializer {
 public:
  OAKDSerializer() : num_bytes_written_(0) {
    num_packets_[DataStream::IMU] = 0;
    num_packets_[DataStream::LEFT_MONO] = 0;
    num_packets_[DataStream::RIGHT_MONO] = 0;
    num_packets_[DataStream::RGB] = 0;
  }
  bool prepare_output_log(std::string_view output_path) {
    out_file_path_ = std::string(output_path);
    out_file_.open(out_file_path_, std::ios::out | std::ios::binary);
    return out_file_.is_open();
  }

  std::string info() {
    std::stringstream msg;
    msg << "Output file " << out_file_path_
        << ", is_open: " << out_file_.is_open() << ", wrote "
        << num_bytes_written_ * 1e-9 << " Gb with " << std::endl;
    for (const auto& [type, num_packets] : num_packets_) {
      msg << "Type: " << magic_enum::enum_name(type)
          << ", num packets: " << num_packets << std::endl;
    }

    return msg.str();
  }

  void write_imu_packet(const DataStream& type, const double timestamp,
                        const dai::IMUPacket& packet) {
    if (!out_file_.is_open()) {
      return;
    }
    // Write type
    write_data(reinterpret_cast<const char*>(&type), sizeof type);

    // Write timestamp
    write_data(reinterpret_cast<const char*>(&timestamp), sizeof timestamp);

    // Write accelerometer
    auto& accelerometer = packet.acceleroMeter;
    write_data(reinterpret_cast<const char*>(&accelerometer.sequence),
               sizeof accelerometer.sequence);
    write_data(reinterpret_cast<const char*>(&accelerometer.x),
               sizeof accelerometer.x);
    write_data(reinterpret_cast<const char*>(&accelerometer.y),
               sizeof accelerometer.y);
    write_data(reinterpret_cast<const char*>(&accelerometer.z),
               sizeof accelerometer.z);

    // Write Gyroscope
    auto& gyroscope = packet.gyroscope;
    write_data(reinterpret_cast<const char*>(&gyroscope.sequence),
               sizeof gyroscope.sequence);
    write_data(reinterpret_cast<const char*>(&gyroscope.x), sizeof gyroscope.x);
    write_data(reinterpret_cast<const char*>(&gyroscope.y), sizeof gyroscope.y);
    write_data(reinterpret_cast<const char*>(&gyroscope.z), sizeof gyroscope.z);

    num_packets_.at(type) += 1;
  }

  void write_camera_packet(const DataStream& type, const double timestamp,
                           const std::shared_ptr<dai::ImgFrame> packet) {
    if (!out_file_.is_open()) {
      return;
    }
    // Write type
    write_data(reinterpret_cast<const char*>(&type), sizeof type);

    // Write timestamp
    write_data(reinterpret_cast<const char*>(&timestamp), sizeof timestamp);

    // Write ImgFrame
    const auto& cv_mat = packet->getFrame(/*copy*/ false);

    write_data(reinterpret_cast<const char*>(cv_mat.ptr()),
               cv_mat.elemSize() * cv_mat.rows * cv_mat.cols);
    num_packets_.at(type) += 1;
  }

 private:
  void write_data(const char* data, const size_t num_bytes) {
    if (!out_file_.is_open()) {
      return;
    }
    out_file_.write(data, num_bytes);
    num_bytes_written_ += num_bytes;
  }

 private:
  size_t num_bytes_written_ = 0;
  std::unordered_map<DataStream, size_t, DataStreamHash> num_packets_;

  std::string out_file_path_;
  std::ofstream out_file_;
};
}  // namespace oakd_logger
