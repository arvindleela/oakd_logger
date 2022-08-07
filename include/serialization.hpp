#pragma once
#include <array>
#include <fstream>
#include <functional>
#include <sstream>

#include "config.hpp"
#include "logger_types.hpp"
#include "magic_enum.hpp"

namespace oakd_logger {
class OAKDSerializer {
 public:
  OAKDSerializer() : num_bytes_written_(0), num_bytes_read_(0) {
    std::vector<DataStream> all_types;
    all_types.emplace_back(DataStream::IMU);
    all_types.emplace_back(DataStream::LEFT_MONO);
    all_types.emplace_back(DataStream::RIGHT_MONO);
    all_types.emplace_back(DataStream::RGB);

    for (const auto& type : all_types) {
      num_packets_written_.insert({type, 0});
      num_packets_read_.insert({type, 0});
      const size_t hash = to_hash(type);
      DataStream_to_log_type_.insert({type, hash});
      log_type_to_DataStream_.insert({hash, type});
    }
  }

  size_t to_hash(const DataStream& type) const {
    return std::hash<std::string_view>{}(magic_enum::enum_name(type));
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
    for (const auto& [type, num_packets] : num_packets_written_) {
      msg << "Type: " << magic_enum::enum_name(type)
          << ", num packets: " << num_packets << std::endl;
    }

    msg << "Input file " << out_file_path_
        << ", is_open: " << in_file_.is_open() << ", read "
        << num_bytes_read_ * 1e-9 << " Gb with " << std::endl;
    for (const auto& [type, num_packets] : num_packets_read_) {
      msg << "Type: " << magic_enum::enum_name(type)
          << ", num packets: " << num_packets << std::endl;
    }
    return msg.str();
  }

  void write_imu_packet(const DataStream& type, const double timestamp,
                        const dai::IMUPacket& packet) {
    if (!out_file_.is_open()) {
      LOG(ERROR) << "Write file is not open while writing IMU packet.";
      return;
    }

    // Write type
    const size_t type_hash = DataStream_to_log_type_.at(type);
    write_data<size_t>(type_hash);

    const IMUPacket imu_packet(timestamp, packet);
    write_data<IMUPacket>(imu_packet);

    num_packets_written_.at(type) += 1;
  }

  void write_camera_packet(const DataStream& type, const double timestamp,
                           const std::shared_ptr<dai::ImgFrame> packet) {
    if (!out_file_.is_open()) {
      LOG(ERROR) << "Write file is not open while writing camera packet.";
      return;
    }

    // Write type
    const size_t type_hash = DataStream_to_log_type_.at(type);
    write_data<size_t>(type_hash);

    // Write timestamp
    write_data<float>(timestamp);

    // Write width and height
    const auto& cv_mat = packet->getCvFrame();
    write_data<int>(cv_mat.rows);
    write_data<int>(cv_mat.cols);
    write_data<int>(cv_mat.type());

    out_file_.write(reinterpret_cast<const char*>(cv_mat.ptr()),
                    cv_mat.total() * cv_mat.elemSize());

    num_packets_written_.at(type) += 1;
  }

  bool prepare_input_stream(std::string_view input_path) {
    out_file_path_ = std::string(input_path);
    in_file_.open(out_file_path_, std::ios::in | std::ios::binary);
    return in_file_.is_open();
  }

  bool read_input_file() {
    if (!in_file_.is_open()) {
      return false;
    }

    LOG(INFO) << "In read_input_file with " << in_file_.tellg();

    size_t type_hash;
    read_data<size_t>(type_hash);
    const DataStream type = (log_type_to_DataStream_.find(type_hash) !=
                             log_type_to_DataStream_.end())
                                ? log_type_to_DataStream_.at(type_hash)
                                : DataStream::INVALID;

    IMUPacket imu_packet;
    cv::Mat image;
    if (type == DataStream::IMU) {
      read_data<IMUPacket>(imu_packet);

      LOG(INFO) << "Here with IMU "
                << ", ts: " << imu_packet.timestamp
                << ", accel: " << imu_packet.accelerometer.sequence_num
                << ", x: " << imu_packet.accelerometer.x
                << ", y: " << imu_packet.accelerometer.y
                << ", z: " << imu_packet.accelerometer.z
                << ", gyro: " << imu_packet.gyroscope.sequence_num
                << ", x: " << imu_packet.gyroscope.x
                << ", y: " << imu_packet.gyroscope.y
                << ", z: " << imu_packet.gyroscope.z
                << ", gcount: " << in_file_.gcount()
                << ", tellg: " << in_file_.tellg()
                << ", done: " << (in_file_.peek() == EOF) << std::endl;

    } else if (type != DataStream::INVALID) {
      float timestamp = MAX_FLOAT;
      int nrows = 0;
      int ncols = 0;
      int img_type = 0;

      // Read image dims
      read_data<float>(timestamp);
      read_data<int>(nrows);
      read_data<int>(ncols);
      read_data<int>(img_type);

      CameraPacket cam_packet(nrows, ncols, img_type);
      cv::Mat& image = cam_packet.image;

      const size_t nbytes = image.total() * image.elemSize();
      in_file_.read(reinterpret_cast<char*>(image.ptr()), nbytes);
      num_bytes_read_ += nbytes;

      LOG(INFO) << "Here with " << magic_enum::enum_name(type)
                << ", nrows: " << nrows << ", ncols: " << ncols
                << ", type: " << img_type;
    } else {
      LOG(ERROR) << "Got an INVALID packet hash: " << type_hash;
    }

    // Record read packets
    if (type != DataStream::INVALID) {
      num_packets_read_.at(type) += 1;
    }

    return in_file_.peek() != EOF;
  }

 private:
  template <typename T>
  void write_data(const T& data) {
    if (!out_file_.is_open()) {
      LOG(ERROR) << "Write file is not open.";
      return;
    }
    out_file_.write(reinterpret_cast<const char*>(&data), sizeof data);
    num_bytes_written_ += sizeof data;
  }

  template <typename T>
  void read_data(T& data) {
    if (!in_file_.is_open()) {
      LOG(ERROR) << "Write file is not open.";
      return;
    }
    in_file_.read(reinterpret_cast<char*>(&data), sizeof data);
    num_bytes_read_ += sizeof data;
  }

 private:
  std::unordered_map<DataStream, size_t> DataStream_to_log_type_;
  std::unordered_map<size_t, DataStream> log_type_to_DataStream_;

  size_t num_bytes_written_ = 0;
  std::unordered_map<DataStream, size_t, DataStreamHash> num_packets_written_;
  std::unordered_map<DataStream, size_t, DataStreamHash> num_packets_read_;

  std::string out_file_path_;
  std::ofstream out_file_;

  size_t num_bytes_read_ = 0;
  std::ifstream in_file_;
};
}  // namespace oakd_logger
