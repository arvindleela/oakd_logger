#pragma once
#include <fstream>
#include <functional>
#include <sstream>

#include "config.hpp"
#include "logger_types.hpp"
#include "magic_enum.hpp"

namespace oakd_logger {

/**
 * @brief:  Hash DataStream type to a size_t
 * @param[in]   type:   DataStream type
 * @return: A unique hash corresponding to this type
 */
inline size_t to_hash(const DataStream& type) {
  return std::hash<std::string_view>{}(magic_enum::enum_name(type));
}

class OAKDSerializer {
 public:
  OAKDSerializer() {
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

  /**
   * @brief:    Prepare input stream
   * @param[in] input_path: Input file path
   * @return:   True, if file was opened successfully
   */
  bool prepare_input_stream(std::string_view input_path) {
    in_file_.open(std::string(input_path), std::ios::in | std::ios::binary);
    if (in_file_.is_open()) {
      std::cout << "Successfully opened " << input_path << " for reading"
                << std::endl;
    } else {
      std::cout << "Failed to open " << input_path << " for reading"
                << std::endl;
    }
    return in_file_.is_open();
  }

  /**
   * @brief:    Prepare logging at a given output path
   * @param[in] output_path:    Output path binary file
   * @return:   True, if file is successfully opened
   */
  bool prepare_output_stream(std::string_view output_path) {
    out_file_.open(std::string(output_path), std::ios::out | std::ios::binary);
    if (out_file_.is_open()) {
      std::cout << "Successfully opened file " << output_path << " for logging."
                << std::endl;
    } else {
      std::cout << "Unable to open file " << output_path << " for logging "
                << std::endl;
    }
    return out_file_.is_open();
  }

  /**
   * @brief:    Write IMU packet
   * @param[in] timestamp:  Timestamp
   * @param[in] packet:  Data
   */
  void write_imu_packet(const double timestamp, const dai::IMUPacket& packet) {
    if (!out_file_.is_open()) {
      std::cout << "Write file is not open while writing IMU packet."
                << std::endl;
      return;
    }

    // Write type
    const size_t type_hash = DataStream_to_log_type_.at(DataStream::IMU);
    out_file_.write(reinterpret_cast<const char*>(&type_hash),
                    sizeof type_hash);

    const IMUPacket imu_packet(timestamp, packet);
    out_file_.write(reinterpret_cast<const char*>(&imu_packet),
                    sizeof imu_packet);

    num_packets_written_.at(DataStream::IMU) += 1;
  }

  /**
   * @brief:    Write camera packet
   * @param[in] type:   Sensor type
   * @param[in] timestamp:  Sensor timestamp
   * @param[in] packet:  Image pointer
   */
  void write_camera_packet(const DataStream& type, const float timestamp,
                           const std::shared_ptr<dai::ImgFrame> packet) {
    if (!out_file_.is_open()) {
      LOG(ERROR) << "Write file is not open while writing camera packet.";
      return;
    }

    // Write type
    const size_t type_hash = DataStream_to_log_type_.at(type);
    out_file_.write(reinterpret_cast<const char*>(&type_hash),
                    sizeof type_hash);

    // Write timestamp
    out_file_.write(reinterpret_cast<const char*>(&timestamp),
                    sizeof timestamp);

    // Write width and height
    const auto& cv_mat = packet->getCvFrame();
    const int nrows = cv_mat.rows;
    const int ncols = cv_mat.cols;
    const int image_type = cv_mat.type();
    out_file_.write(reinterpret_cast<const char*>(&nrows), sizeof nrows);
    out_file_.write(reinterpret_cast<const char*>(&ncols), sizeof ncols);
    out_file_.write(reinterpret_cast<const char*>(&image_type),
                    sizeof image_type);

    // Write the actualy image
    out_file_.write(reinterpret_cast<const char*>(cv_mat.ptr()),
                    cv_mat.total() * cv_mat.elemSize());

    num_packets_written_.at(type) += 1;
  }

  /**
   * @brief:    Read input stream
   * @return:   True, if success
   */
  bool read_input_stream() {
    if (!in_file_.is_open()) {
      return false;
    }

    size_t type_hash;
    in_file_.read(reinterpret_cast<char*>(&type_hash), sizeof type_hash);

    // Reach the packet corresponding to the packet type
    const DataStream type = (log_type_to_DataStream_.find(type_hash) !=
                             log_type_to_DataStream_.end())
                                ? log_type_to_DataStream_.at(type_hash)
                                : DataStream::INVALID;

    IMUPacket imu_packet;
    cv::Mat image;
    if (type == DataStream::IMU) {
      in_file_.read(reinterpret_cast<char*>(&imu_packet), sizeof imu_packet);
    } else if (type != DataStream::INVALID) {
      float timestamp = MAX_FLOAT;
      int nrows = 0;
      int ncols = 0;
      int img_type = 0;

      // Read image dims
      in_file_.read(reinterpret_cast<char*>(&timestamp), sizeof timestamp);
      in_file_.read(reinterpret_cast<char*>(&nrows), sizeof nrows);
      in_file_.read(reinterpret_cast<char*>(&ncols), sizeof ncols);
      in_file_.read(reinterpret_cast<char*>(&img_type), sizeof img_type);

      CameraPacket cam_packet(nrows, ncols, img_type);
      cv::Mat& image = cam_packet.image;

      const size_t nbytes = image.total() * image.elemSize();
      in_file_.read(reinterpret_cast<char*>(image.ptr()), nbytes);

    } else {
      std::cout << "Got an INVALID packet hash: " << type_hash << std::endl;
    }

    // Record read packets
    if (type != DataStream::INVALID) {
      num_packets_read_.at(type) += 1;
    }

    return in_file_.peek() != EOF;
  }

  /**
   * @brief:    Write some diagnostic information about the output file
   * @return:   String with information about the output file
   */
  std::string output_file_info() {
    std::stringstream msg;
    msg << "Output file is_open: " << out_file_.is_open() << ", tellp "
        << out_file_.tellp() << " bytes (" << (out_file_.tellp() * 1e-6)
        << " Mb)" << std::endl
        << "Number of packets: " << std::endl;
    for (const auto& [type, num_packets] : num_packets_written_) {
      msg << "Type: " << magic_enum::enum_name(type)
          << ", num packets: " << num_packets << std::endl;
    }
    return msg.str();
  }

  /**
   * @brief:    Write some diagnostic information about the input file
   * @return:   A string with the requeired diagnostic information
   */
  std::string input_file_info() {
    std::stringstream msg;
    msg << "Input file is_open: " << in_file_.is_open() << ", tellg "
        << in_file_.tellg() << " bytes (" << (in_file_.tellg() * 1e-6) << " Mb)"
        << std::endl
        << "Number of packets: " << std::endl;
    for (const auto& [type, num_packets] : num_packets_read_) {
      msg << "Type: " << magic_enum::enum_name(type)
          << ", num packets: " << num_packets << std::endl;
    }
    return msg.str();
  }

 private:
  std::unordered_map<DataStream, size_t> DataStream_to_log_type_;
  std::unordered_map<size_t, DataStream> log_type_to_DataStream_;

  std::unordered_map<DataStream, size_t, DataStreamHash> num_packets_written_;
  std::unordered_map<DataStream, size_t, DataStreamHash> num_packets_read_;

  std::ofstream out_file_;
  std::ifstream in_file_;
};
}  // namespace oakd_logger
