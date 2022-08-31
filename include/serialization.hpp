#pragma once
#include <fstream>
#include <functional>
#include <optional>
#include <sstream>
#include <stdexcept>

#include "config.hpp"
#include "logger_types.hpp"
#include "magic_enum.hpp"
#include "misc_utilities.hpp"

namespace oakd_logger {

class OAKDSerializer {
 public:
  OAKDSerializer() {
      for (const auto &type : ALL_SENSOR_TYPES) {
          num_packets_written_.insert({type, 0});
          num_packets_read_.insert({type, 0});
          const uint64_t hash =
              Utilities::Misc::dirty_string_hash(magic_enum::enum_name(type));

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
   * @brief:    Check if ready to log output stream to a file
   * @return:   True, if ready
   */
  bool ok_to_log() const { return out_file_.is_open(); }

  /**
   * @brief:    Check if input file is ready to read
   * @return:   True, if ready
   */
  bool ok_to_read() const { return in_file_.is_open(); }

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
  void write_imu_packet(const double timestamp, const dai::IMUPacket &packet) {
      if (!out_file_.is_open()) {
          std::cout << "Write file is not open while writing IMU packet."
                    << std::endl;
          return;
      }

      // Write type
      const size_t type_hash = DataStream_to_log_type_.at(DataStream::IMU);
      out_file_.write(reinterpret_cast<const char *>(&type_hash),
                      sizeof type_hash);

      const IMUPacket imu_packet(static_cast<float>(timestamp), packet);
      out_file_.write(reinterpret_cast<const char *>(&imu_packet),
                      sizeof imu_packet);

      num_packets_written_.at(DataStream::IMU) += 1;
  }

  /**
   * @brief:    Write camera packet
   * @param[in] type:   Sensor type
   * @param[in] timestamp:  Sensor timestamp
   * @param[in] packet:  Image pointer
   */
  void write_camera_packet(const DataStream &type, const float timestamp,
                           const std::shared_ptr<dai::ImgFrame> &packet) {
      if (!out_file_.is_open()) {
          throw std::runtime_error(
              "Write file is not open while writing camera packet.");
      }

      // Write type
      const size_t type_hash = DataStream_to_log_type_.at(type);
      out_file_.write(reinterpret_cast<const char *>(&type_hash),
                      sizeof type_hash);

      // Write timestamp
      out_file_.write(reinterpret_cast<const char *>(&timestamp),
                      sizeof timestamp);

      // Write width and height
      const auto cv_mat = packet->getCvFrame();
      const int sequence_num = static_cast<int>(packet->getSequenceNum());
      const int num_rows = cv_mat.rows;
      const int num_cols = cv_mat.cols;
      const int image_type = cv_mat.type();
      out_file_.write(reinterpret_cast<const char *>(&sequence_num),
                      sizeof sequence_num);
      out_file_.write(reinterpret_cast<const char *>(&num_rows), sizeof num_rows);
      out_file_.write(reinterpret_cast<const char *>(&num_cols), sizeof num_cols);
      out_file_.write(reinterpret_cast<const char *>(&image_type),
                      sizeof image_type);

      // Write the actual image
      out_file_.write(reinterpret_cast<const char *>(cv_mat.ptr()),
                      static_cast<long>(cv_mat.total() * cv_mat.elemSize()));

      num_packets_written_.at(type) += 1;
  }

  /**
   * @brief:    Read input stream
   * @param[in] imu_packet: IMU packet
   * @param[in] cam_packet: Camera packet
   * @return:   DataStream type if read successfully
   */
  std::optional<DataStream> read_input_stream(IMUPacket &imu_packet,
                                              CameraPacket &cam_packet) {
      if (!in_file_.is_open() || in_file_.peek() == EOF) {
          return std::nullopt;
      }

      size_t type_hash;
      in_file_.read(reinterpret_cast<char *>(&type_hash), sizeof type_hash);

      // Reach the packet corresponding to the packet type
      const DataStream type = (log_type_to_DataStream_.find(type_hash) !=
          log_type_to_DataStream_.end())
                              ? log_type_to_DataStream_.at(type_hash)
                              : DataStream::INVALID;

      if (type == DataStream::IMU) {
          in_file_.read(reinterpret_cast<char *>(&imu_packet), sizeof imu_packet);
      } else if (image_type(type)) {
          float timestamp = MAX_FLOAT;
          int sequence_num = -1;
          int num_cols = -1;
          int num_rows = -1;
          int img_type = -1;

          // Read image dims
          in_file_.read(reinterpret_cast<char *>(&timestamp), sizeof timestamp);
          in_file_.read(reinterpret_cast<char *>(&sequence_num),
                        sizeof sequence_num);
          in_file_.read(reinterpret_cast<char *>(&num_cols), sizeof num_cols);
          in_file_.read(reinterpret_cast<char *>(&num_rows), sizeof num_rows);
          in_file_.read(reinterpret_cast<char *>(&img_type), sizeof img_type);

          cv::Mat img(num_cols, num_rows, img_type);
          const size_t num_bytes = img.total() * img.elemSize();
          in_file_.read(reinterpret_cast<char *>(img.ptr()), static_cast<long>(num_bytes));
          cam_packet = CameraPacket(type, timestamp, sequence_num, img);
      } else {
          std::cout << "Got an INVALID packet hash: " << type_hash << std::endl;
      }

      // Record read packets
      if (type != DataStream::INVALID) {
          num_packets_read_.at(type) += 1;
          return type;
      }

      return std::nullopt;
  }

  /**
   * @brief:    Write some diagnostic information about the output file
   * @return:   String with information about the output file
   */
  std::string output_file_info() {
      std::stringstream msg;
      msg << "Output file is_open: " << out_file_.is_open() << ", num_bytes "
          << out_file_.tellp() << " bytes (" << (static_cast<double>(out_file_.tellp()) * 1e-6)
          << " Mb)" << std::endl
          << "Number of packets: " << std::endl;
      for (const auto &[type, num_packets] : num_packets_written_) {
          msg << "Type: " << magic_enum::enum_name(type)
              << ", num packets: " << num_packets << std::endl;
      }
      return msg.str();
  }

  /**
   * @brief:    Write some diagnostic information about the input file
   * @return:   A string with the required diagnostic information
   */
  std::string input_file_info() {
      std::stringstream msg;
      msg << "Input file is_open: " << in_file_.is_open() << ", num_bytes "
          << in_file_.tellg() << " bytes (" << (static_cast<double>(in_file_.tellg()) * 1e-6) << " Mb)"
          << std::endl
          << "Number of packets: " << std::endl;
      for (const auto &[type, num_packets] : num_packets_read_) {
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
