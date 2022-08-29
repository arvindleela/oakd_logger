#pragma once
#include "pybind11/pybind11.h"
#include "pybind11/eigen.h"

#include "config.hpp"
#include "logger.hpp"

namespace oakd_logger {
namespace py = pybind11;

class OAKDLoggerWrapper {
 public:
  /**
   * @brief:        C'tor that takes some arguments.
   * @param args:   Arguments passed in through python.
   */
  explicit OAKDLoggerWrapper(const py::dict &args);

  /**
   * @brief:    Initialize logger.
   * @return:   True, if success
   */
  bool initialize() { return logger_->initialize(); }

  /**
   * @brief:    Start logging
   */
  void start_logging() { return logger_->start_logging(); }

  /**
   * @brief:    Replay a raw binary file.
   * @param input_file: Output of the logger
   */
  void replay(std::string_view input_file) {
      return logger_->replay(input_file);
  }

  /**
   * @brief:    Sequential read of an input file
   * @param[in] input_file: Input filename
   * @param[out] cam_dict:  A placeholder where camera data is returned
   * @param[out] imu_dict:  A placeholder where IMU data is returned
   * @return:   Type of the next packet
   */
  DataStream sequential_read(std::string_view input_file,
                             py::dict &cam_dict,
                             py::dict &imu_dict);

  /**
   * @brief:    Prepare logging at a given output path
   * @param[in] output_path:    Output path binary file
   * @return:   True, if file is successfully opened
   */
  bool prepare_output_stream(std::string_view output_path) {
      return logger_->prepare_output_stream(output_path);
  }

 private:
  std::unique_ptr<Logger> logger_;
};
}  // namespace oakd_logger
