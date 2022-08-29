#pragma once
#include <pybind11/pybind11.h>

#include "Eigen/Dense"
#include "config.hpp"
#include "logger.hpp"

namespace oakd_logger {
namespace py = pybind11;

class OAKDLoggerWrapper {
 public:
  OAKDLoggerWrapper(const py::dict& args);

  bool initialize() { return logger_->initialize(); }

  void start_logging() { return logger_->start_logging(); }

  void replay(std::string_view input_file) {
    return logger_->replay(input_file);
  }

  std::string sequential_read(
      std::string_view input_file,
      Eigen::Ref<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> img) {
    return logger_->sequential_read(input_file, img);
  }

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
