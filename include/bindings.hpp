#pragma once
#include <pybind11/pybind11.h>

#include "config.hpp"
#include "logger.hpp"

namespace oakd_logger {
namespace py = pybind11;

class OAKDLoggerWrapper {
 public:
  OAKDLoggerWrapper(const py::dict& args);

  bool initialize() { return logger_->initialize(); }

  void start_logging() { return logger_->start_logging(); }

  bool prepare_output_log(std::string_view output_path) {
    return logger_->prepare_output_log(output_path);
  }

 private:
  std::unique_ptr<Logger> logger_;
};
}  // namespace oakd_logger
