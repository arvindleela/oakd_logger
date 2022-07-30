#include "bindings.hpp"

namespace oakd_logger {
namespace py = pybind11;

OAKDLoggerWrapper::OAKDLoggerWrapper(const py::dict& args) {
  Config logger_config;
  logger_ = std::make_unique<Logger>(args["logdir"].cast<std::string>(),
                                     logger_config);
}

// Declare a stereo pair
PYBIND11_MODULE(OAKDLogger, m) {
  py::class_<OAKDLoggerWrapper>(m, "OAKDLogger")
      .def(py::init<const py::dict&>())
      .def("initialize", &OAKDLoggerWrapper::initialize)
      .def("start_logging", &OAKDLoggerWrapper::start_logging);
};
}  // namespace oakd_logger
