#include "bindings.hpp"

namespace oakd_logger {
namespace py = pybind11;

OAKDLoggerWrapper::OAKDLoggerWrapper(const py::dict &args) {
    Config logger_config;

    logger_ = std::make_unique<Logger>(args["logdir"].cast<std::string>(),
                                       logger_config);
}

DataStream OAKDLoggerWrapper::sequential_read(std::string_view input_file,
                                              py::dict &cam_dict,
                                              py::dict &imu_dict) {
    IMUPacket imu_packet;
    CameraPacket cam_packet;
    const auto type = logger_->sequential_read(input_file, imu_packet, cam_packet);
    if (!type) {
        return DataStream::INVALID;
    }

    if (image_type(*type)) {
        auto img = cam_dict["img"].cast<Eigen::Ref<EigenImage>>();
        std::copy(cam_packet.image().data, cam_packet.image().data + cam_packet.image().total(), img.data());
        cam_dict["timestamp"] = cam_packet.timestamp();
    } else if (*type == DataStream::IMU) {
        auto accelerometer = imu_dict["accelerometer"].cast<Eigen::Ref<Eigen::Matrix<double, 3, 1>>>();
        std::copy(imu_packet.accelerometer.data().cbegin(),
                  imu_packet.accelerometer.data().cend(),
                  accelerometer.data());

        auto gyroscope = imu_dict["gyroscope"].cast<Eigen::Ref<Eigen::Matrix<double, 3, 1>>>();
        std::copy(imu_packet.gyroscope.data().cbegin(), imu_packet.gyroscope.data().cend(), gyroscope.data());

        imu_dict["timestamp"] = imu_packet.timestamp;
    }
    return *type;
}

PYBIND11_MODULE(OAKDLogger, m) {
    py::class_<OAKDLoggerWrapper>(m, "OAKDLogger")
        .def(py::init<const py::dict &>())
        .def("initialize", &OAKDLoggerWrapper::initialize)
        .def("prepare_output_stream", &OAKDLoggerWrapper::prepare_output_stream)
        .def("start_logging", &OAKDLoggerWrapper::start_logging)
        .def("replay", &OAKDLoggerWrapper::replay)
        .def("sequential_read", &OAKDLoggerWrapper::sequential_read);

    py::enum_<DataStream>(m, "DataStream")
        .value(magic_enum::enum_name(DataStream::IMU).data(), DataStream::IMU)
        .value(magic_enum::enum_name(DataStream::LEFT_MONO).data(), DataStream::LEFT_MONO)
        .value(magic_enum::enum_name(DataStream::RIGHT_MONO).data(), DataStream::RIGHT_MONO)
        .value(magic_enum::enum_name(DataStream::RGB).data(), DataStream::RGB)
        .value(magic_enum::enum_name(DataStream::INVALID).data(), DataStream::INVALID)
        .export_values();
};
}  // namespace oakd_logger
