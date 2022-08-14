#pragma once
#include <opencv2/core/core.hpp>
#include <sstream>
#include <thread>

#include "config.hpp"
#include "logger_types.hpp"
#include "magic_enum.hpp"

namespace oakd_logger {

class OAKDPreviewer {
  static constexpr float scale = 0.5;
  static constexpr size_t PANE_ROWS = scale * RGB_PREVIEW_ROWS;
  static constexpr size_t PANE_COLS = scale * RGB_PREVIEW_COLS;

public:
  // clang-format off
  // The preview window has this layout
  //  M = RGB_PREVIEW_ROWS
  //  N = RGB_PREVIEW_COLS
  //  _________________________________________
  //  |                   |                   |
  //  | LEFT mono         |  RIGHT mono       |
  //  | M x N             |  M x N            |
  //  |___________________|___________________|
  //  |                   |                   |
  //  |  RGB color        |  IMU + flags      |
  //  |  M x N            |  M x N            |
  //  |___________________|___________________|
  // clang-format on

  OAKDPreviewer()
      : preview_img_{
            std::make_unique<cv::Mat>(2 * PANE_ROWS, 2 * PANE_COLS, CV_8UC3)} {
    mask_.insert({DataStream::LEFT_MONO, cv::Rect(0, 0, PANE_COLS, PANE_ROWS)});
    mask_.insert(
        {DataStream::RIGHT_MONO, cv::Rect(PANE_COLS, 0, PANE_COLS, PANE_ROWS)});
    mask_.insert(
        {DataStream::RGB, cv::Rect(0, PANE_ROWS, PANE_COLS, PANE_ROWS)});
    mask_.insert({DataStream::IMU,
                  cv::Rect(PANE_COLS, PANE_ROWS, PANE_COLS, PANE_ROWS)});

    // Initialize sensor_timestamp_
    for (const auto &type : ALL_SENSOR_TYPES) {
      sensor_timestamp_.insert({type, -1.0});
    }
  }

  void write_text(cv::Mat &image, int x, int y, const std::string &text) {
    const cv::Scalar COLOR_WHITE(255, 255, 255);
    cv::putText(image, text,
                /*location*/ cv::Point(x, y),
                /*font*/ cv::FONT_HERSHEY_PLAIN,
                /*scale*/ 2.0 * scale,
                /*color*/ COLOR_WHITE,
                /*thickness*/ 1,
                /*linttype*/ false);
  }

  void write_col_block(cv::Mat &diag_image,
                       const std::vector<std::string> &lines,
                       const int row_offset, const int col_offset) {
    const int drow = 30 * scale;
    int roff = row_offset;
    for (const auto &line : lines) {
      write_text(diag_image, col_offset, roff, line);
      roff += drow;
    }
  }

  void update_diagnostic_frame() {
    cv::Mat diag_image(PANE_ROWS, PANE_COLS, CV_8UC3);
    diag_image = 0;

    // Write last timestamps
    int row_offset = 15;
    int col_offset = 10;

    // Write names of sensors
    std::vector<std::string> lines;
    lines.emplace_back("Type");
    for (const auto &type : ALL_SENSOR_TYPES) {
      lines.emplace_back(magic_enum::enum_name(type));
    }
    write_col_block(diag_image, lines, row_offset, col_offset);

    // Write timestamps
    lines.clear();
    col_offset += scale * 200;
    lines.emplace_back("|Timestamp(s)");
    for (const auto &type : ALL_SENSOR_TYPES) {
      lines.emplace_back("|" + std::to_string(sensor_timestamp_.at(type)));
    }
    write_col_block(diag_image, lines, row_offset, col_offset);

    // Finalyy write diagnostic frame to preview
    diag_image.copyTo(preview_img_->operator()(mask_.at(DataStream::IMU)));
  }

  void update_image(const CameraPacket &cam_packet) {
    if (!image_type(cam_packet.type)) {
      return;
    }
    sensor_timestamp_[cam_packet.type] = cam_packet.timestamp;
    cv::Mat resized = cam_packet.image;

    if (mono_image_type(cam_packet.type)) {
      cv::cvtColor(resized, resized, cv::COLOR_GRAY2BGR);
    }

    cv::resize(resized, resized, cv::Size(PANE_COLS, PANE_ROWS),
               cv::INTER_LINEAR);
    resized.copyTo(preview_img_->operator()(mask_.at(cam_packet.type)));

    // Update diagnostics information
    update_diagnostic_frame();
  }

  void update_imu(const float timestamp) {
    sensor_timestamp_[DataStream::IMU] = timestamp;
  }

  /**
   * @brief:   Get preview image
   * @return:  Pointer to the collated image
   */
  cv::Mat *preview_img() const { return preview_img_.get(); };

private:
  std::unordered_map<DataStream, float> sensor_timestamp_;

  std::unordered_map<DataStream, cv::Rect> mask_;
  std::unique_ptr<cv::Mat> preview_img_;
};
}  // namespace oakd_logger
