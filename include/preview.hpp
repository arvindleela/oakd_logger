#pragma once
#include <opencv2/core/core.hpp>
#include <sstream>
#include <thread>

#include "config.hpp"
#include "logger_types.hpp"
#include "magic_enum.hpp"
#include "packet_statistics.hpp"

namespace oakd_logger {

class OAKDPreviewer {
  static constexpr float SCALE = 0.5;
  static constexpr size_t PANE_ROWS = SCALE * CAM_ROWS;
  static constexpr size_t PANE_COLS = SCALE * CAM_COLS;

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
          /*scale*/ 2.0 * SCALE,
          /*color*/ COLOR_WHITE,
          /*thickness*/ 1,
          /*linttype*/ false);
  }

  void write_col_block(cv::Mat &diag_image,
                       const std::vector<std::string> &lines,
                       const int row_offset, const int col_offset) {
      const int drow = 30 * SCALE;
      int roff = row_offset;
      for (const auto &line : lines) {
          write_text(diag_image, col_offset, roff, line);
          roff += drow;
      }
  }

  void draw_quality_bar(cv::Mat &diag_image,
                        const std::vector<StatDataPoint> &stats,
                        const double min, const double max,
                        const size_t row_offset, const size_t col_offset) {
      const cv::Scalar RED(0, 0, 255);
      const cv::Scalar GREEN(0, 255, 0);
      static constexpr double dcol = 20 * SCALE;
      static constexpr double BOX_HEIGHT = 10 * SCALE;
      static constexpr double BOX_WIDTH = 10 * SCALE;

      // Draw little rectangles for the IMU
      const double max_delta = max - min;
      double co = col_offset;
      for (const auto &stat : stats) {
          const double color_scale =
              (std::clamp(stat.dt, min, max) - min) / max_delta;

          const cv::Scalar color = color_scale * RED + (1.0 - color_scale) * GREEN;
          cv::Rect rect(co, row_offset, BOX_WIDTH, BOX_HEIGHT);

          co += dcol;
          cv::rectangle(diag_image, rect, color, /* thickness */ -1.0);
      }
  }

  void draw_statistics(
      cv::Mat &diag_image, std::shared_ptr<IMUStat> imu_stat,
      std::unordered_map<DataStream, std::shared_ptr<CAMStat>> cam_stats,
      const size_t row_offset, const size_t col_offset) {
      const size_t drow = 30 * SCALE;
      size_t ro = row_offset;
      std::vector<StatDataPoint> stats;

      imu_stat->get_statistics(stats);

      // Draw IMU quality bar
      draw_quality_bar(diag_image, stats, imu_stat->min_dt_s(),
                       imu_stat->max_dt_s(), ro, col_offset);
      ro += drow;

      // Draw cam quality bar
      for (const auto &type : ALL_SENSOR_TYPES) {
          if (!image_type(type)) {
              continue;
          }

          auto cam_stat = cam_stats.at(type);
          cam_stat->get_statistics(stats);
          draw_quality_bar(diag_image, stats, cam_stat->min_dt_s(),
                           cam_stat->max_dt_s(), ro, col_offset);
          ro += drow;
      }
  }

  void update_diagnostic_frame(
      std::shared_ptr<IMUStat> imu_stat,
      std::unordered_map<DataStream, std::shared_ptr<CAMStat>> &cam_stats,
      const bool record) {
      cv::Mat diag_image(PANE_ROWS, PANE_COLS, CV_8UC3);
      diag_image = 0;

      // Write last timestamps
      const int INIT_ROW_OFFSET = 30 * SCALE;
      const int INIT_COL_OFFSET = 20 * SCALE;
      int row_offset = INIT_ROW_OFFSET;
      int col_offset = INIT_COL_OFFSET;

      // Write names of sensors
      std::vector<std::string> lines;
      lines.emplace_back("Type");
      for (const auto &type : ALL_SENSOR_TYPES) {
          lines.emplace_back(magic_enum::enum_name(type));
      }
      write_col_block(diag_image, lines, row_offset, col_offset);

      // Write timestamps
      lines.clear();
      col_offset += SCALE * 200;
      lines.emplace_back("|Timestamp(s)");
      for (const auto &type : ALL_SENSOR_TYPES) {
          lines.emplace_back("|" + std::to_string(sensor_timestamp_.at(type)));
      }
      write_col_block(diag_image, lines, row_offset, col_offset);

      // Draw statistics
      col_offset += SCALE * 200;
      draw_statistics(diag_image, imu_stat, cam_stats, row_offset + SCALE * 20,
                      col_offset);

      // Write min and max times
      lines.clear();
      col_offset += SCALE * 200;
      std::stringstream msg;
      msg << "|Threshold(s)";
      lines.emplace_back(msg.str());

      msg.str("");
      msg << std::setprecision(3) << "|(" << imu_stat->min_dt_s() << ","
          << imu_stat->max_dt_s() << ")";
      lines.emplace_back(msg.str());

      for (const auto &type : ALL_SENSOR_TYPES) {
          if (!image_type(type)) {
              continue;
          }
          msg.str("");
          msg << std::setprecision(3) << "|(" << cam_stats.at(type)->min_dt_s()
              << "," << cam_stats.at(type)->max_dt_s() << ")";
          lines.emplace_back(msg.str());
      }
      write_col_block(diag_image, lines, row_offset, col_offset);

      // Write record statistics
      row_offset += SCALE * 200;
      col_offset = INIT_COL_OFFSET;
      lines.clear();
      msg.str("");
      msg << "Record [" << (record ? "x" : " ") << "]";
      lines.emplace_back(msg.str());
      write_col_block(diag_image, lines, row_offset, col_offset);

      // Finaly write diagnostic frame to preview
      diag_image.copyTo(preview_img_->operator()(mask_.at(DataStream::IMU)));
  }

  void update_image(
      const CameraPacket &cam_packet, std::shared_ptr<IMUStat> imu_stat,
      std::unordered_map<DataStream, std::shared_ptr<CAMStat>> cam_stats,
      const bool record) {
      if (!image_type(cam_packet.type())) {
          return;
      }
      sensor_timestamp_[cam_packet.type()] = cam_packet.timestamp();
      cv::Mat resized = cam_packet.image();

      if (mono_image_type(cam_packet.type())) {
          cv::cvtColor(resized, resized, cv::COLOR_GRAY2BGR);
      }

      cv::resize(resized, resized, cv::Size(PANE_COLS, PANE_ROWS),
                 cv::INTER_LINEAR);
      resized.copyTo(preview_img_->operator()(mask_.at(cam_packet.type())));

      // Update diagnostics information
      update_diagnostic_frame(imu_stat, cam_stats, record);
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
