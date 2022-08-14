#pragma once
#include <opencv2/core/core.hpp>
#include <thread>

#include "config.hpp"
#include "logger_types.hpp"
#include "magic_enum.hpp"

namespace oakd_logger {

// void update_view(std::mutex& mtx, int& key, const cv::Mat& preview) {
class OAKDPreviewer {
 public:
   OAKDPreviewer()
       : preview_img_{std::make_unique<cv::Mat>(RGB_PREVIEW_ROWS,
                                                RGB_PREVIEW_COLS, CV_8UC3)} {
     mask_.insert({DataStream::LEFT_MONO, cv::Rect(0, 0, 0.5 * RGB_PREVIEW_COLS,
                                                   0.5 * RGB_PREVIEW_ROWS)});
     mask_.insert({DataStream::RIGHT_MONO,
                   cv::Rect(0.5 * RGB_PREVIEW_COLS, 0, 0.5 * RGB_PREVIEW_COLS,
                            0.5 * RGB_PREVIEW_ROWS)});
     mask_.insert({DataStream::RGB,
                   cv::Rect(0, 0.5 * RGB_PREVIEW_ROWS, 0.5 * RGB_PREVIEW_COLS,
                            0.5 * RGB_PREVIEW_ROWS)});
     mask_.insert({DataStream::IMU,
                   cv::Rect(0.5 * RGB_PREVIEW_COLS, 0.5 * RGB_PREVIEW_ROWS,
                            0.5 * RGB_PREVIEW_COLS, 0.5 * RGB_PREVIEW_ROWS)});
   }
   void update(const DataStream &type, const IMUPacket &imu_packet,
               const CameraPacket &cam_packet) {
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

     if (type == DataStream::IMU) {
       (void)imu_packet;
     }
     // Check if it is a camera packet
     else if (type != DataStream::INVALID) {
       cv::Mat resized;
       if (type == DataStream::LEFT_MONO || type == DataStream::RIGHT_MONO) {
         // Convert GRAY to BGR
         cv::Mat gray_resized;
         cv::resize(cam_packet.image, gray_resized,
                    cv::Size(0.5 * RGB_PREVIEW_COLS, 0.5 * RGB_PREVIEW_ROWS),
                    cv::INTER_LINEAR);
         cv::cvtColor(gray_resized, resized, cv::COLOR_GRAY2BGR);
       } else {
         // Resize RGB image
         cv::resize(cam_packet.image, resized,
                    cv::Size(0.5 * RGB_PREVIEW_COLS, 0.5 * RGB_PREVIEW_ROWS),
                    cv::INTER_LINEAR);
       }

       resized.copyTo(preview_img_->operator()(mask_.at(type)));
     } else {
       std::cout << "Trying to draw an invalid type." << std::endl;
     }
   }

   /**
    * @brief:   Get preview image
    * @return:  Pointer to the collated image
    */
   cv::Mat *preview_img() const { return preview_img_.get(); };

 private:
   std::unordered_map<DataStream, cv::Rect> mask_;
   std::unique_ptr<cv::Mat> preview_img_;
};
}  // namespace oakd_logger
