#pragma once
#include <opencv2/core/core.hpp>
#include <thread>

#include "config.hpp"

namespace oakd_logger {

// void update_view(std::mutex& mtx, int& key, const cv::Mat& preview) {
class OAKDPreviewer {
 public:
  OAKDPreviewer()
      //: viewer_(std::thread(&update_view, viewer_mutex_, key_, preview_img_))
      //: {
      : viewer_(std::make_unique<std::thread>(OAKDPreviewer::update_view)) {
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
    preview_img_ = cv::Mat(RGB_PREVIEW_ROWS, RGB_PREVIEW_COLS, CV_8UC3);
    (void)key_;
  }

  static void update_view() {
    int key = 0;
    // mtx.lock();
    // cv::namedWindow("Preview", cv::WINDOW_NORMAL);
    // cv::imshow("Preview", preview);
    // key = cv::waitKey(1);
    std::cout << "Update viewer with " << ++key << std::endl;
    // mtx.unlock();
  }

  int update() const { return -1; }

 private:
  std::unique_ptr<std::thread> viewer_;
  std::mutex viewer_mutex_;

  cv::Mat preview_img_;
  int key_ = -1;
};
}  // namespace oakd_logger
