#pragma once

#include "circular_buffer.hpp"

namespace oakd_logger {

struct StatDataPoint {
  size_t ds{};  // Delta sequence num
  double dt{};  // Delta timestamp
};

template<size_t N>
class PacketStatistics {
 public:
  static constexpr size_t MAX_SEQ_NUM = 255;
  static constexpr size_t MAX_SLOTS = 10;
  static constexpr size_t NUM_SLOT_ITEMS = N / MAX_SLOTS;

  PacketStatistics(const double min_dt_s, const double max_dt_s)
      : min_dt_s_(min_dt_s), max_dt_s_(max_dt_s) {}

  void update_statistics(const size_t seq_num, const double timestamp) {
      if (nullptr == last_timestamp_) {
          last_timestamp_ = std::make_unique<double>(timestamp);
      }
      if (nullptr == last_seq_num_) {
          last_seq_num_ = std::make_unique<size_t>(seq_num);
      }

      size_t ds = (seq_num + MAX_SEQ_NUM - *last_seq_num_) % MAX_SEQ_NUM;
      double dt = timestamp - *last_timestamp_;
      delta_.push({.ds = ds, .dt = dt});

      *last_timestamp_ = timestamp;
      *last_seq_num_ = seq_num;
  }

  void get_statistics(std::vector<StatDataPoint> &stat) const {
      stat.clear();
      auto start_iter = delta_.cbegin();

      while (start_iter != delta_.cend()) {
          auto end_iter = std::distance(start_iter, delta_.cend()) <
              static_cast<long>(NUM_SLOT_ITEMS)
                          ? delta_.cend()
                          : std::next(start_iter, NUM_SLOT_ITEMS);

          if (start_iter == end_iter) {
              break;
          }

          // Compute max ds and dt
          const auto max_iter = std::max_element(
              start_iter, end_iter,
              [](const auto &a, const auto &b) { return a.dt < b.dt; });
          stat.push_back(*max_iter);
          start_iter = end_iter;
      }
  }

  void info(const std::string_view identifier) {
      std::stringstream msg;
      msg << "Statistics, " << identifier;
      for (const auto &delta : delta_) {
          msg << "(" << delta.dt << ", " << delta.ds << "), ";
      }
      LOG(INFO) << msg.str();
  }

  // Some getters
  double min_dt_s() const { return min_dt_s_; }
  double max_dt_s() const { return max_dt_s_; }

 private:
  // Store some thresholds for plotting
  double min_dt_s_ = 0.0;
  double max_dt_s_ = 1e-4;

  Utilities::Containers::CircularBuffer<StatDataPoint, N> delta_;
  std::unique_ptr<double> last_timestamp_ = nullptr;
  std::unique_ptr<size_t> last_seq_num_ = nullptr;
};
using IMUStat = PacketStatistics<IMU_STAT_BUFFER_SIZE>;
using CAMStat = PacketStatistics<CAM_STAT_BUFFER_SIZE>;
}  // namespace oakd_logger
