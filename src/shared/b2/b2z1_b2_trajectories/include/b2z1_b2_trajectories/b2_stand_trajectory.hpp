#pragma once

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace b2z1_b2_trajectories {

class B2StandTrajectory {
public:
  explicit B2StandTrajectory(rclcpp::Node& node)
  : target_pos_1_(load_array_param<12>(
      node, "b2_stand_target_1_q",
      {0.0, 1.36, -2.65, 0.0, 1.36, -2.65, 0.2, 1.36, -2.65, 0.2, 1.36, -2.65})),
    target_pos_2_(load_array_param<12>(
      node, "b2_stand_target_2_q",
      {0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3})),
    target_pos_3_(load_array_param<12>(
      node, "b2_stand_target_3_q",
      {0.0, 1.36, -2.65, 0.0, 1.36, -2.65, 0.0, 1.36, -2.65, 0.0, 1.36, -2.65})),
    target_pos_4_(load_array_param<12>(
      node, "b2_stand_target_4_q",
      {-0.5, 1.36, -2.65, 0.5, 1.36, -2.65, -0.5, 1.36, -2.65, 0.5, 1.36, -2.65})),
    down_pose_(load_array_param<12>(
      node, "b2_down_q",
      {0.0, 1.28, -2.80, 0.0, 1.28, -2.80, 0.0, 1.28, -2.80, 0.0, 1.28, -2.80})),
    kp_by_joint_(load_array_param<12>(
      node, "b2_kp_by_joint",
      {260.0, 260.0, 260.0, 260.0, 260.0, 260.0, 430.0, 430.0, 430.0, 430.0, 430.0, 430.0})),
    kd_by_joint_(load_array_param<12>(
      node, "b2_kd_by_joint",
      {7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0})),
    durations_(load_array_param<5>(
      node, "b2_stand_phase_durations",
      {1.0, 1.8, 2.0, 2.2, 1.0}))
  {
    start_pos_ = down_pose_;
  }

  template <std::size_t N>
  static std::array<double, N> load_array_param(
    rclcpp::Node& node,
    const std::string& name,
    const std::array<double, N>& default_value)
  {
    const auto vec = node.declare_parameter<std::vector<double>>(
      name, std::vector<double>(default_value.begin(), default_value.end()));
    if (vec.size() != N) {
      throw std::runtime_error(
              "Parameter '" + name + "' must have length " + std::to_string(N) +
              ", got " + std::to_string(vec.size()));
    }

    std::array<double, N> out{};
    std::copy(vec.begin(), vec.end(), out.begin());
    return out;
  }

  void set_start_position(const std::array<double, 12>& start_q)
  {
    start_pos_ = start_q;
  }

  std::array<double, 12> sample(double elapsed_sec) const
  {
    std::array<double, 12> q{};
    const double t1 = durations_[0];
    const double t2 = t1 + durations_[1];
    const double t3 = t2 + durations_[2];
    const double t4 = t3 + durations_[3];

    for (int i = 0; i < 12; ++i) {
      if (elapsed_sec < t1) {
        const double alpha = elapsed_sec / std::max(1.0e-3, durations_[0]);
        q[i] = start_pos_[i] + (target_pos_1_[i] - start_pos_[i]) * alpha;
      } else if (elapsed_sec < t2) {
        const double alpha = (elapsed_sec - t1) / std::max(1.0e-3, durations_[1]);
        q[i] = target_pos_1_[i] + (target_pos_2_[i] - target_pos_1_[i]) * alpha;
      } else if (elapsed_sec < t3) {
        q[i] = target_pos_2_[i];
      } else if (elapsed_sec < t4) {
        const double alpha = (elapsed_sec - t3) / std::max(1.0e-3, durations_[3]);
        q[i] = target_pos_2_[i] + (target_pos_3_[i] - target_pos_2_[i]) * alpha;
      } else {
        const double alpha = std::clamp(
          (elapsed_sec - t4) / std::max(1.0e-3, durations_[4]),
          0.0,
          1.0);
        q[i] = target_pos_3_[i] + (target_pos_4_[i] - target_pos_3_[i]) * alpha;
      }
    }
    return q;
  }

  const std::array<double, 12>& down_pose() const { return down_pose_; }
  const std::array<double, 12>& final_stand_pose() const { return target_pos_4_; }
  const std::array<double, 12>& kp_by_joint() const { return kp_by_joint_; }
  const std::array<double, 12>& kd_by_joint() const { return kd_by_joint_; }

  double total_duration() const
  {
    double total = 0.0;
    for (double d : durations_) {
      total += d;
    }
    return total;
  }
  double first_phase_duration() const { return durations_[0]; }

private:
  std::array<double, 12> start_pos_{};
  std::array<double, 12> target_pos_1_{};
  std::array<double, 12> target_pos_2_{};
  std::array<double, 12> target_pos_3_{};
  std::array<double, 12> target_pos_4_{};
  std::array<double, 12> down_pose_{};
  std::array<double, 12> kp_by_joint_{};
  std::array<double, 12> kd_by_joint_{};
  std::array<double, 5> durations_{};
};

}  // namespace b2z1_b2_trajectories
