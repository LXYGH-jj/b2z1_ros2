#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#include "b2z1_b2_trajectories/b2_stand_trajectory.hpp"
#include "b2z1_msgs/msg/mujoco_demo_command.hpp"

namespace b2z1_mujoco_bridge {

class MujocoBridgeNode : public rclcpp::Node {
public:
  MujocoBridgeNode();

private:
  void on_command(const b2z1_msgs::msg::MujocoDemoCommand::SharedPtr msg);
  void publish_lowcmd();
  void init_lowcmd_message();
  void set_motor_hold(int index, double q, double kp, double kd, uint8_t mode = 0x01);
  void set_motor_stop(int index, uint8_t mode = 0x01);
  void sample_current_targets(
    std::array<double, 12>& b2_q,
    std::array<double, 6>& z1_q,
    double& gain_scale) const;
  void log_z1_debug_snapshot(const char* reason);
  static uint32_t crc32_core(uint32_t* ptr, uint32_t len);

  rclcpp::Subscription<b2z1_msgs::msg::MujocoDemoCommand>::SharedPtr demo_command_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  unitree_go::msg::dds_::LowCmd_ low_cmd_{};
  unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_sub_;
  mutable std::mutex lowstate_mutex_;
  unitree_go::msg::dds_::LowState_ latest_lowstate_{};
  bool have_lowstate_;

  b2z1_b2_trajectories::B2StandTrajectory b2_trajectory_;
  std::array<double, 12> b2_from_q_{};
  std::array<double, 12> b2_to_q_{};
  std::array<double, 6> z1_target_q_{};
  std::array<double, 6> z1_from_q_{};
  std::array<double, 6> z1_to_q_{};
  std::array<double, 6> z1_kp_by_joint_{};
  std::array<double, 6> z1_kd_by_joint_{};

  std::string current_b2_mode_;
  std::string network_interface_;
  double ramp_duration_sec_;
  double ground_support_kp_scale_;
  double ground_support_kd_scale_;
  bool debug_z1_state_;
  rclcpp::Time transition_start_time_;
  rclcpp::Time gain_ramp_start_time_;
  mutable rclcpp::Time last_debug_log_time_;
  double transition_duration_sec_;
  bool have_active_command_;
  bool warned_move_mode_;
  bool use_official_stand_sequence_;
};

}  // namespace b2z1_mujoco_bridge
