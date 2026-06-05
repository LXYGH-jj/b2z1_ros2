#include <chrono>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

#include "b2z1_b2_lowcmd/b2/b2_motion_switch_client.hpp"
#include "b2z1_b2_trajectories/b2_stand_trajectory.hpp"
#include "b2z1_b2_lowcmd/common/motor_crc.h"

namespace b2z1_b2_lowcmd {

class B2RealLowcmdStandNode : public rclcpp::Node {
public:
  B2RealLowcmdStandNode()
  : Node("b2_real_lowcmd_stand_node"),
    msc_(this),
    trajectory_(*this),
    kp_(declare_parameter<double>("kp", 1000.0)),
    kd_(declare_parameter<double>("kd", 10.0)),
    topic_lowcmd_(declare_parameter<std::string>("topic_lowcmd", "/lowcmd")),
    topic_lowstate_(declare_parameter<std::string>("topic_lowstate", "/lowstate"))
  {
    init_lowcmd();

    lowcmd_publisher_ = create_publisher<unitree_go::msg::LowCmd>(topic_lowcmd_, 10);
    lowstate_subscriber_ = create_subscription<unitree_go::msg::LowState>(
      topic_lowstate_, 10,
      [this](const unitree_go::msg::LowState::SharedPtr msg) {
        low_state_ = *msg;
        have_lowstate_ = true;
      });

    initialization_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&B2RealLowcmdStandNode::process_initialization, this));
  }

private:
  void init_lowcmd()
  {
    low_cmd_.head[0] = 0xFE;
    low_cmd_.head[1] = 0xEF;
    low_cmd_.level_flag = 0xFF;
    low_cmd_.gpio = 0;

    for (int i = 0; i < 20; ++i) {
      low_cmd_.motor_cmd[i].mode = 0x0A;
      low_cmd_.motor_cmd[i].q = PosStopF;
      low_cmd_.motor_cmd[i].kp = 0.0f;
      low_cmd_.motor_cmd[i].dq = VelStopF;
      low_cmd_.motor_cmd[i].kd = 0.0f;
      low_cmd_.motor_cmd[i].tau = 0.0f;
    }
  }

  void process_initialization()
  {
    if (control_enabled_) {
      return;
    }

    if (!have_lowstate_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for the first /lowstate message before enabling lowcmd control.");
      return;
    }

    const auto motion_active = query_motion_status();
    if (!motion_active.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Motion-switch status is unavailable. Will retry before enabling lowcmd control.");
      return;
    }

    if (*motion_active) {
      RCLCPP_INFO(get_logger(), "Try to deactivate the motion control-related service.");
      const int32_t ret = msc_.ReleaseMode();
      if (ret == 0) {
        RCLCPP_INFO(get_logger(), "ReleaseMode succeeded. Waiting for confirmation.");
      } else {
        RCLCPP_WARN(get_logger(), "ReleaseMode failed. Error code: %d", ret);
      }
      return;
    }

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(2),
      std::bind(&B2RealLowcmdStandNode::lowcmd_write, this));
    control_enabled_ = true;
    initialization_timer_->cancel();
    RCLCPP_INFO(get_logger(), "Lowcmd control enabled after lowstate and motion-switch checks.");
  }

  std::optional<bool> query_motion_status()
  {
    std::string robot_form;
    std::string motion_name;
    const int32_t ret = msc_.CheckMode(robot_form, motion_name);
    if (ret == 0) {
      RCLCPP_INFO(get_logger(), "CheckMode succeeded.");
    } else {
      RCLCPP_WARN(get_logger(), "CheckMode failed. Error code: %d", ret);
      return std::nullopt;
    }

    if (motion_name.empty()) {
      RCLCPP_INFO(get_logger(), "The motion control-related service is deactivated.");
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "A motion control-related service is still active (form=%s, name=%s).",
      robot_form.c_str(), motion_name.c_str());
    return true;
  }

  void lowcmd_write()
  {
    ++motion_time_;
    if (motion_time_ < 500) {
      get_crc(low_cmd_);
      lowcmd_publisher_->publish(low_cmd_);
      return;
    }

    if (first_run_) {
      for (int i = 0; i < 12; ++i) {
        start_pos_[i] = low_state_.motor_state[i].q;
      }
      trajectory_.set_start_position(start_pos_);
      first_run_ = false;
    }

    const double elapsed_sec = static_cast<double>(motion_time_ - 500) * 0.002;
    const auto target_q = trajectory_.sample(elapsed_sec);

    for (int j = 0; j < 12; ++j) {
      low_cmd_.motor_cmd[j].q = static_cast<float>(target_q[j]);
      low_cmd_.motor_cmd[j].dq = 0.0f;
      low_cmd_.motor_cmd[j].kp = static_cast<float>(kp_);
      low_cmd_.motor_cmd[j].kd = static_cast<float>(kd_);
      low_cmd_.motor_cmd[j].tau = 0.0f;
    }

    get_crc(low_cmd_);
    lowcmd_publisher_->publish(low_cmd_);
  }

  unitree::robot::b2::MotionSwitchClient msc_;
  b2z1_b2_trajectories::B2StandTrajectory trajectory_;
  unitree_go::msg::LowCmd low_cmd_;
  unitree_go::msg::LowState low_state_;

  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmd_publisher_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscriber_;
  rclcpp::TimerBase::SharedPtr initialization_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::array<double, 12> start_pos_{};
  double kp_;
  double kd_;
  std::string topic_lowcmd_;
  std::string topic_lowstate_;
  int motion_time_ = 0;
  bool first_run_ = true;
  bool have_lowstate_ = false;
  bool control_enabled_ = false;
};

}  // namespace b2z1_b2_lowcmd

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<b2z1_b2_lowcmd::B2RealLowcmdStandNode>());
  rclcpp::shutdown();
  return 0;
}
